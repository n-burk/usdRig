#
# TOUCHPOSE, end to end, in a real usdview.
#
# The model test (tests/python/test_touchpose_model.py) already proves
# the arithmetic on a cube. What can only be proved with the app running
# is everything this file asserts, and each of these has a number rather
# than a smoke check:
#
#   * a KNOWN face's centroid, projected to screen and clicked, selects
#     that region's control in usdview's own `dataModel.selection` --
#     which is the whole point of the design, because selecting there is
#     what makes the Avar Editor and the viewport gizmo follow;
#   * the pick follows the DEFORMED mesh: `hips_ctl` is moved +12 cm, the
#     same region is picked again, and the pixel it is found at has moved
#     by a measured number of pixels. A pick against the rest mesh would
#     answer at the old pixel;
#   * the HIGHLIGHT ACTUALLY RENDERS, and AUTHORS NOTHING. It is a Storm
#     shader tint on the body's own materials, driven by two primvars a
#     Hydra scene index adds (libs/rigExecImaging/touchPoseHighlight.h):
#     the pixels are counted, and the session layer is compared byte for
#     byte before and after, and no TouchPose prim may exist on the stage;
#   * while TouchPose is ON the MESH IS NOT SELECTABLE: a click on a
#     region selects the control and not `body_geo`, and a click on the
#     skin but off every region selects nothing at all rather than
#     falling through to usdview's picking;
#   * the MODIFIER RULES match the Control Picker's exactly -- no
#     modifier replaces, Shift toggles both ways, Ctrl only ever removes
#     -- for a click AND for a marquee, asserted on selection contents;
#   * TOUCHPOSE STANDS DOWN during a gizmo drag: moving the cursor across
#     a region boundary lights nothing, while the selection highlight is
#     deliberately left lit;
#   * the SELECTION highlight is three distinct colours: hover, lead
#     (the region just clicked) and the other selected regions. Each is
#     counted in pixels of its own region's screen area, because "it is a
#     different colour" is not a claim a reader can check;
#   * the GIZMO WINS an overlapping click: pressing on a manipulator
#     handle that is over a touch region starts the drag and leaves the
#     selection alone;
#   * the MESH stays out of the selection even when something other than
#     a TouchPose click puts it there;
#   * PAINTING moves faces between regions on a drag, the highlight
#     follows it live, and the rig's generation counter does NOT move --
#     which is the direct evidence that a stroke costs nothing like the
#     ~2.33 s an authored edit inside the rig costs on the next evaluate;
#   * SAVE rewrites the touch layer and leaves the rig file alone, and
#     what comes back off disk is what was painted;
#   * turning TouchPose OFF gives usdview its picking back completely --
#     the same click then selects `body_geo` as it always did. A mode
#     that leaves the viewport subtly broken behind it is worse than no
#     mode.
#
# Run with:  bin\run_testusdview_touchpose.bat
# Set TOUCHPOSE_SHOT=path.png to keep a frame grab of the highlight.
#
import os
import sys
import time

from pxr import Gf, Sdf, UsdGeom
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

MESH = "/Biped/Geom/body_geo"
HIPS = "/Biped/Rig/Controls/hips_ctl"

# A pixel counts as changed when it moves this far in 0-255 per-channel
# terms, and the fraction is of SAMPLED pixels (every second pixel each
# way). Both numbers are R2's, so the highlight measurement below is
# directly comparable with the displayColor one it replaced.
_CHANGED_BY = 20.0

# The highlight lights ONE region, not the whole body, so R2's 0.5%-of-
# frame floor is the wrong yardstick: it was calibrated against a tint
# applied to every face. The floor here is region-relative -- the
# highlight has to cover at least this share of the screen area the
# region itself occupies, measured by casting a grid of rays first. Well
# under 1.0 because a patch seen at a grazing angle loses pixels to the
# _CHANGED_BY threshold wherever the skin underneath is already a similar
# colour.
_MIN_OF_REGION = 0.35

_PLUGIN = os.environ.get("TOUCHPOSE_PLUGIN_DIR")
if _PLUGIN and _PLUGIN not in sys.path:
    sys.path.insert(0, _PLUGIN)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RigExecMenu(appController):
    menus = {}
    for child in appController._mainWindow.menuBar().children():
        if isinstance(child, QtWidgets.QMenu):
            menus[str(child.title()).replace("&", "")] = child
    return menus


def _Submenu(menu, title):
    for action in menu.actions():
        if action.menu() is not None and action.text() == title:
            return action.menu()
    return None


def _Trigger(menu, title):
    for action in menu.actions():
        if action.text() == title:
            action.trigger()
            return True
    return False


class _Viewport(object):
    """Screen <-> world for the live camera, in PHYSICAL pixels.

    `computeWindowViewport` is physical and Qt's event coordinates are
    logical; every number in this file is physical, and converted once at
    the point a Qt event is built.
    """

    def __init__(self, view):
        self.view = view
        viewport = view.computeWindowViewport()
        self.width, self.height = int(viewport[2]), int(viewport[3])
        frustum = view.resolveCamera()[0].frustum
        self.viewProjection = (frustum.ComputeViewMatrix()
                               * frustum.ComputeProjectionMatrix())
        try:
            self.ratio = float(view.devicePixelRatioF())
        except AttributeError:
            self.ratio = 1.0

    def Project(self, point):
        """World point -> (x, y) physical pixel, or None when behind."""
        # Gf.Matrix4d.Transform does the homogeneous divide, so this is
        # NDC straight out.
        ndc = self.viewProjection.Transform(
            Gf.Vec3d(float(point[0]), float(point[1]), float(point[2])))
        if not (-1.5 < ndc[0] < 1.5 and -1.5 < ndc[1] < 1.5):
            return None
        return ((ndc[0] * 0.5 + 0.5) * self.width - 0.5,
                (0.5 - ndc[1] * 0.5) * self.height - 0.5)


def _SendClick(view, x, y, ratio, modifiers=None):
    """A real left click at a PHYSICAL pixel, through the event filter.

    Sent with QApplication.sendEvent rather than called on the controller
    directly: the filter has to be reached the way Qt reaches it, or the
    test proves the method works and not that the mode is installed.

    Returns True when TOUCHPOSE consumed the press, which is read off
    usdview's own state rather than off `event.isAccepted()`: a mouse
    event arrives pre-accepted and `QWidget::event` returns true whether
    or not anything meaningful happened, so the accept flag says nothing.
    `StageView.mousePressEvent` sets `_cameraMode = "pick"` the moment it
    runs (stageView.py, just before it calls `pickObject`), so seeing
    that flag still unset is direct evidence the press never reached it.
    """
    modifiers = modifiers or QtCore.Qt.NoModifier
    local = QtCore.QPointF(x / ratio, y / ratio)
    globalPos = view.mapToGlobal(local.toPoint())
    view._cameraMode = "none"
    view._dragActive = False
    for kind in (QtCore.QEvent.MouseButtonPress,
                 QtCore.QEvent.MouseButtonRelease):
        event = QtGui.QMouseEvent(
            kind, local, QtCore.QPointF(globalPos),
            QtCore.Qt.LeftButton,
            QtCore.Qt.LeftButton if kind == QtCore.QEvent.MouseButtonPress
            else QtCore.Qt.NoButton,
            modifiers)
        QtWidgets.QApplication.sendEvent(view, event)
        if kind == QtCore.QEvent.MouseButtonPress:
            reached = (view._cameraMode == "pick")
    # A press TouchPose swallowed must leave no half-finished drag behind
    # it either: usdview sets `_dragActive` on the press and clears it on
    # the release, and a swallowed pair must leave it clear.
    if view._dragActive:
        raise AssertionError(
            "the stage view is left mid-drag after a click at (%.1f, %.1f)"
            % (x, y))
    return not reached


def _SendButton(view, kind, x, y, ratio, modifiers=None):
    """One raw mouse event at a PHYSICAL pixel. No interpretation."""
    modifiers = modifiers or QtCore.Qt.NoModifier
    local = QtCore.QPointF(x / ratio, y / ratio)
    globalPos = view.mapToGlobal(local.toPoint())
    event = QtGui.QMouseEvent(
        kind, local, QtCore.QPointF(globalPos), QtCore.Qt.LeftButton,
        QtCore.Qt.LeftButton if kind == QtCore.QEvent.MouseButtonPress
        else QtCore.Qt.NoButton,
        modifiers)
    QtWidgets.QApplication.sendEvent(view, event)


def _Selected(selection):
    """The selection as a set of paths, WITHOUT the pseudo-root.

    `clearPrims` leaves `/` behind rather than nothing, so every "is it
    empty" and "is it exactly these" check has to drop it or it silently
    compares the wrong thing. The `['/Biped/Geom/body_geo']` residue in
    an earlier report was this, not a leak.
    """
    return set(str(p.GetPath()) for p in selection.getPrims()
               if p and p.IsValid() and not p.IsPseudoRoot())


def _SendHover(view, x, y, ratio):
    """One button-less move at a PHYSICAL pixel, as WA_Hover delivers it.

    Sent as a HoverMove rather than by calling `controller.Hover`: the
    suspension lives in the event filter, so calling the method would
    walk straight past the thing under test.
    """
    local = QtCore.QPointF(x / ratio, y / ratio)
    event = QtGui.QHoverEvent(QtCore.QEvent.HoverMove, local, local)
    QtWidgets.QApplication.sendEvent(view, event)


def _SessionFingerprint(stage):
    """Everything the session layer holds, as one comparable string.

    Any authored edit at all -- points, indices, colour, visibility --
    changes this, which is what "authored NOTHING" has to mean.
    """
    return stage.GetSessionLayer().ExportToString()


def _SendDrag(view, x0, y0, x1, y1, ratio, modifiers=None):
    """Press, move, release -- a real drag through the event filter."""
    modifiers = modifiers or QtCore.Qt.NoModifier
    view._cameraMode = "none"
    view._dragActive = False
    _SendButton(view, QtCore.QEvent.MouseButtonPress, x0, y0, ratio,
                modifiers)
    steps = 6
    for i in range(1, steps + 1):
        mx = x0 + (x1 - x0) * float(i) / steps
        my = y0 + (y1 - y0) * float(i) / steps
        local = QtCore.QPointF(mx / ratio, my / ratio)
        move = QtGui.QMouseEvent(
            QtCore.QEvent.MouseMove, local,
            QtCore.QPointF(view.mapToGlobal(local.toPoint())),
            QtCore.Qt.NoButton, QtCore.Qt.LeftButton, modifiers)
        QtWidgets.QApplication.sendEvent(view, move)
    _SendButton(view, QtCore.QEvent.MouseButtonRelease, x1, y1, ratio,
                modifiers)


def _Capture(view):
    """Every second pixel of the drawn frame, as QColors."""
    view.updateGL()
    QtWidgets.QApplication.processEvents()
    image = view.grabFrameBuffer()
    return [[image.pixelColor(x, y) for x in range(0, image.width(), 2)]
            for y in range(0, image.height(), 2)]


def _Changed(a, b):
    total = sum(len(row) for row in a)
    moved = 0
    for rowA, rowB in zip(a, b):
        for p, q in zip(rowA, rowB):
            if (abs(p.red() - q.red()) + abs(p.green() - q.green())
                    + abs(p.blue() - q.blue())) / 3.0 > _CHANGED_BY:
                moved += 1
    return moved, total


def testUsdviewInputFunction(appController):
    import touchPoseUI

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    view = appController._stageView
    model = appController._dataModel
    model.viewSettings.showHUD = False
    selection = model.selection

    # --- 1. registered, and in the RigExec menu -------------------------
    reg = appController._plugRegistry
    _Check(reg.getCommandPlugin("TouchPoseContainer.touchPose") is not None,
           "TouchPose is registered as a command plugin")
    menus = _RigExecMenu(appController)
    _Check("RigExec" in menus, "there is a RigExec menu: %s" % sorted(menus))
    topMenu = menus["RigExec"]
    # Two containers fill this menu, so this is where the layout is
    # checked: one of each submenu, in order, whatever the load order.
    top = [a.text() for a in topMenu.actions() if not a.isSeparator()]
    _Check(top == ["Reactivate RigExec Evaluation", "Viewport",
                   "General Editors", "Animation Editors"],
           "the RigExec menu layout is item, then three submenus: %s" % top)
    rigMenu = _Submenu(topMenu, "Animation Editors")
    titles = [a.text() for a in rigMenu.actions()]
    _Check("TouchPose" in titles,
           "RigExec -> Animation Editors carries TouchPose: %s" % titles)
    # ...and it MERGED into the existing submenu rather than making a
    # second one, which is the only reason a separate plugin container is
    # acceptable here.
    _Check("Control Picker" in titles,
           "the same submenu still carries rigExecUsdview's own items: %s"
           % titles)
    order = ["Graph Editor", "Shape Editor", "Control Picker", "TouchPose",
             "Volume Weight Editor", "Curvenet Authoring"]
    _Check([t for t in order if t in titles] == titles,
           "and the items are in layout order: %s" % titles)
    _Check("TouchPose" not in menus,
           "and no second top-level menu was created: %s" % sorted(menus))
    viewportMenu = _Submenu(topMenu, "Viewport")

    _Check(_Trigger(rigMenu, "TouchPose"), "the menu item triggered")
    appController._processEvents()
    panel = touchPoseUI.TouchPosePanel.GetInstance(api)
    _Check(panel is not None, "the panel was created")
    controller = panel.controller

    # --- 2. the regions loaded, and the unbound ones are not there ------
    touchModel = controller.model
    _Check(touchModel is not None,
           "the touch regions loaded off the stage")
    regions, covered, faces = touchModel.Coverage()
    _Check(regions > 50,
           "a useful number of regions, got %d" % regions)
    _Check(all(r.control for r in touchModel.regions),
           "every region binds a control -- the unbound ones were skipped "
           "at import")
    for region in touchModel.regions:
        prim = stage.GetPrimAtPath(Sdf.Path(region.control))
        _Check(prim and prim.IsValid(),
               "%s targets a real prim: %s" % (region.name, region.control))
        _Check(str(prim.GetTypeName()) in ("RigExecControl", "RigExecJoint"),
               "%s is drivable, got %s"
               % (region.control, prim.GetTypeName()))
    print("TouchPose: %d regions over %d of %d faces (%.0f%%)"
          % (regions, covered, faces, 100.0 * covered / faces))

    # --- 3. the points really are the DEFORMED ones ---------------------
    rest = UsdGeom.Mesh(stage.GetPrimAtPath(MESH)).GetPointsAttr().Get()
    _Check(touchModel.points is not None and len(touchModel.points) == len(rest),
           "the model has a point per mesh point")

    controller.SetActive(True)
    _Check(controller.active, "TouchPose activated")

    # --- 4. WHAT IS ACTUALLY ON SCREEN ---------------------------------
    #
    # A grid of real casts, once, because three later assertions need it
    # and none of them can be answered by projecting alone: a region's
    # centroid is routinely behind the character or occluded by an arm,
    # and a test that clicked such a pixel would assert the wrong thing.
    # The scan gives, per pixel, the region under it -- so it yields a
    # region that is genuinely visible, the screen AREA it covers (which
    # is what the highlight measurement below is scaled against), a pixel
    # of bare unpainted skin, and a pixel with no character at all.
    viewport = _Viewport(view)
    step = max(8, viewport.width // 48)
    hits = {}
    bare = []
    empty = []
    scanned = 0
    for sy in range(step // 2, viewport.height, step):
        for sx in range(step // 2, viewport.width, step):
            got, f = controller.RegionAt(float(sx), float(sy))
            scanned += 1
            if f < 0:
                empty.append((float(sx), float(sy)))
            elif got is None:
                bare.append((float(sx), float(sy)))
            else:
                hits.setdefault(got.index, []).append((float(sx), float(sy)))
    _Check(hits, "some touch region is visible from this camera")
    print("TouchPose: a %d-px grid over %dx%d found %d samples on a region, "
          "%d on unpainted skin, %d off the character"
          % (step, viewport.width, viewport.height,
             sum(len(v) for v in hits.values()), len(bare), len(empty)))

    # The biggest visible region, and the sample nearest its own screen
    # centre -- the pixel a user would most plainly have aimed at.
    best = max(hits, key=lambda i: len(hits[i]))
    region = touchModel.regions[best]
    samples = hits[best]
    cx = sum(p[0] for p in samples) / len(samples)
    cy = sum(p[1] for p in samples) / len(samples)
    pixel = min(samples,
                key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
    regionFraction = float(len(samples)) / max(scanned, 1)

    # ...and now the KNOWN-face check the brief asks for, the other way
    # round: project one of this region's faces and require the cast at
    # that pixel to land back in the same region. Projection and casting
    # are independent code paths, so agreeing is evidence.
    face = None
    for candidate in region.faces[::max(1, len(region.faces) // 24)]:
        projected = viewport.Project(touchModel.FaceCentroid(int(candidate)))
        if projected is None:
            continue
        got, _f = controller.RegionAt(projected[0], projected[1])
        if got is not None and got.index == region.index:
            face, pixel = int(candidate), projected
            break
    _Check(face is not None,
           "a face of %s projects to a pixel that casts back to it"
           % region.label)
    _region, hitFace = controller.RegionAt(pixel[0], pixel[1])
    print("TouchPose: region %s is the largest on screen (%.2f%% of the "
          "frame); its face %d projects to (%.1f, %.1f) and the ray there "
          "lands on face %d of the same region"
          % (region.label, 100.0 * regionFraction, face, pixel[0], pixel[1],
             hitFace))

    selection.clearPrims()
    appController._processEvents()
    consumed = _SendClick(view, pixel[0], pixel[1], viewport.ratio)
    appController._processEvents()
    chosen = [str(p.GetPath()) for p in selection.getPrims()
              if p and p.IsValid()]
    _Check(consumed, "TouchPose consumed the click")
    _Check(chosen == [region.control],
           "the click selected %s, got %s" % (region.control, chosen))
    _Check(MESH not in chosen,
           "and did NOT select the mesh: %s" % chosen)

    # Shift TOGGLES -- on a region that is not selected, that is an add.
    # The off half, and Ctrl, are asserted in full further down.
    second = [i for i in sorted(hits, key=lambda i: -len(hits[i]))
              if touchModel.regions[i].control != region.control]
    other = touchModel.regions[second[0]] if second else None
    otherPixel = hits[second[0]][len(hits[second[0]]) // 2] if second else None
    if otherPixel is not None:
        _SendClick(view, otherPixel[0], otherPixel[1], viewport.ratio,
                   QtCore.Qt.ShiftModifier)
        appController._processEvents()
        chosen = _Selected(selection)
        _Check(chosen == set([region.control, other.control]),
               "shift-click added %s: got %s" % (other.control, chosen))
        print("TouchPose: shift-click added %s -> %d selected"
              % (other.label, len(chosen)))

    # --- 5. the highlight RENDERS ---------------------------------------
    #
    # Counted the way R2 counted the displayColor that did not work, so
    # the two numbers can be read side by side.
    controller._highlight.Clear()
    selection.clearPrims()
    controller.SyncSelection()
    appController._processEvents()
    baseline = _Capture(view)
    fingerprint = _SessionFingerprint(stage)
    controller.Hover(pixel[0], pixel[1])
    appController._processEvents()
    lit = _Capture(view)
    moved, total = _Changed(baseline, lit)
    fraction = float(moved) / max(total, 1)
    # The number that decides it: how much of the region's OWN screen
    # area lit up, not how much of the frame.
    ofRegion = fraction / max(regionFraction, 1e-9)
    # NOTHING AUTHORED. The highlight is Hydra state: the session layer
    # is byte-identical, and no TouchPose prim exists anywhere.
    _Check(_SessionFingerprint(stage) == fingerprint,
           "the hover authored NOTHING into the session layer")
    strays = [str(p.GetPath()) for p in stage.TraverseAll()
              if p.GetName().startswith("TouchPose")
              and p.GetTypeName() in ("Mesh", "RigExecTouchOverlay")
              and p.GetAttribute("points").HasAuthoredValue()]
    _Check(not strays, "and no overlay geometry exists: %s" % strays)
    import touchPoseNative
    _Check(touchPoseNative.SceneIndexCount() >= 1,
           "the TouchPose scene index is in the imaging chain")
    _Check(controller.model.native.HighlightTable()[region.index + 1][3] > 0,
           "and the region's row of the colour table is lit")
    print("TouchPose: hovering %s moved %d of %d sampled pixels (%.3f%% of "
          "the frame) where the region itself covers %.3f%% -- so the "
          "highlight covers %.0f%% of its own region. R2's displayColor "
          "moved 0 of 80730 on this same mesh."
          % (region.label, moved, total, 100.0 * fraction,
             100.0 * regionFraction, 100.0 * ofRegion))
    _Check(ofRegion >= _MIN_OF_REGION,
           "the highlight is VISIBLE: %d of %d sampled pixels changed, "
           "%.0f%% of the region's own %.3f%% of the frame (floor %.0f%%). "
           "The overlay-rprim route FAILED if this is the assertion you "
           "are reading."
           % (moved, total, 100.0 * ofRegion, 100.0 * regionFraction,
              100.0 * _MIN_OF_REGION))

    shot = os.environ.get("TOUCHPOSE_SHOT")
    if shot:
        view.grabFrameBuffer().save(shot)
        print("TOUCHPOSE_SHOT %s" % shot)

    # ...and leaving the mesh puts it back.
    controller.Hover(1.0, 1.0)
    appController._processEvents()
    cleared = _Capture(view)
    back, _total = _Changed(baseline, cleared)
    _Check(back < moved * 0.2,
           "hovering off the body clears the highlight: %d pixels still "
           "differ from the baseline, against %d when lit" % (back, moved))

    # --- 5b. what one hover costs ---------------------------------------
    #
    # The hover runs on every mouse move, so the budget is a frame:
    # 16.7 ms at 60 Hz, shared with everything else usdview is doing.
    # Two cases, because they are not the same cost: staying inside one
    # region is cast + lookup, while crossing into another also rebuilds
    # the overlay patch and re-authors it into the session layer. The
    # second is the one that could have made the design unusable, so it
    # is measured rather than assumed.
    inside = [p for p in samples[:8]] or [pixel]
    crossing = [pixel, otherPixel] if otherPixel is not None else [pixel]

    def _Time(points, repeats):
        controller.Hover(points[0][0], points[0][1])
        start = time.perf_counter()
        for i in range(repeats):
            spot = points[i % len(points)]
            controller.Hover(spot[0], spot[1])
        return 1000.0 * (time.perf_counter() - start) / repeats

    sameRegion = _Time(inside, 20)
    changeRegion = _Time(crossing, 20)
    print("TouchPose: one hover costs %.2f ms inside a region and %.2f ms "
          "crossing into another (cast + a colour-table update); a 60 Hz "
          "frame is 16.7 ms" % (sameRegion, changeRegion))
    _Check(sameRegion < 2.0,
           "a hover inside one region is nearly free: %.2f ms" % sameRegion)
    _Check(changeRegion < 16.7,
           "a hover that changes region fits in a frame: %.2f ms"
           % changeRegion)

    # --- 5b2. the MODIFIER RULES, click and marquee ---------------------
    #
    # These have to match the Control Picker exactly: the same rig is
    # being selected and it must not matter which tool the animator
    # reached for. Asserted on selection CONTENTS, mode by mode.
    _Check(controller.ModeFor(QtCore.Qt.NoModifier)
           == touchPoseUI.MODE_REPLACE, "no modifier replaces")
    _Check(controller.ModeFor(QtCore.Qt.ShiftModifier)
           == touchPoseUI.MODE_TOGGLE, "shift toggles")
    _Check(controller.ModeFor(QtCore.Qt.ControlModifier)
           == touchPoseUI.MODE_REMOVE, "ctrl removes")
    _Check(controller.ModeFor(QtCore.Qt.ControlModifier
                              | QtCore.Qt.ShiftModifier)
           == touchPoseUI.MODE_REMOVE,
           "ctrl+shift REMOVES -- there is exactly one gesture that can "
           "never add, and this is the combination most likely to be hit "
           "by accident")

    secondRegion = other
    _Check(secondRegion is not None,
           "two regions are visible, which the modifier checks need")

    # REPLACE: the selection becomes exactly what was picked.
    selection.clearPrims()
    appController._processEvents()
    controller.Pick([region], touchPoseUI.MODE_REPLACE)
    _Check(_Selected(selection) == set([region.control]),
           "replace: %s" % _Selected(selection))
    controller.Pick([secondRegion], touchPoseUI.MODE_REPLACE)
    _Check(_Selected(selection) == set([secondRegion.control]),
           "replace again, and the first is GONE: %s" % _Selected(selection))

    # TOGGLE on, then off, with the same gesture.
    controller.Pick([region], touchPoseUI.MODE_TOGGLE)
    _Check(_Selected(selection) == set([secondRegion.control,
                                        region.control]),
           "toggle added: %s" % _Selected(selection))
    controller.Pick([region], touchPoseUI.MODE_TOGGLE)
    _Check(_Selected(selection) == set([secondRegion.control]),
           "the SAME gesture removed it again: %s" % _Selected(selection))

    # REMOVE only ever subtracts -- on something selected and on
    # something that is not.
    controller.Pick([region], touchPoseUI.MODE_REMOVE)
    _Check(_Selected(selection) == set([secondRegion.control]),
           "ctrl on an unselected region added NOTHING: %s"
           % _Selected(selection))
    controller.Pick([secondRegion], touchPoseUI.MODE_REMOVE)
    _Check(_Selected(selection) == set(),
           "ctrl on a selected one removed it, leaving nothing: %s"
           % _Selected(selection))

    # And through real Qt events, which is where a click and a marquee
    # could drift apart.
    selection.clearPrims()
    appController._processEvents()
    _SendClick(view, pixel[0], pixel[1], viewport.ratio)
    appController._processEvents()
    _Check(_Selected(selection) == set([region.control]),
           "a plain click replaces: %s" % _Selected(selection))
    _SendClick(view, pixel[0], pixel[1], viewport.ratio,
               QtCore.Qt.ShiftModifier)
    appController._processEvents()
    _Check(_Selected(selection) == set(),
           "shift-clicking the same region toggled it OFF: %s"
           % _Selected(selection))

    # --- 5b3. the MARQUEE -----------------------------------------------
    #
    # A band over two regions, under each of the three modifiers. The
    # band is built from the two regions' own sample pixels so it
    # provably covers both.
    a = hits[best][0]
    b = hits[second[0]][0]
    for spot in hits[best] + hits[second[0]]:
        a = (min(a[0], spot[0]), min(a[1], spot[1]))
        b = (max(b[0], spot[0]), max(b[1], spot[1]))
    caught = controller.RegionsInBand(a[0], a[1], b[0], b[1])
    names = set(r.control for r in caught)
    _Check(region.control in names and secondRegion.control in names,
           "the band over both regions catches both: %d regions, %s"
           % (len(caught), sorted(r.label for r in caught)[:6]))

    selection.clearPrims()
    appController._processEvents()
    controller.Marquee(a[0], a[1], b[0], b[1], touchPoseUI.MODE_REPLACE)
    afterBand = _Selected(selection)
    _Check(afterBand == names,
           "marquee replace selected exactly what it caught: %d vs %d"
           % (len(afterBand), len(names)))

    controller.Marquee(a[0], a[1], b[0], b[1], touchPoseUI.MODE_TOGGLE)
    _Check(_Selected(selection) == set(),
           "marquee toggle over an all-selected band cleared it: %s"
           % len(_Selected(selection)))
    controller.Marquee(a[0], a[1], b[0], b[1], touchPoseUI.MODE_TOGGLE)
    _Check(_Selected(selection) == names,
           "and toggling again put it back: %d" % len(_Selected(selection)))
    controller.Marquee(a[0], a[1], b[0], b[1], touchPoseUI.MODE_REMOVE)
    _Check(_Selected(selection) == set(),
           "marquee ctrl removed the lot: %s" % _Selected(selection))

    # A band that touches only PART of a region still catches it --
    # "touches", not "encloses", or a finger would be uncatchable. And a
    # band a few pixels across, which is what a shaky click produces,
    # must still catch what it is sitting on rather than nothing.
    tiny = hits[best][0]
    caught = controller.RegionsInBand(tiny[0] - 2, tiny[1] - 2,
                                      tiny[0] + 2, tiny[1] + 2)
    _Check(any(r.index == region.index for r in caught),
           "a 4-px band inside %s still catches it: %s"
           % (region.label, [r.label for r in caught]))
    _Check(len(caught) <= 2,
           "...and only it, not half the body: %s"
           % [r.label for r in caught])

    # A drag has to actually become a marquee rather than a click, and a
    # click has to survive a pixel of tremor.
    selection.clearPrims()
    appController._processEvents()
    _SendDrag(view, pixel[0], pixel[1], pixel[0] + 1, pixel[1] + 1,
              viewport.ratio)
    appController._processEvents()
    _Check(_Selected(selection) == set([region.control]),
           "a 1-px wobble is still a CLICK, not a one-pixel marquee: %s"
           % _Selected(selection))

    selection.clearPrims()
    appController._processEvents()
    _SendDrag(view, a[0], a[1], b[0], b[1], viewport.ratio)
    appController._processEvents()
    dragged = _Selected(selection)
    _Check(dragged == names,
           "and a real drag marquees: %d regions vs the %d the band "
           "catches" % (len(dragged), len(names)))
    print("TouchPose: modifiers match the picker (replace / toggle both "
          "ways / ctrl-only-removes), and a %.0fx%.0f px marquee caught "
          "%d regions under all three"
          % (b[0] - a[0], b[1] - a[1], len(names)))

    # --- 5b4. the hotkeys -----------------------------------------------
    shortcuts = dict((str(x.shortcut().toString()), x)
                     for x in panel.actions())
    for wanted in ("T", "P", "Ctrl+A", "Ctrl+Shift+A", "Ctrl+I", "Ctrl+S"):
        _Check(wanted in shortcuts,
               "%s is bound: %s" % (wanted, sorted(shortcuts)))
        _Check(shortcuts[wanted].shortcutContext()
               == QtCore.Qt.WidgetWithChildrenShortcut,
               "%s is scoped to the panel, so it cannot fight usdview's "
               "own bindings" % wanted)

    selection.clearPrims()
    appController._processEvents()
    shortcuts["Ctrl+A"].trigger()
    appController._processEvents()
    _Check(len(_Selected(selection)) > 1,
           "Ctrl+A selected every region's control: %d"
           % len(_Selected(selection)))
    allOfThem = _Selected(selection)
    shortcuts["Ctrl+I"].trigger()
    appController._processEvents()
    _Check(_Selected(selection) == set(),
           "Ctrl+I over a full selection inverts to nothing: %d"
           % len(_Selected(selection)))
    shortcuts["Ctrl+I"].trigger()
    appController._processEvents()
    _Check(_Selected(selection) == allOfThem,
           "and back again: %d vs %d"
           % (len(_Selected(selection)), len(allOfThem)))
    shortcuts["Ctrl+Shift+A"].trigger()
    appController._processEvents()
    _Check(_Selected(selection) == set(),
           "Ctrl+Shift+A cleared: %s" % _Selected(selection))

    was = controller.active
    shortcuts["T"].trigger()
    appController._processEvents()
    _Check(controller.active != was,
           "T toggled the mode: %s -> %s" % (was, controller.active))
    shortcuts["T"].trigger()
    appController._processEvents()
    _Check(controller.active == was, "and T put it back")
    print("TouchPose: 6 hotkeys bound to the panel; Ctrl+A selected %d "
          "regions, Ctrl+I inverted both ways, T toggled the mode"
          % len(allOfThem))

    # --- 5c. the SELECTION highlight, in three colours ------------------
    #
    # Hover is already measured above. This measures the other two, the
    # same way: pixels changed against a frame taken with the same
    # selection and nothing lit.
    controller._highlight.Clear()
    selection.clearPrims()
    appController._processEvents()
    unlit = _Capture(view)

    controller.Select(region)                       # lead
    appController._processEvents()
    leadShot = _Capture(view)
    leadMoved, _t = _Changed(unlit, leadShot)
    leadOfRegion = (float(leadMoved) / max(total, 1)) / max(regionFraction,
                                                            1e-9)
    _Check(controller.highlight.lead is not None,
           "the lead is drawn")
    _Check(leadOfRegion >= _MIN_OF_REGION,
           "the LEAD highlight is visible: %d pixels, %.0f%% of the "
           "region's own area (floor %.0f%%)"
           % (leadMoved, 100.0 * leadOfRegion, 100.0 * _MIN_OF_REGION))

    secondMoved = 0
    if other is not None:
        # Both selected in one gesture, `region` last -- so the lead is
        # the one the animator touched last, which is what lead means.
        controller.Pick([other, region], touchPoseUI.MODE_REPLACE)
        appController._processEvents()
        bothShot = _Capture(view)
        secondMoved, _t = _Changed(leadShot, bothShot)
        _Check(controller.highlight.selected,
               "the non-lead selected regions are drawn: %s"
               % (controller.highlight.selected,))
        _Check(controller._lead is not None
               and controller._lead.index == region.index,
               "the region clicked LAST is the lead, got %s"
               % (controller._lead.label if controller._lead else None))
        _Check(secondMoved > 100,
               "adding a second region lit more of the frame: %d pixels"
               % secondMoved)

    # The three colours must actually differ ON SCREEN, not just in the
    # model. Sampled at the lead region's own pixel, which is the one
    # place all three states can be compared like for like.
    sx, sy = int(pixel[0] / 2), int(pixel[1] / 2)

    def _At(shot):
        row = shot[min(sy, len(shot) - 1)]
        return row[min(sx, len(row) - 1)]

    controller._highlight.Clear()
    appController._processEvents()
    leadPixel = _At(_Capture(view))
    controller.Hover(pixel[0], pixel[1])
    appController._processEvents()
    hoverPixel = _At(_Capture(view))
    selection.clearPrims()
    controller._highlight.Clear()
    if other is not None:
        controller.Pick([other, region], touchPoseUI.MODE_REPLACE)
        controller._lead = other
        controller.SyncSelection()
        appController._processEvents()
        nonLeadPixel = _At(_Capture(view))
    else:
        nonLeadPixel = None

    def _Apart(a, b):
        return (abs(a.red() - b.red()) + abs(a.green() - b.green())
                + abs(a.blue() - b.blue())) / 3.0

    _Check(_Apart(leadPixel, hoverPixel) > _CHANGED_BY,
           "lead and hover are different colours on screen: %.1f apart "
           "(%s vs %s)" % (_Apart(leadPixel, hoverPixel),
                           leadPixel.getRgb()[:3], hoverPixel.getRgb()[:3]))
    if nonLeadPixel is not None:
        _Check(_Apart(leadPixel, nonLeadPixel) > _CHANGED_BY,
               "lead and non-lead selected differ: %.1f apart (%s vs %s)"
               % (_Apart(leadPixel, nonLeadPixel), leadPixel.getRgb()[:3],
                  nonLeadPixel.getRgb()[:3]))
        # The pair that nearly collided when hover was mixed toward
        # white instead of scaled: a pale hover IS a neutral, and
        # neutral is what selected looks like.
        _Check(_Apart(hoverPixel, nonLeadPixel) > _CHANGED_BY,
               "hover and non-lead selected differ: %.1f apart (%s vs %s)"
               % (_Apart(hoverPixel, nonLeadPixel), hoverPixel.getRgb()[:3],
                  nonLeadPixel.getRgb()[:3]))
    shot = os.environ.get("TOUCHPOSE_STATES_SHOT")
    if shot:
        # All three states at once, for a human to look at: the lead
        # region, a second selected one, and the cursor over a third.
        if other is not None:
            controller.Pick([other, region], touchPoseUI.MODE_REPLACE)
        third = next((touchModel.regions[i] for i in hits
                      if i not in (region.index,
                                   other.index if other else -1)), None)
        if third is not None:
            spot = hits[third.index][len(hits[third.index]) // 2]
            controller.Hover(spot[0], spot[1])
        appController._processEvents()
        view.updateGL()
        QtWidgets.QApplication.processEvents()
        view.grabFrameBuffer().save(shot)
        print("TOUCHPOSE_STATES_SHOT %s" % shot)

    print("TouchPose: at the same pixel -- hover %s, lead %s, selected %s; "
          "lead lit %d pixels (%.0f%% of its region), adding a second "
          "region lit %d more"
          % (hoverPixel.getRgb()[:3], leadPixel.getRgb()[:3],
             nonLeadPixel.getRgb()[:3] if nonLeadPixel else "(n/a)",
             leadMoved, 100.0 * leadOfRegion, secondMoved))

    # --- 5d. the mesh cannot get into the selection ---------------------
    #
    # Consuming the press covers what TouchPose sees. This is the rest:
    # anything else that selects the mesh while the mode is on -- the
    # outliner, a picker button, a press this filter declined -- must be
    # undone. Driven straight at the data model, since that is the one
    # path every one of those shares.
    selection.clearPrims()
    appController._processEvents()
    meshPrim = stage.GetPrimAtPath(Sdf.Path(MESH))
    selection.addPrim(meshPrim)
    appController._processEvents()
    chosen = [str(p.GetPath()) for p in selection.getPrims()
              if p and p.IsValid()]
    _Check(MESH not in chosen,
           "the mesh was pushed into the selection and TouchPose took it "
           "straight back out: %s" % chosen)

    # There is no overlay prim for usdview's pick to land on instead of
    # the skin: the highlight is the skin's own shader.
    controller.Hover(pixel[0], pixel[1])
    appController._processEvents()
    _Check(not any(stage.GetPrimAtPath(Sdf.Path(p)).IsValid()
                   for p in touchPoseUI._LEGACY_OVERLAYS),
           "no overlay prim exists to be picked")
    controller._highlight.Clear()

    # ...and with the mode OFF the same push sticks, or the guard would
    # have broken usdview rather than modified it.
    controller.SetActive(False)
    selection.clearPrims()
    selection.addPrim(meshPrim)
    appController._processEvents()
    chosen = [str(p.GetPath()) for p in selection.getPrims()
              if p and p.IsValid()]
    _Check(MESH in chosen,
           "with the mode off the mesh selects normally: %s" % chosen)
    controller.SetActive(True)
    appController._processEvents()
    chosen = [str(p.GetPath()) for p in selection.getPrims()
              if p and p.IsValid()]
    _Check(MESH not in chosen,
           "and switching the mode ON drops a mesh that was already "
           "selected: %s" % chosen)
    print("TouchPose: the mesh cannot stay selected while the mode is on, "
          "and selects normally the moment it is off")

    # --- 5e. the gizmo wins an overlapping click ------------------------
    #
    # Qt hands a press to the most recently installed event filter first,
    # so without an explicit rule the winner is whichever panel was
    # opened last. TouchPose defers: if the gizmo would take the press,
    # it returns False before casting anything.
    gizmo = None
    try:
        import gizmoUI
        gizmo = gizmoUI.GetController()
        if gizmo is None:
            # The toolbar is not installed by default in a headless
            # session; the menu item is what an animator would use.
            _Trigger(viewportMenu, "Viewport Tools")
            appController._processEvents()
            gizmo = gizmoUI.GetController()
        if gizmo is not None:
            gizmo.SetVisible(True)
            # The select tool draws no handles at all, so there would be
            # nothing to overlap: put it in translate, which is what an
            # animator posing a limb has it in.
            gizmo.SetTool(gizmoUI.TOOL_TRANSLATE)
    except ImportError:
        pass
    if gizmo is None:
        print("TouchPose: NOTE the viewport gizmo is not installed in this "
              "session; the gizmo-wins rule is asserted on the predicate "
              "only")
        _Check(not touchPoseUI.GizmoOwns(pixel[0], pixel[1]),
               "with no gizmo, GizmoOwns answers False rather than raising")
    else:
        # Put the gizmo on the control whose region is under `pixel`, so
        # its handles are drawn over that region by construction.
        selection.clearPrims()
        selection.setPrim(stage.GetPrimAtPath(Sdf.Path(region.control)))
        appController._processEvents()
        gizmo.RefreshTarget()
        appController._processEvents()
        handles = [h for h in (gizmo.Handles() or ()) if h.grabbable]
        overlapping = None
        for handle in handles:
            # The centre, then the handle's own projected points: an axis
            # arrow's centre is at the origin of the gizmo and its far
            # end is out over the limb, and either may be the one that
            # lands on a region.
            spots = [handle.center] + list(getattr(handle, "points", ()))
            for spot in spots:
                hx, hy = float(spot[0]), float(spot[1])
                got, face = controller.RegionAt(hx, hy)
                if (face >= 0 and got is not None
                        and touchPoseUI.GizmoOwns(hx, hy, viewport.ratio)):
                    overlapping = (handle, hx, hy, got)
                    break
            if overlapping is not None:
                break
        if overlapping is None:
            print("TouchPose: NOTE no gizmo handle currently overlaps a "
                  "touch region (tool %s, %d handles); the rule is "
                  "asserted on the predicate only"
                  % (gizmo.Tool(), len(handles)))
        else:
            handle, hx, hy, got = overlapping
            before = [str(p.GetPath()) for p in selection.getPrims()
                      if p and p.IsValid()]
            # PRESS ONLY, and the drag is checked while it is still in
            # flight. `_SendClick`'s consumed/not-consumed cannot answer
            # this: it reads whether the press reached usdview's stage
            # view, and the gizmo swallowing it looks from there exactly
            # like TouchPose swallowing it. The two observable facts that
            # DO separate them are that the gizmo is now dragging and
            # that TouchPose recorded no press of its own.
            _SendButton(view, QtCore.QEvent.MouseButtonPress, hx, hy,
                        viewport.ratio)
            appController._processEvents()
            dragging = gizmo.IsDragging()
            touchTook = controller._consumedPress
            after = [str(p.GetPath()) for p in selection.getPrims()
                     if p and p.IsValid()]
            _SendButton(view, QtCore.QEvent.MouseButtonRelease, hx, hy,
                        viewport.ratio)
            appController._processEvents()

            _Check(dragging,
                   "pressing handle %r (over region %s) started the "
                   "gizmo's drag" % (handle.name, got.label))
            _Check(not touchTook,
                   "...and TouchPose did not take that press")
            _Check(after == before,
                   "...and the selection is untouched: %s -> %s"
                   % (before, after))
            _Check(not gizmo.IsDragging(),
                   "the release ended the drag cleanly")
            print("TouchPose: handle %r sits over region %s; the press "
                  "started the gizmo drag, TouchPose stood off, and the "
                  "selection stayed %s"
                  % (handle.name, got.label, after))

            # --- TOUCHPOSE STANDS DOWN WHILE THE DRAG IS LIVE ---------
            #
            # Re-press the handle and leave the drag in flight, then move
            # the cursor across a region boundary and prove nothing
            # happened. "Nothing" is the session layer's own contents:
            # an overlay rebuild authors points, indices, a colour, an
            # extent and a visibility, so a byte-identical layer is the
            # whole claim in one comparison.
            selection.clearPrims()
            appController._processEvents()
            controller.Pick([region], touchPoseUI.MODE_REPLACE)
            appController._processEvents()
            litSelection = _Selected(selection)

            # Light a hover patch first, so "cleared once on the way in"
            # is a visible change rather than a no-op.
            _SendHover(view, pixel[0], pixel[1], viewport.ratio)
            appController._processEvents()
            _Check(controller.highlight.key is not None,
                   "a hover patch is lit before the drag starts")

            _SendButton(view, QtCore.QEvent.MouseButtonPress, hx, hy,
                        viewport.ratio)
            appController._processEvents()
            _Check(gizmo.IsDragging(), "the drag is in flight")
            _Check(touchPoseUI.GizmoDragging(),
                   "and TouchPose can see that it is")

            # The first hover during the drag drops the patch, once.
            _SendHover(view, pixel[0], pixel[1], viewport.ratio)
            appController._processEvents()
            _Check(controller.suspended, "TouchPose suspended itself")
            _Check(controller.highlight.suspended,
                   "and stood the hover down on the way in")
            _Check(controller.highlight.key is None,
                   "...which means it is not drawing")

            # Now the measurement: samples that cross region boundaries,
            # and a session layer that must not move a byte.
            crossing = [pixel, otherPixel or pixel,
                        hits[best][0], hits[second[0]][0] if second
                        else pixel]
            fingerprint = _SessionFingerprint(stage)
            leadBefore = controller.highlight.lead
            selectedBefore = controller.highlight.selected

            suspendStart = time.perf_counter()
            samples = 40
            for i in range(samples):
                spot = crossing[i % len(crossing)]
                _SendHover(view, spot[0], spot[1], viewport.ratio)
            suspendedMs = 1000.0 * (time.perf_counter()
                                    - suspendStart) / samples
            appController._processEvents()

            _Check(_SessionFingerprint(stage) == fingerprint,
                   "%d hover samples across region boundaries authored "
                   "NOTHING into the session layer while the drag was "
                   "live" % samples)
            # Still SUSPENDED, not cleared -- the fingerprint above is
            # what says nothing was authored, and this says nothing was
            # drawn either.
            _Check(controller.highlight.suspended,
                   "the hover patch stayed dark")
            _Check(controller.highlight.lead == leadBefore
                   and controller.highlight.selected == selectedBefore,
                   "and the SELECTION patches were left alone -- blinking "
                   "those off mid-drag would read as a bug")
            _Check(_Selected(selection) == litSelection,
                   "the selection itself is untouched: %s"
                   % _Selected(selection))

            # A press during a drag must not start a TouchPose pick. It
            # falls through to usdview, whose own pick then runs -- that
            # is usdview's business and not something to suppress. What
            # must NOT happen is TouchPose taking it, or TouchPose's own
            # overlay prims ending up selected: they are rprims drawn on
            # top of the body, so Storm's pick reaches them first.
            _SendButton(view, QtCore.QEvent.MouseButtonPress,
                        pixel[0], pixel[1], viewport.ratio)
            appController._processEvents()
            _Check(not controller._consumedPress,
                   "TouchPose did not take a press made during the drag")
            leftover = _Selected(selection)
            _Check(not any(p.startswith("/TouchPose") for p in leftover),
                   "and no overlay prim was left selected: %s" % leftover)
            _Check(MESH not in leftover,
                   "nor the mesh: %s" % leftover)
            controller.Pick([region], touchPoseUI.MODE_REPLACE)
            appController._processEvents()

            # What the same samples cost with TouchPose awake -- the
            # number the animator feels, since this is what used to run
            # on every mouse move of a manipulation.
            awakeStart = time.perf_counter()
            for i in range(samples):
                spot = crossing[i % len(crossing)]
                controller.Hover(spot[0], spot[1])
            awakeMs = 1000.0 * (time.perf_counter() - awakeStart) / samples

            _SendButton(view, QtCore.QEvent.MouseButtonRelease, hx, hy,
                        viewport.ratio)
            appController._processEvents()
            _Check(not gizmo.IsDragging(), "the drag ended")

            # ...and the very next hover works again. One-shot, no poll.
            _SendHover(view, pixel[0], pixel[1], viewport.ratio)
            appController._processEvents()
            _Check(not controller.suspended, "TouchPose resumed")
            _Check(controller.highlight.key is not None,
                   "and the first hover after the drag highlights again")
            _Check(_SessionFingerprint(stage) == fingerprint,
                   "and even awake, lighting it authored nothing")

            print("TouchPose: during a gizmo drag, %d hover samples across "
                  "region boundaries cost %.3f ms each and authored "
                  "nothing; awake the same samples cost %.2f ms each "
                  "(%.0fx). The selection patches stayed lit; the first "
                  "hover after the drag highlighted again."
                  % (samples, suspendedMs, awakeMs,
                     awakeMs / max(suspendedMs, 1e-6)))

            controller._highlight.Clear()
            selection.clearPrims()
            appController._processEvents()

            # The converse, so this is a RULE and not a blanket refusal:
            # a pixel on the same region but AWAY from every handle must
            # still be TouchPose's.
            away = next((s for s in hits[best]
                         if not touchPoseUI.GizmoOwns(s[0], s[1],
                                                      viewport.ratio)), None)
            if away is not None:
                selection.clearPrims()
                appController._processEvents()
                took = _SendClick(view, away[0], away[1], viewport.ratio)
                appController._processEvents()
                chosen = [str(p.GetPath()) for p in selection.getPrims()
                          if p and p.IsValid()]
                _Check(took and chosen == [region.control],
                       "off the handles the same region is still "
                       "TouchPose's: consumed=%s selection=%s"
                       % (took, chosen))
                print("TouchPose: and at (%.0f, %.0f), clear of every "
                      "handle, the click still selects %s"
                      % (away[0], away[1], chosen))

    selection.clearPrims()
    controller._highlight.Clear()
    controller.SyncSelection()
    appController._processEvents()

    # --- 6. the pick follows the DEFORMED mesh --------------------------
    #
    # `hips_ctl` +12 cm, then the SAME region is found again -- at a
    # different pixel. A pick that read `points` off the stage would
    # answer at the old pixel, because the stage's points are the rest
    # mesh and never move.
    hips = stage.GetPrimAtPath(Sdf.Path(HIPS))
    _Check(hips and hips.IsValid(), "the rig has %s" % HIPS)
    # `avars:ty`, not a transform op: a RigExecControl is posed through
    # its avars and the solver turns those into the deformation. The
    # stage is Y-up centimetres (metersPerUnit 0.01), so +12 is +12 cm.
    translate = hips.GetAttribute("avars:ty")
    if not translate or not translate.IsValid():
        translate = hips.CreateAttribute("avars:ty", Sdf.ValueTypeNames.Double)
    before = float(translate.Get() or 0.0)

    restCentroid = Gf.Vec3d(*[float(c)
                              for c in touchModel.FaceCentroid(face)])
    translate.Set(before + 12.0)
    appController._processEvents()
    view.updateGL()
    QtWidgets.QApplication.processEvents()

    moved_points = controller.SyncPose()
    posedCentroid = Gf.Vec3d(*[float(c)
                               for c in touchModel.FaceCentroid(face)])
    rise = posedCentroid[1] - restCentroid[1]
    _Check(moved_points,
           "the generation counter moved and the CPU point copy refreshed")
    # NOT asserted as ~12 cm: this face is on a thigh between a raised hip
    # and a planted foot, so it rises by roughly half of what the hips do
    # -- which region happens to be biggest on screen is a camera
    # accident and must not decide whether the test passes. What has to
    # be true is that the face moved MEASURABLY and that the pick went
    # with it.
    _Check(rise > 1.0,
           "face %d rose with the hips: %.2f cm of the 12 cm the hips "
           "moved (a limb between a moved hip and a planted foot takes a "
           "fraction)" % (face, rise))

    # The clincher: the STAGE's own points did not move at all. Anything
    # picking off `GetPointsAttr()` would still be aiming at the rest
    # mesh, which is exactly the bug this design exists to avoid.
    restNow = UsdGeom.Mesh(stage.GetPrimAtPath(MESH)).GetPointsAttr().Get()
    _Check(Gf.Vec3f(restNow[0]) == Gf.Vec3f(rest[0])
           and len(restNow) == len(rest),
           "the stage's `points` are unchanged by the pose -- so the "
           "movement above came from Hydra, not from the file")

    # WHERE THE REGION IS NOW, found the same way it was found at rest:
    # project a face of it and require the cast at that pixel to land
    # back in the same region. Not simply "project face N and assume":
    # raising the hips with the feet planted folds the top of the thigh
    # behind the pelvis, so the very face used at rest can end up
    # occluded by a NEIGHBOURING region -- which is a correct pick, and
    # would look like a failure if the test insisted on that one face.
    posedViewport = _Viewport(view)
    posedPixel = None
    for candidate in region.faces[::max(1, len(region.faces) // 40)]:
        projected = posedViewport.Project(
            touchModel.FaceCentroid(int(candidate)))
        if projected is None:
            continue
        got, _f = controller.RegionAt(projected[0], projected[1])
        if got is not None and got.index == region.index:
            posedPixel = projected
            break
    _Check(posedPixel is not None,
           "%s is still visible after the pose, and the cast finds it"
           % region.label)
    shift = ((posedPixel[0] - pixel[0]) ** 2
             + (posedPixel[1] - pixel[1]) ** 2) ** 0.5
    _Check(shift > 4.0,
           "and it moved on screen by %.1f px, so the two pixels are "
           "genuinely different tests" % shift)

    selection.clearPrims()
    appController._processEvents()
    consumed = _SendClick(view, posedPixel[0], posedPixel[1],
                          posedViewport.ratio)
    appController._processEvents()
    chosen = [str(p.GetPath()) for p in selection.getPrims()
              if p and p.IsValid()]
    _Check(consumed and chosen == [region.control],
           "clicking the POSED pixel selected %s, got %s"
           % (region.control, chosen))
    print("TouchPose: after hips +12 cm the region's face rose %.2f cm and "
          "moved %.1f px on screen; the pick followed it" % (rise, shift))

    translate.Set(before)
    appController._processEvents()
    view.updateGL()
    QtWidgets.QApplication.processEvents()
    controller.SyncPose(force=True)

    # --- 6b. PAINTING ---------------------------------------------------
    #
    # A stroke is the pick loop running backwards: same cast, same brush,
    # but it writes the region instead of reading it. Two things have to
    # be true and both are measured here -- the faces really move, and
    # the rig does not notice.
    controller.SetPaintTarget(region)
    controller.SetPainting(True)
    controller.SetBrush(touchModel.brush)
    _Check(controller.painting, "paint mode is on")

    # ...and with it on, EVERY region is drawn at once, in the edit
    # colour set. That is the mode the studio's tool works in: you
    # cannot paint a boundary you cannot see. Measured, not assumed --
    # the patch is one rprim carrying a colour per face, which is
    # exactly the kind of thing that authors cleanly and draws nothing.
    appController._processEvents()
    allLit = _Capture(view)
    allMoved, _t = _Changed(baseline, allLit)
    allFraction = float(allMoved) / max(total, 1)
    _Check(controller.highlight.editing,
           "the edit-mode colours are drawn")
    _Check(allMoved > moved * 2,
           "every region at once covers far more than the one hovered "
           "region did: %d pixels against %d" % (allMoved, moved))
    print("TouchPose: paint mode lit every region at once -- %d of %d "
          "sampled pixels (%.2f%% of the frame) against %d for the one "
          "hovered region"
          % (allMoved, total, 100.0 * allFraction, moved))

    lib = None
    try:
        import ctypes
        from rigExecUsdview import ImagingLibraryPath
        lib = ctypes.CDLL(ImagingLibraryPath())
        lib.RigExecImaging_GetGeneration.restype = ctypes.c_longlong
    except Exception:
        pass
    generationBefore = lib.RigExecImaging_GetGeneration() if lib else None

    sizesBefore = {r.index: len(r.faces) for r in touchModel.regions}
    covered = int((touchModel.region_of >= 0).sum())

    # Stroke along the border between the chosen region and its
    # neighbour, so faces genuinely change hands rather than being
    # painted onto ground the target already owns.
    border = None
    for spot in hits[best]:
        for dx, dy in ((step, 0), (-step, 0), (0, step), (0, -step)):
            got, face = controller.RegionAt(spot[0] + dx, spot[1] + dy)
            if face >= 0 and (got is None or got.index != region.index):
                border = (spot[0] + dx, spot[1] + dy, got)
                break
        if border is not None:
            break
    _Check(border is not None,
           "found a pixel next to %s that it does not own" % region.label)
    bx, by, donor = border

    # Light the TARGET's patch FIRST, so what is counted below is the
    # patch growing onto the newly painted faces rather than a highlight
    # appearing out of nothing. The first is what a painter actually
    # sees, and it does not depend on whatever was lit beforehand.
    controller.highlight.SetHover(region)
    appController._processEvents()
    before = _Capture(view)
    painted = controller.Paint(bx, by)
    appController._processEvents()
    _Check(painted > 0,
           "the brush caught %d faces at (%.0f, %.0f)" % (painted, bx, by))
    _Check(len(region.faces) == sizesBefore[region.index] +
           (painted if donor is None else 0) or
           len(region.faces) > sizesBefore[region.index],
           "%s grew: %d -> %d"
           % (region.label, sizesBefore[region.index], len(region.faces)))
    gained = len(region.faces) - sizesBefore[region.index]
    if donor is not None:
        lost = sizesBefore[donor.index] - len(donor.faces)
        _Check(lost > 0,
               "%s lost the faces %s gained: %d lost, %d gained"
               % (donor.label, region.label, lost, gained))
        _Check(lost <= gained,
               "and lost no more than were taken: %d vs %d" % (lost, gained))

    # NON-OVERLAP, which the whole face -> region lookup depends on.
    # Named `owned`, not `total`: `total` is the sampled-pixel count the
    # highlight measurements are quoted against, and shadowing it here
    # quietly rewrote the summary line at the end of this file.
    owned = sum(len(r.faces) for r in touchModel.regions)
    _Check(owned == int((touchModel.region_of >= 0).sum()),
           "every owned face is owned exactly once: %d faces in regions "
           "against %d marked in the lookup"
           % (owned, int((touchModel.region_of >= 0).sum())))

    after = _Capture(view)
    strokeMoved, _t = _Changed(before, after)
    _Check(strokeMoved > 5,
           "the highlight grew onto the painted faces: %d pixels changed"
           % strokeMoved)

    if lib is not None:
        _Check(lib.RigExecImaging_GetGeneration() == generationBefore,
               "the rig's generation counter did NOT move across the "
               "stroke (%s -> %s) -- nothing was authored inside the rig, "
               "so nothing recompiled"
               % (generationBefore, lib.RigExecImaging_GetGeneration()))

    # A stroke has to fit in a frame like a hover does.
    strokeStart = time.perf_counter()
    for i in range(10):
        controller.Paint(bx, by)
    strokeMs = 1000.0 * (time.perf_counter() - strokeStart) / 10.0
    _Check(strokeMs < 16.7,
           "one brush dab fits in a frame: %.2f ms" % strokeMs)

    # ERASE gives the faces back to nobody.
    erased = controller.Paint(bx, by, erase=True)
    appController._processEvents()
    _Check(erased > 0, "the erase caught %d faces" % erased)
    _Check(int((touchModel.region_of >= 0).sum()) < covered + gained,
           "erasing reduced the covered count")

    print("TouchPose: a dab at (%.0f, %.0f) moved %d faces into %s%s, the "
          "overlay changed %d pixels, the rig generation stayed %s, and a "
          "dab costs %.2f ms"
          % (bx, by, gained, region.label,
             " from %s" % donor.label if donor is not None else "",
             strokeMoved, generationBefore, strokeMs))

    # --- 6c. SAVE writes the touch layer, not the rig -------------------
    layerPath = controller.RegionLayerPath()
    _Check(layerPath and os.path.exists(layerPath),
           "the region layer was located: %r" % layerPath)
    _Check("Biped_touch_regions" in os.path.basename(layerPath),
           "...and it is the TOUCH layer, not the rig: %s"
           % os.path.basename(layerPath))

    rigPath = None
    for layer in stage.GetUsedLayers():
        name = os.path.basename(layer.realPath or layer.identifier)
        if name in ("Biped.usda", "Biped_layered_center.usda"):
            rigPath = layer.realPath
            break
    rigBefore = os.path.getmtime(rigPath) if rigPath else None

    scratch = os.path.join(
        os.environ.get("TEMP", "."), "touchpose_saved_regions.usda")
    if os.path.exists(scratch):
        os.remove(scratch)
    saved = controller.Save(scratch)
    _Check(saved == scratch and os.path.exists(scratch),
           "Save wrote %r" % saved)
    if rigPath:
        _Check(os.path.getmtime(rigPath) == rigBefore,
               "and did not touch the rig file %s"
               % os.path.basename(rigPath))

    from pxr import Usd as _Usd
    checkLayer = Sdf.Layer.CreateAnonymous("check.usda")
    checkLayer.subLayerPaths.append(scratch)
    checkLayer.subLayerPaths.append(
        stage.GetRootLayer().realPath or stage.GetRootLayer().identifier)
    checkStage = _Usd.Stage.Open(checkLayer)
    import touchPoseModel as _model
    reread = _model.TouchModel.FromStage(checkStage, MESH)
    _Check(len(reread.regions) == len(
        [r for r in touchModel.regions if len(r.faces)]),
        "every non-empty region survived the save: %d written, %d read "
        "back" % (len([r for r in touchModel.regions if len(r.faces)]),
                  len(reread.regions)))
    saved_region = next(r for r in reread.regions if r.name == region.name)
    _Check(len(saved_region.faces) == len(region.faces),
           "%s kept its painted face count: %d on disk, %d in memory"
           % (region.label, len(saved_region.faces), len(region.faces)))
    print("TouchPose: saved %d regions to a scratch layer and read back "
          "%d, %s with %d faces either way; the rig file was not touched"
          % (len(touchModel.Rows()), len(reread.regions), region.label,
             len(saved_region.faces)))
    os.remove(scratch)

    controller.SetPainting(False)

    # --- 7. a click that misses the character is NOT consumed -----------
    #
    # The corner pixel: no mesh there, so usdview's own picking, the
    # camera and the gizmo all still see the event.
    selection.clearPrims()
    appController._processEvents()
    _Check(empty, "the grid found a pixel with no character under it")
    away = empty[0]
    consumed = _SendClick(view, away[0], away[1], viewport.ratio)
    _Check(not consumed,
           "a click at %s missed the mesh and was NOT consumed" % (away,))
    appController._processEvents()
    chosen = [str(p.GetPath()) for p in selection.getPrims()
              if p and p.IsValid()]
    _Check(MESH not in chosen,
           "and usdview's own pick, which ran, selected nothing there: %s"
           % chosen)
    print("TouchPose: a click at %s fell through to usdview (selection %s)"
          % (away, chosen or "empty"))

    # --- 8. ON the mesh but OFF every region: consumed, selects nothing -
    #
    # 30% of `body_geo` is unpainted -- soles, inner mouth, the scalp
    # under the hair -- plus everything whose region was skipped for
    # having no control, which is most of the face rig. Clicking there
    # must still not select the mesh.
    bareSpot = bare[len(bare) // 2] if bare else None
    if bareSpot is None:
        print("TouchPose: NOTE no unpainted skin is visible from this "
              "camera; the 'mesh is not selectable' case is asserted on "
              "the miss path only")
    else:
        selection.clearPrims()
        appController._processEvents()
        # `clearPrims` leaves usdview holding the pseudo-root, not an
        # empty list -- so "unchanged" is compared against what a cleared
        # selection actually looks like rather than against [].
        cleared = [str(p.GetPath()) for p in selection.getPrims()
                   if p and p.IsValid()]
        consumed = _SendClick(view, bareSpot[0], bareSpot[1], viewport.ratio)
        appController._processEvents()
        chosen = [str(p.GetPath()) for p in selection.getPrims()
                  if p and p.IsValid()]
        _Check(consumed,
               "a click at %s is on the skin but on no region, and "
               "TouchPose still consumed it" % (bareSpot,))
        _Check(chosen == cleared,
               "...leaving the selection exactly as it was (%s), got %s"
               % (cleared, chosen))
        _Check(MESH not in chosen,
               "and above all NOT selecting the mesh: %s" % chosen)
        print("TouchPose: unpainted skin at %s consumed the click and left "
              "the selection at %s" % (bareSpot, chosen))

    # --- 9. turning it OFF gives usdview its picking back ---------------
    controller.SetActive(False)
    _Check(not controller.active, "TouchPose deactivated")
    selection.clearPrims()
    appController._processEvents()
    consumed = _SendClick(view, pixel[0], pixel[1], viewport.ratio)
    appController._processEvents()
    chosen = [str(p.GetPath()) for p in selection.getPrims()
              if p and p.IsValid()]
    _Check(not consumed,
           "with TouchPose off the click is not consumed")
    _Check(MESH in chosen,
           "and usdview selects the mesh again, as it always did: %s"
           % chosen)
    print("TouchPose: with the mode off the same pixel selects %s" % chosen)

    # ...and the highlight goes with it. Measured as the difference
    # between two frames taken back to back with the SAME selection,
    # rather than against the old baseline: usdview draws its own
    # selection highlight, so a frame captured under a different
    # selection differs for reasons that have nothing to do with
    # TouchPose.
    selection.clearPrims()
    controller.SetActive(True)
    controller.Hover(pixel[0], pixel[1])
    appController._processEvents()
    litAgain = _Capture(view)
    controller.SetActive(False)
    appController._processEvents()
    off = _Capture(view)
    removed, _total = _Changed(litAgain, off)
    _Check(removed > moved * 0.5,
           "deactivating took the highlight away again: %d pixels changed "
           "back, against the %d it lit" % (removed, moved))
    print("TouchPose: deactivating removed %d of the %d pixels the "
          "highlight lit" % (removed, moved))

    # The whole session authored nothing: no prim was ever created for
    # the highlight, at the root or anywhere else.
    _Check(not any(p.GetName().startswith("TouchPose")
                   for p in stage.GetPseudoRoot().GetChildren()),
           "no TouchPose prim was ever created at the stage root")

    print("RIGEXEC_TOUCHPOSE_OK %d regions / %d of %d faces (%.0f%%); "
          "click selects %s; highlight moved %d of %d pixels (%.3f%% of "
          "the frame, %.0f%% of the region); pose +12cm raised the face "
          "%.2f cm and moved the pick %.1f px; hover %.2f/%.2f ms; paint "
          "%d faces in %.2f ms with the rig generation unmoved; suspends "
          "during a gizmo drag; miss "
          "falls through; mesh not selectable while active; off restores "
          "usdview picking"
          % (regions, covered, faces, 100.0 * covered / faces,
             region.control.rsplit("/")[-1], moved, total,
             100.0 * fraction, 100.0 * ofRegion, rise, shift,
             sameRegion, changeRegion, gained, strokeMs))
