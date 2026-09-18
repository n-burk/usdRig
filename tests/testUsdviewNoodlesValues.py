"""testusdview script: read and edit simple attribute values on the node
rows of the Noodles Editor, inside a real usdview.

Nothing here is visible to the headless unit tests in
plugin/usdNoodles/testenv, because all of it needs a laid-out node with real
row geometry, a real Qt event loop and a real undo stack:

  * a CLICK on a value cell opens the inline editor; typing and Return
    author the value and push exactly one undo entry.
  * a DRAG on the same cell changes the value along a Houdini-style step
    ladder, live, and commits as exactly ONE undo entry on release.
  * a token with ``allowedTokens`` opens a popup listing EXACTLY those
    tokens, and picking one authors at Default because the attribute is
    uniform.
  * a CONNECTED attribute is read-only: no editor, no drag, no undo entry,
    and -- the part that matters -- the press must not fall through into a
    connection drag.
  * the PIN GUTTER still starts a connection drag. This is the regression
    the whole design is arranged around: value cells sit a padding short of
    the outer tenth of the node on each side, and that tenth must keep
    meaning "start a link".
  * Esc cancels, both in the editor and mid-drag, leaving the value and the
    undo stack exactly as they were.
  * matrix4d / float[] / asset / quatf rows get NO cell at all, and a press
    where one would be falls through to a node drag.

Prints RIGEXEC_NOODLES_VALUES_OK.

Set RIGEXEC_NOODLES_SHOT=/path.png to keep a grab of the editor with the
cells drawn.

The editor is closed before returning: a Noodles GL widget left open while
testusdview tears the app down reports GL errors from usdview's own
viewport, which testusdview treats as a failure.
"""
import os


VALUES = "/Values"
CHAIN = "/Chain"


def _pump(app, n=20):
    for _ in range(n):
        app.processEvents()


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def testUsdviewInputFunction(appController):
    import sys

    from pxr import Sdf, Usd
    from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

    app = QtWidgets.QApplication.instance()

    registry = appController._plugRegistry
    _Check(registry is not None,
           "usdview loaded no plugins -- a duplicate command name?")
    command = registry.getCommandPlugin("NoodlesPluginContainer.ShowNoodlesEditor")
    _Check(command is not None, "no Noodles Editor command")
    _Check(command._callback.__module__.startswith("UsdNoodles."),
           "the Noodles Editor is %s, not the in-repo UsdNoodles"
           % command._callback.__module__)
    installed = sys.modules.get("pxr.UsdNoodles")
    _Check(installed is None or getattr(installed, "SUPERSEDED_BY", None),
           "the installed pxr.UsdNoodles was imported: %s"
           % getattr(installed, "__file__", installed))

    command.run()
    _pump(app, 50)

    from UsdNoodles._usdNoodles import NoodlesUndoManager
    from UsdNoodles.graphView import GraphView

    views = [w for w in app.allWidgets() if isinstance(w, GraphView)]
    _Check(bool(views), "no GraphView after opening the Noodles Editor")
    view = views[0]
    _pump(app, 30)

    stage = appController._dataModel.stage
    valuesPrim = stage.GetPrimAtPath(Sdf.Path(VALUES))
    _Check(bool(valuesPrim) and valuesPrim.IsValid(), "the fixture has no /Values")

    # --- the prims become nodes -----------------------------------------
    selection = appController._dataModel.selection
    selection.clearPrims()
    for path in (VALUES, CHAIN):
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if prim and prim.IsValid():
            selection.addPrim(prim)
    _pump(app, 10)
    view.addNodesFromPrimTreeSelection()
    _pump(app, 40)
    view.repaint()
    _pump(app, 20)

    _Check(VALUES in view.nodes,
           "%s did not become a node: %s" % (VALUES, sorted(view.nodes)))
    valuesNode = view.nodes[VALUES]

    undo = NoodlesUndoManager.instance()

    # --- helpers ---------------------------------------------------------
    def screenPoint(worldX, worldY):
        return QtCore.QPointF((worldX - view.panX) * view.zoom,
                              (worldY - view.panY) * view.zoom)

    def cellPoint(node, prop, comp=0):
        """The middle of one value cell, in widget coordinates.

        Derived from the view's own geometry, never from hard-coded pixels:
        the node width depends on the font metrics of whatever machine this
        runs on.
        """
        rect = view._valueCellRectFor(node, prop, comp)
        _Check(rect is not None, "%s has no value cell" % prop)
        return screenPoint((rect.left + rect.right) * 0.5,
                           (rect.top + rect.bottom) * 0.5)

    def mouse(kind, point, buttons=None, modifiers=QtCore.Qt.NoModifier):
        if buttons is None:
            buttons = (QtCore.Qt.NoButton
                       if kind == QtCore.QEvent.MouseButtonRelease
                       else QtCore.Qt.LeftButton)
        app.sendEvent(view, QtGui.QMouseEvent(
            kind, point, view.mapToGlobal(point.toPoint()),
            QtCore.Qt.LeftButton, buttons, modifiers))
        _pump(app, 2)

    def press(point, modifiers=QtCore.Qt.NoModifier):
        mouse(QtCore.QEvent.MouseButtonPress, point, modifiers=modifiers)

    def move(point, modifiers=QtCore.Qt.NoModifier):
        mouse(QtCore.QEvent.MouseMove, point, buttons=QtCore.Qt.LeftButton,
              modifiers=modifiers)

    def release(point, modifiers=QtCore.Qt.NoModifier):
        mouse(QtCore.QEvent.MouseButtonRelease, point, modifiers=modifiers)

    def key(code, text="", modifiers=QtCore.Qt.NoModifier):
        """Deliver a real QKeyEvent to the view's own event() override.

        Not ``app.sendEvent``: a key event that did not come from the window
        system is run through QApplication's shortcut-override plumbing
        first, and for Key_Escape specifically that plumbing consumes it --
        the KeyPress never reaches the widget at all, however the override is
        answered. In a real session the window system sends the override and
        the key press as two separate events, which is the path
        ``_shouldInterceptShortcut`` exists for; that contract is asserted
        directly below instead.
        """
        view.event(QtGui.QKeyEvent(
            QtCore.QEvent.KeyPress, code, modifiers, text))
        _pump(app, 2)

    def settle():
        view.repaint()
        _pump(app, 10)

    def typeText(chars):
        for ch in chars:
            if ch.isdigit():
                code = getattr(QtCore.Qt, "Key_" + ch)
            elif ch == ".":
                code = QtCore.Qt.Key_Period
            elif ch == "-":
                code = QtCore.Qt.Key_Minus
            else:
                code = getattr(QtCore.Qt, "Key_" + ch.upper())
            key(code, ch)

    def clearEditor():
        for _ in range(len(view._valueEditInput.text) + 4):
            key(QtCore.Qt.Key_Backspace)

    # --- the cells exist, and only for the simple types ------------------
    for prop in ("plain", "dbl", "count", "flag", "vec", "tint", "label",
                 "driven", "animated"):
        _Check(view._valueCellRectFor(valuesNode, prop, 0) is not None,
               "%s should have a value cell" % prop)
    for prop in ("xf", "arr", "tex", "q"):
        _Check(view._valueCellRectFor(valuesNode, prop, 0) is None,
               "%s must have NO value cell -- it is not a simple type" % prop)
    for comp in range(3):
        _Check(view._valueCellRectFor(valuesNode, "vec", comp) is not None,
               "float3 component %d has no cell" % comp)

    # --- the cell is disjoint from the connection gutter -----------------
    nx = float(valuesNode.position[0])
    nw = float(valuesNode.size[0])
    rect = view._valueCellRectFor(valuesNode, "plain", 0)
    _Check(rect.right < nx + nw * 0.9,
           "the value cell (right=%.1f) reaches into the right connection "
           "gutter (starts at %.1f)" % (rect.right, nx + nw * 0.9))
    _Check(rect.left > nx + nw * 0.1,
           "the value cell (left=%.1f) reaches into the left connection "
           "gutter (ends at %.1f)" % (rect.left, nx + nw * 0.1))

    shot = os.environ.get("RIGEXEC_NOODLES_SHOT")
    if shot:
        view.repaint()
        _pump(app, 5)
        view.grabFramebuffer().save(shot)

    # --- 1. click-edit a float -------------------------------------------
    undo.clear()
    point = cellPoint(valuesNode, "plain")
    press(point)
    release(point)
    _Check(view._valueEditTarget == (VALUES, "plain", 0),
           "a click did not open the editor (target=%r)" % (view._valueEditTarget,))
    _Check(view._mungState is None, "a click left a drag in progress")
    clearEditor()
    typeText("2.5")
    key(QtCore.Qt.Key_Return)
    _pump(app, 20)

    _Check(view._valueEditTarget is None, "the editor is still open")
    _Check(abs(valuesPrim.GetAttribute("plain").Get() - 2.5) < 1e-5,
           "plain is %r, expected 2.5" % valuesPrim.GetAttribute("plain").Get())
    _Check(valuesPrim.GetAttribute("plain").GetNumTimeSamples() == 0,
           "an attribute with no samples must author at Default")
    _Check(undo.canUndo(), "the typed edit pushed no undo entry")
    _Check(undo.undoDescription().startswith("Set "),
           "undo says %r" % undo.undoDescription())

    # An interaction in progress must claim the non-printable keys it uses,
    # or Qt hands Escape to the enclosing dock and "Esc cancels" does nothing.
    view._beginValueEdit(VALUES, "plain", 0)
    override = QtGui.QKeyEvent(QtCore.QEvent.ShortcutOverride,
                               QtCore.Qt.Key_Escape, QtCore.Qt.NoModifier, "")
    _Check(view._shouldInterceptShortcut(override),
           "an open value editor does not claim Escape")
    view._cancelValueEdit()
    _pump(app, 5)

    # --- 2. drag the same cell -------------------------------------------
    undo.clear()
    settle()
    _Check(not undo.canUndo(), "the undo stack did not clear")
    point = cellPoint(valuesNode, "plain")
    press(point)
    _Check(view._mungState is not None,
           "pressing a float cell did not arm a value drag")
    sawActive = False
    for step in (30.0, 70.0, 100.0):
        move(QtCore.QPointF(point.x() + step, point.y()))
        sawActive = sawActive or view._mungActive
    _Check(sawActive, "the drag never became active")
    release(QtCore.QPointF(point.x() + 100.0, point.y()))
    _pump(app, 20)

    _Check(not view._mungActive, "the drag is still active after release")
    _Check(view._mungState is None, "the drag state survived the release")
    _Check(view._valueEditTarget is None, "a drag opened the keyboard editor")
    # 2.5 -> ladder_step(2.5) == 0.01 -> 100 px adds exactly 1.0
    got = valuesPrim.GetAttribute("plain").Get()
    _Check(abs(got - 3.5) < 1e-4,
           "a 100 px drag from 2.5 gave %r, expected 3.5" % got)
    _Check(undo.canUndo(), "the drag pushed no undo entry")
    _Check(undo.undoDescription().startswith("Set "),
           "undo says %r" % undo.undoDescription())
    undo.undo()
    _pump(app, 10)
    got = valuesPrim.GetAttribute("plain").Get()
    _Check(abs(got - 2.5) < 1e-4,
           "ONE undo left plain at %r, expected the pre-drag 2.5 -- the drag "
           "pushed more than one entry" % got)
    _Check(not undo.canUndo(),
           "the drag pushed more than one undo entry (%r still there)"
           % undo.undoDescription())

    # --- 3. Shift multiplies the step by ten -----------------------------
    undo.clear()
    settle()
    point = cellPoint(valuesNode, "count")
    before = valuesPrim.GetAttribute("count").Get()
    press(point)
    _Check(view._mungState is not None,
           "pressing an int cell did not arm a value drag")
    move(QtCore.QPointF(point.x() + 10.0, point.y()),
         modifiers=QtCore.Qt.ShiftModifier)
    _Check(view._mungActive, "10 px past a 3 px threshold did not start a drag")
    release(QtCore.QPointF(point.x() + 10.0, point.y()),
            modifiers=QtCore.Qt.ShiftModifier)
    _pump(app, 10)
    got = valuesPrim.GetAttribute("count").Get()
    _Check(got == before + 100,
           "Shift + 10 px on an int gave %r, expected %r (step 1 x10)"
           % (got, before + 100))

    # --- 4. the token popup lists EXACTLY the allowed tokens -------------
    chainNode = view.nodes.get(CHAIN)
    chainPrim = stage.GetPrimAtPath(Sdf.Path(CHAIN))
    hasChain = (chainNode is not None and chainPrim and chainPrim.IsValid()
                and bool(chainPrim.GetAttribute("rigExec:controlSpace")))
    if hasChain:
        undo.clear()
        spaceAttr = chainPrim.GetAttribute("rigExec:controlSpace")
        point = cellPoint(chainNode, "rigExec:controlSpace")
        press(point)
        release(point)
        _Check(view._tokenPopup is not None,
               "clicking an allowedTokens cell opened no popup")
        _Check(list(view._tokenPopup["tokens"]) == ["world", "parentRelative"],
               "the popup lists %r, expected exactly "
               "['world', 'parentRelative']" % (view._tokenPopup["tokens"],))
        _Check(view._valueEditTarget is None,
               "an enum token opened the free-text editor as well")
        if shot:
            view.repaint()
            _pump(app, 5)
            view.grabFramebuffer().save(
                shot.replace(".png", "_tokens.png"))
        key(QtCore.Qt.Key_Down)
        key(QtCore.Qt.Key_Return)
        _pump(app, 20)
        _Check(view._tokenPopup is None, "the popup is still open")
        got = spaceAttr.Get()
        _Check(got in ("world", "parentRelative"),
               "controlSpace is %r, not one of its allowed tokens" % got)
        _Check(spaceAttr.GetNumTimeSamples() == 0,
               "a uniform attribute must always author at Default")
        _Check(undo.canUndo(), "the token pick pushed no undo entry")
    else:
        print("RIGEXEC_NOODLES_VALUES: RigExecFkChain unavailable, "
              "skipping the token-popup case")

    # --- 5. a connected attribute is read-only ---------------------------
    undo.clear()
    drivenAttr = valuesPrim.GetAttribute("driven")
    _Check(drivenAttr.HasAuthoredConnections(),
           "the fixture's driven attribute has no connection")
    point = cellPoint(valuesNode, "driven")
    press(point)
    _Check(view._mungState is None,
           "a connected attribute started a value drag")
    _Check(not view._draggingLink,
           "a press on a read-only value cell started a CONNECTION drag")
    move(QtCore.QPointF(point.x() + 100.0, point.y()))
    release(QtCore.QPointF(point.x() + 100.0, point.y()))
    _pump(app, 10)
    _Check(view._valueEditTarget is None,
           "a connected attribute opened the editor")
    _Check(not view._draggingLink,
           "dragging a read-only value cell started a connection drag")
    _Check(not undo.canUndo(),
           "a read-only cell pushed an undo entry: %r" % undo.undoDescription())
    _Check(drivenAttr.HasAuthoredConnections(),
           "the connection was clobbered")

    # --- 6. the pin gutter still starts a connection ---------------------
    view.clearSelection()
    rect = view._valueCellRectFor(valuesNode, "plain", 0)
    rowY = (rect.top + rect.bottom) * 0.5
    for fraction, expectOutput in ((0.95, True), (0.05, False)):
        undo.clear()
        point = screenPoint(nx + nw * fraction, rowY)
        press(point)
        _Check(view._draggingLink,
               "a press at %.0f%% of the node width did not start a "
               "connection drag" % (fraction * 100.0))
        _Check(bool(view._dragLinkSourceIsOutput) == expectOutput,
               "the gutter drag went the wrong way (isOutput=%r, expected %r)"
               % (view._dragLinkSourceIsOutput, expectOutput))
        _Check(view._mungState is None,
               "a gutter press also started a value drag")
        _Check(view._valueEditTarget is None,
               "a gutter press also opened the value editor")
        # Drop on empty canvas to clean up.
        empty = screenPoint(nx - 4000.0, rowY)
        move(empty)
        release(empty)
        _pump(app, 10)
        _Check(not view._draggingLink, "the connection drag did not finish")

    # --- 7. Esc cancels, in the editor and mid-drag ----------------------
    undo.clear()
    dblAttr = valuesPrim.GetAttribute("dbl")
    _Check(abs(dblAttr.Get() - 2.5) < 1e-9, "dbl started at %r" % dblAttr.Get())

    point = cellPoint(valuesNode, "dbl")
    press(point)
    release(point)
    _Check(view._valueEditTarget is not None, "the editor did not open on dbl")
    clearEditor()
    typeText("9")
    key(QtCore.Qt.Key_Escape)
    _pump(app, 10)
    _Check(view._valueEditTarget is None, "Esc left the editor open")
    _Check(abs(dblAttr.Get() - 2.5) < 1e-9,
           "Esc in the editor still authored: dbl is %r" % dblAttr.Get())
    _Check(not undo.canUndo(),
           "Esc in the editor pushed an undo entry: %r" % undo.undoDescription())

    point = cellPoint(valuesNode, "dbl")
    press(point)
    move(QtCore.QPointF(point.x() + 150.0, point.y()))
    _Check(abs(dblAttr.Get() - 2.5) > 1e-6,
           "the drag never previewed a new value")
    key(QtCore.Qt.Key_Escape)
    _pump(app, 10)
    _Check(view._mungState is None, "Esc left the drag in progress")
    _Check(abs(dblAttr.Get() - 2.5) < 1e-6,
           "Esc mid-drag left dbl at %r, expected the pre-drag 2.5"
           % dblAttr.Get())
    _Check(not undo.canUndo(),
           "a cancelled drag pushed an undo entry: %r" % undo.undoDescription())
    release(QtCore.QPointF(point.x() + 150.0, point.y()))
    _pump(app, 10)

    # --- 8. a row with no cell falls through to a node drag --------------
    undo.clear()
    view.clearSelection()
    # Aim at the middle of the node, on the row that xf (matrix4d) occupies:
    # the same X a cell would use, on a row that has none.
    rect = view._valueCellRectFor(valuesNode, "plain", 0)
    cellX = (rect.left + rect.right) * 0.5
    port_start_y = float(valuesNode.position[1]) + valuesNode.layoutPortStartY
    lineHeight = valuesNode.layoutPortLineHeight
    xfSlot = None
    pins = list(valuesNode.inputPins)
    slots = list(valuesNode.inputRowSlots)
    if "xf" in pins:
        i = pins.index("xf")
        xfSlot = slots[i] if i < len(slots) else i
    if xfSlot is not None:
        point = screenPoint(cellX, port_start_y + (xfSlot + 0.5) * lineHeight)
        press(point)
        _Check(view.draggingNodes,
               "a press where a matrix4d's cell would be did not fall "
               "through to a node drag")
        _Check(view._mungState is None, "a matrix4d row started a value drag")
        release(point)
        _pump(app, 10)
        view.clearSelection()

    # --- 9. a time-sampled attribute keys at the current frame -----------
    undo.clear()
    animatedAttr = valuesPrim.GetAttribute("animated")
    _Check(animatedAttr.GetNumTimeSamples() == 2,
           "the fixture's animated attribute has %d samples"
           % animatedAttr.GetNumTimeSamples())
    view._currentTimeCode = Usd.TimeCode(12.0)
    for node in view.nodes.values():
        node.invalidate_value_row()
    view.repaint()
    _pump(app, 10)
    beforeDefault = animatedAttr.Get(Usd.TimeCode.Default())
    beforeAt12 = animatedAttr.Get(Usd.TimeCode(12.0))
    point = cellPoint(valuesNode, "animated")
    press(point)
    _Check(view._mungState is not None,
           "pressing a time-sampled cell did not arm a value drag")
    move(QtCore.QPointF(point.x() + 120.0, point.y()))
    release(QtCore.QPointF(point.x() + 120.0, point.y()))
    _pump(app, 20)
    # The sample lands in the current EDIT TARGET, which is its own layer;
    # what has to be true is that it is a sample AT the current frame and
    # that the Default opinion was left alone.
    _Check(12.0 in list(animatedAttr.GetTimeSamples()),
           "the drag authored no sample at frame 12 (samples %r)"
           % (list(animatedAttr.GetTimeSamples()),))
    _Check(animatedAttr.Get(Usd.TimeCode(12.0)) != beforeAt12,
           "the value at frame 12 did not change")
    _Check(animatedAttr.Get(Usd.TimeCode.Default()) == beforeDefault,
           "keying at a frame also rewrote the Default opinion")

    # --- 10. a bool cell is a checkbox -----------------------------------
    undo.clear()
    flagAttr = valuesPrim.GetAttribute("flag")
    before = bool(flagAttr.Get())
    point = cellPoint(valuesNode, "flag")
    press(point)
    release(point)
    _pump(app, 10)
    _Check(bool(flagAttr.Get()) is (not before),
           "clicking the checkbox did not toggle flag")
    _Check(view._valueEditTarget is None, "a bool opened the text editor")
    _Check(view._mungState is None, "a bool started a drag")
    _Check(undo.canUndo(), "the checkbox pushed no undo entry")

    # --- 11. a vector edits one component and leaves the rest alone ------
    undo.clear()
    vecAttr = valuesPrim.GetAttribute("vec")
    point = cellPoint(valuesNode, "vec", 1)
    press(point)
    release(point)
    _Check(view._valueEditTarget == (VALUES, "vec", 1),
           "clicking the middle component opened %r" % (view._valueEditTarget,))
    clearEditor()
    typeText("9")
    key(QtCore.Qt.Key_Return)
    _pump(app, 20)
    got = vecAttr.Get()
    _Check(abs(got[0] - 1.0) < 1e-5 and abs(got[1] - 9.0) < 1e-5
           and abs(got[2] - 3.0) < 1e-5,
           "editing vec[1] gave %r, expected (1, 9, 3)" % (got,))

    if shot:
        view.repaint()
        _pump(app, 5)
        view.grabFramebuffer().save(shot.replace(".png", "_edited.png"))

    for widget in list(app.topLevelWidgets()):
        if type(widget).__module__.startswith("UsdNoodles"):
            widget.close()
    view.close()
    _pump(app, 20)
    print("RIGEXEC_NOODLES_VALUES_OK")
