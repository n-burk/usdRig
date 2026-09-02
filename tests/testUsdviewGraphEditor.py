#
# THE GRAPH EDITOR, end to end.
#
# Written for the same reason as testUsdviewGizmo.py: the headless tests
# in tests/python call graphModel and graphScreen directly and never go
# near a widget, so they cannot see the one thing a graph editor is -- a
# PLOT that a mouse has to be able to hit. This script opens
# examples/ArmShotAnim.usda, selects the HandIK control, opens the
# editor on the viewport toolbar's undo stack and drives synthetic mouse
# and key events at the pixels the canvas itself reports for its keys
# and tangent handles. Every assertion is about the spline that landed
# in the session layer and the value the attribute resolves to
# afterwards, not about what the panel thinks it did.
#
# Set RIGEXEC_GRAPH_SHOT=/path.png to save a window grab for inspection.
#
import os

from pxr import Sdf, Ts, Usd

CONTROL = "/Shot/HeroArm/Rig/Controls/HandIK"
TX = "HandIK.avars:tx"
TY = "HandIK.avars:ty"
TZ = "HandIK.avars:tz"
RX = "HandIK.avars:rx"

# The knots examples/ArmShotAnim.usda authors on the three translate
# channels, and the frames usdview opens on.
FILE_FRAMES = (1001.0, 1024.0, 1048.0)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Mouse(QtCore, QtGui, widget, kind, x, y, button=None, buttons=None,
           modifiers=None):
    """
    One synthetic mouse event in the canvas's LOGICAL pixels.

    Logical, not physical: graphEditorUI._Position:139-153 deliberately
    does NOT scale by devicePixelRatioF the way gizmoUI._Position must,
    because graphScreen is handed the widget's logical size. KeyPixels()
    and TangentPixels() report the same units, so their output can be
    fed straight in here.
    """
    pos = QtCore.QPointF(float(x), float(y))
    glob = widget.mapToGlobal(QtCore.QPoint(int(x), int(y)))
    globF = QtCore.QPointF(float(glob.x()), float(glob.y()))
    if button is None:
        button = QtCore.Qt.LeftButton
    if buttons is None:
        buttons = button
    if modifiers is None:
        modifiers = QtCore.Qt.NoModifier
    return QtGui.QMouseEvent(kind, pos, globF, button, buttons, modifiers)


class _Driver(object):
    """Mouse, keyboard, selection and the stage, in the panel's units."""

    def __init__(self, appController, panel):
        from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets
        self.QtCore, self.QtGui, self.QtWidgets = QtCore, QtGui, QtWidgets
        self.app = appController
        self.api = appController._usdviewApi
        self.panel = panel
        self.canvas = panel.canvas

    def Pump(self):
        self.app._processEvents()

    def Send(self, kind, x, y, **kw):
        self.QtWidgets.QApplication.sendEvent(
            self.canvas, _Mouse(self.QtCore, self.QtGui, self.canvas,
                                kind, x, y, **kw))
        self.Pump()

    def Press(self, point, **kw):
        self.Send(self.QtCore.QEvent.Type.MouseButtonPress, *point, **kw)

    def Move(self, point, **kw):
        self.Send(self.QtCore.QEvent.Type.MouseMove, *point, **kw)

    def Release(self, point, button=None, modifiers=None):
        self.Send(self.QtCore.QEvent.Type.MouseButtonRelease, *point,
                  button=button or self.QtCore.Qt.LeftButton,
                  buttons=self.QtCore.Qt.NoButton, modifiers=modifiers)

    def Drag(self, start, end, steps=4, button=None, modifiers=None):
        """A press, `steps` moves and a release, as Qt would deliver them."""
        self.Press(start, button=button, modifiers=modifiers)
        for i in range(1, steps + 1):
            t = i / float(steps)
            self.Move((start[0] + (end[0] - start[0]) * t,
                       start[1] + (end[1] - start[1]) * t),
                      button=self.QtCore.Qt.NoButton,
                      buttons=button or self.QtCore.Qt.LeftButton,
                      modifiers=modifiers)
        self.Release(end, button=button, modifiers=modifiers)

    def Key(self, key, modifiers=None):
        # QtTest is the one Qt module pxr.Usdviewq.qt does not re-export,
        # so it comes from the binding directly. That is fine in a test;
        # the plugin modules themselves still go through Usdviewq.qt.
        try:
            from PySide6 import QtTest
        except ImportError:
            from PySide2 import QtTest
        if modifiers is None:
            modifiers = self.QtCore.Qt.NoModifier
        QtTest.QTest.keyClick(self.canvas, key, modifiers)
        self.Pump()

    # -- the stage ------------------------------------------------------

    def Attr(self, name):
        return self.api.stage.GetAttributeAtPath(
            Sdf.Path(CONTROL + "." + name))

    def Spline(self, name):
        return self.Attr(name).GetSpline()

    def Knot(self, name, time):
        knot = self.Spline(name).GetKnot(float(time))
        _Check(knot is not None,
               "%s has a knot at %s; it has %s"
               % (name, time, sorted(self.Spline(name).GetKnots())))
        return knot

    def Value(self, name, frame):
        return self.Attr(name).Get(Usd.TimeCode(float(frame)))

    def SessionSpec(self, name):
        return self.api.stage.GetSessionLayer().GetAttributeAtPath(
            Sdf.Path(CONTROL + "." + name))

    # -- the canvas -----------------------------------------------------

    def KeyPixel(self, label, time):
        """The (x, y) the canvas draws label's key at `time`."""
        keys = self.canvas.KeyPixels().get(label) or []
        hit = [k for k in keys if abs(k[2] - float(time)) < 1e-9]
        _Check(hit, "%s draws a key at %s; it draws %s"
               % (label, time, [k[2] for k in keys]))
        return (hit[0][0], hit[0][1])

    def TangentPixel(self, label, time, side):
        """The (x, y) of one tangent handle end of a SELECTED key."""
        handles = self.canvas.TangentPixels().get(label) or []
        hit = [h for h in handles
               if abs(h[2] - float(time)) < 1e-9 and h[3] == side]
        _Check(hit, "%s draws a %s handle at %s; it draws %s"
               % (label, side, time, handles))
        return (hit[0][0], hit[0][1])

    def RulerPoint(self, frame):
        """A point on the bottom frame ruler under `frame`."""
        plot = self.canvas.PlotRectF()
        return (self.canvas.transform.TimeToX(float(frame)),
                plot.bottom() + 6.0)

    def Rewind(self, stack):
        """Undo back to a clean stack, for sections that follow."""
        while stack.CanUndo():
            stack.Undo()
        self.Pump()


def _KnotCount(panel, labels):
    """How many knots the given curves hold, from the panel's splines."""
    total = 0
    for index, ref in enumerate(panel.Curves()):
        if ref.Label() in labels:
            spline = panel.DrawSplines()[index]
            if spline is not None:
                total += len(spline.GetKnots())
    return total


def testUsdviewInputFunction(appController):
    import gizmoUI
    import graphEditorUI
    import graphModel

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    session = stage.GetSessionLayer()
    _Check(stage.GetEditTarget().GetLayer() == session,
           "usdview's edit target is the session layer, which is where "
           "every assertion below looks for the authored spline")

    controller = gizmoUI.GetController()
    _Check(controller is not None,
           "the container installed the viewport tools on stage load")

    prim = stage.GetPrimAtPath(CONTROL)
    _Check(prim, "the arm shot has %s" % CONTROL)
    api.ClearPrimSelection()
    api.AddPrimToSelection(prim)
    appController._processEvents()

    # The editor is opened on the VIEWPORT TOOLBAR's stack on purpose.
    # graphEditorUI._GizmoUndoActions:1252 then lets the panel adopt the
    # toolbar's Ctrl+Z / Ctrl+Shift+Z rather than registering rivals for
    # the same sequence -- two actions carrying one application shortcut
    # make every Ctrl+Z ambiguous and Qt fires neither. Opening with a
    # private stack here would test a key path no artist ever gets.
    stack = controller.undoStack
    _Check(graphEditorUI.GetGraphEditor() is None,
           "no graph editor exists before this test opens one")
    panel = graphEditorUI.OpenGraphEditor(api, stack)
    panel.resize(1200, 720)
    appController._processEvents()
    _Check(graphEditorUI.GetGraphEditor() is panel,
           "GetGraphEditor hands back the panel this test opened")
    _Check(panel.undoStack is stack,
           "the panel pushes onto the viewport toolbar's stack, so one "
           "Ctrl+Z history covers gizmo drags and graph edits alike")

    # testusdview's windows are never activated by the window manager, so
    # QApplication.activeWindow() is None and Qt's shortcut map refuses
    # EVERY shortcut before it looks at the key -- Ctrl+Z would silently
    # do nothing for reasons that have nothing to do with the editor.
    # Supplying the activation is what lets the undo assertion below go
    # through the real key path instead of calling the action by hand.
    d = _Driver(appController, panel)
    d.QtWidgets.QApplication.setActiveWindow(panel)
    d.Pump()
    _Check(d.QtWidgets.QApplication.activeWindow() is not None,
           "a window is active, so application shortcuts dispatch")
    panel.FrameAll()
    d.Pump()

    # --- 1. The curve list ----------------------------------------------
    labels = [ref.Label() for ref in panel.Curves()]
    _Check(labels[:3] == [TX, TY, TZ],
           "the three channels the file splines come first, in order: %s"
           % (labels[:3],))
    _Check(RX in labels and "HandIK.rest:tx" in labels,
           "the unanimated avar and rest channels are listed too, so an "
           "unkeyed channel can be keyed from the editor: %s" % (labels,))
    # Curves() is deliberately longer than the drawn set (spec 1.3), so
    # the key-count assertions all go through KeyPixels().
    _Check(sorted(d.canvas.KeyPixels()) == sorted([TX, TY, TZ]),
           "only the three splined channels draw keys: %s"
           % (sorted(d.canvas.KeyPixels()),))
    for label in (TX, TY, TZ):
        times = sorted(k[2] for k in d.canvas.KeyPixels()[label])
        _Check(times == list(FILE_FRAMES),
               "%s draws the file's three knots: %s" % (label, times))

    # --- 2. Property selection narrows the curve set --------------------
    api.dataModel.selection.setPropPath(Sdf.Path(CONTROL + ".avars:tx"))
    d.Pump()
    _Check([ref.Label() for ref in panel.Curves()] == [TX],
           "picking one property in the browser leaves one curve: %s"
           % ([ref.Label() for ref in panel.Curves()],))
    _Check(sorted(d.canvas.KeyPixels()) == [TX],
           "and only that curve draws")
    api.dataModel.selection.clearProps()
    d.Pump()
    _Check(len(panel.Curves()) == len(labels),
           "clearing the property selection falls back to the prim's "
           "channels: %s" % (len(panel.Curves()),))
    panel.FrameAll()
    d.Pump()

    # --- 3. Dragging a key moves it in the session spline ---------------
    before = Ts.Spline(d.Spline("avars:ty"))
    beforeValue = d.Knot("avars:ty", 1024).GetValue()
    beforeAt1024 = d.Value("avars:ty", 1024)
    panel.SelectKeys([(TY, 1024.0)])
    d.Pump()
    _Check(panel.SelectedKeys() == [(TY, 1024.0)],
           "one key selected: %s" % (panel.SelectedKeys(),))
    press = d.KeyPixel(TY, 1024.0)
    # Right by four frames' worth of pixels and up by 60, so the drag
    # crosses whole frames and cannot be mistaken for jitter.
    perFrame = (d.canvas.transform.TimeToX(1025.0)
                - d.canvas.transform.TimeToX(1024.0))
    d.Drag(press, (press[0] + perFrame * 4.0, press[1] - 60.0))

    spec = d.SessionSpec("avars:ty")
    _Check(spec is not None and spec.HasSpline(),
           "the drag authored a whole spline into the session layer")
    moved = panel.SelectedKeys()
    _Check(len(moved) == 1 and moved[0][0] == TY,
           "the dragged key stayed selected: %s" % (moved,))
    movedTime = moved[0][1]
    _Check(abs(movedTime - 1028.0) < 1e-9,
           "the key moved four whole frames, snapped: 1024 -> %s"
           % (movedTime,))
    _Check(d.Spline("avars:ty").GetKnot(1024.0) is None,
           "and nothing was left behind on the frame it came from")
    knot = d.Knot("avars:ty", movedTime)
    _Check(knot.GetValue() - beforeValue > 1e-6,
           "dragging up raised the knot's value: %s -> %s"
           % (beforeValue, knot.GetValue()))
    _Check(abs(d.Value("avars:ty", movedTime) - knot.GetValue()) < 1e-9,
           "the attribute resolves to the moved knot's value at its new "
           "frame: %s" % (d.Value("avars:ty", movedTime),))
    _Check(abs(d.Value("avars:ty", 1024) - beforeAt1024) > 1e-6,
           "and the value the viewport shows at frame 1024 changed: "
           "%s -> %s" % (beforeAt1024, d.Value("avars:ty", 1024)))

    # --- 4. One Ctrl+Z restores the spline exactly ----------------------
    _Check(stack.CanUndo(), "the whole drag is one undo step")
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    _Check(d.Spline("avars:ty") == before,
           "Ctrl+Z restored the pre-drag spline exactly, knots, tangents "
           "and infinities alike")
    _Check(d.SessionSpec("avars:ty") is None,
           "the undo removed the session spec the drag created")
    _Check(abs(d.Value("avars:ty", 1024) - beforeAt1024) < 1e-9,
           "so the attribute is back to %s at frame 1024" % (beforeAt1024,))
    d.Rewind(stack)

    # --- 5. Insert Key holds the value the curve already had ------------
    appController.setFrame(1030)
    d.Pump()
    evaluated = dict((name, d.Spline(name).Eval(1030.0))
                     for name in ("avars:tx", "avars:ty", "avars:tz"))
    restingRx = d.Value("avars:rx", 1030)
    _Check(panel.InsertKeyAtCurrentFrame(), "Insert Key ran")
    d.Pump()
    for name, value in evaluated.items():
        knot = d.Knot(name, 1030)
        _Check(abs(knot.GetValue() - value) < 1e-9,
               "the inserted %s key holds the value the curve already "
               "evaluated to at 1030: %s vs %s"
               % (name, knot.GetValue(), value))
        _Check(abs(d.Value(name, 1030) - value) < 1e-9,
               "so inserting a key did not move the character")
        _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmAutoEase
               and knot.GetPostTanAlgorithm()
               == Ts.TangentAlgorithmAutoEase,
               "and it came in with Maya's Auto tangents on both sides: "
               "%s / %s" % (knot.GetPreTanAlgorithm(),
                            knot.GetPostTanAlgorithm()))
    # An unanimated channel has nothing to evaluate, so its first key
    # comes from the attribute's resolved value instead.
    rx = d.Spline("avars:rx").GetKnot(1030.0)
    _Check(rx is not None
           and abs(rx.GetValue() - (restingRx or 0.0)) < 1e-9,
           "the unanimated avars:rx was keyed at its resting value %s: %s"
           % (restingRx, rx))
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    _Check(d.Spline("avars:tx").GetKnot(1030.0) is None
           and not d.Spline("avars:rx").GetKnots(),
           "one Ctrl+Z took back the whole insert, every curve at once")
    d.Rewind(stack)

    # --- 6. Delete ------------------------------------------------------
    panel.SelectKeys([(TZ, 1024.0)])
    heldBefore = d.Value("avars:tz", 1024)
    _Check(panel.DeleteSelectedKeys(), "Delete Keys ran")
    d.Pump()
    _Check(d.Spline("avars:tz").GetKnot(1024.0) is None,
           "the key at 1024 is gone from the tz spline")
    _Check(abs(d.Value("avars:tz", 1024) - heldBefore) > 1e-6,
           "and the curve now interpolates straight through 1024: "
           "%s -> %s" % (heldBefore, d.Value("avars:tz", 1024)))
    _Check(panel.SelectedKeys() == [],
           "nothing is left selected: %s" % (panel.SelectedKeys(),))
    d.Rewind(stack)
    _Check(d.Spline("avars:tz").GetKnot(1024.0) is not None,
           "undo brought the deleted key back")

    # --- 7. Tangent types: Flat and Step --------------------------------
    panel.SetTangentSide("both")
    panel.SelectKeys([(TX, 1024.0)])
    panel.SetTangentType("flat")
    d.Pump()
    knot = d.Knot("avars:tx", 1024)
    _Check(abs(knot.GetPreTanSlope()) < 1e-9
           and abs(knot.GetPostTanSlope()) < 1e-9,
           "Flat authored a zero slope on both sides: %s / %s"
           % (knot.GetPreTanSlope(), knot.GetPostTanSlope()))
    _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmCustom,
           "a flat tangent is an authored one, not an automatic one")
    _Check(abs(d.Spline("avars:tx").EvalDerivative(1024.0)) < 1e-6,
           "and the curve really is level there: derivative %s"
           % (d.Spline("avars:tx").EvalDerivative(1024.0),))
    d.Rewind(stack)

    panel.SelectKeys([(TX, 1024.0)])
    panel.SetTangentType("step")
    d.Pump()
    _Check(d.Knot("avars:tx", 1024).GetNextInterpolation() == Ts.InterpHeld,
           "Step held the segment AFTER the key: %s"
           % (d.Knot("avars:tx", 1024).GetNextInterpolation(),))
    _Check(abs(d.Value("avars:tx", 1040)
               - d.Knot("avars:tx", 1024).GetValue()) < 1e-9,
           "so every frame up to the next key resolves to the stepped "
           "value: %s" % (d.Value("avars:tx", 1040),))
    d.Rewind(stack)

    # --- 8. Dragging a tangent handle -----------------------------------
    panel.SetVisibleCurves([TX])
    panel.SelectKeys([(TX, 1024.0)])
    panel.SetTangentType("auto")
    d.Pump()
    knot = d.Knot("avars:tx", 1024)
    _Check(knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmAutoEase,
           "the handle starts out automatic, so the canvas draws it "
           "hollow and a drag has to unlock it")
    slopeBefore = knot.GetPostTanSlope()
    handle = d.TangentPixel(TX, 1024.0, "out")
    d.Drag(handle, (handle[0], handle[1] - 50.0))
    knot = d.Knot("avars:tx", 1024)
    _Check(knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmCustom,
           "dragging a locked handle switched the key to a custom "
           "tangent: %s" % (knot.GetPostTanAlgorithm(),))
    _Check(knot.GetPostTanSlope() - slopeBefore > 1e-6,
           "dragging the handle UP steepened the out slope: %s -> %s"
           % (slopeBefore, knot.GetPostTanSlope()))
    _Check(abs(d.Knot("avars:tx", 1024).GetValue()
               - d.Spline("avars:tx").Eval(1024.0)) < 1e-9,
           "and the key itself did not move while its handle did")
    d.Rewind(stack)

    # Weighted: the same gesture now changes the handle's LENGTH too.
    panel.SelectKeys([(TX, 1024.0)])
    panel.SetTangentType("auto")
    d.Pump()
    panel.SetWeighted(True)
    _Check(panel.Weighted(), "Weighted is on")
    widthBefore = d.Knot("avars:tx", 1024).GetPostTanWidth()
    handle = d.TangentPixel(TX, 1024.0, "out")
    d.Drag(handle, (handle[0] + 70.0, handle[1] - 10.0))
    widthAfter = d.Knot("avars:tx", 1024).GetPostTanWidth()
    _Check(widthAfter - widthBefore > 1e-6,
           "dragging the handle outward with Weighted on lengthened it: "
           "%s -> %s" % (widthBefore, widthAfter))
    panel.SetWeighted(False)
    d.Rewind(stack)

    # --- 9. Break, then Unify -------------------------------------------
    panel.SelectKeys([(TX, 1024.0)])
    panel.SetTangentSide("both")
    panel.SetTangentType("spline")
    d.Pump()
    _Check(panel.Break(), "Break ran")
    d.Pump()
    _Check(not graphModel.IsUnified(d.Knot("avars:tx", 1024)),
           "the two sides are now independent")
    panel.SetTangentSide("out")
    panel.SetTangentType("flat")
    d.Pump()
    knot = d.Knot("avars:tx", 1024)
    _Check(abs(knot.GetPostTanSlope()) < 1e-9
           and abs(knot.GetPreTanSlope()) > 1e-6,
           "flattening only the Out side left a corner at the key: "
           "%s in, %s out" % (knot.GetPreTanSlope(),
                              knot.GetPostTanSlope()))
    panel.SetTangentSide("both")
    _Check(panel.Unify(), "Unify ran")
    d.Pump()
    knot = d.Knot("avars:tx", 1024)
    _Check(abs(knot.GetPreTanSlope() - knot.GetPostTanSlope()) < 1e-9,
           "Unify copied the out slope to the in side, so the curve runs "
           "smoothly through the key again: %s / %s"
           % (knot.GetPreTanSlope(), knot.GetPostTanSlope()))
    _Check(graphModel.IsUnified(knot), "and the key reads as unified")
    d.Rewind(stack)

    # --- 10. Infinity ---------------------------------------------------
    # SetInfinity applies to the curves the editor is SHOWING, because
    # picking rows in the curve list is what isolates them; tx alone is
    # visible from section 8, so this is a one-curve edit.
    _Check(panel.VisibleIndices()
           and [panel.Curves()[i].Label()
                for i in panel.VisibleIndices()] == [TX],
           "tx is the only visible curve, so the infinity edit is "
           "confined to it")
    _Check(panel.SetInfinity("cycle", "oscillate"), "Set Infinity ran")
    d.Pump()
    spline = d.Spline("avars:tx")
    _Check(spline.GetPreExtrapolation().mode == Ts.ExtrapLoopReset,
           "Maya's Cycle is TsExtrapLoopReset -- the curve repeated "
           "exactly -- not LoopRepeat, which is Cycle with Offset: %s"
           % (spline.GetPreExtrapolation().mode,))
    _Check(spline.GetPostExtrapolation().mode == Ts.ExtrapLoopOscillate,
           "and Oscillate is TsExtrapLoopOscillate: %s"
           % (spline.GetPostExtrapolation().mode,))
    # The mapping is only worth anything if the stage evaluates that way.
    period = FILE_FRAMES[-1] - FILE_FRAMES[0]
    inside = spline.Eval(1024.0)
    _Check(abs(d.Value("avars:tx", 1024.0 - period) - inside) < 1e-6,
           "one period before the first key the cycle repeats frame "
           "1024's value %s, not the held end value: %s"
           % (inside, d.Value("avars:tx", 1024.0 - period)))
    mirrored = FILE_FRAMES[-1] + (FILE_FRAMES[-1] - 1024.0)
    _Check(abs(d.Value("avars:tx", mirrored) - inside) < 1e-6,
           "and past the last key the oscillation plays the curve "
           "backwards, so frame %s resolves to %s: %s"
           % (mirrored, inside, d.Value("avars:tx", mirrored)))
    d.Rewind(stack)
    _Check(d.Spline("avars:tx").GetPreExtrapolation().mode == Ts.ExtrapHeld,
           "undo put the infinities back to Constant")
    panel.SetVisibleCurves([ref.Label() for ref in panel.Curves()])
    d.Pump()

    # --- 11. Scrubbing the frame ruler ----------------------------------
    appController.setFrame(1010)
    d.Pump()
    _Check(api.dataModel.currentFrame.GetValue() == 1010,
           "the playhead starts at 1010: %s" % (api.dataModel.currentFrame,))
    d.Drag(d.RulerPoint(1012), d.RulerPoint(1036))
    _Check(api.dataModel.currentFrame.GetValue() == 1036,
           "dragging along the bottom ruler scrubbed usdview itself to "
           "1036: %s" % (api.dataModel.currentFrame,))
    _Check(panel.CurrentFrame() == 1036,
           "and the editor's playhead followed: %s" % (panel.CurrentFrame(),))
    _Check(not stack.CanUndo(),
           "scrubbing is not an edit, so it pushed nothing onto the stack")

    # --- 12. Marquee selection ------------------------------------------
    panel.SetSelection([])
    panel.FrameAll()
    d.Pump()
    expected = _KnotCount(panel, (TX, TY, TZ))
    _Check(expected == 9,
           "the three curves hold the file's nine knots again: %s"
           % (expected,))
    plot = d.canvas.PlotRectF()
    d.Drag((plot.left() + 2.0, plot.top() + 2.0),
           (plot.right() - 2.0, plot.bottom() - 2.0))
    selected = panel.SelectedKeys()
    _Check(len(selected) == expected,
           "a marquee over the whole plot took all %d keys: %s"
           % (expected, selected))
    _Check(sorted(set(label for label, _ in selected))
           == sorted([TX, TY, TZ]),
           "from all three curves: %s" % (selected,))

    # A marquee round one key takes exactly that one.
    point = d.KeyPixel(TY, 1024.0)
    d.Drag((point[0] - 12.0, point[1] - 12.0),
           (point[0] + 12.0, point[1] + 12.0))
    _Check(panel.SelectedKeys() == [(TY, 1024.0)],
           "a marquee round one key takes exactly it: %s"
           % (panel.SelectedKeys(),))

    shot = os.environ.get("RIGEXEC_GRAPH_SHOT")
    if shot:
        panel.SelectKeys([(TY, 1024.0), (TX, 1048.0)])
        panel.SetTangentType("auto")
        panel.FrameAll()
        d.Pump()
        panel.window().grab().save(shot)
        d.Rewind(stack)

    print("RIGEXEC_GRAPH_OK curves, property filter, key drag + undo, "
          "insert, delete, tangent types, handle drag, break/unify, "
          "infinity, scrub, marquee")
