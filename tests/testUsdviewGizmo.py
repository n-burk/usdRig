#
# THE VIEWPORT GIZMO TOOLBAR, end to end.
#
# Written for the same reason as testUsdviewCurvenetMove.py: the headless
# tests in tests/python call the math and the edit helpers directly and
# never go near a camera, so they cannot see the one thing a manipulator
# is -- a PROJECTION that a mouse has to be able to hit. This script opens
# examples/ArmShotAnim.usda (the container installs the toolbar when the
# stage loads), selects the HandIK control and drives synthetic mouse and
# key events through the gizmo's own projected handle positions. Every
# assertion is about what landed on the stage afterwards, not about what
# the controller thinks it did.
#
# Set RIGEXEC_GIZMO_SHOT=/path.png to save a window grab for inspection.
#
import math
import os

from pxr import Gf, Sdf, UsdGeom

CONTROL = "/Shot/HeroArm/Rig/Controls/HandIK"
XFORM = "/Shot/HeroArm"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Mouse(QtCore, QtGui, view, kind, x, y, button=None, buttons=None,
           modifiers=None):
    """
    One synthetic mouse event in the view's LOGICAL pixels, which is what
    HandleScreenPositions() reports and what Qt delivers.

    `button` is separate from `buttons` because the controller reads
    event.button() to tell a left grab from the conventional middle-drag-repeats-
    the-selected-handle; a helper that hard-codes LeftButton can send a
    "middle" press the controller will never recognise as one.
    """
    pos = QtCore.QPointF(float(x), float(y))
    glob = view.mapToGlobal(QtCore.QPoint(int(x), int(y)))
    globF = QtCore.QPointF(float(glob.x()), float(glob.y()))
    if button is None:
        button = QtCore.Qt.LeftButton
    if buttons is None:
        buttons = button
    if modifiers is None:
        modifiers = QtCore.Qt.NoModifier
    return QtGui.QMouseEvent(kind, pos, globF, button, buttons, modifiers)


def _Lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


class _Driver(object):
    """Mouse, keyboard and selection, in the units the controller uses."""

    def __init__(self, appController, controller):
        from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets
        import gizmoUI
        self.QtCore, self.QtGui, self.QtWidgets = QtCore, QtGui, QtWidgets
        self.app = appController
        self.api = appController._usdviewApi
        self.controller = controller
        self.view = gizmoUI.StageView(self.api)

    def Pump(self):
        self.app._processEvents()

    def Ratio(self):
        return self.view.devicePixelRatioF()

    def Send(self, kind, x, y, **kw):
        self.QtWidgets.QApplication.sendEvent(
            self.view, _Mouse(self.QtCore, self.QtGui, self.view, kind,
                              x, y, **kw))
        self.Pump()

    def Press(self, point, **kw):
        self.Send(self.QtCore.QEvent.Type.MouseButtonPress, *point, **kw)

    def Move(self, point, **kw):
        self.Send(self.QtCore.QEvent.Type.MouseMove, *point, **kw)

    def Release(self, point, button=None):
        self.Send(self.QtCore.QEvent.Type.MouseButtonRelease, *point,
                  button=button, buttons=self.QtCore.Qt.NoButton)

    def Drag(self, start, end, steps=4, button=None):
        self.Press(start, button=button)
        _Check(self.controller.IsDragging(),
               "a press at %s started a drag (handles: %s)" % (
                   (start,), sorted(self.controller.HandleScreenPositions())))
        for i in range(1, steps + 1):
            self.Move(_Lerp(start, end, i / float(steps)),
                      button=self.QtCore.Qt.NoButton,
                      buttons=button or self.QtCore.Qt.LeftButton)
        self.Release(end, button=button)
        _Check(not self.controller.IsDragging(), "the release ended the drag")

    def AxisPoints(self, name):
        positions = self.controller.HandleScreenPositions()
        _Check(name in positions, "handle %r is present, not just %s" % (
            name, sorted(positions)))
        return positions[name][0], positions[name][-1]

    def DragAxis(self, name, fraction=0.35, start=0.5):
        """Slide along an axis handle by `fraction` of its screen length."""
        a, b = self.AxisPoints(name)
        self.Drag(_Lerp(a, b, start), _Lerp(a, b, start + fraction))

    def DragRing(self, name, quarter=6):
        positions = self.controller.HandleScreenPositions()
        _Check(name in positions, "ring %r is present, not just %s" % (
            name, sorted(positions)))
        points = positions[name]
        self.Drag(points[0], points[quarter % len(points)])

    def Key(self, key, modifiers=None):
        # QtTest is the one Qt module pxr.Usdviewq.qt does not re-export, so
        # it has to come from the binding directly. That is fine in a test;
        # the plugin modules themselves still go through Usdviewq.qt.
        try:
            from PySide6 import QtTest
        except ImportError:
            from PySide2 import QtTest
        if modifiers is None:
            modifiers = self.QtCore.Qt.NoModifier
        self.view.setFocus()
        QtTest.QTest.keyClick(self.view, key, modifiers)
        self.Pump()

    def KeyDown(self, key, modifiers=None):
        # A HOLD, not Key()'s click: Key() is QTest.keyClick, a press
        # immediately followed by a release, so the hold is armed and
        # cleared before the next mouse move (probed: _holdGrid is
        # False after a mid-drag keyClick(Key_X) and True across a
        # move after keyPress(Key_X)). Every hold assertion hand-rolls
        # Press / Move / KeyDown / Move / KeyUp / Release, never Drag().
        try:
            from PySide6 import QtTest
        except ImportError:
            from PySide2 import QtTest
        if modifiers is None:
            modifiers = self.QtCore.Qt.NoModifier
        self.view.setFocus()
        QtTest.QTest.keyPress(self.view, key, modifiers)
        self.Pump()

    def KeyUp(self, key, modifiers=None):
        try:
            from PySide6 import QtTest
        except ImportError:
            from PySide2 import QtTest
        if modifiers is None:
            modifiers = self.QtCore.Qt.NoModifier
        self.view.setFocus()
        QtTest.QTest.keyRelease(self.view, key, modifiers)
        self.Pump()

    def Select(self, path):
        prim = self.api.stage.GetPrimAtPath(path)
        _Check(prim, "the stage has a prim at %s" % path)
        self.api.ClearPrimSelection()
        self.api.AddPrimToSelection(prim)
        self.Pump()
        return prim

    def SelectMany(self, paths):
        """
        Several prims, in pick order, the way the viewport does it.

        Through dataModel.selection rather than the UsdviewApi pair:
        ClearPrimSelection() followed by AddPrimToSelection() leaves the
        PSEUDO-ROOT in the selection and focused (measured -- a four-
        control script selection reads back as ['/', ...] with `/` as
        api.prim), and a test that only ever exercised that path would
        not notice the gizmo mis-reading the lead. setPrim + addPrim is
        what a shift-click and the Control Picker both do.
        """
        selection = self.app._dataModel.selection
        prims = []
        for index, path in enumerate(paths):
            prim = self.api.stage.GetPrimAtPath(path)
            _Check(prim, "the stage has a prim at %s" % path)
            if index == 0:
                selection.setPrim(prim)
            else:
                selection.addPrim(prim)
            prims.append(prim)
        self.Pump()
        return prims

    def HandleAt(self, point):
        """The handle a left click at `point` would grab, by name."""
        import gizmoScreen
        import gizmoUI
        ratio = self.Ratio()
        hit = gizmoScreen.HitTest(self.controller.Handles(),
                                  point[0] * ratio, point[1] * ratio,
                                  gizmoUI.HIT_PIXELS * ratio)
        return hit.name if hit is not None else None

    def FreeRotatePoint(self):
        """
        A point inside the free-rotate ball that is not on a ring.

        The rings outrank the sphere in HitTest by design (the ball
        covers all three of them), and an obliquely viewed ring projects
        to an ellipse that passes close to the centre, so "somewhere near
        the middle" is not good enough -- the answer depends on the
        camera. Ask the same HitTest the controller uses instead of
        guessing.
        """
        import gizmoScreen
        import gizmoUI
        handles = self.controller.Handles()
        free = None
        for handle in handles:
            if handle.name == "free":
                free = handle
        _Check(free is not None, "the rotate manipulator has a free-rotate "
               "ball (handles: %s)" % [h.name for h in handles])
        ratio = self.Ratio()
        cx, cy = free.points[0]
        radius = free.radiusPixels
        for fraction in (0.45, 0.3, 0.6, 0.15, 0.75):
            for step in range(24):
                angle = step * math.pi / 12.0
                x = cx + radius * fraction * math.cos(angle)
                y = cy + radius * fraction * math.sin(angle)
                hit = gizmoScreen.HitTest(handles, x, y,
                                          gizmoUI.HIT_PIXELS * ratio)
                if hit is not None and hit.name == "free":
                    return (x / ratio, y / ratio)
        raise AssertionError(
            "some point inside the free-rotate ball resolves to it rather "
            "than to a ring; from this camera none of 120 candidates did")


def _Values(prim, names, frame):
    return [prim.GetAttribute(n).Get(frame) for n in names]


def _World(stage, path, frame):
    """The world-space translation of the prim at `path`."""
    cache = UsdGeom.XformCache(frame)
    return cache.GetLocalToWorldTransform(
        stage.GetPrimAtPath(path)).ExtractTranslation()


def _Landed(controller, stage, path, frame):
    """
    Where the dragged prim is NOW: the manipulator while a drag is live, the
    stage once it has been released.

    A drag in progress authors nothing (see _TestPreviewThenCommit), so
    mid-gesture the stage still holds the pre-drag transform while the
    manipulator -- drawn from the previewed values -- is where the artist sees
    the prim. Both branches answer the same question about the same prim, and
    asking it of the stage mid-drag would answer about the prim's past.
    """
    if controller.IsDragging() and controller.Target() is not None:
        return controller.Target().GizmoMatrix().ExtractTranslation()
    return _World(stage, path, frame)


def _ProjectLogical(controller, world):
    """
    A world point as LOGICAL pixels, which is what _Driver.Send takes.

    Through the controller's own camera -- the one the live drag
    resolves against. ProjectPoint is physical, so the ratio comes
    back off on the way out, the inverse of _Position's way in
    (gizmoUI.py:2075-2091).
    """
    import gizmoScreen
    camera, viewport, ratio = controller._Camera()
    screen = gizmoScreen.ProjectPoint(
        gizmoScreen.ViewProjection(camera), viewport,
        Gf.Vec3d(*world))
    if screen is None:
        return None
    return (screen[0] / ratio, screen[1] / ratio)


def _Changed(before, after, tolerance=1e-6):
    return [i for i in range(len(before))
            if abs((before[i] or 0.0) - (after[i] or 0.0)) > tolerance]


GROUP = ["/Shot/HeroArm/Rig/Controls/ShoulderFK",
         "/Shot/HeroArm/Rig/Controls/ElbowFK",
         "/Shot/HeroArm/Rig/Controls/WristFK"]
SOLVED_JOINT = "/Shot/HeroArm/Rig/Joints/Shoulder"


def _TestGroupSelection(d, controller, stage, session, frame):
    """
    SEVERAL controls, one manipulator, through the real application.

    The headless tests assert the maths; this asserts the wiring the
    maths hangs off, which is where two reported bugs actually lived:

      * usdview's focus prim is getPrimPaths()[0] -- the FIRST prim in
        the selection, not the last (selectionDataModel.py:623-627). The
        group took its lead from the focus prim, so the FIRST control
        picked was orienting the gizmo and answering to the Last
        Selected pivot. Measured in a live usdview: shift-picking
        shoulder, elbow, wrist reported api.prim == ShoulderFK.
      * Individual Origins applied the carried-member filter, which is
        right for the rigid pivots and wrong for this one. Every FK
        chain in a RigExec rig is a namespace chain
        (controlSpace="parentRelative"), so on Biped.usda a three-
        control finger, arm or leg selection had every member but the
        topmost dropped: measured, a delta reached 1 of 3 in all three
        modes, which is indistinguishable from the mode doing nothing.

    The controls here are SIBLINGS, which is what makes the pivot modes
    separable in one drag: with nobody carrying anybody, Centre orbits
    all three about their centroid and Individual leaves every one of
    them exactly where it was.
    """
    import gizmoMath
    import gizmoSettings
    import gizmoUI

    def Posed(path):
        # The RIG's frame, not UsdGeomXformCache's: a control's world
        # transform is composed by the evaluator from the avars above
        # it, and the USD xform cache knows nothing about that. _World
        # above is for the plain xform cases.
        frames = gizmoMath.ComputeRigFrames(
            stage, stage.GetPrimAtPath(path), frame)
        return frames.posed * frames.assetToWorld

    def Origin(path):
        return Gf.Vec3d(Posed(path).ExtractTranslation())

    def Turned(path, before):
        return (before.GetInverse()
                * Posed(path).GetOrthonormalized(False)
                ).ExtractRotation().GetAngle()

    def Authored(path):
        return [session.GetAttributeAtPath(Sdf.Path(path + "." + n))
                is not None for n in gizmoMath.AVAR_R]

    def Centroid(points):
        return (points[0] + points[1] + points[2]) / 3.0

    # -- the selection, and which end of it leads ----------------------
    d.SelectMany(GROUP)
    controller.SetTool(gizmoUI.TOOL_ROTATE)
    d.Pump()
    target = controller.Target()
    _Check(target is not None and target.kind == "group",
           "three selected controls give one group gizmo: %s"
           % controller.Reason())
    _Check(target.label == "3 controls", "label: %r" % target.label)
    _Check([m.prim.GetPath() for m in target.members]
           == [Sdf.Path(p) for p in GROUP],
           "in selection order: %s" % [m.label for m in target.members])
    _Check(target.lead.prim.GetPath() == Sdf.Path(GROUP[-1]),
           "the LAST-selected control leads, not usdview's focus prim "
           "(which is the first): lead is %r" % target.lead.label)
    d.SelectMany(list(reversed(GROUP)))
    d.Pump()
    _Check(controller.Target().lead.prim.GetPath() == Sdf.Path(GROUP[0]),
           "and it follows the order the artist picked in")

    # -- a real ring drag, in each pivot mode --------------------------
    results = {}
    for mode in gizmoMath.GROUP_PIVOT_MODES:
        d.SelectMany(GROUP)
        controller.SetTool(gizmoUI.TOOL_ROTATE)
        controller.SetGroupPivot(mode)
        d.Pump()
        target = controller.Target()
        _Check(target.pivotMode == mode,
               "the toolbar setting reached the target: %r"
               % target.pivotMode)
        before = [Origin(p) for p in GROUP]
        beforeWorld = [Posed(p).GetOrthonormalized(False)
                       for p in GROUP]
        undoDepth = controller.undoStack.CanUndo()
        d.DragRing("z")
        d.Pump()
        after = [Origin(p) for p in GROUP]
        turns = [Turned(p, w) for p, w in zip(GROUP, beforeWorld)]
        results[mode] = (before, after, turns)
        _Check(min(turns) > 1.0,
               "%s: every member turned (%s)" % (mode, turns))
        _Check(max(turns) - min(turns) < 1e-6,
               "%s: by the SAME angle (%s)" % (mode, turns))
        for path in GROUP:
            _Check(any(Authored(path)),
                   "%s: %s got its own avars:r" % (mode, path))
        # ONE undo entry for the whole group, not one per control.
        _Check(controller.undoStack.CanUndo() and not undoDepth
               or controller.undoStack.CanUndo(),
               "%s: the drag pushed an undo entry" % mode)
        _Check(controller.undoStack.UndoText().endswith("3 controls"),
               "%s: labelled for the group: %r"
               % (mode, controller.undoStack.UndoText()))
        controller.Undo()
        d.Pump()
        for path, start in zip(GROUP, before):
            _Check((Origin(path) - start).GetLength() < 1e-6,
                   "%s: one Undo put %s back" % (mode, path))

    # -- and the modes differ in the way they are supposed to ----------
    before, after, _turns = results[gizmoMath.GROUP_PIVOT_CENTER]
    drift = Centroid(after) - Centroid(before)
    _Check(drift.GetLength() < 1e-6,
           "Centre: the centroid does not move (%s)" % (drift,))
    _Check(max((a - b).GetLength() for a, b in zip(after, before)) > 1e-3,
           "Centre: but the controls orbit it")
    before, after, _turns = results[gizmoMath.GROUP_PIVOT_INDIVIDUAL]
    _Check(max((a - b).GetLength() for a, b in zip(after, before)) < 1e-6,
           "Individual Origins: nobody moves; each turns in place")
    before, after, _turns = results[gizmoMath.GROUP_PIVOT_LEAD]
    _Check((after[-1] - before[-1]).GetLength() < 1e-6,
           "Last Selected: the lead stays put")
    _Check((after[0] - before[0]).GetLength() > 1e-3,
           "Last Selected: the others swing about it")

    # -- a control the rig writes is skipped, not dragged --------------
    d.SelectMany(GROUP[:2] + [SOLVED_JOINT])
    controller.SetTool(gizmoUI.TOOL_ROTATE)
    d.Pump()
    target = controller.Target()
    _Check(target is not None and target.kind == "group",
           "a mixed selection still gives a group: %s" % controller.Reason())
    _Check([m.prim.GetPath() for m in target.members]
           == [Sdf.Path(p) for p in GROUP[:2]],
           "the solver-posed joint is not a member: %s"
           % [m.label for m in target.members])
    _Check("solver" in target.Advisory(),
           "and the status says why: %r" % target.Advisory())
    jointBefore = Origin(SOLVED_JOINT)
    d.DragRing("z")
    d.Pump()
    _Check((Origin(SOLVED_JOINT) - jointBefore).GetLength() < 1e-9,
           "the refused prim did not move")
    for name in gizmoMath.AVAR_R + gizmoMath.AVAR_T:
        _Check(session.GetAttributeAtPath(
            Sdf.Path(SOLVED_JOINT + "." + name)) is None,
            "and nothing was authored onto its %s" % name)
    controller.Undo()
    d.Pump()

    # -- the toolbar button is the same setting, and cycles ------------
    bar = controller.toolbar
    controller.SetTool(gizmoUI.TOOL_ROTATE)
    controller.SetGroupPivot(gizmoMath.GROUP_PIVOT_CENTER)
    d.Pump()
    _Check(bar._groupAction.isEnabled(),
           "Rotate offers a group pivot")
    _Check(not bar._groupAction.isChecked(),
           "the default is not highlighted")
    bar._groupAction.trigger()
    d.Pump()
    _Check(controller.GroupPivot() == gizmoMath.GROUP_PIVOT_LEAD
           and bar._groupAction.isChecked(),
           "one click cycles to %r" % controller.GroupPivot())
    _Check(controller.Target().pivotMode == gizmoMath.GROUP_PIVOT_LEAD,
           "and reaches the live target")
    bar._groupAction.trigger()
    bar._groupAction.trigger()
    d.Pump()
    _Check(controller.GroupPivot() == gizmoMath.GROUP_PIVOT_CENTER,
           "three clicks wrap: %r" % controller.GroupPivot())
    d.Key(d.QtCore.Qt.Key_P)
    _Check(controller.GroupPivot() == gizmoMath.GROUP_PIVOT_LEAD,
           "P is the keyboard twin: %r" % controller.GroupPivot())
    controller.SetGroupPivot(gizmoMath.GROUP_PIVOT_CENTER)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    # Move offers Centre and Individual Origins (each control along its own
    # axes); Last Selected is left out because it moves a selection exactly
    # as the centre does, so the cycle has two stops.
    _Check(bar._groupAction.isEnabled(),
           "Move offers a group pivot: centre or individual origins")
    bar._groupAction.trigger()
    d.Pump()
    _Check(controller.GroupPivot() == gizmoMath.GROUP_PIVOT_INDIVIDUAL,
           "Move cycles from the centre to individual origins: %r"
           % controller.GroupPivot())
    bar._groupAction.trigger()
    d.Pump()
    _Check(controller.GroupPivot() == gizmoMath.GROUP_PIVOT_CENTER,
           "and back to the centre, skipping Last Selected: %r"
           % controller.GroupPivot())

    # -- Global / Local, the other toolbar toggle ----------------------
    controller.SetTool(gizmoUI.TOOL_ROTATE)
    d.SelectMany(GROUP)
    d.Pump()
    # Turn the LEAD off axis first. Global and Local name different
    # frames, but on a control whose posed frame happens to be axis-
    # aligned they name the same AXES, and an assertion that the drawn
    # ring changed would be vacuous -- or fail -- for a reason that has
    # nothing to do with the toggle.
    leadPath = Sdf.Path(GROUP[-1])
    stage.GetPrimAtPath(leadPath).GetAttribute("avars:ry").Set(40.0)
    d.Pump()
    controller.SetOrientation(gizmoSettings.ORIENT_WORLD)
    d.Pump()
    _Check(bar._orientAction.text() == "Global"
           and not bar._orientAction.isChecked(),
           "the button names the mode in force: %r"
           % bar._orientAction.text())
    worldRing = controller._Handle("z").worldAxis
    d.Key(d.QtCore.Qt.Key_L)
    _Check(controller.Orientation() == gizmoSettings.ORIENT_OBJECT,
           "L switches to Local")
    d.Pump()
    _Check(bar._orientAction.text() == "Local"
           and bar._orientAction.isChecked(),
           "and the button follows: %r" % bar._orientAction.text())
    localRing = controller._Handle("z").worldAxis
    _Check((Gf.Vec3d(worldRing) - Gf.Vec3d(localRing)).GetLength() > 1e-3,
           "and the DRAWN ring axis actually changed: %s -> %s"
           % (worldRing, localRing))
    leadZ = Posed(GROUP[-1]).GetOrthonormalized(False).TransformDir(
        Gf.Vec3d(0, 0, 1))
    _Check((Gf.Vec3d(localRing) - leadZ).GetLength() < 1e-6,
           "Local rings ARE the lead control's own axes: %s vs %s"
           % (localRing, leadZ))
    controller.SetOrientation(gizmoSettings.ORIENT_WORLD)
    spec = session.GetAttributeAtPath(
        leadPath.AppendProperty("avars:ry"))
    if spec is not None:
        spec.owner.RemoveProperty(spec)
    d.Pump()

def _TestPreviewThenCommit(d, controller, stage, session, frame):
    """
    A drag in progress does not touch the document; the release does, once
    (docs/superpowers/specs/2026-09-10-hydra-preview-manipulation-design.md).

    The assertions the headless tests cannot make: that a real mouse drag
    through the real application leaves the edit target layer alone while the
    manipulator and the viewport both follow it, and that letting go is what
    writes -- which is the difference between authoring an unfinished gesture
    hundreds of times and authoring the result of it once.
    """
    import gizmoMath
    import gizmoUI

    prim = d.Select(CONTROL)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    specPath = Sdf.Path(CONTROL + ".avars:tx")
    tx = prim.GetAttribute("avars:tx")

    # Start from a known, committed state so "nothing was authored" below means
    # nothing at all rather than nothing new.
    session.RemovePropertyIfHasOnlyRequiredFields(
        session.GetAttributeAtPath(specPath)) \
        if session.GetAttributeAtPath(specPath) else None
    before = tx.Get(frame)
    undosBefore = controller.undoStack.UndoCount() \
        if hasattr(controller.undoStack, "UndoCount") else None

    a, b = d.AxisPoints("x")
    start, end = _Lerp(a, b, 0.5), _Lerp(a, b, 0.85)
    d.Press(start, button=d.QtCore.Qt.LeftButton)
    _Check(controller.IsDragging(), "the press started a drag")
    gizmoBefore = controller.Target().GizmoMatrix().ExtractTranslation()

    # --- mid-drag ---------------------------------------------------------
    for i in (1, 2, 3):
        d.Move(_Lerp(start, end, i / 3.0),
               button=d.QtCore.Qt.NoButton,
               buttons=d.QtCore.Qt.LeftButton)
    _Check(session.GetAttributeAtPath(specPath) is None,
           "mid-drag the edit target layer has NO spec for the channel being "
           "dragged; found %s" % session.GetAttributeAtPath(specPath))
    _Check(abs(tx.Get(frame) - before) < 1e-12,
           "and the composed value has not moved either: %s -> %s"
           % (before, tx.Get(frame)))
    # The value IS real everywhere the artist can see it: the preview channel
    # holds it, and the manipulator is drawn from it.
    pending = gizmoMath.PreviewValues()
    _Check(specPath in pending,
           "the dragged channel is in the preview: %s"
           % sorted(str(p) for p in pending))
    gizmoMid = controller.Target().GizmoMatrix().ExtractTranslation()
    _Check((gizmoMid - gizmoBefore).GetLength() > 1e-6,
           "the manipulator followed the uncommitted drag: %s -> %s"
           % (gizmoBefore, gizmoMid))

    # --- release ----------------------------------------------------------
    d.Release(end, button=d.QtCore.Qt.LeftButton)
    _Check(not controller.IsDragging(), "the release ended the drag")
    spec = session.GetAttributeAtPath(specPath)
    _Check(spec is not None,
           "the release authored the channel into the edit target layer")
    _Check(abs(tx.Get(frame) - before) > 1e-6,
           "and the value moved: %s -> %s" % (before, tx.Get(frame)))
    _Check(gizmoMath.PreviewValues() == {},
           "the preview is over; the stage is the authority again")
    _Check(controller.undoStack.CanUndo(),
           "the whole drag is one undo entry")
    after = tx.Get(frame)

    # --- an aborted drag authors nothing ----------------------------------
    d.Press(start, button=d.QtCore.Qt.LeftButton)
    for i in (1, 2):
        d.Move(_Lerp(start, end, i / 2.0),
               button=d.QtCore.Qt.NoButton,
               buttons=d.QtCore.Qt.LeftButton)
    d.Key(d.QtCore.Qt.Key_Escape)
    _Check(not controller.IsDragging(), "Escape ended the drag")
    _Check(gizmoMath.PreviewValues() == {},
           "an aborted drag stops previewing")
    _Check(abs(tx.Get(frame) - after) < 1e-9,
           "and leaves the committed value exactly as it was: %s vs %s"
           % (tx.Get(frame), after))

    # Undo the committed drag, so the rest of the suite starts where it did.
    controller.Undo()
    d.Pump()
    _Check(abs(tx.Get(frame) - before) < 1e-9,
           "undo restored the pre-drag value: %s" % tx.Get(frame))


def testUsdviewInputFunction(appController):
    import gizmoMath
    import gizmoSettings
    import gizmoUI

    appController._processEvents()
    controller = gizmoUI.GetController()
    _Check(controller is not None,
           "the container installed the viewport tools on stage load")
    d = _Driver(appController, controller)
    stage = d.api.stage
    session = stage.GetSessionLayer()
    _Check(stage.GetEditTarget().GetLayer() == session,
           "usdview's edit target is the session layer, which is where "
           "every assertion below looks for the authored spec")
    frame = d.api.frame

    # testusdview's window is never activated by the window manager, so
    # QApplication.activeWindow() is None and Qt's shortcut map refuses
    # EVERY shortcut before it looks at the key -- Ctrl+Z would silently
    # do nothing here for reasons that have nothing to do with the gizmo.
    # Supplying the activation the window manager would is what lets the
    # undo/redo assertions below exercise the real key path rather than
    # calling the actions by hand.
    d.QtWidgets.QApplication.setActiveWindow(appController._mainWindow)
    d.Pump()
    _Check(d.QtWidgets.QApplication.activeWindow() is not None,
           "the main window is active, so application shortcuts dispatch")

    # The toolbar fits usdview's DEFAULT width: at the designer
    # default (mainWindowUI.py:37, 1145x1002) the viewport frame is
    # 598 logical px, and the Snap: button once pushed Undo/Redo into
    # the overflow chevron. Asserted here, before the resize below --
    # at 1800 px everything fits and the check would be vacuous.
    # NOTHING folds into QToolBar's overflow chevron at usdview's default
    # width. This used to ask only that Undo and Redo survive, and it was the
    # most the row could promise: text buttons are as wide as the platform's
    # UI font makes them, and on Windows at 10pt the row wanted 856 px against
    # the ~600 it gets, so Snap, Undo, Redo, Settings and Graph all went into
    # the chevron -- where, as the toolbar's own comment says, nobody finds
    # them. The row is glyphs now (gizmoIcons), which are the width we choose
    # rather than the width a font imposes, so every control fits everywhere
    # and the assertion can say so.
    d.Pump()
    bar = controller.toolbar
    _Check(bar.width() > 0, "the toolbar is laid out already")
    _Check(bar.sizeHint().width() <= bar.width(),
           "the toolbar fits the default viewport width (needs %d px, has "
           "%d)" % (bar.sizeHint().width(), bar.width()))
    for action in bar.actions():
        # A separator has a widget of its own (a plain QWidget), so it has to
        # be filtered on the ACTION, not on the widget being None.
        if action.isSeparator():
            continue
        widget = bar.widgetForAction(action)
        name = action.text() or getattr(widget, "text", lambda: "")()             or type(widget).__name__
        _Check(widget is not None and widget.isVisibleTo(bar),
               "%r stays out of the overflow chevron at the default width "
               "(toolbar %d px, sizeHint %d)" % (
                   name, bar.width(), bar.sizeHint().width()))

    # Every button says what it is twice over: a glyph to look at and a
    # tooltip to read. With the words gone the tooltip is the only text an
    # artist can reach, so a button without one is unlabelled.
    for action in (list(bar._toolActions.values())
                   + list(bar._channelActions.values())
                   + list(bar._writeActions.values())
                   + [bar.undoAction, bar.redoAction,
                      bar.settingsAction, bar.graphAction]):
        _Check(not action.icon().isNull(),
               "%r has a glyph" % action.text())
        _Check(action.toolTip(), "%r has a tooltip" % action.text())

    # The ARTWORK is complete. gizmoIcons falls back to drawing a glyph when a
    # file is missing, which is what keeps a broken install from showing a row
    # of blank buttons -- and would also let a set that quietly lost half its
    # art ship looking almost right. Asserted here so "almost right" fails.
    import gizmoIcons
    missing = [n for n in gizmoIcons.Names() if not gizmoIcons.HasArt(n)]
    _Check(not missing,
           "every glyph is served by artwork rather than the drawn fallback; "
           "missing: %s" % missing)

    # Widen the window so the toolbar is not folded into QToolBar's
    # overflow chevron, which is where testusdview's default width puts
    # the status label -- the one control a reader of the grab most wants
    # to see. The rendered viewport stays 597x540 whatever this says:
    # testusdview pins it with SetPhysicalWindowSize for reproducible test
    # images, which is why the grab shows a small render in a wide window.
    # Done here rather than before the grab because every drag below reads
    # its coordinates from the controller's own projections, so the layout
    # must settle before anything is projected.
    appController._mainWindow.resize(1800, 1000)
    d.Pump()

    # Frame the control and tumble off the default straight-down-Z view.
    # Not cosmetic: down -Z the Z axis projects to a point, so its handle
    # is locked and ungrabbable, the rotate rings degenerate to lines, and
    # the X arrow lands outside testusdview's 597 px viewport. Every drag
    # below needs three usable axes, so the test asks for the view an
    # animator would actually work in.
    d.Select(CONTROL)
    appController._frameSelection()
    camera = d.api.dataModel.viewSettings.freeCamera
    if camera is not None:
        camera.rotTheta = 35.0
        camera.rotPhi = -25.0
        camera.dist = camera.dist * 2.6
    d.Pump()

    # --- 1. Translate a control in Animation mode, undo, redo ------------
    prim = d.Select(CONTROL)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    tx = prim.GetAttribute("avars:tx")
    before = tx.Get(frame)
    d.DragAxis("x")
    afterX = tx.Get(frame)
    _Check(abs(afterX - before) > 1e-6, "the x drag changed avars:tx at the "
           "frame: %s -> %s" % (before, afterX))
    spec = session.GetAttributeAtPath(Sdf.Path(CONTROL + ".avars:tx"))
    _Check(spec is not None and spec.HasSpline(),
           "animation mode authored a spline knot in the session layer")
    _Check(controller.undoStack.CanUndo(), "the drag pushed one edit")
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    _Check(abs(tx.Get(frame) - before) < 1e-9,
           "Ctrl+Z restored avars:tx: %s" % tx.Get(frame))
    _Check(session.GetAttributeAtPath(Sdf.Path(CONTROL + ".avars:tx"))
           is None, "the undo removed the session spec the drag created")
    d.Key(d.QtCore.Qt.Key_Z,
          d.QtCore.Qt.ControlModifier | d.QtCore.Qt.ShiftModifier)
    _Check(abs(tx.Get(frame) - afterX) < 1e-9,
           "Ctrl+Shift+Z re-applied avars:tx: %s" % tx.Get(frame))
    controller.Undo()
    d.Pump()

    # --- 2. Centre handle moves in the camera plane ----------------------
    positions = controller.HandleScreenPositions()
    _Check("center" in positions, "the move manipulator has a centre handle")
    c = positions["center"][0]
    beforeT = _Values(prim, gizmoMath.AVAR_T, frame)
    d.Drag(c, (c[0] + 30, c[1] - 30))
    afterT = _Values(prim, gizmoMath.AVAR_T, frame)
    _Check(len(_Changed(beforeT, afterT)) >= 2,
           "the centre drag moved the control in the camera plane, which "
           "is oblique to every axis, so at least two avars change: "
           "%s -> %s" % (beforeT, afterT))
    controller.Undo()
    d.Pump()

    # --- 3. Default mode writes the default and warns --------------------
    controller.SetWriteMode(gizmoMath.WRITE_DEFAULT)
    d.Pump()
    d.DragAxis("x")
    spec = session.GetAttributeAtPath(Sdf.Path(CONTROL + ".avars:tx"))
    _Check(spec is not None and spec.HasDefaultValue()
           and not spec.HasSpline(),
           "default mode authored a default, not a knot")
    _Check("outranked" in controller.Status(),
           "the status warns that the file's spline outranks the default: "
           "%r" % controller.Status())
    controller.Undo()
    d.Pump()
    controller.SetWriteMode(gizmoMath.WRITE_ANIMATION)
    d.Pump()

    # --- 4. Rotate and scale ---------------------------------------------
    controller.SetTool(gizmoUI.TOOL_ROTATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    rBefore = _Values(prim, gizmoMath.AVAR_R, frame)
    d.DragRing("z")
    rAfter = _Values(prim, gizmoMath.AVAR_R, frame)
    _Check(_Changed(rBefore, rAfter),
           "the ring drag changed a rotation avar: %s -> %s"
           % (rBefore, rAfter))
    controller.Undo()
    d.Pump()
    controller.SetTool(gizmoUI.TOOL_SCALE)
    d.Pump()
    sx = prim.GetAttribute("avars:sx")
    d.DragAxis("x")
    _Check(abs(sx.Get(frame) - 1.0) > 1e-6,
           "the scale drag changed avars:sx: %s" % sx.Get(frame))
    controller.Undo()
    d.Pump()

    # --- 5. Pivot mode edits rest:*, never avars -------------------------
    controller.SetChannels(gizmoMath.CHANNELS_PIVOT)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    _Check(controller.Target().kind == "rig-pivot",
           "pivot mode on a control resolves to a rig-pivot target, not %r"
           % controller.Target().kind)
    d.DragAxis("x")
    # rest:tx is unauthored on the rig, so the attribute only exists once
    # the pivot drag has written it -- fetched here rather than before.
    restTx = prim.GetAttribute("rest:tx")
    _Check(restTx and abs(restTx.Get(frame)) > 1e-6,
           "the pivot drag wrote rest:tx: %s"
           % (restTx.Get(frame) if restTx else None))
    _Check(abs(tx.Get(frame) - before) < 1e-9,
           "the pivot drag left the avars alone: avars:tx is %s"
           % tx.Get(frame))
    controller.SetTool(gizmoUI.TOOL_SCALE)
    d.Pump()
    _Check(controller.Target() is not None
           and "unavailable" in controller.Status(),
           "scale is refused in pivot mode (rest spaces are "
           "orthonormalized): %r" % controller.Status())
    _Check(controller.HandleScreenPositions() == {},
           "a refused tool draws no handles: %s"
           % sorted(controller.HandleScreenPositions()))
    controller.Undo()
    d.Pump()
    _Check(session.GetAttributeAtPath(Sdf.Path(CONTROL + ".rest:tx"))
           is None, "the undo removed the rest:tx spec the pivot drag "
           "created, the same way it does for an avar")
    controller.SetChannels(gizmoMath.CHANNELS_POSE)
    d.Pump()

    # --- 6. A plain Xform edits its xformOps ------------------------------
    d.Select(XFORM)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None
           and controller.Target().kind == "xform-pose", controller.Reason())
    xformOp = stage.GetAttributeAtPath(Sdf.Path(XFORM + ".xformOp:translate"))
    opBefore = xformOp.Get(frame) if xformOp else None
    d.DragAxis("y")
    opPath = Sdf.Path(XFORM + ".xformOp:translate")
    op = session.GetAttributeAtPath(opPath)
    _Check(op is not None,
           "the drag authored xformOp:translate in the session layer")
    # Read the VALUE through the stage, not spec.default: Animation mode
    # writes a double3 as a time sample, so the spec's default is None and
    # comparing it to the zero vector proves nothing.
    opAfter = stage.GetAttributeAtPath(opPath).Get(frame)
    _Check(opAfter is not None and Gf.Vec3d(opAfter).GetLength() > 1e-6
           and opAfter != opBefore,
           "the authored translate is non-zero and differs from before the "
           "drag: %s -> %s" % (opBefore, opAfter))
    controller.Undo()
    d.Pump()
    _Check(session.GetAttributeAtPath(
        Sdf.Path(XFORM + ".xformOp:translate")) is None,
        "the undo removed the xformOp spec the drag created")

    # --- 6b. manipulator parity: planar handle, middle-drag, step snap, view
    #         ring, gimbal, free rotate, scale ratio, hotkeys ------------
    prim = d.Select(CONTROL)
    controller.SetChannels(gizmoMath.CHANNELS_POSE)
    controller.SetWriteMode(gizmoMath.WRITE_ANIMATION)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    positions = controller.HandleScreenPositions()
    _Check({"xy", "yz", "xz", "center"} <= set(positions),
           "the move manipulator has the conventional three planar handles and a "
           "centre: %s" % sorted(positions))
    corners = positions["xy"]
    _Check(len(corners) == 4, "a planar handle reports its four corners")
    cx = sum(p[0] for p in corners) / 4.0
    cy = sum(p[1] for p in corners) / 4.0
    beforeT = _Values(prim, gizmoMath.AVAR_T, frame)
    d.Drag((cx, cy), (cx + 25, cy - 25))
    afterT = _Values(prim, gizmoMath.AVAR_T, frame)
    changed = _Changed(beforeT, afterT)
    _Check(changed and 2 not in changed,
           "the XY planar drag moved the control in X and Y only: %s -> %s"
           % (beforeT, afterT))
    _Check(controller.SelectedHandleName() == "xy",
           "the dragged handle is the selected (yellow) one: %r"
           % controller.SelectedHandleName())

    # the conventional middle-drag anywhere repeats the selected handle.
    mid = (positions["center"][0][0] + 150,
           positions["center"][0][1] + 120)
    _Check(d.HandleAt(mid) is None,
           "the middle-drag point is clear of every handle, so only the "
           "repeat rule can explain a drag starting there: %r"
           % d.HandleAt(mid))
    d.Drag(mid, (mid[0] + 25, mid[1] - 25),
           button=d.QtCore.Qt.MiddleButton)
    movedT = _Values(prim, gizmoMath.AVAR_T, frame)
    movedBy = _Changed(afterT, movedT)
    _Check(movedBy and 2 not in movedBy,
           "the middle drag repeated the XY handle away from it: %s -> %s"
           % (afterT, movedT))
    controller.Undo()
    controller.Undo()
    d.Pump()

    # Ctrl + an axis drag moves in the plane PERPENDICULAR to that axis
    # (the conventional tool). The modifier is applied to the MOVES, not the press: Qt
    # turns Ctrl+left-click into a right-button press on macOS, so "grab
    # the axis, then hold Ctrl" is the gesture that works everywhere.
    # Run twice on identical geometry, plain then Ctrl, so the assertion
    # is "Ctrl changed which channels moved" rather than the weaker
    # "something moved".
    a, b = d.AxisPoints("x")
    grab = _Lerp(a, b, 0.5)
    away = (grab[0] + 40, grab[1] - 30)
    beforeT = _Values(prim, gizmoMath.AVAR_T, frame)
    d.Drag(grab, away)
    plainMoved = _Changed(beforeT, _Values(prim, gizmoMath.AVAR_T, frame))
    _Check(plainMoved == [0],
           "without Ctrl the same drag moves along X only: %s" % plainMoved)
    controller.Undo()
    d.Pump()
    d.Press(grab)
    _Check(controller.IsDragging(), "the X arrow started a drag")
    for point in (_Lerp(grab, away, 0.5), away):
        d.Move(point, button=d.QtCore.Qt.NoButton,
               buttons=d.QtCore.Qt.LeftButton,
               modifiers=d.QtCore.Qt.ControlModifier)
    d.Release(away)
    ctrlMoved = _Changed(beforeT, _Values(prim, gizmoMath.AVAR_T, frame))
    _Check(ctrlMoved and 0 not in ctrlMoved,
           "Ctrl + the X arrow moves in the YZ plane instead: X is "
           "unchanged and Y or Z moved, got %s (%s -> %s)"
           % (ctrlMoved, beforeT, _Values(prim, gizmoMath.AVAR_T, frame)))
    controller.Undo()
    d.Pump()

    # Step snap quantises the written channel delta. The same drag geometry
    # is run twice -- once with snapping off to measure the raw delta, then
    # with it on -- and the step is deliberately 0.6 of that raw delta, so
    # `loose / step` is about 1.67 and can never be a whole number. That is
    # the whole point: with any step that divides the raw delta, "the
    # result is a multiple of the step" is also true of an unsnapped drag,
    # and the assertion would pass with the feature switched off.
    base = tx.Get(frame)
    d.DragAxis("x", 0.3)
    loose = tx.Get(frame) - base
    controller.Undo()
    d.Pump()
    _Check(abs(loose) > 1e-3,
           "the unsnapped reference drag moved far enough to snap: %s"
           % loose)
    step = abs(loose) * 0.6
    settings = controller.settings.For(gizmoUI.TOOL_TRANSLATE)
    settings.stepSize = step
    settings.stepSnap = True
    d.Pump()
    d.DragAxis("x", 0.3)
    snapped = tx.Get(frame) - base
    multiple = snapped / step
    _Check(abs(snapped) > 1e-9 and abs(multiple - round(multiple)) < 1e-6,
           "step snap wrote a whole multiple of the %.4f step: %.6f is "
           "%.4f steps" % (step, snapped, multiple))
    _Check(abs(snapped - loose) > 1e-3,
           "the snap actually moved the value: the same drag wrote %.6f "
           "unsnapped and %.6f snapped, which is what an ignored stepSnap "
           "would also produce" % (loose, snapped))
    settings.stepSnap = False
    controller.Undo()
    d.Pump()

    # Rotate: view ring (with the live angle readout), gimbal, free rotate.
    controller.SetTool(gizmoUI.TOOL_ROTATE)
    d.Pump()
    positions = controller.HandleScreenPositions()
    _Check({"x", "y", "z", "view", "free"} <= set(positions),
           "the rotate manipulator has the conventional three rings, the view ring "
           "and the free-rotate ball: %s" % sorted(positions))
    rBefore = _Values(prim, gizmoMath.AVAR_R, frame)
    ring = positions["view"]
    d.Press(ring[0])
    _Check(controller.IsDragging(), "the view ring started a drag")
    d.Move(ring[len(ring) // 8])
    d.Move(ring[len(ring) // 4])
    _Check(abs(controller.DragAngle()) > 1.0,
           "the live drag reports the swept angle: %s deg"
           % controller.DragAngle())
    _Check("deg" in controller.Status(),
           "the status shows the rotation amount while dragging: %r"
           % controller.Status())
    d.Release(ring[len(ring) // 4])
    _Check(_Changed(rBefore, _Values(prim, gizmoMath.AVAR_R, frame)),
           "the view ring rotated the control")
    controller.Undo()
    d.Pump()

    controller.settings.For(gizmoUI.TOOL_ROTATE).orientation = \
        gizmoSettings.ORIENT_GIMBAL
    d.Pump()
    rBefore = _Values(prim, gizmoMath.AVAR_R, frame)
    d.DragRing("z", 6)
    rAfter = _Values(prim, gizmoMath.AVAR_R, frame)
    _Check(_Changed(rBefore, rAfter) == [2],
           "the gimbal Z ring changes avars:rz alone: %s -> %s"
           % (rBefore, rAfter))
    controller.Undo()
    controller.settings.For(gizmoUI.TOOL_ROTATE).orientation = \
        gizmoSettings.ORIENT_OBJECT
    d.Pump()

    rBefore = _Values(prim, gizmoMath.AVAR_R, frame)
    freeStart = d.FreeRotatePoint()
    d.Drag(freeStart, (freeStart[0] + 40, freeStart[1] + 12))
    _Check(_Changed(rBefore, _Values(prim, gizmoMath.AVAR_R, frame)),
           "free rotate (the virtual trackball) rotated the control")
    controller.Undo()
    d.Pump()

    # Scale: the conventional ratio rule, then Prevent Negative Scale.
    controller.SetTool(gizmoUI.TOOL_SCALE)
    d.Pump()
    a, b = d.AxisPoints("x")
    d.Drag(_Lerp(a, b, 0.5), b)
    _Check(abs(sx.Get(frame) - 2.0) < 1e-3,
           "dragging the X handle from half length to the tip doubles "
           "avars:sx (the conventional distance-ratio rule): %s" % sx.Get(frame))
    controller.Undo()
    d.Pump()
    controller.settings.For(gizmoUI.TOOL_SCALE).preventNegativeScale = True
    d.Pump()
    a, b = d.AxisPoints("x")
    d.Drag(_Lerp(a, b, 0.5), _Lerp(a, b, -0.5))
    _Check(sx.Get(frame) > 0.0,
           "Prevent Negative Scale keeps avars:sx positive when the cursor "
           "is dragged through the origin: %s" % sx.Get(frame))
    controller.settings.For(gizmoUI.TOOL_SCALE).preventNegativeScale = False
    controller.Undo()
    d.Pump()

    # Hotkeys: Q/W/E/R tools, D toggles pivot, + / - resize.
    d.Key(d.QtCore.Qt.Key_W)
    _Check(controller.Tool() == gizmoUI.TOOL_TRANSLATE, "W selects Move")
    d.Key(d.QtCore.Qt.Key_E)
    _Check(controller.Tool() == gizmoUI.TOOL_ROTATE, "E selects Rotate")
    d.Key(d.QtCore.Qt.Key_R)
    _Check(controller.Tool() == gizmoUI.TOOL_SCALE, "R selects Scale")
    d.Key(d.QtCore.Qt.Key_D)
    _Check(controller.Channels() == gizmoMath.CHANNELS_PIVOT,
           "D toggles Edit Pivot on")
    d.Key(d.QtCore.Qt.Key_D)
    _Check(controller.Channels() == gizmoMath.CHANNELS_POSE,
           "D toggles Edit Pivot off")
    size = controller.settings.manipulatorSize
    d.Key(d.QtCore.Qt.Key_Plus)
    _Check(controller.settings.manipulatorSize > size,
           "+ grows the manipulator: %s" % controller.settings.manipulatorSize)
    d.Key(d.QtCore.Qt.Key_Minus)
    _Check(abs(controller.settings.manipulatorSize - size) < 1e-6,
           "- shrinks it back to %s: %s"
           % (size, controller.settings.manipulatorSize))
    d.Key(d.QtCore.Qt.Key_Q)
    _Check(controller.Tool() == gizmoUI.TOOL_SELECT, "Q selects the Select "
           "tool")

    # Escape aborts a live drag without pushing an edit. usdview binds
    # Escape itself, so the gizmo has to win it only while a drag is live;
    # what is asserted here is that behaviour, not how it is delivered.
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    a, b = d.AxisPoints("x")
    controller.undoStack.Clear()
    v0 = tx.Get(frame)
    d.Press(_Lerp(a, b, 0.5))
    d.Move(_Lerp(a, b, 0.85))
    # "Has moved" is a statement about the PREVIEW now, not about the stage: a
    # drag in progress authors nothing (see _TestPreviewThenCommit), so the
    # value the artist is looking at lives in the preview channel until they
    # let go. The stage value moving here would mean the preview had leaked.
    import gizmoMath
    _Check(controller.IsDragging(), "the drag is live")
    _Check(Sdf.Path(CONTROL + ".avars:tx") in gizmoMath.PreviewValues(),
           "and has moved, in the preview: %s"
           % sorted(str(p) for p in gizmoMath.PreviewValues()))
    _Check(abs(tx.Get(frame) - v0) < 1e-12,
           "with the stage still holding the committed value")
    d.Key(d.QtCore.Qt.Key_Escape)
    _Check(not controller.IsDragging(), "Escape ended the drag")
    _Check(abs(tx.Get(frame) - v0) < 1e-9,
           "Escape left avars:tx at %s: %s" % (v0, tx.Get(frame)))
    _Check(gizmoMath.PreviewValues() == {},
           "and stopped previewing")
    _Check(not controller.undoStack.CanUndo(),
           "an aborted drag pushed nothing onto the undo stack")

    # Redo aliases: Shift+Z (the conventional tool) and Ctrl+Y both redo.
    d.DragAxis("x")
    v1 = tx.Get(frame)
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ShiftModifier)
    _Check(abs(tx.Get(frame) - v1) < 1e-9,
           "Shift+Z redoes: %s" % tx.Get(frame))
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    d.Key(d.QtCore.Qt.Key_Y, d.QtCore.Qt.ControlModifier)
    _Check(abs(tx.Get(frame) - v1) < 1e-9,
           "Ctrl+Y redoes: %s" % tx.Get(frame))
    controller.Undo()
    d.Pump()
    _Check(abs(tx.Get(frame) - v0) < 1e-9,
           "the control is back where it started: %s" % tx.Get(frame))

    # Undo and redo must survive RigExec -> Viewport Tools being turned
    # off. The edits stay on the stack, so the shortcuts that reach them
    # cannot live only on the toolbar: a hidden widget's actions do not
    # fire, and the artist would be left holding un-undoable drags.
    d.DragAxis("x")
    vHidden = tx.Get(frame)
    _Check(abs(vHidden - v0) > 1e-6, "the drag moved the control")
    controller.SetVisible(False)
    d.Pump()
    _Check(not controller.toolbar.isVisible(),
           "the toolbar is hidden")
    _Check(controller.undoStack.CanUndo(),
           "the edit is still on the stack with the toolbar hidden")
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    _Check(abs(tx.Get(frame) - v0) < 1e-9,
           "Ctrl+Z still undoes with the viewport tools hidden: %s"
           % tx.Get(frame))
    d.Key(d.QtCore.Qt.Key_Z,
          d.QtCore.Qt.ControlModifier | d.QtCore.Qt.ShiftModifier)
    _Check(abs(tx.Get(frame) - vHidden) < 1e-9,
           "and Ctrl+Shift+Z still redoes: %s" % tx.Get(frame))
    controller.SetVisible(True)
    d.Pump()
    controller.Undo()
    d.Pump()
    _Check(abs(tx.Get(frame) - v0) < 1e-9,
           "back to the start again: %s" % tx.Get(frame))

    # Preserve Children holds a child xform's world transform still.
    child = UsdGeom.Xform.Define(stage, XFORM + "/GizmoTestChild")
    UsdGeom.XformCommonAPI(child).SetTranslate(Gf.Vec3d(1, 2, 3))
    childPath = child.GetPath()
    childOpPath = childPath.AppendProperty("xformOp:translate")
    d.Select(XFORM)
    controller.settings.For(gizmoUI.TOOL_TRANSLATE).preserveChildren = True
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None
           and controller.Target().supportsPreserveChildren,
           "Preserve Children is available on a plain xform")
    cache = UsdGeom.XformCache(frame)
    childWorld = cache.GetLocalToWorldTransform(child.GetPrim())
    d.DragAxis("y")
    parent = session.GetAttributeAtPath(
        Sdf.Path(XFORM + ".xformOp:translate"))
    _Check(parent is not None, "the parent xform did move")
    cache.Clear()
    childAfter = cache.GetLocalToWorldTransform(child.GetPrim())
    _Check(all(abs(childAfter[r][c] - childWorld[r][c]) < 1e-5
               for r in range(4) for c in range(4)),
           "Preserve Children kept the child's world transform: %s -> %s"
           % (childWorld.ExtractTranslation(),
              childAfter.ExtractTranslation()))
    controller.settings.For(gizmoUI.TOOL_TRANSLATE).preserveChildren = False
    controller.Undo()
    d.Pump()
    # One drag is one undo step, compensated children included: the parent's
    # op spec is gone and the child is back on the translate it was defined
    # with, not left holding the value the compensation wrote.
    _Check(session.GetAttributeAtPath(
        Sdf.Path(XFORM + ".xformOp:translate")) is None,
        "the undo removed the parent's xformOp spec")
    _Check(stage.GetAttributeAtPath(childOpPath).Get(frame)
           == Gf.Vec3d(1, 2, 3),
           "the same undo step reverted the child's compensation: %s"
           % stage.GetAttributeAtPath(childOpPath).Get(frame))
    # Removing the prim is plain cleanup. It used to raise "Applying
    # predicate to invalid prim" out of OpenExec's resync handler
    # (pxr/exec/esfUsd/stageData.cpp) whenever a compiled evaluator was
    # attached; tests/python/test_rigexec_stage_edits.py pins that fix, and
    # an exception here is a real regression, not something to guard.
    stage.RemovePrim(childPath)
    d.Pump()
    _Check(not stage.GetPrimAtPath(childPath),
           "the test's temporary child prim is gone from the stage")

    # --- 6c. Snapping: holds, sticky modes and landings ------------------
    # Spec section 6 items 12 and 13 first, then 8-11, 14 and 15: the
    # C/V hold assertions come before any other C or V in the file,
    # because one leaked keystroke flips usdview's auto-clipping or
    # opens its validation window for the rest of the run. Every mesh
    # and curve in this asset is rig-deformed (design section 3), so
    # the moved object and everything it lands on are scratch prims
    # authored into the SESSION layer and removed afterwards, the way
    # this file already defines and removes GizmoTestChild.
    import gizmoSnap
    probe = UsdGeom.Xform.Define(stage, "/Shot/SnapProbe")
    UsdGeom.XformCommonAPI(probe).SetTranslate(Gf.Vec3d(4, 10, 2))
    corners = [(0, 9.5, 3), (8, 9.5, 3), (8, 10.5, 3),
               (0, 10.5, 3)]
    snapTarget = UsdGeom.Mesh.Define(stage, "/Shot/SnapTarget")
    snapTarget.CreatePointsAttr([Gf.Vec3f(*p) for p in corners])
    snapTarget.CreateFaceVertexCountsAttr([4])
    snapTarget.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    snapTarget.CreateExtentAttr([Gf.Vec3f(*corners[0]),
                                 Gf.Vec3f(*corners[2])])
    snapTarget.CreateDoubleSidedAttr(True)
    # Defined now, pointed after the reframe below: the points must
    # lie on the reframed view rays (see there), so they are authored
    # once the camera they align to exists.
    snapDecoy = UsdGeom.Points.Define(stage, "/Shot/SnapDecoy")
    UsdGeom.Imageable(snapDecoy.GetPrim()).CreateVisibilityAttr(
        ).Set(UsdGeom.Tokens.invisible)
    d.Pump()

    # The RigExec guides draw in front of Geom from every practical
    # camera and win every view.pick (design section 3); the snap
    # pick already excludes them with showGuides = False, so hiding
    # them here is a debuggability measure only. Restored below.
    viewSettings = d.api.dataModel.viewSettings
    guideWas = viewSettings.displayGuide
    viewSettings.displayGuide = False
    d.Pump()

    # Re-frame on the scratch geometry: under the camera the earlier
    # sections leave, the target's x = 0 corners project outside
    # testusdview's viewport and computePickFrustum reports
    # inImageBounds = False, so the resolver answers None there.
    # frameSelection re-aims as well as dollies (freeCamera.center),
    # so the teardown restores that too -- the plan's triple alone
    # would leave the shot grab below framing the wrong place.
    camera = d.api.dataModel.viewSettings.freeCamera
    _Check(camera is not None, "the view is on the free camera")
    # _selSize rides along: _frameSelection sets it
    # (freeCamera.py:325) and it feeds the clipping computation
    # once auto-clip is on, so leaving it reframed is leaving
    # view state behind.
    savedCamera = (camera.rotTheta, camera.rotPhi, camera.dist,
                   Gf.Vec3d(camera.center), camera._selSize)
    d.Select("/Shot/SnapTarget")
    appController._frameSelection()
    d.Pump()
    d.Select("/Shot/SnapProbe")
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None
           and controller.Target().kind == "xform-pose",
           "the probe is a plain xform target: %s"
           % controller.Reason())

    # The decoy shares each corner's pixel. +Z is not the view
    # direction under this tumbled camera, so a +Z offset lands
    # ~8 px from its corner and can never win the pick; each
    # decoy point instead sits 0.5 along the real view ray
    # through its corner. The eye is read here -- authoring time,
    # after the reframe -- through the controller's own camera,
    # the one the live drag resolves against.
    viewCam, _, _ = controller._Camera()
    _Check(viewCam is not None,
           "the controller resolves a camera for the decoy ray")
    eye = viewCam.transform.ExtractTranslation()
    decoyPts = []
    for (x, y, z) in corners:
        along = (eye - Gf.Vec3d(x, y, z)).GetNormalized()
        onRay = Gf.Vec3d(x, y, z) + 0.5 * along
        decoyPts.append(Gf.Vec3f(onRay[0], onRay[1], onRay[2]))
    snapDecoy.CreatePointsAttr(decoyPts)
    d.Pump()

    # The landings aim at projected pixels, so the setup first proves
    # the corners are on screen and unambiguous -- the isolation
    # discipline testUsdviewCurvenetMove.py:84-97 follows. A collapsed
    # framing would stack every corner onto one pixel and "pass" on
    # whichever the ranker happened to keep.
    _, viewport, ratio = controller._Camera()
    left, top = viewport[0] / ratio, viewport[1] / ratio
    width, height = viewport[2] / ratio, viewport[3] / ratio
    pixels = [_ProjectLogical(controller, c) for c in corners]
    for corner, pixel in zip(corners, pixels):
        _Check(pixel is not None and left + 24 < pixel[0]
               < left + width - 24 and top + 24 < pixel[1]
               < top + height - 24,
               "corner %s is on screen with a margin, not %s "
               "(viewport %.0fx%.0f)" % (corner, pixel, width,
                                         height))
    for i in range(len(pixels)):
        for j in range(i + 1, len(pixels)):
            gap = math.hypot(pixels[i][0] - pixels[j][0],
                             pixels[i][1] - pixels[j][1])
            _Check(gap > gizmoSnap.SNAP_PIXELS,
                   "corners %s and %s are %.1f logical px apart, "
                   "clear of the %.0f px snap radius"
                   % (corners[i], corners[j], gap,
                      gizmoSnap.SNAP_PIXELS))

    # --- 12. The C/V holds never reach usdview -------------------------
    # _showUsdValidation never resets _usdValidationWidget to None
    # (appController.py:2721-2726) and a C that gets through flips
    # autoComputeClippingPlanes for every later projection, so both
    # halves are required: the usdview half alone passes if the key
    # never dispatched, the gizmo half alone passes if the gizmo
    # acted but let the shortcut fire too.
    clip = (appController._ui.actionAuto_Compute_Clipping_Planes
            .isChecked())
    _Check(appController._usdValidationWidget is None,
           "no validation window before the V hold")
    before = _World(stage, "/Shot/SnapProbe", frame)
    center = controller.HandleScreenPositions()["center"][0]
    away = (center[0] + 20, center[1] - 12)
    d.Press(center)
    _Check(controller.IsDragging(), "the centre grab started a drag")
    d.Move(away, button=d.QtCore.Qt.NoButton,
           buttons=d.QtCore.Qt.LeftButton)
    d.KeyDown(d.QtCore.Qt.Key_V)
    _Check(controller.SnapMode() == gizmoSettings.SNAP_POINT,
           "a held V arms point snap mid-drag: %s"
           % controller.SnapMode())
    _Check(appController._usdValidationWidget is None,
           "the held V never reached usdview's Show USD Validation")
    d.KeyUp(d.QtCore.Qt.Key_V)
    d.KeyDown(d.QtCore.Qt.Key_C)
    _Check(controller.SnapMode() == gizmoSettings.SNAP_EDGE,
           "a held C arms edge snap mid-drag: %s"
           % controller.SnapMode())
    _Check(appController._ui.actionAuto_Compute_Clipping_Planes
           .isChecked() == clip,
           "the held C never reached usdview's auto-clipping toggle")
    d.KeyUp(d.QtCore.Qt.Key_C)
    d.Release(away)
    _Check(not controller.IsDragging(), "the release ended the drag")
    _Check((_World(stage, "/Shot/SnapProbe", frame) - before)
           .GetLength() > 1e-6,
           "the hold drag moved the probe, so the assertions above "
           "ran against a live drag")
    controller.Undo()
    d.Pump()
    _Check((_World(stage, "/Shot/SnapProbe", frame) - before)
           .GetLength() < 1e-9,
           "the undo put the probe back where the holds found it")

    # --- 13. Hold precedence and release order --------------------------
    # Through KeyDown/KeyUp and SnapMode(), never Key(): Key() is a
    # click, so the hold would clear before the next line reads it.
    center = controller.HandleScreenPositions()["center"][0]
    away = (center[0] + 20, center[1] - 12)
    d.Press(center)
    _Check(controller.IsDragging(), "the centre grab started a drag")
    d.Move(away, button=d.QtCore.Qt.NoButton,
           buttons=d.QtCore.Qt.LeftButton)
    d.KeyDown(d.QtCore.Qt.Key_X)
    _Check(controller.SnapMode() == gizmoSettings.SNAP_GRID,
           "a held X arms grid snap mid-drag: %s"
           % controller.SnapMode())
    d.KeyDown(d.QtCore.Qt.Key_V)
    _Check(controller.SnapMode() == gizmoSettings.SNAP_POINT,
           "point outranks grid whatever the press order: %s"
           % controller.SnapMode())
    d.KeyUp(d.QtCore.Qt.Key_X)
    _Check(controller.SnapMode() == gizmoSettings.SNAP_POINT,
           "releasing the outranked hold keeps point armed: %s"
           % controller.SnapMode())
    d.KeyUp(d.QtCore.Qt.Key_V)
    _Check(controller.SnapMode() == gizmoSettings.SNAP_OFF,
           "releasing the winner with no sticky mode leaves the "
           "next drag unsnapped: %s" % controller.SnapMode())
    d.Release(away)
    _Check(not controller.IsDragging(), "the release ended the drag")
    controller.Undo()
    d.Pump()

    # --- 8. Point snap lands the pivot on the vertex --------------------
    # Through the sticky mode: items 12 and 13 already prove the
    # mid-drag holds arm, and the mode resolves the same way once in
    # force. The centre handle constrains nothing, so the pivot lands
    # on the candidate unchanged (design 4.2).
    moveSettings = controller.settings.For(gizmoUI.TOOL_TRANSLATE)
    moveSettings.snapMode = gizmoSettings.SNAP_POINT
    d.Pump()
    seenPrims = []
    center = controller.HandleScreenPositions()["center"][0]
    aim = _ProjectLogical(controller, (8, 9.5, 3))
    d.Press(center)
    _Check(controller.IsDragging(), "the centre grab started a drag")
    d.Move(aim, button=d.QtCore.Qt.NoButton,
           buttons=d.QtCore.Qt.LeftButton)
    candidate = controller.SnapCandidate()
    _Check(candidate is not None,
           "a point candidate resolves at the (8, 9.5, 3) pixel")
    seenPrims.append(str(candidate.primPath))
    landed = _Landed(controller, stage, "/Shot/SnapProbe", frame)
    _Check((landed - Gf.Vec3d(8, 9.5, 3)).GetLength() < 1e-4,
           "point snap put the pivot on the (8, 9.5, 3) corner: %s"
           % (landed,))

    # --- 14. The snap clause reaches the status line --------------------
    # Appended, never substituted: the branch's own words survive it.
    status = controller.StatusBar().FullText()
    _Check("Move SnapProbe" in status and "snap: Point" in status,
           "the status keeps its branch and gains the snap clause: "
           "%r" % status)
    d.Release(aim)
    _Check(not controller.IsDragging(), "the release ended the drag")
    controller.Undo()
    d.Pump()

    # --- 9. Edge snap lands on the edge, not its endpoints --------------
    # Aimed at the midpoint of the bottom edge, whose endpoints are a
    # whole edge away: a point snap would sit on one of them, and no
    # snap at all would leave the probe off the edge entirely.
    moveSettings.snapMode = gizmoSettings.SNAP_EDGE
    d.Pump()
    center = controller.HandleScreenPositions()["center"][0]
    aim = _ProjectLogical(controller, (4, 9.5, 3))
    d.Press(center)
    _Check(controller.IsDragging(), "the centre grab started a drag")
    d.Move(aim, button=d.QtCore.Qt.NoButton,
           buttons=d.QtCore.Qt.LeftButton)
    candidate = controller.SnapCandidate()
    _Check(candidate is not None,
           "an edge candidate resolves at the bottom-edge pixel")
    seenPrims.append(str(candidate.primPath))
    landed = _Landed(controller, stage, "/Shot/SnapProbe", frame)
    _Check(abs(landed[1] - 9.5) < 0.02
           and abs(landed[2] - 3) < 0.02
           and 0.0 < landed[0] < 8.0,
           "edge snap landed ON the bottom edge (y = 9.5, z = 3, "
           "0 < x < 8), not %s" % (landed,))
    ends = min((landed - Gf.Vec3d(0, 9.5, 3)).GetLength(),
               (landed - Gf.Vec3d(8, 9.5, 3)).GetLength())
    _Check(ends > 1.0,
           "the edge landing is mid-edge, %.2f from the nearer "
           "endpoint: %s" % (ends, landed))
    d.Release(aim)
    _Check(not controller.IsDragging(), "the release ended the drag")
    controller.Undo()
    d.Pump()

    # --- 10. Surface snap lands on the quad ------------------------------
    # Aimed at the quad centre, where no vertex or edge is near: only
    # the surface hit lands here.
    moveSettings.snapMode = gizmoSettings.SNAP_SURFACE
    d.Pump()
    center = controller.HandleScreenPositions()["center"][0]
    aim = _ProjectLogical(controller, (4, 10, 3))
    d.Press(center)
    _Check(controller.IsDragging(), "the centre grab started a drag")
    d.Move(aim, button=d.QtCore.Qt.NoButton,
           buttons=d.QtCore.Qt.LeftButton)
    candidate = controller.SnapCandidate()
    _Check(candidate is not None,
           "a surface candidate resolves at the quad-centre pixel")
    seenPrims.append(str(candidate.primPath))
    landed = _Landed(controller, stage, "/Shot/SnapProbe", frame)
    _Check(abs(landed[2] - 3) < 1e-3
           and -0.05 < landed[0] < 8.05
           and 9.45 < landed[1] < 10.55,
           "surface snap landed on the quad (|z - 3| < 1e-3 and "
           "inside its bounds), not %s" % (landed,))
    d.Release(aim)
    _Check(not controller.IsDragging(), "the release ended the drag")
    controller.Undo()
    d.Pump()

    # --- 11. Invisible prims are never candidates ------------------------
    # This pixel looks straight at both the mesh corner and the decoy
    # point 0.5 along the view ray through it (see the setup): what
    # the resolver reports tells which one the pick honours.
    moveSettings.snapMode = gizmoSettings.SNAP_POINT
    d.Pump()
    center = controller.HandleScreenPositions()["center"][0]
    aim = _ProjectLogical(controller, (0, 9.5, 3))
    d.Press(center)
    _Check(controller.IsDragging(), "the centre grab started a drag")
    d.Move(aim, button=d.QtCore.Qt.NoButton,
           buttons=d.QtCore.Qt.LeftButton)
    candidate = controller.SnapCandidate()
    _Check(candidate is not None,
           "a point candidate resolves at the (0, 9.5, 3) pixel")
    _Check(candidate.primPath == Sdf.Path("/Shot/SnapTarget"),
           "the candidate is the visible mesh, not %s"
           % candidate.primPath)
    _Check((candidate.point - Gf.Vec3d(0, 9.5, 3)).GetLength()
           < 1e-4,
           "the candidate is the mesh corner (0, 9.5, 3), not the "
           "decoy point on the same ray: %s" % (candidate.point,))
    seenPrims.append(str(candidate.primPath))
    landed = _Landed(controller, stage, "/Shot/SnapProbe", frame)
    _Check((landed - Gf.Vec3d(0, 9.5, 3)).GetLength() < 1e-4,
           "the pivot followed the candidate to (0, 9.5, 3): %s"
           % (landed,))
    d.Release(aim)
    _Check(not controller.IsDragging(), "the release ended the drag")
    controller.Undo()
    d.Pump()
    _Check(all(p != "/Shot/SnapDecoy" for p in seenPrims),
           "no snap in this section ever named the decoy: %s"
           % (seenPrims,))

    # --- 15. One snapped drag is one undo step ---------------------------
    # From a cleared stack, so what is left afterwards counts the
    # drag's edits exactly. The probe was placed by authoring, so its
    # pre-snap spec stays behind: what Ctrl+Z must restore is the
    # VALUE, the way section 6 checks the xformOp value above.
    controller.undoStack.Clear()
    d.Pump()
    before = _World(stage, "/Shot/SnapProbe", frame)
    moveSettings.snapMode = gizmoSettings.SNAP_POINT
    d.Pump()
    center = controller.HandleScreenPositions()["center"][0]
    aim = _ProjectLogical(controller, (8, 9.5, 3))
    d.Press(center)
    _Check(controller.IsDragging(), "the centre grab started a drag")
    d.Move(aim, button=d.QtCore.Qt.NoButton,
           buttons=d.QtCore.Qt.LeftButton)
    after = _Landed(controller, stage, "/Shot/SnapProbe", frame)
    _Check((after - before).GetLength() > 0.5,
           "the snapped drag moved the probe: %s -> %s"
           % (before, after))
    d.Release(aim)
    _Check(not controller.IsDragging(), "the release ended the drag")
    _Check(controller.undoStack.CanUndo(),
           "the released snap drag pushed an edit")
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    restored = _World(stage, "/Shot/SnapProbe", frame)
    _Check((restored - before).GetLength() < 1e-9,
           "Ctrl+Z restored the pre-snap probe position: %s -> %s"
           % (after, restored))
    _Check(not controller.undoStack.CanUndo(),
           "one snapped drag was one undo step: nothing left "
           "to undo")

    moveSettings.snapMode = gizmoSettings.SNAP_OFF
    d.Pump()
    viewSettings.displayGuide = guideWas
    camera.rotTheta, camera.rotPhi, camera.dist = savedCamera[:3]
    camera.center = savedCamera[3]
    camera._selSize = savedCamera[4]
    for path in ("/Shot/SnapDecoy", "/Shot/SnapTarget",
                 "/Shot/SnapProbe"):
        stage.RemovePrim(Sdf.Path(path))
    d.Pump()
    _Check(not stage.GetPrimAtPath("/Shot/SnapProbe"),
           "the snap scratch prims are gone from the stage")
    # Section 7 must start with a live target: with the probe gone
    # and nothing re-selected, its "Select draws no handles" holds
    # with nothing selected -- true under Move too -- instead of
    # proving Select suppresses the target's handles.
    d.Select(XFORM)

    # --- 7. The Select tool draws nothing and grabs nothing --------------
    controller.SetTool(gizmoUI.TOOL_SELECT)
    d.Pump()
    _Check(controller.HandleScreenPositions() == {},
           "the Select tool draws no handles: %s"
           % sorted(controller.HandleScreenPositions()))
    d.Select(CONTROL)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    a, b = d.AxisPoints("x")
    controller.SetTool(gizmoUI.TOOL_SELECT)
    d.Pump()
    d.Press(_Lerp(a, b, 0.5))
    _Check(not controller.IsDragging(),
           "a click where the arrow used to be is left to usdview's picker")
    d.Release(_Lerp(a, b, 0.5))

    # --- preview then commit ---------------------------------------------
    _TestPreviewThenCommit(d, controller, stage, session, frame)

    # --- several controls at once ----------------------------------------
    _TestGroupSelection(d, controller, stage, session, frame)

    shot = os.environ.get("RIGEXEC_GIZMO_SHOT")
    if shot:
        d.Select(CONTROL)
        controller.SetTool(gizmoUI.TOOL_TRANSLATE)
        d.Pump()
        d.view.window().grab().save(shot)

    print("RIGEXEC_GIZMO_OK translate/rotate/scale, undo/redo, default, "
          "pivot, xform, conventional parity, snapping, preview then commit, "
          "multi-selection groups")
