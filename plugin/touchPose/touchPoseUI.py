"""TouchPose: touch the character, select the control that moves it.

Hover the skin and the painted region under the cursor lights up; click
and that region's control becomes usdview's selection -- which is the
whole trick, and the same one the Control Picker uses: selecting through
`dataModel.selection` means the Avar Editor and the viewport gizmo follow
for free and this panel never has to know what an avar is.

THIS FILE IS UI GLUE. The mouse, the menu, the selection rules and the
panel live here; the data a test can assert lives in `touchPoseModel`; and
everything that costs anything runs in C++ (rigExecImaging, bound by
`touchPoseNative`):

  * THE PICK is a BVH over the POSED triangles -- the points RigExec
    publishes to Hydra, not the stage's rest `points` -- refit in parallel
    when a new pose is published. A cast is microseconds. It replaced a
    numpy cast over every triangle that cost 1.6 ms per hover sample and
    ignored the mesh's world transform.

  * THE HIGHLIGHT IS A SHADER, not geometry. A Hydra scene index adds two
    primvars to the touched mesh -- a per-face region id, published once,
    and a small per-region colour table -- and wraps the terminal of every
    material bound to it in a generated glslfx that mixes table[region]
    over the lit colour (libs/rigExecImaging/touchPoseHighlight.h). A hover
    is one constant-primvar upload. NOTHING is authored on the stage: no
    prim is created, deleted or made visible, the session layer is never
    touched, the rig never sees a change notice, and the highlight sits
    exactly on the deforming skin because it IS the skin.

    That removed, measured on the biped: a region crossing that took
    71-91 ms to reach the screen (a Hydra resync of the overlay rprim), a
    ~270 ms stage-edit tax on every selection change, a +22 ms re-author of
    the overlay's points on every pose change, a lift off the skin that
    z-fought or floated, and an overlay prim usdview could pick instead of
    the body.

HOVER USES `WA_Hover`, NOT `setMouseTracking`. Turning mouse tracking on
for usdview's stage view also turns on its own GPU `pickObject` per mouse
move. WA_Hover delivers button-less moves as HoverMove events, which
nothing else in usdview listens for.

TOUCHPOSE STANDS DOWN WHILE A GIZMO DRAG IS IN FLIGHT: no cast, no hover
highlight. The SELECTION highlight stays lit -- it is drawn by the body's
own shader, so it follows the pose being dragged with no work at all.

SELECTION SEMANTICS ARE THE CONTROL PICKER'S, to the letter. No modifier
REPLACES, Shift TOGGLES, Ctrl REMOVES. A drag marquees, catching every
region the band TOUCHES, and the three rules apply to it unchanged. Click
and marquee share one code path (`Pick`) with the mode decided in one place
(`ModeFor`).

WHY THE CLICK IS SWALLOWED. While TouchPose is on, a click anywhere on the
character belongs to TouchPose: on a region it selects that region's
control, and OFF every region it still selects nothing rather than letting
usdview pick the mesh. A click that misses the mesh ENTIRELY returns False
untouched, so the camera, the gizmo and usdview's own picking still work.

PAINTING is the same loop writing instead of reading. With paint on, a drag
gives the faces under the brush to the region selected in the panel, and
Shift-drag takes them away. It edits the MODEL, never the stage; Save
writes the touch layer beside the rig, never the rig itself.
"""
import os
import sys

from pxr import Gf, Sdf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

if __name__ != "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Per-session state (one controller and one panel per usdview window) lives
# in rigExecUsdview's registry, like gizmoUI's controllers: several usdview
# sessions can share this module in one process. rigExecUsdview's directory
# is on the module search path wherever TouchPose runs (it is a RigExec
# plugin: bin/_env.bat and the pipeline launchers put it there), which is
# also how the gizmo helpers below reach gizmoUI.
import sessionRegistry
import touchPoseModel

# Voice-to-select. OPTIONAL IN THE STRONGEST SENSE, and the dependency
# runs one way: TouchPose does not need voice, voice needs TouchPose.
# The module may be missing, the speech subpackage beside this checkout
# may be missing, the machine may have no speech engine at all -- and
# TouchPose opens, picks and paints exactly as it always did. Caught as
# `Exception` and not `ImportError` on purpose: a syntax error or a
# missing stdlib module in a file nobody here wrote must cost the
# checkbox, not the panel. The reason ends up on the disabled checkbox,
# where the animator is already looking, rather than in the terminal.
try:
    import touchPoseVoice
except Exception:                                   # pragma: no cover
    touchPoseVoice = None



# What the highlight OPENS at, and the slider moves it live from there.
# The authored file says 0.478 (`touchpose:alpha`), drawn over an
# UNSHADED viewport; this is the value asked for over the shaded body.
#
# It is the opening value only. `Load` used to assign it on every load,
# so switching layer -- or any reload at all -- threw the slider's value
# away and snapped back to the default.
HIGHLIGHT_OPACITY = 0.30



_LEGACY_OVERLAYS = ("/TouchPoseHighlight", "/TouchPoseSelected",
                    "/TouchPoseLead", "/TouchPoseRegions")

# A few pixels of slop before a press counts as a marquee. Without it
# every click is a one-pixel band and the click path never runs at all.
DRAG_SLOP = 3.0

class FaceTableEdit(object):
    """One paint stroke: the face -> region table before it and after.

    Shaped to sit on rigExecUndo's stack next to a gizmo drag and a
    channel scrub, which is why it carries a label: the toolbar's Undo
    tooltip reads it.
    """

    def __init__(self, controller, before, after, label="Paint regions"):
        self._controller = controller
        self._before = before
        self._after = after
        self.label = label

    def Undo(self):
        self._controller.ApplyFaceTable(self._before)

    def Redo(self):
        self._controller.ApplyFaceTable(self._after)


class UndoStack(object):
    """A bounded linear stack of strokes; a push drops the redo branch.

    The same shape as rigExecUndo.UndoStack, kept here rather than
    imported: TouchPose loads as its own usdview plugin and has to open
    on a session where the RigExec plugin is absent. Twenty lines is
    cheaper than a soft import that silently disables undo when the
    import misses.
    """

    LIMIT = 50

    def __init__(self):
        self._undo = []
        self._redo = []

    def Push(self, edit):
        self._undo.append(edit)
        del self._undo[:-self.LIMIT]
        self._redo = []

    def CanUndo(self):
        return bool(self._undo)

    def CanRedo(self):
        return bool(self._redo)

    def Undo(self):
        if not self._undo:
            return False
        edit = self._undo.pop()
        edit.Undo()
        self._redo.append(edit)
        return True

    def Redo(self):
        if not self._redo:
            return False
        edit = self._redo.pop()
        edit.Redo()
        self._undo.append(edit)
        return True

    def Clear(self):
        self._undo = []
        self._redo = []


# How often the gizmo is asked whether a drag is in flight.
DRAG_POLL_MS = 33
# How long the timeline has to sit still after a frame change (playback or
# a scrub) before TouchPose lights up again.
TIME_QUIET_MS = 250

# The floor between two hover evaluations, in milliseconds: one 60 Hz
# frame, the fastest the highlight can be seen to change.
HOVER_MIN_MS = 16

# The three selection modes, shared verbatim with the Control Picker.
MODE_REPLACE = "replace"
MODE_TOGGLE = "toggle"
MODE_REMOVE = "remove"


def RegionMatches(region, query):
    """True when *region* should survive the panel's filter.

    Every space-separated word has to appear somewhere in the region's
    name or in the path of the control it selects, case-insensitively and
    in any order -- so "l lid" finds the left eyelids whichever way round
    the rig spells them, and a bare "jaw" finds the jaw whether the word
    is in the region's name or only in its control's.

    An empty or whitespace query keeps everything, which is what makes
    clearing the box the way back.

    Pure, and deliberately: the panel's list is rebuilt from this on every
    keystroke, and a rule about which rows to show is a rule worth
    testing without a Qt event loop.
    """
    words = (query or "").split()
    if not words:
        return True
    hay = "%s %s" % (getattr(region, "label", "") or "",
                     getattr(region, "control", "") or "")
    hay = hay.lower()
    return all(word.lower() in hay for word in words)


def StageView(usdviewApi):
    """usdview's stage view widget, or None headless."""
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


class _NullContext(object):
    """Stand-in for `selection.batchPrimChanges` on a usdview without it."""

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def GizmoDragging(usdviewApi=None):
    """True while `usdviewApi`'s session's viewport gizmo has a drag in
    flight (without an api: the current session's, see gizmoUI.GetController).

    Read-only through `gizmoUI`'s public surface, and any failure answers
    False so a session without the viewport tools behaves as it always did.
    """
    try:
        import gizmoUI
        controller = gizmoUI.GetController(usdviewApi)
        return controller is not None and controller.IsDragging()
    except Exception:
        return False


def GizmoOwns(x, y, ratio=1.0, usdviewApi=None):
    """True when `usdviewApi`'s session's viewport gizmo (without an api:
    the current session's) would take a press at this pixel.

    READ-ONLY, and through gizmoUI's public surface only. The conditions
    mirror `GizmoController._OnPress`. TouchPose DEFERS rather than
    competing: the gizmo's handles are small deliberate targets drawn on
    top, and a region is the whole limb behind them. Any failure answers
    False, which is the safe direction.
    """
    try:
        import gizmoUI
        import gizmoScreen
    except ImportError:
        return False
    try:
        controller = gizmoUI.GetController(usdviewApi)
        if controller is None or not controller.IsVisible():
            return False
        if controller.IsDragging():
            return True
        if controller.Tool() == gizmoUI.TOOL_SELECT:
            return False
        handles = controller.Handles()
        if not handles:
            return False
        return gizmoScreen.HitTest(handles, x, y,
                                   gizmoUI.HIT_PIXELS * ratio) is not None
    except Exception:
        return False


class Highlight(object):
    """What is lit, and nothing about how. Storm draws it.

    Four states, composited in C++ from lowest to highest: paint mode's
    every-region edit colours, the selected regions, the lead (the region
    clicked last), and the hover -- so the region under the cursor always
    shows what a click would do.

    Every setter ends in `Flush`, which hands the whole state to the native
    mesh in one call. The native side composes the per-region colour table
    and compares it with what is already drawn, so a state that did not
    change -- a mouse move that stays inside one region -- sends Hydra
    nothing at all.
    """

    def __init__(self, model, opacity=HIGHLIGHT_OPACITY):
        self._model = model
        self._native = model.native
        self.hover = None           # region index
        self.lead = None            # region index
        self.selected = ()          # region indices, lead excluded
        self.editing = False
        self.suspended = False
        self.opacity = opacity
        self.enabled = False
        self.flushes = 0

    # -- lifetime --------------------------------------------------------

    def Enable(self, on):
        """Attach or detach the shader highlight. The only costly step:
        attaching makes Storm compile the wrapped material once."""
        on = bool(on)
        if on == self.enabled:
            return False
        self.enabled = on
        self._native.SetHighlightEnabled(on)
        if on:
            self.Flush(force=True)
        return True

    # -- state -----------------------------------------------------------

    @property
    def key(self):
        """The hover region, or None -- what the hover state is keyed on."""
        return None if self.suspended else self.hover

    def SetHover(self, region):
        index = None if region is None else int(region.index)
        if index == self.hover:
            return False
        self.hover = index
        return self.Flush()

    def Clear(self):
        """Drop the hover."""
        return self.SetHover(None)

    def SetSelection(self, lead, others):
        lead = None if lead is None else int(lead.index)
        others = tuple(sorted(int(r.index) for r in others))
        if lead == self.lead and others == self.selected:
            return False
        self.lead = lead
        self.selected = others
        return self.Flush()

    def SetEditing(self, editing):
        editing = bool(editing)
        if editing == self.editing:
            return False
        self.editing = editing
        return self.Flush()

    def SetOpacity(self, opacity):
        opacity = max(0.0, min(1.0, float(opacity)))
        if opacity == self.opacity:
            return False
        self.opacity = opacity
        return self.Flush()

    def Suspend(self):
        """Stand every highlight down while the rig is being moved: a gizmo
        drag, playback, a scrub. The state is kept, only not drawn."""
        if self.suspended:
            return False
        self.suspended = True
        return self.Flush()

    def Resume(self):
        if not self.suspended:
            return False
        self.suspended = False
        return self.Flush()

    def Flush(self, force=False):
        """Hand the state to the native table. True when Hydra was told."""
        if not self.enabled:
            return False
        colors = self._model.StateColors()
        if self.suspended:
            # Nothing lit while the rig moves: the hover, the lead and the
            # selection all go dark and come back as they were.
            changed = self._native.SetHighlightState(
                None, None, (), self.editing, self.opacity, colors["lead"],
                colors["selected"])
        else:
            changed = self._native.SetHighlightState(
                self.hover, self.lead, self.selected, self.editing,
                self.opacity, colors["lead"], colors["selected"])
        if changed:
            self.flushes += 1
        return changed


class TouchPoseController(QtCore.QObject,
                          metaclass=sessionRegistry.PerSessionInstanceMeta(
                              QtCore.QObject)):
    """The mode: an event filter over the stage view, plus the highlight.

    One per usdview session, filed under the session's main window.
    `TouchPoseController._instance` reads the current session's controller
    (see sessionRegistry.PerSessionInstanceMeta).
    """

    _sessions = sessionRegistry.SessionRegistry("touchPose controllers")

    statusChanged = QtCore.Signal(str)
    strokeFinished = QtCore.Signal()

    @classmethod
    def GetInstance(cls, usdviewApi):
        """This session's controller, created once.

        A different api object of the SAME session (a script's own
        UsdviewApi on the same window) still replaces the controller, as
        it always did; another session's controller is never touched.
        """
        controller = cls._sessions.Get(usdviewApi)
        if controller is None or controller._api is not usdviewApi:
            if controller is not None:
                controller.SetActive(False)
            controller = cls._sessions.Set(usdviewApi, cls(usdviewApi))
        return controller

    @classmethod
    def ForApi(cls, usdviewApi):
        """This session's controller if one was built, else None."""
        return cls._sessions.Get(usdviewApi)

    # `mesh_path` PINS the controller to one mesh and is None by default,
    # which means "every mesh the stage carries regions for". It is only
    # ever read, never written back: writing the resolved mesh back onto
    # it is what used to pin the whole panel to body_geo after the first
    # load, and with it the layer switch -- measured on Biped_stack.usda,
    # the switch listed one entry ("Body") and hid itself, so the two eye
    # sets could not be reached from the panel at all.
    def __init__(self, usdviewApi, mesh_path=None):
        super(TouchPoseController, self).__init__()
        self._api = usdviewApi
        self._requestedMeshPath = mesh_path
        self._mesh_path = mesh_path
        self._view = StageView(usdviewApi)
        self._active = False
        self._installed = False
        # EVERY region set on the stage, all live at once, and the ONE
        # that is active -- listed in the panel, painted into and saved.
        #
        # WHY ALL OF THEM. A set states the mesh its faces index, so the
        # body set indexes body_geo and the two eye sets index l_eye_geo
        # and r_eye_geo. With one model open, the hover cast only ever
        # reached the live set's mesh: measured at the pixel over the
        # front of the left eyeball, the eyeball is hit at t = 332.45 and
        # body_geo at t = 337.57 -- the eye is 5.1 units IN FRONT of the
        # skin there -- and the hover still answered the body's face 489,
        # which is unpainted, so hovering an eye lit nothing. The sets do
        # not have to agree about anything to coexist: each owns its own
        # mesh, its own BVH and its own highlight, and the pick simply
        # takes the nearest hit across all of them.
        self._models = []
        self._highlights = []       # parallel to _models
        self._model = None
        # The layer the panel is standing on, remembered across reloads so
        # a stage edit or a save does not snap the switch back to the
        # rig's first layer. Empty until the first successful load.
        self._layer_name = None
        self._highlight = None
        self._opacity = HIGHLIGHT_OPACITY
        self._error = None
        self._hover = None
        self._lead = None
        self._consumedPress = False
        self._guarding = False
        self._selectionSignal = None
        self._paint = False
        self._paintTarget = None
        self._painting = None       # "add", "erase", or None
        self._brushRadius = None    # None = the model's own default
        self.stroke_faces = 0
        # The face -> region table as the current stroke found it, and
        # the fallback stack. See _EndStroke and _UndoStack.
        self._strokeBefore = None
        self._localUndo = UndoStack()
        self._pressAt = None
        self._pressMode = MODE_REPLACE
        self._suspended = False
        self._band = None
        self._rubber = None
        # POLLED, not event-driven. The gizmo exposes no drag signal, and
        # while a button is held Qt sends MouseMove, not HoverMove -- and
        # the gizmo's own filter may consume those first. A boolean read on
        # a timer does not care who wins the event.
        self._dragPoll = QtCore.QTimer(self)
        self._dragPoll.setInterval(DRAG_POLL_MS)
        self._dragPoll.timeout.connect(self._PollDrag)
        # Playback and scrubbing: every frame change marks the timeline busy
        # and restarts a quiet timer; TouchPose stays dark until it fires.
        self._timeBusy = False
        self._timeQuiet = QtCore.QTimer(self)
        self._timeQuiet.setSingleShot(True)
        self._timeQuiet.setInterval(TIME_QUIET_MS)
        self._timeQuiet.timeout.connect(self._OnTimeQuiet)
        self._frameSignal = None
        # See `_HoverSoon`: the mouse reports far faster than the
        # highlight can usefully change.
        self._lastHover = 0
        self._pendingHover = None
        self._hoverTimer = QtCore.QTimer(self)
        self._hoverTimer.setSingleShot(True)
        self._hoverTimer.timeout.connect(self._FlushHover)

    # -- state -----------------------------------------------------------

    @property
    def active(self):
        return self._active

    @property
    def model(self):
        """The ACTIVE set: the one the panel lists, paints and saves."""
        return self._model

    @property
    def models(self):
        """Every live set, the active one included, in switch order."""
        return list(self._models)

    @property
    def layers(self):
        """(label, scope path) per live set -- what the switch offers.

        Taken from the models rather than re-walked off the stage, so
        the switch can only ever offer a set that is actually open. It
        used to come from `model.layers`, which `TouchModel` fills with
        the layers on ITS OWN mesh: on Biped_stack.usda that is the one
        body set, so the switch had a single entry, hid itself, and the
        two eye sets were unreachable.
        """
        return [(m.layer_name, m.scope_path) for m in self._models]

    @property
    def highlight(self):
        return self._highlight

    def _Stage(self):
        return getattr(self._api, "stage", None)

    def _Time(self):
        """The frame the viewport draws, for the pose sync."""
        try:
            return self._api.frame
        except Exception:
            return None

    def Load(self, layer=None):
        """Open every touch layer on the stage. Returns a status line.

        `layer` names the one that becomes ACTIVE -- listed in the panel,
        painted into and saved. Unset keeps the one already active, and
        failing that takes the rig's first. Keeping it is what makes a
        plain Reload -- a stage edit, a save -- leave the switch where
        the user put it instead of snapping back to the default.

        Every OTHER layer is opened too and is just as live for hover,
        click and marquee. See `__init__`: a layer is one mesh, so an
        inactive layer is the only way the eyes can be touched while the
        body is being worked on.

        Reloading while the mode is ON re-attaches the highlights to the
        new models; the old ones released their native meshes, and with
        them their highlights, first. (This used to build a fresh overlay
        canvas beside the old one and leave the old one lit.)
        """
        stage = self._Stage()
        if stage is None:
            self._error = "no stage"
            return self._error
        wasActive = self._active
        self._ReleaseModel()
        try:
            if layer is None:
                layer = self._layer_name
            self._models, index = self._OpenLayers(stage, layer)
            self._model = self._models[index]
            self._layer_name = self._model.layer_name
        except Exception as exc:
            self._models = []
            self._model = None
            self._error = str(exc)
            return ("No touch regions on this stage (%s). Import them with "
                    "`bin\\run_touchpose.bat import_touch <rig>.usda` and "
                    "open <rig>_touch.usda." % exc)
        # THE OPACITY SURVIVES A LOAD. It used to be put back to
        # HIGHLIGHT_OPACITY here, which is why changing layer snapped the
        # highlight back to 85%: measured, a session set to 0.300 came
        # back from the next Load at 0.850 while the slider still read
        # 30%, so the panel and what was drawn disagreed as well. The
        # slider is the only thing that sets it, so the value it is at IS
        # the answer; `__init__` supplies the default for the first load.
        self._highlights = [Highlight(m, self._opacity) for m in self._models]
        self._highlight = self._highlights[index]
        time = self._Time()
        for model in self._models:
            model.SyncPose(time, force=True)
        if wasActive:
            for highlight in self._highlights:
                highlight.Enable(True)
                highlight.SetEditing(self._paint)
            self.SyncSelection()
        regions, covered, faces = self._model.Coverage()
        self._error = None
        return ("%d regions cover %d of %d faces (%.0f%%)"
                % (regions, covered, faces, 100.0 * covered / max(faces, 1)))

    def _OpenLayers(self, stage, layer):
        """Open a model per region set. Returns (models, active index).

        The sets come from `FindLayers`, filtered by `self._mesh_path`
        when the controller was pinned to one mesh -- which is how a shot
        holding two characters keeps their sets apart. Each model is
        opened BY SCOPE PATH, so the mesh it reads is the one its own set
        names rather than whatever the first set happened to name.

        A stage with no typed scope at all (the legacy GeomSubset layout)
        yields no rows; one model is opened the old way so those files
        keep working.

        A set that will not open is SKIPPED rather than allowed to fail
        the load: a scope whose mesh relationship is broken used to be
        one set out of one, and is now one out of several, so letting it
        throw would take the body down with it. If none of them opens,
        the first failure is re-raised and the panel says so as before.
        """
        rows = touchPoseModel.FindLayers(stage, self._mesh_path)
        if not rows:
            model = touchPoseModel.TouchModel.FromStage(
                stage, self._mesh_path, layer=layer)
            return [model], 0
        want = None if layer is None else str(layer)
        models, kept, failure = [], [], None
        for label, path, scope in rows:
            try:
                # The mesh is taken off the scope ROW rather than left to
                # `FromStage` to re-derive: deriving it walks the whole
                # stage again per set, measured at 72.3 ms for the
                # biped's three against 49.9 ms when each scope is
                # simply asked.
                models.append(touchPoseModel.TouchModel.FromStage(
                    stage,
                    self._mesh_path or touchPoseModel.MeshOfScope(scope),
                    layer=path))
            except Exception as exc:
                failure = failure if failure is not None else exc
                continue
            kept.append((label, path))
        if not models:
            raise failure if failure is not None else ValueError(
                "no touch region set on this stage could be opened")
        index = next((i for i, (label, path) in enumerate(kept)
                      if want in (label, path)), 0)
        return models, index

    def _ReleaseModel(self):
        for highlight in self._highlights:
            highlight.Enable(False)
        for model in self._models:
            model.Close()
        self._highlights = []
        self._models = []
        self._highlight = None
        self._model = None
        self._hover = None
        self._lead = None
        self._paintTarget = None

    def SetActive(self, active):
        active = bool(active)
        if active == self._active:
            return self._active
        if active:
            if self._model is None:
                self.Load()
            if self._model is None:
                return False
            self._Install()
            self._Subscribe()
            # The mesh may already be selected from before the mode was
            # switched on; the rule is about the selection, so enforce it
            # the moment the mode takes over.
            self._active = True
            self._GuardMesh()
            # The one step that compiles a shader. Everything after this is
            # a colour-table upload. Once per live set -- Storm compiles
            # the wrapped material per material, and the eyes' is not the
            # body's.
            for highlight in self._highlights:
                highlight.Enable(True)
                highlight.SetEditing(self._paint)
            self.SyncSelection()
            self._dragPoll.start()
            self._ConnectFrames(True)
        else:
            self._ConnectFrames(False)
            self._timeQuiet.stop()
            self._timeBusy = False
            self._dragPoll.stop()
            self._hoverTimer.stop()
            self._pendingHover = None
            self._Uninstall()
            self._Unsubscribe()
            self._pressAt = None
            self._band = None
            self._suspended = False
            self._HideBand()
            for highlight in self._highlights:
                highlight.Clear()
                highlight.Enable(False)
            self._hover = None
            self._lead = None
        self._active = active
        self._Redraw()
        return self._active

    def _Redraw(self):
        """Ask the viewport for a frame. The highlight is Hydra state, not a
        stage edit, so usdview has no change notice to redraw on."""
        view = self._view
        if view is not None:
            try:
                view.update()
            except Exception:
                pass

    # -- what the highlight looks like -----------------------------------

    @property
    def opacity(self):
        return self._opacity

    def SetOpacity(self, opacity):
        """How much of the body shows through the highlight. Live.

        One slider for every live set: a body patch and an eye patch at
        different opacities would read as two different things, and the
        slider does not say which set it means.
        """
        self._opacity = max(0.0, min(1.0, float(opacity)))
        changed = False
        for highlight in self._highlights:
            changed = highlight.SetOpacity(self._opacity) or changed
        if changed:
            self._Redraw()
        # Written back onto the MODEL so Save persists it as
        # `touchpose:alpha`.
        for model in self._models:
            model.alpha = self._opacity
        return self._opacity

    def SetStateColor(self, state, colour):
        """Change the lead or selected colour and repaint at once.

        On every live set, for the same reason the opacity is: lead and
        selected mean the same thing whichever set the region came from.
        """
        if self._model is None:
            return None
        colour = tuple(float(c) for c in colour)
        changed = False
        for model, highlight in zip(self._models, self._highlights):
            if state == "lead":
                model.lead_color = colour
            else:
                model.selected_color = colour
            changed = highlight.Flush() or changed
        if changed:
            self._Redraw()
        return colour

    # -- selection -------------------------------------------------------

    def _Selection(self):
        model = getattr(self._api, "dataModel", None)
        return getattr(model, "selection", None) if model else None

    def _Subscribe(self):
        selection = self._Selection()
        signal = getattr(selection, "signalPrimSelectionChanged", None)
        if signal is None or self._selectionSignal is not None:
            return
        signal.connect(self._OnSelectionChanged)
        self._selectionSignal = signal

    def _Unsubscribe(self):
        if self._selectionSignal is None:
            return
        try:
            self._selectionSignal.disconnect(self._OnSelectionChanged)
        except (RuntimeError, TypeError):
            pass       # already gone with the stage; nothing to undo
        self._selectionSignal = None

    def _OnSelectionChanged(self, *args):
        if not self._active or self._guarding:
            return
        self._GuardMesh()
        self.SyncSelection()

    def _GuardMesh(self):
        """Keep the mesh out of the selection while the mode is on.

        Consuming the press covers the case TouchPose sees; this covers
        every case it does not -- the outliner, a picker button, a press
        this filter declined. The rule is about the SELECTION, so it is
        enforced on the selection. The legacy overlay prims are swept too,
        in case a session saved by the old version left one behind.

        EVERY live set's mesh, not just the active one's: the eyeballs
        are picked by TouchPose now, so they are no more selectable than
        the body while the mode is on.
        """
        selection = self._Selection()
        stage = self._Stage()
        if selection is None or stage is None:
            return False
        paths = [m.mesh_path for m in self._models] + list(_LEGACY_OVERLAYS)
        held = set(str(p.GetPath()) for p in selection.getPrims() if p)
        unwanted = []
        for path in paths:
            if path not in held:
                continue
            prim = stage.GetPrimAtPath(Sdf.Path(path))
            if prim and prim.IsValid():
                unwanted.append(prim)
        if not unwanted:
            return False
        self._guarding = True
        try:
            for prim in unwanted:
                selection.removePrim(prim)
        except Exception:
            return False
        finally:
            self._guarding = False
        return True

    def SyncSelection(self):
        """Relight the lead and selected regions from usdview's selection.

        Driven off the SELECTION rather than off our own click, so a
        control chosen in the Control Picker, the outliner or the Avar
        Editor lights its region here too.
        """
        if self._model is None or self._highlight is None:
            return False
        selection = self._Selection()
        paths = [str(p.GetPath()) for p in selection.getPrims()
                 if p and p.IsValid()] if selection is not None else []
        # usdview keeps the selection in the order it was made, so the LAST
        # entry is the most recent -- which is the lead. Not `getFocusPrim`,
        # which returns the FIRST. A lead set by our own click wins while it
        # is still selected.
        lead = self._lead if (self._lead and self._lead.control in paths) \
            else None
        if lead is None and paths:
            lead = next(iter(self._RegionsFor([paths[-1]])), None)
        self._lead = lead
        # Per SET, because a region index means nothing outside the set
        # it came from. There is at most one lead across all of them --
        # the others get it as a plain selection, which is what makes a
        # control selected in two sets light green in one and neutral in
        # the other rather than green in both.
        changed = False
        for model, highlight in zip(self._models, self._highlights):
            chosen = model.RegionsFor(paths)
            mine = lead if any(r is lead for r in chosen) else None
            others = [r for r in chosen
                      if mine is None or r.index != mine.index]
            changed = highlight.SetSelection(mine, others) or changed
        if changed:
            self._Redraw()
        return changed

    def _RegionsFor(self, paths):
        """Every region on every live set whose control is in `paths`."""
        found = []
        for model in self._models:
            found.extend(model.RegionsFor(paths))
        return found

    def _ClearHover(self):
        """Drop the hover on every live set. True when one was lit."""
        changed = False
        for highlight in self._highlights:
            changed = highlight.Clear() or changed
        return changed

    # -- installation ----------------------------------------------------

    def _Install(self):
        view = self._view
        if view is None or self._installed:
            return
        view.installEventFilter(self)
        # See the module docstring: WA_Hover, never setMouseTracking.
        view.setAttribute(QtCore.Qt.WA_Hover, True)
        self._installed = True

    def _Uninstall(self):
        view = self._view
        if view is None or not self._installed:
            return
        view.removeEventFilter(self)
        # WA_Hover is deliberately LEFT ON. gizmoUI sets it too and may
        # still be running; clearing it here would silently kill the
        # manipulator's pre-selection highlight. It costs nothing on its
        # own -- the events are only delivered, not acted on.
        self._installed = False

    # -- the ray ---------------------------------------------------------

    def _Ratio(self):
        try:
            return float(self._view.devicePixelRatioF())
        except AttributeError:
            return 1.0

    def _Position(self, event):
        """Cursor in PHYSICAL pixels, which is what the viewport is in.

        Qt reports widget-local LOGICAL pixels; `computeWindowViewport` is
        physical. Skipping the ratio is invisible at 1.0 and puts every
        pick at half the cursor's position on a HiDPI display -- the same
        bridge `gizmoUI._Position` makes.
        """
        try:
            position = event.position()
            x, y = position.x(), position.y()
        except AttributeError:
            x, y = event.x(), event.y()
        ratio = self._Ratio()
        return x * ratio, y * ratio

    def RayAt(self, x, y):
        """World-space (origin, direction) through a physical pixel."""
        view = self._view
        if view is None:
            return None
        try:
            viewport = view.computeWindowViewport()
            width, height = int(viewport[2]), int(viewport[3])
            frustum = view.resolveCamera()[0].frustum
        except Exception:
            return None
        return touchPoseModel.RayThroughPixel(frustum, x, y, width, height)

    def RegionAt(self, x, y):
        """(region, face) under a physical pixel; face < 0 means no mesh."""
        _model, region, face = self.CastAt(x, y)
        return region, face

    def CastAt(self, x, y):
        """(model, region, face) under a physical pixel. Nearest wins.

        Every live set is cast and the SMALLEST t is taken, which is the
        surface actually in front of the cursor. Measured on the biped
        at the pixel over the front of the left eyeball: l_eye_geo at
        t = 332.45, body_geo at t = 337.57, so the eye wins by 5.1 units
        and the hover lands on L_Eye instead of the unpainted body face
        489 that used to answer there.

        Three casts rather than one, and the two extra meshes are 384
        faces each against body_geo's 26,274; a cast is a BVH descent, so
        the cost is in the log of the face count, not the count.
        """
        if not self._models:
            return None, None, -1
        ray = self.RayAt(x, y)
        if ray is None:
            return None, None, -1
        best, bestFace, bestT = None, -1, None
        for model in self._models:
            face, t = model.Cast(ray[0], ray[1])
            if face < 0:
                continue
            if bestT is None or t < bestT:
                best, bestFace, bestT = model, face, t
        if best is None:
            return None, None, -1
        return best, best.RegionOfFace(bestFace), bestFace

    # -- painting --------------------------------------------------------

    @property
    def painting(self):
        return self._paint

    def SetPainting(self, on):
        """Paint mode on or off. On, every region is lit in its own edit
        colour -- you cannot paint a boundary you cannot see -- and the
        hovered one in the same set, so it reads as the same thing lit."""
        self._paint = bool(on)
        if not self._paint:
            self._painting = None
        if self._active:
            changed = False
            for highlight in self._highlights:
                changed = highlight.SetEditing(self._paint) or changed
            if changed:
                self._Redraw()
        return self._paint

    def HoverColorFor(self, region):
        """The hover colour for the mode TouchPose is in.

        Asked of the SET the region belongs to: the palette is per set,
        so reading the active one's for an eye region would answer with
        the body's colours.
        """
        model = self._ModelOf(region)
        if model is None:
            return None
        return (model.EditColor(region) if self._paint
                else model.HoverColor(region))

    def _ModelOf(self, region):
        """Which live set a region came from, by identity."""
        if region is None:
            return None
        for model in self._models:
            if any(r is region for r in model.regions):
                return model
        return None

    @property
    def brush(self):
        if self._brushRadius is not None:
            return self._brushRadius
        return self._model.brush if self._model is not None else 0.0

    def SetBrush(self, radius):
        self._brushRadius = max(float(radius), 1e-4)
        return self._brushRadius

    @property
    def paintTarget(self):
        return self._paintTarget

    # -- undo ----------------------------------------------------------
    #
    # TouchPose paints into the MODEL, not the stage: a stroke moves faces
    # between regions in a numpy table and the stage only hears about it
    # on Save. So rigExecUndo, which snapshots Sdf specs, has nothing to
    # snapshot here and this keeps a stack of its own. What it shares with
    # rigExecUndo is the shape: one entry per gesture, a bounded stack, and
    # a push that discards the redo branch.

    def _UndoStack(self):
        """The stack a stroke goes onto: the shared one when it is there.

        The RigExec plugin's gizmo toolbar owns an application-wide
        Ctrl+Z bound to rigExecUndo's stack, so a stroke kept on a stack
        of our own would be unreachable by the only key an artist will
        press -- and worse, Ctrl+Z would silently undo some earlier gizmo
        drag instead of the stroke just painted. FaceTableEdit is shaped
        to sit on that stack, so it goes there when it exists.

        Read through gizmoUI's public surface, the same soft import
        GizmoDragging already uses: TouchPose is its own usdview plugin
        and has to open on a session without RigExec, which is what the
        local stack is for.
        """
        try:
            import gizmoUI
            controller = gizmoUI.GetController()
            stack = getattr(controller, "undoStack", None)
            if stack is not None:
                return stack
        except Exception:
            pass
        return self._localUndo

    def _BeginStroke(self):
        """Remember the table the brush is about to change."""
        self._strokeBefore = (self._model.SnapshotFaces()
                              if self._model is not None else None)

    def _EndStroke(self):
        """Close the stroke, pushing it only if it changed something."""
        before, self._strokeBefore = self._strokeBefore, None
        if before is None or self._model is None or not self.stroke_faces:
            return
        after = self._model.SnapshotFaces()
        if (before == after).all():
            return
        self._UndoStack().Push(FaceTableEdit(self, before, after))

    def CanUndo(self):
        return self._UndoStack().CanUndo()

    def CanRedo(self):
        return self._UndoStack().CanRedo()

    def Undo(self):
        """Take back the last paint stroke. True when one was there."""
        return self._UndoStack().Undo()

    def Redo(self):
        return self._UndoStack().Redo()

    def ApplyFaceTable(self, table):
        """Undo/redo lands here: restore, republish, tell the panel."""
        if self._model is None:
            return
        self._model.RestoreFaces(table)
        self._Redraw()
        self.strokeFinished.emit()

    def SetPaintTarget(self, region):
        """Which region a stroke gives its faces to."""
        self._paintTarget = region
        return region

    def Paint(self, x, y, erase=False):
        """One brush dab at a physical pixel. Returns faces changed.

        The dab is a world-space ball around the ray hit: one native cast
        plus one parallel pass over the face centroids. The edited face ->
        region table goes to the native side, which republishes the
        per-face region primvar -- one array upload, no geometry.
        """
        if self._model is None:
            return 0
        target = self._paintTarget
        if target is None and not erase:
            return 0
        self.SyncPose()
        ray = self.RayAt(x, y)
        if ray is None:
            return 0
        origin, direction = ray
        face, t = self._model.Cast(origin, direction)
        if face < 0:
            return 0
        length = sum(c * c for c in direction) ** 0.5 or 1.0
        hit = [origin[i] + direction[i] / length * t for i in range(3)]
        faces = self._model.Brush(hit, direction, radius=self._brushRadius)
        if not len(faces):
            return 0
        touched = (self._model.EraseFaces(faces) if erase
                   else self._model.AssignFaces(target, faces))
        if not touched:
            return 0
        self.stroke_faces += len(faces)
        if self._highlight is not None and not erase:
            self._highlight.SetHover(target)
        self._Redraw()
        self.statusChanged.emit(
            "%s %d faces %s %s"
            % ("erased" if erase else "painted", len(faces),
               "from" if erase else "into",
               target.label if target is not None else "(nothing)"))
        return len(faces)

    def Save(self, path=None):
        """Write the regions back to their own layers. Never the rig.

        With no `path`, each region goes back to the layer it was READ
        from, so a character whose body and face each carry a touch layer
        (Biped_body_touch_regions.usda, Biped_face_touch_regions.usda)
        saves each region into its own branch and leaves the rig alone. A
        region painted new this session goes to the layer of the first
        region. With a `path`, every region is written there.

        Returns the first path written; `last_saved_paths` has them all.
        """
        from touchpose import usdexport

        self.last_saved_paths = []
        if self._model is None:
            return None
        if path:
            groups = [(path, self._model.Rows())]
        else:
            groups = self._RowsByLayer()
        if not groups:
            return None
        for layer_path, rows in groups:
            usdexport.save_regions(
                layer_path, self._model.mesh_path, rows,
                palette=self._model.palette, alpha=self._model.alpha,
                lead=self._model.lead_color,
                selected=self._model.selected_color,
                scope_path=getattr(self._model, "scope_path", None))
            self.last_saved_paths.append(layer_path)
        self._model.dirty = False
        return self.last_saved_paths[0]

    def _RowsByLayer(self):
        """[(layer path, rows)] in the order the layers are first met.

        Every layer that held a region is listed, even when all its
        regions were erased, so the rewrite drops them from that file.
        """
        default = self.RegionLayerPath()
        if not default:
            return []
        by_name = {row[0]: row for row in self._model.Rows()}
        groups = {}
        for region in self._model.regions:
            layer = self._RegionLayer(region.name) or default
            rows = groups.setdefault(layer, [])
            if region.name in by_name:
                rows.append(by_name[region.name])
        return list(groups.items())

    def _RegionScope(self):
        stage = self._Stage()
        if stage is None or self._model is None:
            return None
        # The scope the model was READ from, then any scope annotating THIS
        # mesh. (This asked the model for `mesh`, which it never had, so the
        # filter was always None and the first scope on the stage won --
        # the wrong layer as soon as a shot holds two characters.)
        scope_path = getattr(self._model, "scope_path", None)
        scope = stage.GetPrimAtPath(Sdf.Path(scope_path)) if scope_path             else None
        if not scope or not scope.IsValid():
            scopes = touchPoseModel.FindRegionScopes(
                stage, self._model.mesh_path)
            scope = scopes[0] if scopes else None
        return scope

    def _RegionLayer(self, name):
        """The layer holding the strongest opinion on a region's faces."""
        scope = self._RegionScope()
        if scope is None:
            return None
        prim = scope.GetStage().GetPrimAtPath(
            scope.GetPath().AppendChild(name))
        attr = None
        for attr_name in (touchPoseModel.TYPED_FACES,
                          touchPoseModel.FACES_ATTR):
            candidate = prim.GetAttribute(attr_name) if prim else None
            if candidate and candidate.IsValid():
                attr = candidate
                break
        if attr is None:
            return None
        for spec in attr.GetPropertyStack(Usd.TimeCode.Default()):
            identifier = spec.layer.realPath or spec.layer.identifier
            if identifier and not spec.layer.anonymous:
                return identifier
        return None

    def RegionLayerPath(self):
        """The layer the first region prim was authored in, if there is one.

        Found by asking USD which layer holds the strongest opinion on a
        region's face list, rather than by guessing at a filename.
        """
        if self._model is None or not self._model.regions:
            return None
        return self._RegionLayer(self._model.regions[0].name)

    # -- the loop --------------------------------------------------------

    def SyncPose(self, force=False):
        """Follow the pose the viewport draws. True when the mesh moved.

        Native, and cheap when nothing moved (a pointer compare against the
        RigExec snapshot), so it runs per hover rather than on a timer. The
        highlight needs nothing here at all: it is drawn by the mesh's own
        shader, so it moves with the skin by construction.

        Every live set, because every one of them is cast.
        """
        if not self._models:
            return False
        time = self._Time()
        moved = False
        for model in self._models:
            moved = model.SyncPose(time, force=force) or moved
        return moved

    @property
    def suspended(self):
        """True while a gizmo drag, playback or a scrub has TouchPose
        stood down."""
        return self._suspended

    def _ConnectFrames(self, on):
        signal = getattr(self._api.dataModel, "currentFrameChanged", None)
        if on and signal is not None and self._frameSignal is None:
            signal.connect(self._OnFrameChanged)
            self._frameSignal = signal
        elif not on and self._frameSignal is not None:
            try:
                self._frameSignal.disconnect(self._OnFrameChanged)
            except (RuntimeError, TypeError):
                pass
            self._frameSignal = None

    def _OnFrameChanged(self, *args):
        """The timeline moved: dark until it has been still a moment."""
        self._timeBusy = True
        self._timeQuiet.start()
        self._PollDrag()

    def _OnTimeQuiet(self):
        self._timeBusy = False
        self._PollDrag()

    def _PollDrag(self):
        """Stand TouchPose down while the rig is being moved.

        A gizmo drag, playback and a scrub all turn every highlight off --
        hover, lead and selection -- and the picking with them, and turn
        them back on as they were once the rig is still.
        """
        if not self._active or self._model is None:
            return
        dragging = GizmoDragging() or self._timeBusy
        if dragging == self._suspended:
            return
        self._suspended = dragging
        if dragging:
            self._hover = None
            # A hover the rate limit held back must not land on the other
            # side of the suspend and light the hover mid-drag.
            self._hoverTimer.stop()
            self._pendingHover = None
            changed = False
            for highlight in self._highlights:
                changed = highlight.Suspend() or changed
            if changed:
                self._Redraw()
            return
        self.SyncPose(force=True)
        changed = False
        for highlight in self._highlights:
            highlight.Clear()
            changed = highlight.Resume() or changed
        if changed:
            self._Redraw()

    def _HoverSoon(self, x, y):
        """One hover per frame at most, and never the last one dropped.

        LEADING EDGE plus a trailing one-shot, not a plain rate limit: a
        plain one drops the sample where the mouse stopped, which is the
        one that matters.
        """
        now = QtCore.QDateTime.currentMSecsSinceEpoch()
        if now - self._lastHover >= HOVER_MIN_MS:
            self._lastHover = now
            self._pendingHover = None
            return self.Hover(x, y)
        self._pendingHover = (x, y)
        if not self._hoverTimer.isActive():
            self._hoverTimer.start(
                max(1, HOVER_MIN_MS - int(now - self._lastHover)))
        return None

    def _FlushHover(self):
        """Answer the sample the rate limit held back."""
        pending, self._pendingHover = self._pendingHover, None
        if pending is None or not self._active or self._suspended:
            return
        self._lastHover = QtCore.QDateTime.currentMSecsSinceEpoch()
        self.Hover(*pending)

    def Hover(self, x, y):
        """Light the region under a physical pixel. Returns it, or None.

        The hover is set on the set that was HIT and cleared on all the
        others, so crossing from the cheek onto the eyeball puts the
        body's hover out as the eye's comes on -- two lit patches would
        read as two things being aimed at.
        """
        self.SyncPose()
        hit, region, _face = self.CastAt(x, y)
        changed = False
        for model, highlight in zip(self._models, self._highlights):
            changed = highlight.SetHover(
                region if model is hit else None) or changed
        if changed:
            self._Redraw()
        if region is not self._hover:
            self._hover = region
            self.statusChanged.emit(
                region.label if region is not None else "")
            view = self._view
            if view is not None:
                if region is not None:
                    view.setCursor(QtCore.Qt.PointingHandCursor)
                else:
                    view.unsetCursor()
        return region

    def Click(self, x, y, mode=MODE_REPLACE):
        """Pick the region under a physical pixel, in `mode`.

        Returns True when TouchPose owns the click -- which includes a
        click that landed on the SKIN but on no region. See the module
        docstring: while the mode is on, the mesh is not selectable.
        """
        self.SyncPose()
        region, face = self.RegionAt(x, y)
        if face < 0:
            return False            # nothing of the character there
        if region is None:
            # On the skin, off every region. A plain click there CLEARS,
            # the way clicking empty space in any picker does; a Shift or
            # Ctrl click leaves the selection alone, because a modifier
            # says "adjust this selection" and there is nothing to adjust
            # it by.
            if mode == MODE_REPLACE:
                self.Pick([], MODE_REPLACE)
            return True
        self.Pick([region], mode)
        return True

    def _ControlsInBand(self, x0, y0, x1, y1):
        """The rig controls this band catches, independent of the skin.

        TouchPose regions only cover the MESH. Every control that floats
        beside it -- the IK controls, the pole vectors, the param nodes --
        is invisible to a region pick, so while TouchPose was on a box
        drawn round them selected nothing. This is the same call the
        native marquee makes, so both answers come from one piece of
        arithmetic and cannot drift apart.

        Any failure here returns nothing rather than breaking the region
        pick that already worked.
        """
        try:
            import gizmoMarquee
            import gizmoUI
        except Exception:
            return []
        # THIS session's gizmo: its camera, viewport and posed cache are
        # the ones that project this session's stage. (The module's
        # `_controller` view answers for the active window's session, and
        # for nobody when that is ambiguous.)
        controller = gizmoUI.GetController(self._api)
        if controller is None:
            return []
        stage = self._Stage()
        if stage is None:
            return []
        try:
            camera, viewport, _ratio = controller._Camera()
            return gizmoMarquee.ControlsInBand(
                stage, camera, viewport, controller.usdviewApi.frame,
                (x0, y0, x1, y1), controller._solverPosed)
        except Exception:
            return []

    def Marquee(self, x0, y0, x1, y1, mode=MODE_REPLACE):
        """Pick every region AND every control the band touches."""
        self.SyncPose()
        regions = self.RegionsInBand(x0, y0, x1, y1)
        self.Pick(regions, mode,
                  extra=self._ControlsInBand(x0, y0, x1, y1))
        self.statusChanged.emit(
            "%s %d region%s" % (mode, len(regions),
                                "" if len(regions) == 1 else "s"))
        return [r.label for r in regions]

    # -- the rubber band -------------------------------------------------

    def _ShowBand(self):
        """Draw the marquee with Qt's own rubber band over the viewport.

        A QRubberBand rather than a painted overlay widget: the gizmo
        already owns a transparent overlay on this view, and stacking a
        second one under it is a fight about z-order that a stock widget
        does not have.
        """
        view = self._view
        if view is None or self._band is None:
            return
        if self._rubber is None:
            self._rubber = QtWidgets.QRubberBand(
                QtWidgets.QRubberBand.Rectangle, view)
        ratio = self._Ratio()
        x0, y0, x1, y1 = [v / ratio for v in self._band]
        self._rubber.setGeometry(QtCore.QRect(
            QtCore.QPoint(int(min(x0, x1)), int(min(y0, y1))),
            QtCore.QPoint(int(max(x0, x1)), int(max(y0, y1)))))
        self._rubber.show()
        self._rubber.raise_()

    def _HideBand(self):
        if self._rubber is not None:
            self._rubber.hide()

    @staticmethod
    def ModeFor(modifiers):
        """The selection mode a set of modifiers asks for.

        Ctrl is tested FIRST so Ctrl+Shift can only ever subtract: the
        promise is that there is one gesture that never adds, and a
        Shift-wins ordering would break it for the one combination an
        animator is most likely to hit by accident.
        """
        if modifiers & QtCore.Qt.ControlModifier:
            return MODE_REMOVE
        if modifiers & QtCore.Qt.ShiftModifier:
            return MODE_TOGGLE
        return MODE_REPLACE

    def Pick(self, regions, mode=MODE_REPLACE, extra=None):
        """Apply one pick -- a click or a marquee -- to the selection.

        The single path both gestures go through. Returns the prim paths
        the selection ends up holding.

        `extra` is prim paths the same gesture caught by other means --
        the rig controls a marquee band covers, which no region knows
        about. They join the region controls before the mode is applied,
        so a Shift-drag toggles the whole catch as one gesture rather
        than toggling the skin and then fighting over the controls.
        """
        stage = self._Stage()
        selection = self._Selection()
        if stage is None or selection is None:
            return []

        wanted = []
        seen = set()
        for path in ([r.control for r in regions if r.control]
                     + [str(p) for p in (extra or [])]):
            if path in seen:
                continue
            seen.add(path)
            prim = stage.GetPrimAtPath(Sdf.Path(path))
            if prim and prim.IsValid():
                wanted.append(prim)

        # THE PSEUDO-ROOT IS NOT A SELECTION. `clearPrims` leaves `/`
        # behind, so the current set always looks non-empty and a toggle
        # would carry `/` forward for ever. Dropped here rather than at
        # each call site.
        current = [p for p in selection.getPrims()
                   if p and p.IsValid() and not p.IsPseudoRoot()]
        have = set(str(p.GetPath()) for p in current)

        if mode == MODE_REPLACE:
            keep = list(wanted)
        elif mode == MODE_REMOVE:
            drop = set(str(p.GetPath()) for p in wanted)
            keep = [p for p in current if str(p.GetPath()) not in drop]
        else:                                   # toggle
            keep = list(current)
            for prim in wanted:
                path = str(prim.GetPath())
                if path in have:
                    keep = [p for p in keep if str(p.GetPath()) != path]
                else:
                    keep.append(prim)

        # The LEAD is the last region this gesture actually put IN. A
        # remove, or a toggle that turned its region off, leaves the lead
        # to whatever is still selected.
        kept = set(str(p.GetPath()) for p in keep)
        lead = None
        for region in regions:
            if region.control in kept:
                lead = region
        if lead is None and self._lead is not None \
                and self._lead.control in kept:
            lead = self._lead
        self._lead = lead

        with getattr(selection, "batchPrimChanges", _NullContext()):
            selection.clearPrims()
            for prim in keep:
                selection.addPrim(prim)
        return [str(p.GetPath()) for p in keep]

    def RegionsInBand(self, x0, y0, x1, y1):
        """Regions the marquee touches, front-facing only.

        Across every live set, so a band drawn over the head catches the
        eyes along with the face.
        """
        if not self._models or self._view is None:
            return []
        try:
            viewport = self._view.computeWindowViewport()
            width, height = int(viewport[2]), int(viewport[3])
            frustum = self._view.resolveCamera()[0].frustum
            matrix = (frustum.ComputeViewMatrix()
                      * frustum.ComputeProjectionMatrix())
            eye = frustum.ComputeViewMatrix().GetInverse() \
                .ExtractTranslation()
        except Exception:
            return []
        self.SyncPose()
        caught = []
        for model in self._models:
            caught.extend(model.RegionsInRect(
                matrix, width, height, (eye[0], eye[1], eye[2]),
                x0, y0, x1, y1))

        # A band catches a region when one of its face CENTROIDS is
        # inside, which is the right test at any useful size and the
        # wrong one when the band is a few pixels across: the centroids
        # of a limb are further apart than that, so a small band lands
        # between them and catches nothing at all. The band's centre is
        # therefore cast as well -- a tiny marquee then behaves exactly
        # like the click it visually is, which is what stops a slightly
        # shaky click from clearing the selection.
        #
        # Compared by IDENTITY and not by `index`: an index is only
        # unique within one set, and with the eyes live as well region 0
        # exists three times over.
        middle, _face = self.RegionAt((x0 + x1) * 0.5, (y0 + y1) * 0.5)
        if middle is not None and not any(r is middle for r in caught):
            caught.append(middle)
        return caught

    def Select(self, region, add=False):
        """One region, replacing or toggling. A `Pick` of one.

        Kept as its own name because the panel's list rows and the tests
        read better for it, but it is the same path -- there is no second
        implementation of what a selection change means.
        """
        if not region.control:
            return False
        before = self.Pick([region],
                           MODE_TOGGLE if add else MODE_REPLACE)
        if region.control not in before:
            return False
        self.statusChanged.emit("%s -> %s" % (region.label,
                                              region.control.rsplit("/")[-1]))
        return True

    # -- Qt --------------------------------------------------------------

    def eventFilter(self, obj, event):
        if not self._active or self._model is None:
            return False
        kind = event.type()
        if kind == QtCore.QEvent.HoverMove:
            # SUSPENDED: the first thing checked, before the position is
            # even read, so a sample during a drag costs a boolean. The
            # hover patch is dropped once on the way in -- `Clear` is
            # itself a no-op when nothing is lit, so the repeat is free
            # -- and `_suspended` is what makes the resume a one-shot
            # transition instead of a poll.
            if self._suspended:
                return False
            if GizmoDragging(self._api):
                # Stand down now rather than on the next poll tick, so the
                # first hover of a drag already finds TouchPose suspended.
                self._PollDrag()
                return False
            x, y = self._Position(event)
            if GizmoOwns(x, y, self._Ratio(), self._api):
                # The gizmo is drawing its own pre-selection highlight on
                # this pixel; lighting a region behind it as well reads
                # as two things being aimed at.
                if self._ClearHover():
                    self._Redraw()
                self._hover = None
                return False
            self._HoverSoon(x, y)
            return False        # never consume a hover: the camera reads them
        if kind in (QtCore.QEvent.Leave, QtCore.QEvent.HoverLeave):
            self._hover = None
            self._hoverTimer.stop()
            self._pendingHover = None
            if self._ClearHover():
                self._Redraw()
            return False
        if kind == QtCore.QEvent.MouseButtonPress:
            modifiers = event.modifiers()
            # Alt is usdview's camera, untouched. Anything but the left
            # button is the camera or the context menu, likewise.
            if modifiers & QtCore.Qt.AltModifier:
                return False
            if event.button() != QtCore.Qt.LeftButton:
                return False
            x, y = self._Position(event)
            # THE GIZMO WINS. Checked before anything else is done, so a
            # press over a handle costs TouchPose a hit test and nothing
            # else -- no cast, no selection change, no overlay edit.
            if GizmoOwns(x, y, self._Ratio(), self._api):
                self._consumedPress = False
                return False
            # PAINT OWNS THE PLAIN DRAG while its box is ticked, and
            # Shift erases. It deliberately does NOT use Ctrl: Ctrl is
            # the remove gesture now, and a modifier that paints in one
            # mode and subtracts in another is the kind of thing that
            # gets an animator to delete half a selection by reflex.
            # Painting is a mode; the brush takes the drag, and the
            # marquee stands down for as long as it is on.
            if self._paint and self._paintTarget is not None:
                erase = bool(modifiers & QtCore.Qt.ShiftModifier)
                self._painting = "erase" if erase else "add"
                self.stroke_faces = 0
                self._BeginStroke()
                self.Paint(x, y, erase=erase)
                self._consumedPress = True
                return True
            # Not a click yet: it becomes one on RELEASE if the cursor
            # never travelled far enough to be a marquee. Deciding here
            # would make every drag start by selecting whatever was
            # under the press, which the marquee would then replace.
            self._pressAt = (x, y)
            self._pressMode = self.ModeFor(modifiers)
            self._band = None
            self._consumedPress = self.RegionAt(x, y)[1] >= 0
            return self._consumedPress
        if kind == QtCore.QEvent.MouseMove and self._painting is not None:
            x, y = self._Position(event)
            self.Paint(x, y, erase=self._painting == "erase")
            return True
        if kind == QtCore.QEvent.MouseMove and self._pressAt is not None:
            x, y = self._Position(event)
            if (abs(x - self._pressAt[0]) > DRAG_SLOP * self._Ratio()
                    or abs(y - self._pressAt[1]) > DRAG_SLOP * self._Ratio()):
                self._band = (self._pressAt[0], self._pressAt[1], x, y)
                self._ShowBand()
            return self._consumedPress
        if kind in (QtCore.QEvent.MouseButtonRelease,
                    QtCore.QEvent.MouseButtonDblClick):
            if self._painting is not None:
                self._painting = None
                self._EndStroke()
                self.statusChanged.emit(
                    "stroke: %d faces" % self.stroke_faces)
                self.strokeFinished.emit()
                self._consumedPress = False
                return True
            if self._pressAt is not None:
                band, self._band = self._band, None
                start, self._pressAt = self._pressAt, None
                mode, self._pressMode = self._pressMode, MODE_REPLACE
                self._HideBand()
                if band is not None:
                    self.Marquee(band[0], band[1], band[2], band[3], mode)
                    self._consumedPress = False
                    return True
                took = self.Click(start[0], start[1], mode=mode)
                self._consumedPress = False
                return took
            # Consume the release ONLY when this filter consumed the
            # matching press. usdview picks on the PRESS (stageView.py
            # mousePressEvent -> pickObject), so the release carries no
            # selection of its own -- but it does clear `_dragActive`,
            # and swallowing a release whose press went through leaves
            # the stage view convinced a camera drag is still running for
            # the rest of the session. Re-casting the ray here instead
            # would get that wrong the moment the cursor left the body
            # between press and release.
            if event.button() == QtCore.Qt.LeftButton and self._consumedPress:
                self._consumedPress = False
                return True
            return False
        return False


class TouchPosePanel(QtWidgets.QDialog):
    """The toggle, the numbers, and a list of what is paintable."""

    # One panel per usdview session, filed under its main window.
    _sessions = sessionRegistry.SessionRegistry("touchPose panels")

    @classmethod
    def GetInstance(cls, usdviewApi):
        panel = cls._sessions.Get(usdviewApi)
        if panel is None:
            panel = cls._sessions.Set(usdviewApi, cls(usdviewApi))
        else:
            panel._api = usdviewApi
        return panel

    def __init__(self, usdviewApi, parent=None):
        super(TouchPosePanel, self).__init__(
            parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        self._controller = TouchPoseController.GetInstance(usdviewApi)
        self.setWindowTitle("TouchPose")
        self.resize(360, 480)

        layout = QtWidgets.QVBoxLayout(self)
        self._toggle = QtWidgets.QCheckBox(
            "Touch the character to select its control")
        self._toggle.setToolTip(
            "While this is on, clicking the body selects the control that "
            "owns the region under the cursor -- and never the mesh "
            "itself. Turn it off to get usdview's own picking back.")
        self._toggle.toggled.connect(self._OnToggled)
        layout.addWidget(self._toggle)

        # ONE status line for the hover and the voice both. Built here so
        # the voice mode can be wired to it, and added to the layout below
        # the voice checkbox so it reads as the answer to whichever of the
        # two spoke last.
        self._hoverLabel = QtWidgets.QLabel("")
        font = self._hoverLabel.font()
        font.setBold(True)
        self._hoverLabel.setFont(font)

        self._voice = self._MakeVoice(usdviewApi)
        layout.addWidget(self._voiceToggle)
        layout.addWidget(self._hoverLabel)

        # -- the layer switch, and the filter ----------------------------
        #
        # One row, because they answer the same question -- which regions
        # am I looking at -- and a panel this narrow cannot afford two.
        # The switch hides itself on a rig with a single layer rather than
        # showing a combo with one entry nobody can act on.
        pick = QtWidgets.QHBoxLayout()
        self._layerLabel = QtWidgets.QLabel("layer")
        pick.addWidget(self._layerLabel)
        self._layer = QtWidgets.QComboBox()
        self._layer.setToolTip(
            "Which set of touch regions the list below shows, and which "
            "one a stroke paints into. Every set on the stage is live for "
            "hovering and picking whatever this says -- the body and both "
            "eyeballs at once -- because a set owns one mesh and the eyes "
            "are not the body.")
        self._layer.activated.connect(self._OnLayerChosen)
        pick.addWidget(self._layer, 1)
        self._search = QtWidgets.QLineEdit()
        self._search.setPlaceholderText("find a region...")
        self._search.setClearButtonEnabled(True)
        self._search.setToolTip(
            "Filter the list below. Matches the region's name and the "
            "control it selects, case-insensitively; space-separated "
            "words all have to match, in any order.")
        self._search.textChanged.connect(self._OnSearchChanged)
        pick.addWidget(self._search, 1)
        layout.addLayout(pick)

        self._list = QtWidgets.QListWidget()
        self._list.itemClicked.connect(self._OnRowClicked)
        layout.addWidget(self._list, 1)

        # -- painting ----------------------------------------------------
        paint = QtWidgets.QHBoxLayout()
        self._paint = QtWidgets.QCheckBox("Paint")
        self._paint.setToolTip(
            "Dragging on the body gives the faces under the brush to the "
            "region selected above; Shift-drag takes them away. "
            "Nothing is written until Save.")
        self._paint.toggled.connect(self._OnPaintToggled)
        paint.addWidget(self._paint)
        paint.addWidget(QtWidgets.QLabel("brush"))
        self._brush = QtWidgets.QDoubleSpinBox()
        self._brush.setRange(0.05, 100.0)
        self._brush.setSingleStep(0.5)
        self._brush.setSuffix(" cm")
        self._brush.setToolTip(
            "Brush radius in stage units. Defaults to 2% of the mesh, "
            "which is about a fingertip on this character.")
        self._brush.valueChanged.connect(self._controller.SetBrush)
        paint.addWidget(self._brush)
        self._save = QtWidgets.QPushButton("Save regions")
        self._save.setToolTip(
            "Rewrite the touch layer the regions were read from. The rig "
            "is never touched.")
        self._save.clicked.connect(self._OnSave)
        paint.addWidget(self._save)
        layout.addLayout(paint)

        # -- how the patches look ----------------------------------------
        #
        # All three are SAVED with the regions (`touchpose:alpha`,
        # `leadColor`, `selectedColor` on the scope), so this row is not
        # a viewer preference -- it is editing the touch layer, the same
        # way the region list above it does.
        look = QtWidgets.QHBoxLayout()
        look.addWidget(QtWidgets.QLabel("opacity"))
        self._opacity = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._opacity.setRange(5, 100)
        self._opacity.setToolTip(
            "How much of the body shows through the highlight. Takes "
            "effect as you drag; saved with the regions as "
            "touchpose:alpha.")
        self._opacity.valueChanged.connect(self._OnOpacity)
        look.addWidget(self._opacity, 1)
        self._opacityLabel = QtWidgets.QLabel("")
        self._opacityLabel.setMinimumWidth(34)
        look.addWidget(self._opacityLabel)
        self._leadSwatch = QtWidgets.QPushButton("Lead")
        self._leadSwatch.setToolTip(
            "The colour of the region whose control was selected LAST. "
            "The studio's file makes it green.")
        self._leadSwatch.clicked.connect(lambda: self._OnColor("lead"))
        look.addWidget(self._leadSwatch)
        self._selectedSwatch = QtWidgets.QPushButton("Selected")
        self._selectedSwatch.setToolTip(
            "The colour of every other selected region. "
            "The studio's file makes it a neutral grey.")
        self._selectedSwatch.clicked.connect(
            lambda: self._OnColor("selected"))
        look.addWidget(self._selectedSwatch)
        layout.addLayout(look)

        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self._InstallShortcuts()
        self._controller.statusChanged.connect(self._hoverLabel.setText)
        # The face counts in the list go stale the moment a stroke lands.
        self._controller.strokeFinished.connect(self.Refresh)
        self.Reload()

    @property
    def controller(self):
        return self._controller

    @property
    def voice(self):
        return self._voice

    # -- voice -----------------------------------------------------------

    def _MakeVoice(self, usdviewApi):
        """The voice mode and its checkbox, or a checkbox that says why not.

        NOTHING HERE CAN STOP THE PANEL OPENING. The speech subpackage
        lives in the shared TouchPose repo, and a checkout without it -- or a machine with no speech
        engine -- gets a disabled box whose tooltip names the reason,
        which is more use to whoever meets it than a feature that is
        simply not there. Every path out of here returns a checkbox.
        """
        self._voiceToggle = QtWidgets.QCheckBox("Voice (hold %s)"
                                                % (touchPoseVoice.
                                                   PUSH_TO_TALK_LABEL
                                                   if touchPoseVoice else "N"))
        if touchPoseVoice is None:
            self._voiceToggle.setEnabled(False)
            self._voiceToggle.setToolTip(
                "Voice-to-select is not available in this checkout.")
            return None

        try:
            core, reason = touchPoseVoice.Available()
        except Exception as error:                  # pragma: no cover
            core, reason = None, "Voice-to-select is unavailable: %s" % error
        if core is None:
            self._voiceToggle.setEnabled(False)
            self._voiceToggle.setToolTip(reason)
            return None

        self._voiceToggle.setToolTip(
            "Hold %s anywhere in usdview and say a control's name -- "
            "\"left clavicle\", \"right arm pole vector\". The control is "
            "selected, its region lights on the skin, and the Avar Editor "
            "follows. Saying a name without a side picks the one on the "
            "side of the current selection, or both."
            % touchPoseVoice.PUSH_TO_TALK_LABEL)
        try:
            voice = touchPoseVoice.TouchPoseVoice(self._controller,
                                                  usdviewApi, self)
        except Exception as error:                  # pragma: no cover
            self._voiceToggle.setEnabled(False)
            self._voiceToggle.setToolTip("Voice-to-select is unavailable: %s"
                                         % error)
            return None
        voice.statusChanged.connect(self._hoverLabel.setText)
        self._voiceToggle.toggled.connect(self._OnVoiceToggled)
        # IT COMES UP WITH THE PANEL. The files are there and the engine
        # works, so there is nothing for anybody to turn on: holding N
        # just works. Deferred by one event-loop turn so starting the
        # helper -- about 0.9 s, once -- never delays the window
        # appearing, and so a failure to start cannot fail the
        # constructor. The checkbox stays, because turning it OFF is a
        # real thing to want: it gives N and the microphone back.
        # OFF BY DEFAULT. It used to switch itself on once the panel was
        # up, on the reasoning that a feature nobody has to enable is one
        # nobody has to find. In practice it claims a hotkey and opens the
        # microphone for everyone who opens TouchPose, including the many
        # sessions that never say a word -- so it waits to be asked. The
        # checkbox is the way in, and TOUCHPOSE_VOICE_AUTOSTART=1 brings
        # the old behaviour back for anyone who wants it.
        if os.environ.get("TOUCHPOSE_VOICE_AUTOSTART", "").strip() not in (
                "", "0", "false", "False"):
            QtCore.QTimer.singleShot(0, self._AutoStartVoice)
        return voice

    def _AutoStartVoice(self):
        """Switch voice on by itself, quietly, once the panel is up."""
        if self._voice is None or self._voice.enabled:
            return
        try:
            if self._voice.SetEnabled(True):
                self._SetChecked(self._voiceToggle, True)
        except Exception as error:                  # pragma: no cover
            # Nobody asked for this, so nobody gets a terminal line about
            # it: the reason is already on the status line, and TouchPose
            # carries on without it. Warned only because reaching here at
            # all means SetEnabled raised, which it is written not to.
            Tf.Warn("touchPose: voice did not start: %s" % error)

    def _OnVoiceToggled(self, checked):
        if self._voice is None:
            return
        try:
            got = self._voice.SetEnabled(checked)
        except Exception as error:                  # pragma: no cover
            # SetEnabled is written not to raise; if it ever does, the
            # checkbox goes back and TouchPose carries on regardless.
            self._hoverLabel.setText("Voice unavailable: %s" % error)
            got = False
        if got != checked:
            # Starting failed -- no microphone, no engine, no regions. The
            # box goes back so it never claims a mode that is not on; the
            # reason is already in the status line.
            self._SetChecked(self._voiceToggle, got)

    # -- hotkeys ---------------------------------------------------------

    #  T          turn TouchPose on and off -- the mode switch, and the
    #             one an animator hits most
    #  P          paint on and off
    #  Ctrl+A     select every region's control
    #  Ctrl+Shift+A  clear
    #  Ctrl+I     invert over the regions
    #  Ctrl+S     save the regions
    #
    # Scoped WidgetWithChildrenShortcut so they only fire while this
    # panel has focus: usdview binds plain letters of its own over the
    # viewport, and an application-wide `T` would fight them.
    _SHORTCUTS = (
        ("T", "_OnToggleKey"),
        ("P", "_OnPaintKey"),
        ("Ctrl+A", "_OnSelectAll"),
        ("Ctrl+Shift+A", "_OnClearSelection"),
        ("Ctrl+I", "_OnInvertSelection"),
        ("Ctrl+S", "_OnSave"),
    )

    def _InstallShortcuts(self):
        # QAction moved from QtWidgets to QtGui in Qt6 and usdview builds
        # against either, so it is looked up rather than imported.
        factory = getattr(QtGui, "QAction", None) or QtWidgets.QAction
        self._actions = []
        for sequence, handler in self._SHORTCUTS:
            action = factory(sequence, self)
            action.setShortcut(QtGui.QKeySequence(sequence))
            action.setShortcutContext(
                QtCore.Qt.WidgetWithChildrenShortcut)
            action.triggered.connect(getattr(self, handler))
            self.addAction(action)
            self._actions.append(action)
        return self._actions

    @staticmethod
    def _SetChecked(box, value):
        """Set a checkbox without re-entering its own handler."""
        box.blockSignals(True)
        box.setChecked(bool(value))
        box.blockSignals(False)

    def _OnToggleKey(self):
        # Driven off the CONTROLLER, not off the checkbox. The two can be
        # out of step -- anything else may have called SetActive -- and
        # `setChecked` to a value the box already holds emits nothing, so
        # a key press would silently do nothing at all.
        got = self._controller.SetActive(not self._controller.active)
        self._SetChecked(self._toggle, got)
        if not got:
            self._hoverLabel.setText("")

    def _OnPaintKey(self):
        want = not self._controller.painting
        self._SetChecked(self._paint, want)
        self._OnPaintToggled(want)

    def SyncToggles(self):
        """Put both boxes back in step with the controller."""
        self._SetChecked(self._toggle, self._controller.active)
        self._SetChecked(self._paint, self._controller.painting)

    # -- look ------------------------------------------------------------

    def _OnOpacity(self, value):
        self._controller.SetOpacity(value / 100.0)
        self._opacityLabel.setText("%d%%" % value)

    def _OnColor(self, state):
        """Pick a new lead or selected colour, and show it immediately."""
        model = self._controller.model
        if model is None:
            return
        current = (model.lead_color if state == "lead"
                   else model.selected_color)
        chosen = QtWidgets.QColorDialog.getColor(
            QtGui.QColor(*[int(round(c * 255)) for c in current]), self,
            "TouchPose %s colour" % state)
        if not chosen.isValid():
            return
        self._controller.SetStateColor(
            state, (chosen.redF(), chosen.greenF(), chosen.blueF()))
        self._SyncLook()

    def _SyncLook(self):
        """Put the slider and the two swatches back where they belong.

        The swatches are driven off the MODEL, not off the widgets:
        `Load` re-reads the touch layer, and they must show what the
        layer says rather than what the last session left in the widget.

        The slider is driven off the CONTROLLER, which keeps the opacity
        across a load, so this puts the widget back on a value the load
        did not disturb rather than on one it reset.
        """
        self._opacity.blockSignals(True)
        self._opacity.setValue(int(round(self._controller.opacity * 100)))
        self._opacity.blockSignals(False)
        self._opacityLabel.setText("%d%%" % self._opacity.value())
        model = self._controller.model
        if model is None:
            return
        for button, colour in ((self._leadSwatch, model.lead_color),
                               (self._selectedSwatch, model.selected_color)):
            rgb = [int(round(c * 255)) for c in colour]
            # The label goes light on a dark swatch, which both of these
            # start out as; without it "Lead" is unreadable on the
            # studio's own near-black green.
            text = "#fff" if sum(rgb) < 3 * 128 else "#000"
            button.setStyleSheet(
                "background-color: rgb(%d,%d,%d); color: %s;"
                % (rgb[0], rgb[1], rgb[2], text))

    def _AllRegions(self):
        model = self._controller.model
        return list(model.regions) if model is not None else []

    def _OnSelectAll(self):
        regions = self._AllRegions()
        self._controller.Pick(regions, MODE_REPLACE)
        self._status.setText("selected %d regions" % len(regions))

    def _OnClearSelection(self):
        self._controller.Pick([], MODE_REPLACE)
        self._status.setText("selection cleared")

    def _OnInvertSelection(self):
        """Everything not selected becomes selected, and vice versa.

        Over the REGIONS, not over the stage: inverting a selection of
        one control into "every prim in the rig" is not what anyone
        means by it here.
        """
        model = self._controller.model
        if model is None:
            return
        selection = self._controller._Selection()
        have = set()
        if selection is not None:
            have = set(str(p.GetPath()) for p in selection.getPrims()
                       if p and p.IsValid() and not p.IsPseudoRoot())
        wanted = [r for r in model.regions if r.control not in have]
        self._controller.Pick(wanted, MODE_REPLACE)
        self._status.setText("inverted to %d regions" % len(wanted))

    def Refresh(self):
        """Repaint the list from the model, without re-reading the stage.

        `Reload` calls `Load`, which rebuilds the model off the stage and
        would throw away an unsaved stroke.
        """
        model = self._controller.model
        if model is None:
            return
        keep = self._list.currentRow()
        self._list.clear()
        self._Fill(model)
        if 0 <= keep < self._list.count():
            self._list.setCurrentRow(keep)

    def _Fill(self, model):
        query = self._search.text() if hasattr(self, "_search") else ""
        for region in sorted(model.regions, key=lambda r: r.label):
            if not RegionMatches(region, query):
                continue
            item = QtWidgets.QListWidgetItem(
                "%-28s %5d faces" % (region.label, len(region.faces)))
            item.setData(QtCore.Qt.UserRole, region.index)
            item.setForeground(QtGui.QBrush(QtGui.QColor(
                *[int(round(c * 255))
                  for c in model.EditColor(region)])))
            self._list.addItem(item)

    def _OnSearchChanged(self, _text):
        """Refill the list against the filter.

        The filter is a VIEW of the model and touches nothing else: the
        regions, the paint and what the viewport draws are all unchanged,
        so a search cannot lose work and clearing it puts everything
        back. Refilling beats hiding rows because the list is hundreds of
        items, not thousands, and a rebuild keeps one code path.
        """
        model = self._controller.model
        self._list.clear()
        if model is not None:
            self._Fill(model)

    def _SyncLayers(self):
        """Put the switch where the model is standing.

        Read off the CONTROLLER, which lists every set it has open --
        `model.layers` lists only the sets on that model's own mesh, so
        on the biped it was ['Body'] alone and the switch hid itself
        with the two eye sets behind it.

        Signals are blocked while the combo is repopulated: `activated`
        does not fire on a programmatic change, but `setCurrentIndex`
        with a stale handler attached has bitten this panel before, and
        the block costs nothing.
        """
        model = self._controller.model
        layers = list(getattr(self._controller, "layers", None)
                      or getattr(model, "layers", []) or [])
        self._layer.blockSignals(True)
        self._layer.clear()
        for label, path in layers:
            self._layer.addItem(label, path)
        live = getattr(model, "layer_name", "")
        index = next((i for i, (label, _p) in enumerate(layers)
                      if label == live), -1)
        if index >= 0:
            self._layer.setCurrentIndex(index)
        self._layer.blockSignals(False)
        # A single layer is not a choice; a rig with none has no switch
        # to offer either.
        multiple = len(layers) > 1
        self._layer.setVisible(multiple)
        self._layerLabel.setVisible(multiple)

    def _OnLayerChosen(self, index):
        """Make the chosen set active, keeping the filter and the toggles.

        Every set stays live for hover and picking; this only moves which
        one the list below shows and a stroke paints into.

        `_SyncLook` is called at the end because the swatches belong to
        the set -- lead and selected are read off it -- and because the
        slider has to agree with what is drawn. It was missing, so a
        layer change left the slider reading the old percentage while
        `Load` had quietly put the highlight back to 85%.
        """
        label = self._layer.itemText(index)
        if not label or label == getattr(self._controller.model,
                                         "layer_name", ""):
            return
        self._status.setText(self._controller.Load(layer=label) or "")
        self._list.clear()
        model = self._controller.model
        if model is not None:
            self._Fill(model)
        self._SyncLayers()
        self.SyncToggles()
        self._SyncLook()

    def Reload(self):
        self._status.setText(self._controller.Load() or "")
        self._list.clear()
        model = self._controller.model
        self._SyncLayers()
        # A new stage is a new vocabulary. Cheap enough to do on every
        # reload -- the grammar took 0.12 s to load at 474 phrases -- and
        # leaving it stale would have the recogniser answering with prim
        # paths that are no longer on the stage.
        if self._voice is not None:
            self._voice.Refresh()
        if model is None:
            return
        self._Fill(model)
        self.SyncToggles()
        self._SyncLook()

    def _OnToggled(self, checked):
        got = self._controller.SetActive(checked)
        if got != checked:
            self._toggle.setChecked(got)
            self._status.setText(
                "Could not activate: no touch regions on this stage.")
        elif not checked:
            self._hoverLabel.setText("")

    def _OnRowClicked(self, item):
        model = self._controller.model
        if model is None:
            return
        index = item.data(QtCore.Qt.UserRole)
        region = model.regions[index]
        # A row is both "select this control" and "paint into this
        # region" -- the same row means the same region either way, and
        # two lists for one thing would be worse.
        self._controller.SetPaintTarget(region)
        self._controller.Select(region)

    def _OnPaintToggled(self, checked):
        model = self._controller.model
        if checked and model is not None:
            if self._controller.paintTarget is None and model.regions:
                row = self._list.currentRow()
                item = self._list.item(row if row >= 0 else 0)
                if item is not None:
                    self._controller.SetPaintTarget(
                        model.regions[item.data(QtCore.Qt.UserRole)])
            self._brush.setValue(round(model.brush, 2))
        self._controller.SetPainting(checked)
        target = self._controller.paintTarget
        if checked:
            self._status.setText(
                "Drag paints into %s, Shift-drag erases. Nothing is written "
                "until Save." % (target.label if target
                                 else "(pick a region above)"))
            return
        # NOT a reload. This called `Load()`, which re-read the regions off
        # the stage and threw away every unsaved stroke the moment paint
        # was switched off.
        if model is not None:
            regions, covered, faces = model.Coverage()
            self._status.setText(
                "%d regions cover %d of %d faces (%.0f%%)%s"
                % (regions, covered, faces, 100.0 * covered / max(faces, 1),
                   "; unsaved paint" if model.dirty else ""))
            self.Refresh()

    def _OnSave(self):
        path = self._controller.Save()
        if not path:
            self._status.setText(
                "Nothing to save: no region layer was found for this "
                "stage.")
            return
        paths = getattr(self._controller, "last_saved_paths", None) or [path]
        self.Reload()
        self._status.setText("Saved %d regions to %s"
                             % (len(self._controller.model.Rows()),
                                ", ".join(os.path.basename(p)
                                          for p in paths)))

    def closeEvent(self, event):
        # THE MODE OUTLIVES THE WINDOW, deliberately, and this used to be
        # the other way round. The old rule was that closing the window
        # turned the mode off, on the grounds that a mode with no visible
        # control which still eats every click is the worst version of
        # this feature. The animator's answer is that the toggle IS the
        # control: highlighting, clicking, dragging and moving controls
        # in the viewport should keep working for as long as TouchPose is
        # set active in the editor, panel open or not. So the toggle
        # alone decides, and closing the window changes nothing.
        #
        # VOICE IS NOT INCLUDED and the old rule still holds for it: it
        # owns a live microphone, an application-wide event filter and a
        # subprocess, none of which may outlive the window that started
        # them. None of the three is viewport interaction.
        if self._voice is not None:
            self._voice.SetEnabled(False)
            self._SetChecked(self._voiceToggle, False)
        super(TouchPosePanel, self).closeEvent(event)


def OpenTouchPosePanel(usdviewApi):
    panel = TouchPosePanel.GetInstance(usdviewApi)
    panel.Reload()
    panel.show()
    panel.raise_()
    return panel
