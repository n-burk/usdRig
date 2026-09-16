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

from pxr import Gf, Sdf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

if __name__ != "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import touchPoseModel


MESH = "/Biped/Geom/body_geo"

# What the highlight is drawn at when the touch layer does not say. The
# authored file says 0.478 (`touchpose:alpha`), drawn over an UNSHADED
# viewport; over a shaded body a neutral grey selection at 0.478 stops
# reading, so the panel opens here and its slider moves it live.
HIGHLIGHT_OPACITY = 0.85

# Prims TouchPose used to author at the stage root, kept only so a session
# saved by the old version does not leave one of them selected.
_LEGACY_OVERLAYS = ("/TouchPoseHighlight", "/TouchPoseSelected",
                    "/TouchPoseLead", "/TouchPoseRegions")

# A few pixels of slop before a press counts as a marquee. Without it
# every click is a one-pixel band and the click path never runs at all.
DRAG_SLOP = 3.0

# How often the gizmo is asked whether a drag is in flight.
DRAG_POLL_MS = 33

# The floor between two hover evaluations, in milliseconds: one 60 Hz
# frame, the fastest the highlight can be seen to change.
HOVER_MIN_MS = 16

# The three selection modes, shared verbatim with the Control Picker.
MODE_REPLACE = "replace"
MODE_TOGGLE = "toggle"
MODE_REMOVE = "remove"


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


def GizmoDragging():
    """True while the viewport gizmo has a drag in flight.

    Read-only through `gizmoUI`'s public surface, and any failure answers
    False so a session without the viewport tools behaves as it always did.
    """
    try:
        import gizmoUI
        controller = gizmoUI.GetController()
        return controller is not None and controller.IsDragging()
    except Exception:
        return False


def GizmoOwns(x, y, ratio=1.0):
    """True when the viewport gizmo would take a press at this pixel.

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
        controller = gizmoUI.GetController()
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
        """Stand the hover down for a gizmo drag. The selection stays."""
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
        changed = self._native.SetHighlightState(
            None if self.suspended else self.hover, self.lead, self.selected,
            self.editing, self.opacity, colors["lead"], colors["selected"])
        if changed:
            self.flushes += 1
        return changed


class TouchPoseController(QtCore.QObject):
    """The mode: an event filter over the stage view, plus the highlight."""

    _instance = None

    statusChanged = QtCore.Signal(str)
    strokeFinished = QtCore.Signal()

    @classmethod
    def GetInstance(cls, usdviewApi):
        if cls._instance is None or cls._instance._api is not usdviewApi:
            if cls._instance is not None:
                cls._instance.SetActive(False)
            cls._instance = cls(usdviewApi)
        return cls._instance

    def __init__(self, usdviewApi, mesh_path=MESH):
        super(TouchPoseController, self).__init__()
        self._api = usdviewApi
        self._mesh_path = mesh_path
        self._view = StageView(usdviewApi)
        self._active = False
        self._installed = False
        self._model = None
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
        return self._model

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

    def Load(self):
        """Read the touch regions off the stage. Returns a status line.

        Reloading while the mode is ON re-attaches the highlight to the new
        model; the old one released its native mesh, and with it its
        highlight, first. (This used to build a fresh overlay canvas beside
        the old one and leave the old one lit.)
        """
        stage = self._Stage()
        if stage is None:
            self._error = "no stage"
            return self._error
        wasActive = self._active
        self._ReleaseModel()
        try:
            self._model = touchPoseModel.TouchModel.FromStage(
                stage, self._mesh_path)
        except Exception as exc:
            self._model = None
            self._error = str(exc)
            return ("No touch regions on this stage (%s). Import them with "
                    "`bin\\run_touchpose.bat import_touch <rig>.usda` and "
                    "open <rig>_touch.usda." % exc)
        self._opacity = HIGHLIGHT_OPACITY
        self._highlight = Highlight(self._model, self._opacity)
        self._model.SyncPose(self._Time(), force=True)
        if wasActive:
            self._highlight.Enable(True)
            self._highlight.SetEditing(self._paint)
            self.SyncSelection()
        regions, covered, faces = self._model.Coverage()
        self._error = None
        return ("%d regions cover %d of %d faces (%.0f%%)"
                % (regions, covered, faces, 100.0 * covered / max(faces, 1)))

    def _ReleaseModel(self):
        if self._highlight is not None:
            self._highlight.Enable(False)
        if self._model is not None:
            self._model.Close()
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
            # a colour-table upload.
            self._highlight.Enable(True)
            self._highlight.SetEditing(self._paint)
            self.SyncSelection()
            self._dragPoll.start()
        else:
            self._dragPoll.stop()
            self._hoverTimer.stop()
            self._pendingHover = None
            self._Uninstall()
            self._Unsubscribe()
            self._pressAt = None
            self._band = None
            self._suspended = False
            self._HideBand()
            if self._highlight is not None:
                self._highlight.Clear()
                self._highlight.Enable(False)
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
        """How much of the body shows through the highlight. Live."""
        self._opacity = max(0.0, min(1.0, float(opacity)))
        if self._highlight is not None and self._highlight.SetOpacity(
                self._opacity):
            self._Redraw()
        # Written back onto the MODEL so Save persists it as
        # `touchpose:alpha`.
        if self._model is not None:
            self._model.alpha = self._opacity
        return self._opacity

    def SetStateColor(self, state, colour):
        """Change the lead or selected colour and repaint at once."""
        if self._model is None:
            return None
        colour = tuple(float(c) for c in colour)
        if state == "lead":
            self._model.lead_color = colour
        else:
            self._model.selected_color = colour
        if self._highlight is not None and self._highlight.Flush():
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
        """
        selection = self._Selection()
        stage = self._Stage()
        if selection is None or stage is None:
            return False
        paths = [self._mesh_path] + list(_LEGACY_OVERLAYS)
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
            lead = next((r for r in self._model.RegionsFor([paths[-1]])),
                        None)
        self._lead = lead
        chosen = self._model.RegionsFor(paths)
        others = [r for r in chosen
                  if lead is None or r.index != lead.index]
        changed = self._highlight.SetSelection(lead, others)
        if changed:
            self._Redraw()
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
        if self._model is None:
            return None, -1
        ray = self.RayAt(x, y)
        if ray is None:
            return None, -1
        face, _t = self._model.Cast(ray[0], ray[1])
        return self._model.RegionOfFace(face), face

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
        if self._highlight is not None and self._active:
            if self._highlight.SetEditing(self._paint):
                self._Redraw()
        return self._paint

    def HoverColorFor(self, region):
        """The hover colour for the mode TouchPose is in."""
        if self._model is None:
            return None
        return (self._model.EditColor(region) if self._paint
                else self._model.HoverColor(region))

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
        """Write the regions back to their own layer. Never the rig.

        Defaults to the layer the regions were READ from, which is the
        touch layer of the open stage -- so Save on a stage opened as
        `Biped_all.usda` rewrites `Biped_touch_regions.usda` and leaves
        the rig alone.
        """
        from touchpose import usdexport

        if self._model is None:
            return None
        path = path or self.RegionLayerPath()
        if not path:
            return None
        rows = self._model.Rows()
        usdexport.save_regions(
            path, self._mesh_path, rows,
            palette=self._model.palette, alpha=self._model.alpha,
            lead=self._model.lead_color, selected=self._model.selected_color,
            scope_path=getattr(self._model, "scope_path", None))
        self._model.dirty = False
        return path

    def RegionLayerPath(self):
        """The layer the region prims were authored in, if there is one.

        Found by asking USD which layer holds the strongest opinion on a
        region's face list, rather than by guessing at a filename.
        """
        stage = self._Stage()
        if stage is None or self._model is None or not self._model.regions:
            return None
        # The scope the model was READ from, then any scope annotating THIS
        # mesh. (This asked the model for `mesh`, which it never had, so the
        # filter was always None and the first scope on the stage won --
        # the wrong layer as soon as a shot holds two characters.)
        scope_path = getattr(self._model, "scope_path", None)
        scope = stage.GetPrimAtPath(Sdf.Path(scope_path)) if scope_path \
            else None
        if not scope or not scope.IsValid():
            scopes = touchPoseModel.FindRegionScopes(
                stage, self._model.mesh_path)
            scope = scopes[0] if scopes else None
        if scope is None:
            return None
        prim = stage.GetPrimAtPath(
            scope.GetPath().AppendChild(self._model.regions[0].name))
        attr = None
        for name in (touchPoseModel.TYPED_FACES, touchPoseModel.FACES_ATTR):
            candidate = prim.GetAttribute(name) if prim else None
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

    # -- the loop --------------------------------------------------------

    def SyncPose(self, force=False):
        """Follow the pose the viewport draws. True when the mesh moved.

        Native, and cheap when nothing moved (a pointer compare against the
        RigExec snapshot), so it runs per hover rather than on a timer. The
        highlight needs nothing here at all: it is drawn by the mesh's own
        shader, so it moves with the skin by construction.
        """
        if self._model is None:
            return False
        return self._model.SyncPose(self._Time(), force=force)

    @property
    def suspended(self):
        """True while a gizmo drag has TouchPose stood down."""
        return self._suspended

    def _PollDrag(self):
        """Stand the HOVER down for the length of a gizmo drag.

        The selection highlight stays lit: it is part of the body's shader,
        so it follows the pose being dragged at no cost.
        """
        if not self._active or self._model is None:
            return
        dragging = GizmoDragging()
        if dragging == self._suspended:
            return
        self._suspended = dragging
        if dragging:
            self._hover = None
            # A hover the rate limit held back must not land on the other
            # side of the suspend and light the hover mid-drag.
            self._hoverTimer.stop()
            self._pendingHover = None
            if self._highlight is not None and self._highlight.Suspend():
                self._Redraw()
            return
        self.SyncPose(force=True)
        if self._highlight is not None:
            self._highlight.Clear()
            if self._highlight.Resume():
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
        """Light the region under a physical pixel. Returns it, or None."""
        self.SyncPose()
        region, _face = self.RegionAt(x, y)
        if self._highlight is not None and self._highlight.SetHover(region):
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
        controller = getattr(gizmoUI, "_controller", None)
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
        """Regions the marquee touches, front-facing only."""
        if self._model is None or self._view is None:
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
        caught = self._model.RegionsInRect(
            matrix, width, height, (eye[0], eye[1], eye[2]), x0, y0, x1, y1)

        # A band catches a region when one of its face CENTROIDS is
        # inside, which is the right test at any useful size and the
        # wrong one when the band is a few pixels across: the centroids
        # of a limb are further apart than that, so a small band lands
        # between them and catches nothing at all. The band's centre is
        # therefore cast as well -- a tiny marquee then behaves exactly
        # like the click it visually is, which is what stops a slightly
        # shaky click from clearing the selection.
        middle, _face = self.RegionAt((x0 + x1) * 0.5, (y0 + y1) * 0.5)
        if middle is not None and not any(r.index == middle.index
                                          for r in caught):
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
            if GizmoDragging():
                # Stand down now rather than on the next poll tick, so the
                # first hover of a drag already finds TouchPose suspended.
                self._PollDrag()
                return False
            x, y = self._Position(event)
            if GizmoOwns(x, y, self._Ratio()):
                # The gizmo is drawing its own pre-selection highlight on
                # this pixel; lighting a region behind it as well reads
                # as two things being aimed at.
                if self._highlight is not None and self._highlight.Clear():
                    self._Redraw()
                self._hover = None
                return False
            self._HoverSoon(x, y)
            return False        # never consume a hover: the camera reads them
        if kind in (QtCore.QEvent.Leave, QtCore.QEvent.HoverLeave):
            self._hover = None
            self._hoverTimer.stop()
            self._pendingHover = None
            if self._highlight is not None and self._highlight.Clear():
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
            if GizmoOwns(x, y, self._Ratio()):
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

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi):
        if cls._instance is None:
            cls._instance = cls(usdviewApi)
        else:
            cls._instance._api = usdviewApi
        return cls._instance

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

        self._hoverLabel = QtWidgets.QLabel("")
        font = self._hoverLabel.font()
        font.setBold(True)
        self._hoverLabel.setFont(font)
        layout.addWidget(self._hoverLabel)

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
            "the conventional kLeadSelected; the studio's file makes it green.")
        self._leadSwatch.clicked.connect(lambda: self._OnColor("lead"))
        look.addWidget(self._leadSwatch)
        self._selectedSwatch = QtWidgets.QPushButton("Selected")
        self._selectedSwatch.setToolTip(
            "The colour of every other selected region. the conventional kSelected; "
            "the studio's file makes it a neutral grey.")
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
        """Put the slider and the two swatches back on the model.

        Driven off the MODEL, not off the widgets: `Load` re-reads the
        touch layer, and the panel must show what the layer says rather
        than what the last session left in the widget.
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
        for region in sorted(model.regions, key=lambda r: r.label):
            item = QtWidgets.QListWidgetItem(
                "%-28s %5d faces" % (region.label, len(region.faces)))
            item.setData(QtCore.Qt.UserRole, region.index)
            item.setForeground(QtGui.QBrush(QtGui.QColor(
                *[int(round(c * 255))
                  for c in model.EditColor(region)])))
            self._list.addItem(item)

    def Reload(self):
        self._status.setText(self._controller.Load() or "")
        self._list.clear()
        model = self._controller.model
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
        self.Reload()
        self._status.setText("Saved %d regions to %s"
                             % (len(self._controller.model.Rows()), path))

    def closeEvent(self, event):
        # Closing the window turns the mode off. A mode whose only
        # visible control has been dismissed but which still eats every
        # click on the character is the worst version of this feature.
        self._controller.SetActive(False)
        self._toggle.setChecked(False)
        super(TouchPosePanel, self).closeEvent(event)


def OpenTouchPosePanel(usdviewApi):
    panel = TouchPosePanel.GetInstance(usdviewApi)
    panel.Reload()
    panel.show()
    panel.raise_()
    return panel
