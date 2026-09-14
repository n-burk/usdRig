"""TouchPose: touch the character, select the control that moves it.

Hover the skin and the painted region under the cursor lights up; click
and that region's control becomes usdview's selection -- which is the
whole trick, and the same one the Control Picker uses: selecting through
`dataModel.selection` means the Avar Editor and the viewport gizmo follow
for free and this panel never has to know what an avar is.

Everything with a right answer lives in `touchPoseModel` and is tested
headlessly. This file is the mouse, the menu and the session layer.

THREE THINGS HERE THAT A MEASUREMENT DECIDED, not a preference:

  * THE HIGHLIGHT IS A SECOND MESH, not a `displayColor` on `body_geo`.
    The obvious route is the one RigExec's volume-weight overlay already
    uses, and it is dead on this asset: `body_geo` carries five
    `materialBind` subsets bound to `UsdPreviewSurface`, Storm shades
    from `diffuseColor`, and an authored `displayColor` moved 0 of 80,730
    sampled pixels -- while the same colour with the material bindings
    blocked moved 7,383 (9.1%). The primvar reaches the terminal scene
    index either way, so it is shading, not plumbing. A NEW mesh prim has
    no material binding at all, so Storm's fallback material applies to
    it, and that one does read `displayColor`.

    It is ONE second mesh, resident, carrying every painted face -- see
    `OverlayCanvas`. It used to be four, each re-authoring its own
    points, topology, extent and `visibility` on every region crossing,
    and the animator could see the Hydra resync behind that as a flicker
    under the cursor. Now the geometry is authored once when the mode is
    enabled and a hover writes ONE per-face array; "unlit" is per-face
    `primvars:displayOpacity` = 0 rather than an invisible prim.

    THE FLOOR IS ONE AUTHORED EDIT, and it is not the array that costs.
    Measured on this stage: a 1-element array Set on the overlay prim
    costs 3.8 ms and a 16,739-element one costs 4.0 ms, and a second Set
    while the prim is still dirty costs 0.15 ms. So the toll is the
    change notice, paid once however much is written -- which is why the
    design is "author once, then one array" and not "author less".

    It stays a second mesh even now that the shader route is known to
    exist. Spike R5 built it -- a `UsdPrimvarReader_float3` spliced into
    the bound material's `diffuseColor`, reading a per-face tint -- and
    it renders exactly right: 53 of 80,730 pixels from the untouched
    asset with the tint set to the original colours, which is the frame's
    own noise. It costs 164 ms per region crossing against the overlay's
    8.9 ms, because the tint has to be authored ON `body_geo`, which is
    inside the rig's read roots; the identical array authored on a
    root-level prim costs 2.0 ms. The lift the animator was seeing was
    fixed instead, and is now 5.5x to 15x smaller (spike R6).

  * THE PATCHES ARE DRAWN IN TWO DIFFERENT COLOUR SETS, because the
    `.touch` file carries two. Control mode lights ONE region and uses
    the six-colour `touchpose:palette` the file indexes per region;
    paint mode draws EVERY region at once and uses their own
    `touchpose:color`, which the studio's editor randomises precisely so
    that neighbours can be told apart. See `TouchModel.HoverColor`.

  * HOVER USES `WA_Hover`, NOT `setMouseTracking`. Turning mouse tracking
    on for usdview's stage view also turns on its own GPU `pickObject`
    per mouse move (stageView.mouseMoveEvent's "none" camera mode).
    WA_Hover delivers button-less moves as HoverMove events, which
    nothing else in usdview listens for. Taken from `gizmoUI`, which hit
    this first.

  * THE CPU POINT COPY REFRESHES ON THE RIG'S GENERATION COUNTER, not per
    hover. Reading the deformed points out of `HydraObserver` is 0.10 ms
    but rebuilding what the cast needs from them is 6.13 ms, and nothing
    between two poses changes them. `RigExecImaging_GetGeneration` is the
    signal, polled where a hover would otherwise pay for it.

PAINTING (phase 2) IS THE SAME LOOP, WRITING INSTEAD OF READING. With
paint on, Ctrl-drag gives the faces under the brush to the region the
panel has selected and Ctrl-Shift-drag takes them away again. Two things
make that safe to do on a drag:

  * it edits the MODEL, not the stage. Nothing is authored until Save,
    so a stroke costs a brush test and an overlay rebuild, both of which
    the hover path already pays for;
  * Save writes the TOUCH layer beside the rig, never the rig itself.
    An authored edit on any prim inside the rig's read roots makes
    OpenExec uncompile and recompile the network -- ~2.33 s on the next
    evaluate -- and a paint tool that paid that per stroke would be
    unusable. The test asserts the rig's generation counter does not move
    across a stroke, which is the direct evidence that it does not.

TOUCHPOSE STANDS DOWN WHILE A GIZMO DRAG IS IN FLIGHT. Not just the
click -- the whole loop: no cast, no hover evaluation, no overlay
authoring. A hover that crosses a region boundary costs a ray cast plus
about 7 ms of session-layer authoring and the Hydra resync behind it, and
paying that on top of a live manipulation buys nothing anyone can see:
the animator is watching the thing they are dragging, not the region
under the cursor. The hover patch is cleared ONCE on entering the drag
rather than per sample, and the SELECTION patches are left alone -- they
are static geometry that is not being recomputed, and blinking the
selection off mid-drag would read as a bug.

SELECTION SEMANTICS ARE THE CONTROL PICKER'S, to the letter, because the
same rig is being selected and it must not matter which tool the animator
reached for. No modifier REPLACES, Shift TOGGLES (the same gesture adds a
control that is not selected and removes one that is, so a selection can
be built up and trimmed back down without changing hands), and Ctrl
REMOVES whatever it touches -- exactly one gesture that can never add.
A drag marquees, catching every region the band TOUCHES rather than only
the ones it encloses, and the three rules apply to it unchanged. Click
and marquee share one code path (`Pick`) with the mode decided in one
place (`ModeFor`), which is the only way the two stay in step.

WHY THE CLICK IS SWALLOWED. While TouchPose is on, a click anywhere on
the character belongs to TouchPose: on a region it selects that region's
control, and OFF every region it still selects nothing rather than
letting usdview pick `/Biped/Geom/body_geo`. Selecting the skin is never
what the animator meant while this mode is on, and a mode that silently
lets the mesh through is worse than one that does nothing. A click that
misses the mesh ENTIRELY returns False untouched, so the camera, the
gizmo and usdview's own picking all still work -- and turning TouchPose
off puts every click back exactly where it was.
"""
import ctypes
import os
import sys

import numpy
from pxr import Gf, Sdf, Usd, UsdGeom, Vt
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

if __name__ != "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import touchPoseModel


MESH = "/Biped/Geom/body_geo"

# The overlay prim SHIPS WITH THE ASSET, under its TouchPose scope, and
# this is only the fallback for a stage whose regions predate that. It
# was the one and only path once, which made it a global: two assets in
# a shot would author into the same prim and fight over it.
#
# Session-only either way: the runtime writes points and opacity into the
# session layer, so nothing the highlight does is ever saved into the
# asset.
HIGHLIGHT = "/TouchPoseHighlight"

# WHERE THE OVERLAY IS DRAWN, and why it is not yet where it belongs.
#
# The asset ships its own overlay prim, `<rig>/TouchPose/Overlay`, typed
# RigExecTouchOverlay, and rigExecImaging registers the UsdImaging
# adapter that makes a custom type draw at all. That path is complete and
# tested: flip this to True and the highlight lights from inside the
# asset, one per character, with no runtime prim creation anywhere.
#
# It is False because of a cliff that has nothing to do with TouchPose.
# Authoring ONE token into a session layer costs, measured in the app on
# the biped with everything else held constant:
#
#     /Probe_root                       stage root          1.58 ms
#     /Biped/Probe_asset                under the asset   161.36 ms
#     /Biped/Rig/Probe_rig              under the rig     163.53 ms
#     /Biped/Rig/TouchPose/Probe_touch  in the scope      165.24 ms
#     /Biped/Geom/Probe_geom            beside the mesh   173.10 ms
#
# A single token, not an array: the cost is the notice, not the data, and
# it applies to any authored change anywhere under the asset. A hover
# that crosses a region writes two of them, which is the 193 ms an
# animator would feel against a 16.7 ms frame. At the stage root the same
# hover is 6.66 ms.
#
# So the overlay stays at the root until that is fixed, named for its
# asset rather than global, which is the half that was a correctness bug
# (two characters in a shot sharing one prim). Fix the cliff, set this
# True, and the prim is already shipped and waiting.
OVERLAY_IN_HIERARCHY = False
SELECTED = "/TouchPoseSelected"
LEAD = "/TouchPoseLead"
ALL_REGIONS = "/TouchPoseRegions"

# What the patches are drawn at when the touch layer does not say. The
# studio's own file says 0.478 (`touchpose:alpha`), which is what the
# the conventional tool shape drew them at; this is the fallback, and the panel's slider
# moves it live either way.
HIGHLIGHT_OPACITY = 0.85

# Four STATES, composited in this order into the one prim at HIGHLIGHT
# (`OverlayCanvas`); the other three names survive as the channel names
# and as paths `_GuardMesh` still sweeps out of the selection, because a
# session that was open before this change can have the old prims.
#
# NONE of them under /Biped/Rig. That is not tidiness: an authored edit
# on any prim inside the rig's read roots makes OpenExec uncompile and
# recompile the network, which was measured at ~2.33 s on the next
# evaluate. A selection highlight that tinted the CONTROL would pay that
# on every single click. Outside the rig, RigExec never sees the change
# and the click stays instant.
_LAYERS = (ALL_REGIONS, SELECTED, LEAD, HIGHLIGHT)

# A few pixels of slop before a press counts as a marquee. Without it
# every click is a one-pixel band and the click path never runs at all.
DRAG_SLOP = 3.0

# How often the gizmo is asked whether a drag is in flight. Short enough
# that the patches are gone before the first re-posed frame is drawn, and
# the cost is one attribute read -- `GizmoController.IsDragging` is
# `self._drag is not None` -- so there is no reason to make it longer.
DRAG_POLL_MS = 33

# The floor between two hover evaluations, in milliseconds. One frame at
# 60 Hz, because that is the fastest the highlight can possibly be seen
# to change, and a hover costs 2.6 ms of ray cast (spike R7) against a
# mouse that reports at 125 Hz or more.
HOVER_MIN_MS = 16

# The three selection modes, shared verbatim with the Control Picker.
MODE_REPLACE = "replace"
MODE_TOGGLE = "toggle"
MODE_REMOVE = "remove"


def StageView(usdviewApi):
    """usdview's stage view widget, or None headless.

    The private-name mangling is usdview's own (UsdviewApi keeps the app
    controller as __appController); `gizmoUI.StageView` and
    `curvenetUI.SurfacePicker` reach it the same way.
    """
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


def _Vec3fArray(points):
    """numpy Nx3 -> Vt.Vec3fArray, whichever conversion this USD has."""
    try:
        return Vt.Vec3fArray.FromNumpy(points)
    except AttributeError:
        return Vt.Vec3fArray([Gf.Vec3f(*row) for row in points.tolist()])


def _IntArray(values):
    try:
        return Vt.IntArray.FromNumpy(values)
    except AttributeError:
        return Vt.IntArray(values.tolist())


def _FloatArray(values):
    try:
        return Vt.FloatArray.FromNumpy(values)
    except AttributeError:
        return Vt.FloatArray(values.tolist())


class _NullContext(object):
    """Stand-in for `selection.batchPrimChanges` on a usdview without it.

    The batch exists so a multi-region marquee emits one selection
    change rather than one per region; an older build simply emits
    several, which is slower and not wrong.
    """

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def GizmoDragging():
    """True while the viewport gizmo has a drag in flight.

    Split out from `GizmoOwns` because it is asked a different question
    at a different moment: `GizmoOwns` decides who gets a PRESS at a
    pixel, this decides whether TouchPose should be doing anything at
    all. Read-only through `gizmoUI`'s public surface, and any failure
    answers False so a session without the viewport tools behaves exactly
    as it always did.
    """
    try:
        import gizmoUI
        controller = gizmoUI.GetController()
        return controller is not None and controller.IsDragging()
    except Exception:
        return False


def GizmoOwns(x, y, ratio=1.0):
    """True when the viewport gizmo would take a press at this pixel.

    READ-ONLY, and through gizmoUI's public surface only: `GetController`,
    `IsVisible`, `Tool`, `Handles`, `IsDragging`, plus `gizmoScreen
    .HitTest` and the same `HIT_PIXELS` radius the gizmo itself uses. The
    conditions mirror `GizmoController._OnPress` (gizmoUI.py:2152-2168) --
    if that changes, this has to follow, which is why it is one function
    and not a condition sprinkled through the filter.

    WHY TOUCHPOSE DEFERS RATHER THAN COMPETING. Qt hands a press to the
    most recently installed event filter first, so without this the
    winner is whichever panel the animator happened to open last --
    arbitrary, and different from one session to the next. The gizmo is
    the thing being aimed at: its handles are small, deliberate targets
    drawn on top, and a region is the whole limb behind them. Dragging a
    handle and having the limb's control get re-selected under you is a
    bug in a way that "the gizmo took a click that was also over a
    region" is not.

    Any failure answers False: TouchPose then behaves exactly as it does
    with no gizmo installed, which is the safe direction.
    """
    try:
        import gizmoUI
        import gizmoScreen
    except ImportError:
        return False              # no viewport tools in this session
    try:
        controller = gizmoUI.GetController()
        if controller is None or not controller.IsVisible():
            return False
        if controller.IsDragging():
            return True           # a drag in flight owns every event
        if controller.Tool() == gizmoUI.TOOL_SELECT:
            return False          # the select tool draws no handles
        handles = controller.Handles()
        if not handles:
            return False
        # `x`, `y` and the radius are all in PHYSICAL pixels, which is
        # what the gizmo projects its handles into; the caller has the
        # device ratio already and hands it over rather than this
        # reaching back through the controller for it.
        return gizmoScreen.HitTest(handles, x, y,
                                   gizmoUI.HIT_PIXELS * ratio) is not None
    except Exception:
        return False


def _Imaging():
    """rigExecImaging, bound for the one call TouchPose makes, or None.

    Only the generation counter is wanted, and a build without the
    library is a session with no live evaluation at all -- in which case
    the rest mesh IS the posed mesh and polling nothing is correct.
    """
    try:
        from rigExecUsdview import ImagingLibraryPath
        lib = ctypes.CDLL(ImagingLibraryPath())
        lib.RigExecImaging_GetGeneration.restype = ctypes.c_longlong
        return lib
    except Exception:
        return None


class _Points(object):
    """The deformed points of one mesh, kept fresh off the generation.

    `HydraObserver` is the only public way to the points RigExec actually
    posed: they live in Hydra, and the stage's own `points` attribute is
    the rest mesh.
    """

    def __init__(self, mesh_path=MESH):
        self.mesh_path = mesh_path
        self._observer = None
        self._lib = _Imaging()
        self._generation = None
        self.source = "none"

    def _Observer(self):
        if self._observer is not None:
            return self._observer
        try:
            from pxr.Usdviewq._usdviewq import HydraObserver
            names = HydraObserver.GetRegisteredSceneIndexNames()
            if not names:
                return None
            observer = HydraObserver()
            observer.TargetToNamedSceneIndex(names[-1])
            self._observer = observer
        except Exception:
            return None
        return self._observer

    def Generation(self):
        if self._lib is None:
            return None
        try:
            return int(self._lib.RigExecImaging_GetGeneration())
        except Exception:
            return None

    def Read(self):
        """The live points, or None when Hydra cannot supply them."""
        observer = self._Observer()
        if observer is None:
            return None
        try:
            _type, source = observer.GetPrim(Sdf.Path(self.mesh_path))
            if not source or "primvars" not in source.GetNames():
                return None
            entry = source.Get("primvars").Get("points")
            value = entry.Get("primvarValue") if entry else None
            if not value:
                return None
            self.source = "hydra"
            return value.GetValue(0.0)
        except Exception:
            return None

    def Sync(self, model, force=False):
        """Re-point `model` if the rig moved. True when it did."""
        generation = self.Generation()
        if not force and generation is not None \
                and generation == self._generation:
            return False
        points = self.Read()
        if points is None:
            return False
        model.SetPoints(points)
        self._generation = generation
        return True


class OverlayCanvas(object):
    """ONE resident overlay rprim. Per hover, only an array moves.

    THE THING THAT MADE THIS NECESSARY: the previous design was one
    prim per logical layer, each authoring its own `points`,
    `faceVertexCounts`, `faceVertexIndices`, `extent` AND `visibility`
    every time the cursor crossed a region boundary. That is a Hydra
    topology resync plus a visibility toggle at mouse-move rate, and it
    measured 9.7 ms on a crossing against 2.7 ms inside a region -- 5.9
    ms of it in `Show` alone. The flicker the animator was seeing was
    that resync landing between two frames.

    So the geometry is authored ONCE, for every painted face at once,
    and never touched again except when the pose moves. A hover writes
    exactly one array.

    "UNLIT" IS PER-FACE `primvars:displayOpacity` = 0, NOT INVISIBILITY.
    That is the whole trick, and it is what lets four logical layers
    share one prim: a face is lit in the hover colour, the lead colour,
    the selected colour or the edit colour, or it is not lit at all, and
    all of those answers live in the same two arrays.

    WHY NOT A SHADER ON `body_geo`. Spike R5 built exactly that -- a
    `UsdPrimvarReader_float3` spliced into the bound material's
    `diffuseColor` -- and it renders correctly and costs 164 ms per
    region crossing, because the tint has to be authored ON `body_geo`,
    which is inside the rig's read roots. The control measurement is the
    decisive one: the IDENTICAL array costs 2.0 ms on a root-level prim
    and 164.2 ms on `body_geo`. This prim is ours and sits outside the
    rig, so it is on the cheap side of that line.

    THE COLOURS ARE BAKED AT REST. `_resting` holds what every face
    would be drawn in if it were lit -- the hover set in control mode,
    the edit set in paint mode -- so hovering a region that is not
    selected changes the OPACITY array and nothing else. The colour
    array is only rewritten when it actually differs, which is what
    `Flush` compares before it enters an edit context at all.
    """

    def __init__(self, stage, path=HIGHLIGHT, opacity=HIGHLIGHT_OPACITY):
        self._stage = stage
        self._path = Sdf.Path(path)
        self._opacity = opacity
        self._mesh = None
        self._attrs = {}
        self._interpolation = {}
        self._channels = []
        self._faces = None          # the resident face list, sorted
        self._offsets = None        # per-face lift, from _Build
        self._editing = None
        self._resting = None        # (N, 3): what a lit face looks like
        self._color = None          # last written
        self._alpha = None          # last written
        self._visible = False
        self.point_count = 0

    # -- the prim --------------------------------------------------------

    def _Mesh(self):
        if self._mesh is not None and self._mesh.GetPrim().IsValid():
            return self._mesh
        with Usd.EditContext(self._stage, self._stage.GetSessionLayer()):
            # DEFINE ONLY WHAT IS NOT THERE. The asset ships its own
            # overlay prim, and defining over it in the session layer
            # would put a second, weaker-typed opinion on top of a prim
            # that is already exactly right.
            existing = self._stage.GetPrimAtPath(self._path)
            if existing and existing.IsValid():
                mesh = UsdGeom.Mesh(existing)
            else:
                mesh = UsdGeom.Mesh.Define(self._stage, self._path)
            prim = mesh.GetPrim()
            # No material binding is the POINT, not an omission: an
            # unbound mesh gets Storm's fallback material, which is the
            # only surface in this scene that reads `displayColor` --
            # and `displayOpacity` with it.
            mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            # Double-sided because the patch is a slice of a closed body:
            # without it the far half of a limb's region vanishes as the
            # limb turns, which reads as the highlight flickering.
            mesh.CreateDoubleSidedAttr(True)
            self._attrs = {
                "points": mesh.CreatePointsAttr(),
                "counts": mesh.CreateFaceVertexCountsAttr(),
                "indices": mesh.CreateFaceVertexIndicesAttr(),
                "color": mesh.CreateDisplayColorAttr(),
                "opacity": prim.CreateAttribute(
                    "primvars:displayOpacity",
                    Sdf.ValueTypeNames.FloatArray),
                "extent": mesh.CreateExtentAttr(),
                "visibility": mesh.CreateVisibilityAttr(),
            }
            self._interpolation = {}
        self._mesh = mesh
        return mesh

    def Register(self, channel):
        """Channels are composited in registration order; last wins."""
        self._channels.append(channel)

    @property
    def opacity(self):
        return self._opacity

    @property
    def faces(self):
        return self._faces

    @property
    def visible(self):
        return self._visible

    def SetOpacity(self, opacity):
        """How much of the body shows through a LIT face. Takes effect now.

        One array, and never the geometry: re-authoring the patch to
        move a slider would rebuild it for nothing.
        """
        opacity = max(0.0, min(1.0, float(opacity)))
        if opacity == self._opacity:
            return False
        self._opacity = opacity
        self.Flush()
        return True

    # -- the resident geometry -------------------------------------------

    def Sync(self, model, editing=False):
        """Make the resident mesh match the model. Authors once.

        Called on enable, when paint mode flips, and after a brush dab.
        The geometry is re-authored ONLY when the painted face SET
        changes -- which a dab that moves faces between two regions does
        not do -- so the usual answer is a colour array and nothing else.
        """
        if model is None:
            return False
        faces = model.AllFaces()
        built = False
        if (self._faces is None or self._mesh is None
                or not self._mesh.GetPrim().IsValid()
                or not numpy.array_equal(self._faces, faces)):
            built = self._Build(model, faces)
        if built or editing != self._editing or self._resting is None:
            self._editing = editing
            self._resting = numpy.ascontiguousarray(
                model.FaceColors(faces, editing=editing),
                dtype=numpy.float32)
        self.Flush()
        return built

    def _Build(self, model, faces):
        self._faces = faces
        for channel in self._channels:
            channel.Forget()
        self._color = None
        self._alpha = None
        if not len(faces):
            self._offsets = None
            self._resting = numpy.zeros((0, 3), numpy.float32)
            return True
        # The lift travels with the FACE, so the resident patch keeps
        # exactly the per-region offset the separate patches had -- see
        # `TouchModel.FaceOffsets`. Cached, because it moves by a
        # fraction of a percent between two poses and re-deriving it on
        # every drag resume would be one extent computation per region
        # for nothing.
        self._offsets = model.FaceOffsets(faces)
        points, counts, indices = model.OverlayGeometry(
            faces, offset=self._offsets)
        self._Mesh()
        with Usd.EditContext(self._stage, self._stage.GetSessionLayer()):
            self._attrs["points"].Set(_Vec3fArray(points))
            self._attrs["counts"].Set(_IntArray(counts))
            self._attrs["indices"].Set(_IntArray(indices))
            self._SetExtent(points)
        self.point_count = len(points)
        return True

    def Repoint(self, model):
        """The pose moved: re-author the points. NOT the topology.

        The old design re-authored topology with the points because two
        prims' worth of arrays measured the same either way. Here the
        patch is every painted face and it is resident, so the narrow
        version is the one that is free.
        """
        if self._faces is None or not len(self._faces):
            return False
        points, _counts, _indices = model.OverlayGeometry(
            self._faces, offset=self._offsets)
        self._Mesh()
        with Usd.EditContext(self._stage, self._stage.GetSessionLayer()):
            self._attrs["points"].Set(_Vec3fArray(points))
            self._SetExtent(points)
        self.point_count = len(points)
        return True

    def _SetExtent(self, points):
        # An extent is not optional: without one, Hydra culls the prim
        # against a default-constructed (empty) bound and it never draws,
        # which looks exactly like the displayColor failure this whole
        # design exists to avoid.
        lo = points.min(axis=0)
        hi = points.max(axis=0)
        self._attrs["extent"].Set(
            Vt.Vec3fArray([Gf.Vec3f(*lo.tolist()), Gf.Vec3f(*hi.tolist())]))

    def Teardown(self):
        """Stop drawing entirely. The mode was turned off."""
        for channel in self._channels:
            channel.Forget()
        self._faces = None
        self._resting = None
        self._color = None
        self._alpha = None
        if self._mesh is None or not self._mesh.GetPrim().IsValid():
            return False
        with Usd.EditContext(self._stage, self._stage.GetSessionLayer()):
            self._attrs["visibility"].Set(UsdGeom.Tokens.invisible)
        self._visible = False
        return True

    # -- the one write per hover -----------------------------------------

    def Rows(self, faces):
        """Which rows of the resident patch `faces` are, or None.

        The resident list is sorted (`AllFaces` is a `nonzero`), so this
        is a binary search and not a mask per face.
        """
        if self._faces is None or faces is None or not len(faces):
            return None
        if not len(self._faces):
            return None
        faces = numpy.asarray(faces, dtype=numpy.int32)
        rows = numpy.searchsorted(self._faces, faces)
        numpy.clip(rows, 0, len(self._faces) - 1, out=rows)
        # A region can hold a face the patch does not, in the window
        # between a paint edit and the Sync that follows it.
        return rows[self._faces[rows] == faces]

    def Flush(self, force=False):
        """Composite every channel into two arrays; write what MOVED.

        The colour array is compared before it is written, which is the
        point of baking `_resting`: a hover onto an unselected region
        lands on faces that already hold the hover colour, so the only
        thing that actually changes is the opacity. One array, no
        topology, no visibility.
        """
        if self._faces is None or self._resting is None:
            return False
        count = len(self._faces)
        color = self._resting.copy()
        alpha = numpy.zeros(count, numpy.float32)
        for channel in self._channels:
            rows = channel.Rows(self)
            if rows is None or not len(rows):
                continue
            colour = channel.colour
            if colour is not None:
                values = numpy.asarray(colour, dtype=numpy.float32)
                if values.ndim == 2:
                    color[rows] = values[:len(rows)]
                else:
                    color[rows] = values
            alpha[rows] = self._opacity
        lit = bool(alpha.any())
        wantColor = force or self._color is None or not numpy.array_equal(
            color, self._color)
        wantAlpha = force or self._alpha is None or not numpy.array_equal(
            alpha, self._alpha)
        wantVisible = lit != self._visible
        if not (wantColor or wantAlpha or wantVisible):
            return False
        self._Mesh()
        # ONE edit context around every write, and the attribute handles
        # cached by `_Mesh`: each `Create*Attr` re-resolves the property
        # and each `EditContext` costs a target swap, and this runs every
        # time the cursor crosses a region boundary.
        with Usd.EditContext(self._stage, self._stage.GetSessionLayer()):
            if wantColor:
                self._SetPrimvar("color", UsdGeom.Tokens.uniform,
                                 _Vec3fArray(color))
                self._color = color
            if wantAlpha:
                self._SetPrimvar("opacity", UsdGeom.Tokens.uniform,
                                 _FloatArray(alpha))
                self._alpha = alpha
            if wantVisible:
                self._attrs["visibility"].Set(
                    UsdGeom.Tokens.inherited if lit
                    else UsdGeom.Tokens.invisible)
                self._visible = lit
        return True

    def _SetPrimvar(self, name, interpolation, value):
        """Set a primvar, writing its interpolation only when it CHANGES.

        The interpolation is a metadata field on the primvar, and
        re-authoring it every hover would resync the prim's primvar
        descriptors for no reason.
        """
        attr = self._attrs[name]
        if self._interpolation.get(name) != interpolation:
            UsdGeom.Primvar(attr).SetInterpolation(interpolation)
            self._interpolation[name] = interpolation
        attr.Set(value)


class Highlight(object):
    """One STATE of the resident patch: hover, lead, selected, or all.

    Not a prim any more. It holds which faces are in this state and what
    colour they take, and asks the canvas to recomposite; the canvas
    decides what actually needs writing. Registration order is z-order:
    the last channel to claim a face wins it, which is how hovering a
    region that is already selected still shows the hover colour.

    `key` is whatever the caller uses to decide the state is unchanged
    (a region index for hover, a tuple of control paths for the
    selection layers); an equal key is a no-op and authors nothing,
    which is what keeps a mouse move that stays inside one region free.
    """

    def __init__(self, canvas, name, opacity=None):
        self._canvas = canvas
        self._name = name
        self._key = None
        self._faces = None
        self._color = None
        self._rows = None
        self._suspended = False
        canvas.Register(self)

    @property
    def key(self):
        return self._key

    @property
    def name(self):
        return self._name

    @property
    def colour(self):
        return self._color

    @property
    def faces(self):
        return self._faces

    @property
    def suspended(self):
        """Stood down for a drag, with what was drawn still remembered."""
        return self._suspended

    @property
    def opacity(self):
        return self._canvas.opacity

    @property
    def point_count(self):
        return self._canvas.point_count

    def SetOpacity(self, opacity):
        return self._canvas.SetOpacity(opacity)

    def Rows(self, canvas):
        if self._suspended or self._faces is None:
            return None
        if self._rows is None:
            self._rows = canvas.Rows(self._faces)
        return self._rows

    def Forget(self):
        """The resident patch was rebuilt: the row cache is meaningless."""
        self._rows = None

    def Invalidate(self):
        """Forget the key so the next `Show` re-authors.

        For the things `Show` does not key on -- a colour the animator
        just changed in the panel is the same faces and the same region
        index, and would otherwise be a no-op.
        """
        self._key = None

    def Clear(self):
        """Stop drawing and forget what was drawn."""
        if self._key is None and self._faces is None:
            return
        self._key = None
        self._faces = None
        self._color = None
        self._rows = None
        self._suspended = False
        self._canvas.Flush()

    def Show(self, model, faces, colour, key, offset=None):
        """Light `faces` in `colour`. A no-op when `key` is unchanged.

        `colour` is one RGB triple, one PER FACE as an (N, 3) array, or
        None to take the canvas's resting colour -- which is what the
        all-regions state does, because while paint mode is on the
        resting colours ARE the edit colours. `offset` is accepted and
        ignored: the lift travels with the face now, see
        `TouchModel.FaceOffsets`.
        """
        if faces is None or not len(faces):
            self.Clear()
            return False
        if self._key is not None and key == self._key and not self._suspended:
            return False
        self._key = key
        self._faces = numpy.asarray(faces, dtype=numpy.int32)
        self._color = colour
        self._rows = None
        self._suspended = False
        self._canvas.Flush()
        return True

    def Recolor(self, colour, faceCount=None):
        """Change the colours and NOTHING else. A face changed owner."""
        if self._key is None:
            return False
        self._color = colour
        self._canvas.Flush()
        return True

    def Suspend(self):
        """Stop drawing, but REMEMBER what was drawn.

        `Clear` is the wrong tool for a drag: it drops the key, the
        faces and the colour, so there is nothing left to come back to.
        When every state is suspended the canvas has nothing lit and
        takes its own visibility down -- which is the one token a whole
        drag costs, where the old design paid a topology resync per
        sample.
        """
        if self._key is None or self._suspended:
            return False
        self._suspended = True
        self._canvas.Flush()
        return True

    def Resume(self, model):
        """Draw again, at the pose the body is in NOW.

        The points are the canvas's business and the controller has
        already re-pointed them; this only puts the state back.
        """
        if not self._suspended:
            return False
        self._suspended = False
        if self._key is None:
            return False
        self._canvas.Flush()
        return True

    def Rebuild(self, model):
        """The faces under this state may have moved. Recomposite.

        Kept because a brush dab moves faces between regions, and the
        row cache is what has to be dropped when that happens. There is
        no geometry to re-author any more, which is the whole point of
        the resident patch.
        """
        if self._key is None:
            return False
        self._rows = None
        self._canvas.Flush()
        return True


class TouchPoseController(QtCore.QObject):
    """The mode: an event filter over the stage view, plus the overlay."""

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
        self._points = _Points(mesh_path)
        self._canvas = None
        self._highlight = None
        self._selectedLayer = None
        self._leadLayer = None
        self._allLayer = None
        self._allFaces = None
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
        # the events that would stand in for one do not arrive: while a
        # button is held Qt sends MouseMove, not HoverMove -- which is
        # where the drag check used to live, so it never once fired during
        # an actual drag -- and the gizmo's own filter may consume the
        # MouseMoves before this one sees them, depending only on which
        # panel was opened last. A boolean read on a timer does not care
        # who wins the event, and `IsDragging` is an attribute test.
        self._dragPoll = QtCore.QTimer(self)
        self._dragPoll.setInterval(DRAG_POLL_MS)
        self._dragPoll.timeout.connect(self._PollDrag)
        # See `_HoverSoon`: the mouse reports far faster than the
        # highlight can usefully change, and the cast is 2.6 ms.
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

    def Load(self):
        """Read the touch regions off the stage. Returns a status line."""
        stage = self._Stage()
        if stage is None:
            self._error = "no stage"
            return self._error
        try:
            self._model = touchPoseModel.TouchModel.FromStage(
                stage, self._mesh_path)
        except Exception as exc:
            self._model = None
            self._error = str(exc)
            return ("No touch regions on this stage (%s). Import them with "
                    "`bin\\run_touchpose.bat import_touch <rig>.usda` and "
                    "open <rig>_touch.usda." % exc)
        # The patches open at HIGHLIGHT_OPACITY and NOT at the layer's own
        # `touchpose:alpha`, which is 0.478 in the studio's file. Tried
        # the other way round first and measured why not: at 0.478 a
        # second selected region moved 97 sampled pixels where
        # testUsdviewTouchPose's floor is 100, i.e. the non-lead selected
        # patch -- a neutral grey over a grey body -- stops reading. 0.478
        # is the number the the conventional tool shape drew over an UNSHADED viewport,
        # the same reason `StateColors` has to scale the file's colours
        # up here. The slider below moves it live, and Save writes
        # whatever it is set to back to `touchpose:alpha`.
        self._opacity = HIGHLIGHT_OPACITY
        # ONE rprim, four states. The channels are registered in DRAW
        # ORDER and the last one to claim a face wins it, which is the
        # z-order the four separate prims used to get from _LAYERS:
        # all-regions under the selection, the selection under the
        # hover. Hovering a region you have already selected should
        # still show the hover colour, because the hover is the thing
        # that answers "what will this click do".
        self._canvas = OverlayCanvas(stage, self._OverlayPath(stage),
                                     self._opacity)
        # Edit mode's every-region state. It is the only one that is not
        # about the cursor: it stays up for as long as paint is on, and
        # the hover has to keep working over the top of it.
        self._allLayer = Highlight(self._canvas, ALL_REGIONS)
        self._selectedLayer = Highlight(self._canvas, SELECTED)
        self._leadLayer = Highlight(self._canvas, LEAD)
        self._highlight = Highlight(self._canvas, HIGHLIGHT)
        self._points.Sync(self._model, force=True)
        regions, covered, faces = self._model.Coverage()
        self._error = None
        return ("%d regions cover %d of %d faces (%.0f%%); points from %s"
                % (regions, covered, faces,
                   100.0 * covered / max(faces, 1), self._points.source))

    def _OverlayPath(self, stage):
        """Where this asset's highlight surface lives.

        PER ASSET, at the stage root: `/TouchPoseHighlight_Biped`. It is
        not global -- three characters in a shot light three different
        prims and none of them fights over one -- and it is not inside
        the asset either, which is the part that is not where it should
        be yet.

        TWO MEASURED REASONS it is not under the asset's TouchPose scope,
        which is where it belongs and where it will go once either is
        lifted:

          * A SCHEMA TYPE OF ITS OWN DOES NOT DRAW. `RigExecTouchOverlay`
            inheriting Mesh composed perfectly -- valid prim, IsA(Mesh)
            true, visibility inherited, 17,466 points, opacity 0.85,
            correct extent -- and changed 0 of 80,730 sampled pixels.
            UsdImaging binds adapters by prim type and a concrete type
            with no registered adapter never becomes an rprim. The
            identical prim as a plain Mesh lit the region immediately.

          * INSIDE THE ASSET IT IS TOO SLOW. At
            /Biped/TouchPose/Overlay a hover crossing into another region
            cost 184.89 ms, of which 0.55 ms was building the patch and
            the rest was AUTHORING it; at the stage root the same hover
            is 3.99 ms, against a 16.7 ms frame. The write is inside what
            the evaluator treats as its read roots, and every one pays
            the invalidation.

        The second is the one worth fixing: the evaluator should be
        watching its own rig and the meshes it writes, not the whole
        asset namespace. Narrow that and the overlay can move home.
        """
        scope = getattr(self._model, "scope_path", None) if self._model             else None
        if not scope:
            return HIGHLIGHT
        path = Sdf.Path(scope).AppendChild("Overlay")
        prim = stage.GetPrimAtPath(path) if stage else None
        if OVERLAY_IN_HIERARCHY and prim and prim.IsValid():
            return str(path)
        # A stage whose regions predate the shipped overlay: fall back to
        # a root-level prim named for the asset, which is still one per
        # asset rather than the single global this started as.
        prefixes = Sdf.Path(scope).GetPrefixes()
        asset = prefixes[0].name if prefixes else ""
        return "%s_%s" % (HIGHLIGHT, asset) if asset else HIGHLIGHT

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
            # the moment the mode takes over rather than only on the next
            # change.
            self._active = True
            self._GuardMesh()
            # The resident patch is authored HERE, once, and not on the
            # first hover: everything after this is one array write.
            self._canvas.Sync(self._model, self._paint)
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
            for layer in self._Layers():
                layer.Clear()
            if self._canvas is not None:
                self._canvas.Teardown()
            self._allFaces = None
            self._lead = None
        self._active = active
        return self._active

    def _Layers(self):
        return [l for l in (self._allLayer, self._selectedLayer,
                            self._leadLayer, self._highlight)
                if l is not None]

    @property
    def canvas(self):
        return self._canvas

    # -- what the patches look like --------------------------------------

    @property
    def opacity(self):
        return self._opacity

    def SetOpacity(self, opacity):
        """How much of the body shows through every patch. Live."""
        self._opacity = max(0.0, min(1.0, float(opacity)))
        if self._canvas is not None:
            self._canvas.SetOpacity(self._opacity)
        # Written back onto the MODEL so Save persists it: the scope's
        # `touchpose:alpha` is where this number came from and where the
        # next session will look for it.
        if self._model is not None:
            self._model.alpha = self._opacity
        return self._opacity

    def SetStateColor(self, state, colour):
        """Change the lead or selected colour and repaint at once.

        The two patches are keyed on which regions they hold, so a
        colour change alone would be a no-op through `Show`; the keys
        are dropped to force the re-author.
        """
        if self._model is None:
            return None
        colour = tuple(float(c) for c in colour)
        if state == "lead":
            self._model.lead_color = colour
        else:
            self._model.selected_color = colour
        for layer in (self._selectedLayer, self._leadLayer):
            if layer is not None:
                layer.Invalidate()
        self.SyncSelection()
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
        """Keep the mesh and TouchPose's own scaffolding out of the
        selection while the mode is on.

        TWO THINGS GET REMOVED, for two different reasons.

        The MESH, because that is the rule: consuming the press covers
        the case TouchPose sees, and this covers every case it does not
        -- a press this filter declined because its own cast said "no
        face" while Storm's says otherwise, the outliner, a picker
        button. The rule is about the SELECTION, so it is enforced on the
        selection.

        The OVERLAY PRIMS, because they are pickable and nobody meant
        them. They are real rprims drawn on top of the body, so usdview's
        own Storm pick hits them in preference to the skin underneath --
        measured: a press that fell through during a gizmo drag selected
        `/TouchPoseLead`. There is no public way to make an rprim
        unpickable without also making it invisible, so it is undone here
        instead. Selecting a highlight patch is never what anyone meant.

        No authoring, and nothing under /Biped/Rig is touched -- this
        edits usdview's own selection, which costs no rig recompile.
        """
        selection = self._Selection()
        stage = self._Stage()
        if selection is None or stage is None:
            return False
        # The canvas's OWN path, not `_LAYERS`. Those four are channel
        # names composited into one prim, and only one of them was ever
        # also a prim path; now that the overlay is named for its asset,
        # none of them is, and the guard silently stopped guarding.
        paths = [self._mesh_path]
        if self._canvas is not None:
            paths.append(str(self._canvas._path))
        paths.extend(_LAYERS)
        unwanted = []
        for path in paths:
            if not path:
                continue
            prim = stage.GetPrimAtPath(Sdf.Path(path))
            if prim and prim.IsValid():
                unwanted.append(prim)
        held = set(p.GetPath() for p in selection.getPrims() if p)
        unwanted = [p for p in unwanted if p.GetPath() in held]
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
        """Repaint the lead and selected layers from usdview's selection.

        Driven off the SELECTION rather than off our own click, so a
        control chosen in the Control Picker, the outliner or the Avar
        Editor lights its region here too.
        """
        if self._model is None or self._selectedLayer is None:
            return False
        selection = self._Selection()
        paths = [str(p.GetPath()) for p in selection.getPrims()
                 if p and p.IsValid()] if selection is not None else []
        # usdview keeps the selection in the order it was made, so the
        # LAST entry is the most recent -- which is the lead. Not
        # `getFocusPrim`, which sounds right and is not: it returns
        # `getPrimPaths()[0]`, the FIRST prim selected, and the conventional lead
        # (which is what the animator means) is the last. A lead set
        # by our own click wins when it is still selected, because a
        # shift-click adds at the end and the animator means the one
        # they just touched.
        lead = self._lead if (self._lead and self._lead.control in paths) \
            else None
        if lead is None and paths:
            lead = next((r for r in self._model.RegionsFor([paths[-1]])),
                        None)
        self._lead = lead

        colors = self._model.StateColors()
        chosen = self._model.RegionsFor(paths)
        others = [r for r in chosen
                  if lead is None or r.index != lead.index]

        changed = self._selectedLayer.Show(
            self._model, self._model.FacesOf(others), colors["selected"],
            tuple(sorted(r.index for r in others)))
        changed = self._leadLayer.Show(
            self._model, lead.faces if lead is not None else None,
            colors["lead"],
            ("lead", lead.index) if lead is not None else None) or changed
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
        self._paint = bool(on)
        if not self._paint:
            self._painting = None
        self._ShowAllRegions()
        # The hover patch changes COLOUR SET with the mode -- edit mode
        # uses the region's own colour, control mode the palette -- and
        # the hover key is the region index either way, so without this
        # the patch keeps the colour of the mode it was drawn in.
        if self._highlight is not None:
            self._highlight.Invalidate()
        return self._paint

    def HoverColorFor(self, region):
        """The hover colour for the mode TouchPose is in.

        Edit mode draws every region at once in the edit set, so the one
        under the cursor has to come from the same set or it reads as a
        different thing rather than the same thing lit. Control mode
        draws one region and one only, which is what lets it use the
        palette the `.touch` file indexes per region.
        """
        if self._model is None:
            return None
        return (self._model.EditColor(region) if self._paint
                else self._model.HoverColor(region))

    def _ShowAllRegions(self):
        """Draw every region at once, in the edit colours. Paint mode.

        One rprim for all 98 regions with a colour PER FACE, not 98
        rprims: the patch is rebuilt on every stroke and every pose, and
        98 resyncs where one will do is the difference between a paint
        tool and a slideshow.

        Lifted by the model's FLOOR rather than by its own extent. The
        extent here is the whole body, so the region-proportional lift
        would put this patch 0.34 cm out -- and above the hover patch,
        which would then be invisible underneath the thing it is meant
        to highlight.
        """
        layer = self._allLayer
        if layer is None or self._model is None or self._canvas is None:
            return False
        # The resting colours ARE the two colour sets: the region's own
        # touchpose:color while paint is on, the touchpose:palette hover
        # colour while it is off. Sync swaps the set and re-authors the
        # geometry only if the painted face SET moved -- which a dab that
        # hands faces from one region to another does not do, so the
        # usual cost of a dab is one colour array. Rebuilding the whole
        # 16,739-face patch instead made one dab 37.9 ms against a
        # 16.7 ms frame.
        self._canvas.Sync(self._model, editing=self._paint)
        if not self._paint:
            layer.Clear()
            self._allFaces = None
            return False
        faces = self._canvas.faces
        self._allFaces = faces
        # Colour None: take the resting colour, which is already the edit
        # set. So this state contributes OPACITY and nothing else.
        return layer.Show(self._model, faces, None, ("all", len(faces)))

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

        The dab is a world-space ball around the ray hit, not a disc of
        pixels: a screen disc would mean one 2.6 ms cast per pixel of the
        brush, where this is one cast plus a vectorised distance test.
        """
        if self._model is None:
            return 0
        target = self._paintTarget
        if target is None and not erase:
            return 0
        ray = self.RayAt(x, y)
        if ray is None:
            return 0
        origin, direction = ray
        face, t = self._model.Cast(origin, direction)
        if face < 0:
            return 0
        hit = [origin[i] + direction[i] * t for i in range(3)]
        faces = self._model.Brush(hit, direction, radius=self._brushRadius)
        if not len(faces):
            return 0
        touched = (self._model.EraseFaces(faces) if erase
                   else self._model.AssignFaces(target, faces))
        if not touched:
            return 0
        self.stroke_faces += len(faces)
        # Every layer showing a region whose faces just moved has to be
        # redrawn, including the hover one -- the patch is built from the
        # face list and is now wrong.
        for layer in (self._highlight, self._selectedLayer,
                      self._leadLayer):
            if layer is not None:
                layer.Forget()
        # `_ShowAllRegions` re-derives the resting colours from the new
        # owners, re-authors the geometry only if the painted face SET
        # moved, and composites every state in one pass.
        self._ShowAllRegions()
        if not erase:
            self._highlight.Show(self._model, target.faces,
                                 self.HoverColorFor(target),
                                 ("paint", target.index,
                                  len(target.faces)))
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
        region's face list, rather than by guessing at a filename: the
        stage may sublayer the regions at any depth, and `Biped_all.usda`
        does it at a different one from `Biped_touch.usda`.
        """
        stage = self._Stage()
        if stage is None or self._model is None or not self._model.regions:
            return None
        # The scope is found by TYPE, not by a hardcoded path: this used
        # to build `/Biped/TouchPose/<region>` by hand, which worked for
        # one character.
        scopes = touchPoseModel.FindRegionScopes(stage, self._model.mesh
                                                 if hasattr(self._model,
                                                            "mesh") else None)
        if not scopes:
            return None
        prim = scopes[0].GetStage().GetPrimAtPath(
            scopes[0].GetPath().AppendChild(self._model.regions[0].name))
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
        """Pull the deformed points if the rig's generation moved."""
        if self._model is None:
            return False
        moved = self._points.Sync(self._model, force=force)
        if moved:
            # Every patch is built from the points, so a new pose has to
            # re-point all of them or a highlight stays on the old
            # silhouette while the body underneath has moved on.
            # ONE prim and POINTS ONLY, where the old design re-authored
            # the topology of three of them: the patch is resident and
            # its topology cannot change with a pose.
            if self._canvas is not None:
                self._canvas.Repoint(self._model)
        return moved

    @property
    def suspended(self):
        """True while a gizmo drag has TouchPose stood down."""
        return self._suspended

    def _PollDrag(self):
        """Stand every patch down for the length of a gizmo drag.

        ALL THREE LAYERS, not just the hover one. During a drag the body
        is being re-posed every frame, so a patch that keeps drawing is
        either stuck to the pose the drag started from -- a coloured
        shell hanging off the character -- or being re-authored at frame
        rate, which is the single most expensive thing TouchPose can do
        and lands on top of the solve the drag is already paying for.

        Resuming forces the pose sync: the points every patch was built
        from are stale by definition, so the rebuild is not optional.
        """
        if not self._active or self._model is None:
            return
        dragging = GizmoDragging()
        if dragging == self._suspended:
            return
        self._suspended = dragging
        if dragging:
            self._hover = None
            # A hover the rate limit held back must not land on the
            # other side of the suspend and light a patch mid-drag.
            self._hoverTimer.stop()
            self._pendingHover = None
            for layer in self._Layers():
                layer.Suspend()
            return
        self.SyncPose(force=True)
        for layer in self._Layers():
            layer.Resume(self._model)

    def _HoverSoon(self, x, y):
        """One hover per frame at most, and never the last one dropped.

        A mouse reports at 125 Hz to 1 kHz and Qt delivers every sample;
        the cast alone is 2.6 ms (R7), so answering all of them is 30% of
        a core spent on a highlight nobody can see move that fast.

        LEADING EDGE plus a trailing one-shot, not a plain rate limit: a
        plain one drops the sample where the mouse stopped, which is the
        one that matters -- the cursor sits in a region with the previous
        region still lit, and it stays that way until the animator moves
        again. The first sample after a gap always goes through, so a
        single hover (which is what every test sends) is never delayed.
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
        if self._highlight is not None:
            self._highlight.Show(
                self._model,
                region.faces if region is not None else None,
                self.HoverColorFor(region) if region is not None
                else None,
                region.index if region is not None else None)
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
        rows = [[matrix[r][c] for c in range(4)] for r in range(4)]
        pixels, valid = self._model.FacePixels(rows, width, height)
        mask = valid & self._model.FrontFacing(
            (eye[0], eye[1], eye[2]))
        caught = self._model.RegionsInRect(pixels, mask, x0, y0, x1, y1)

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
            if self._suspended or GizmoDragging():
                return False
            x, y = self._Position(event)
            if GizmoOwns(x, y, self._Ratio()):
                # The gizmo is drawing its own pre-selection highlight on
                # this pixel; lighting a region behind it as well reads
                # as two things being aimed at.
                if self._highlight is not None:
                    self._highlight.Clear()
                self._hover = None
                return False
            self._HoverSoon(x, y)
            return False        # never consume a hover: the camera reads them
        if kind in (QtCore.QEvent.Leave, QtCore.QEvent.HoverLeave):
            self._hover = None
            self._hoverTimer.stop()
            self._pendingHover = None
            if self._highlight is not None:
                self._highlight.Clear()
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
            "Ctrl-drag on the body gives the faces under the brush to the "
            "region selected above; Ctrl-Shift-drag takes them away. "
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
        self._status.setText(
            "Ctrl-drag paints into %s, Ctrl-Shift-drag erases. Nothing is "
            "written until Save." % (target.label if target else "(pick a "
                                     "region above)")
            if checked else (self._controller.Load() or ""))

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
