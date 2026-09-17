#!/usr/bin/env python
"""Render the docs GIFs: one viewport-accurate loop per RigExec operator.

Usage: python docs/render_media.py [example.usda ...] [--out DIR] [--sheet]

Each docs/examples stage is rendered LIVE -- no bake -- through an
offscreen UsdImagingGL (Storm) engine, so what lands in the GIF is what
usdview draws: the rig evaluates per frame through the RigExec imaging
plugin, the synthesized guides (joint spheres and cones, control shapes)
are drawn with purpose `guide`, and the mesh is shaded with wireframe on
surface so its deformation is legible.

Every frame is composited in PIL on top of a dark usdview-like gradient.
The whole canvas is 640 x 418: a 58 px title strip over the 640x360
viewport, and nothing else.

    +--------------------------------------------------+
    |  Two-Bone IK    one line of summary              |  title  58 px
    |  HandIK -> Shoulder.Elbow.Wrist -> Upper  [key]  |
    +--------------------------------------------------+
    |                                                  |
    |   rest-pose wireframe over the live pose         |  viewport
    |   animated control ringed, labelled, live avars  |  640x360
    |   dashed links driver -> the joint it drives     |
    |   ============================------------       |  progress
    +--------------------------------------------------+

Three rules the whole set obeys, so a reader learns the vocabulary once:

* **one accent.** `ACCENT` means "the control you animate", and nothing
  else in the picture is allowed to be that colour -- not a mesh, not a
  joint, not a second control. Authored `displayColor` is neutralised to
  a grey-blue whose VALUE tracks the authored luminance, so a dark pupil
  stays dark and a white eye stays light without either carrying a hue
  that competes with the rig.
* **guides never hide the geometry they pose.** The live pass draws the
  mesh alone; the guides are a separate pass over the top at partial
  strength, so the silhouette being deformed is always visible.
* **draw the relationship, not just the endpoints.** A constraint is
  about a link, so the link is drawn: pole to elbow, effector to wrist,
  aim to target, lattice cage as a cage.

`prepare_stage`, `fit_camera` and `camera_ops` are exported because
docs/fit_cameras.py authors MainCam from them: the camera the GIF uses and
the camera in the example file have to be one camera, or the page and the
picture disagree. `bake` is exported for anything that still wants an
evaluated, RigExec-free copy of a stage.

Needs ffmpeg (FFMPEG overrides the path), a built tree, and the two
RigExec plugin resource directories, which it puts on PXR_PLUGINPATH_NAME
itself -- ahead of whatever else is already there.
"""
import glob
import math
import os
import shutil
import subprocess
import sys
import tempfile


def _repo_root():
    """The checkout: this file's parent in docs/, or whatever holds one.

    The docs scripts are also run from a scratch copy while a pipeline is
    being reworked, so the answer is CHECKED rather than assumed: the first
    candidate that actually contains docs/examples and icons/ wins, and an
    explicit RIG in the environment is tried before the working directory.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.dirname(here), here, os.environ.get("RIG")]
    walk = os.path.abspath(os.curdir)
    while True:
        candidates.append(walk)
        parent = os.path.dirname(walk)
        if parent == walk:
            break
        walk = parent
    for candidate in candidates:
        if (candidate and os.path.isdir(os.path.join(candidate, "docs",
                                                     "examples")) and
                os.path.isdir(os.path.join(candidate, "icons"))):
            return candidate
    return os.path.dirname(here)


RIG = _repo_root()
USD = os.environ.get("USD", os.path.join(os.path.dirname(RIG), "usd-install"))

# The import environment bin/_env.sh would have set, set here as well so
# the docs scripts run from a plain shell: python modules on sys.path, and
# -- Windows only, since Python 3.8 stopped honouring PATH for extension
# modules -- the directories holding the USD and RigExec DLLs.
#
# The site-packages probe is the SAME ORDER _env.sh uses. The POSIX layout
# carries the interpreter version in the path (lib/python3.11/site-packages)
# and is therefore globbed rather than written down -- pinning a minor
# version leaves the scripts importing nothing the next time USD is rebuilt
# -- and it is probed FIRST, because a machine can carry a stale
# Lib/site-packages beside a live one.
sys.path.insert(0, os.path.join(RIG, "docs"))
sys.path.insert(0, os.path.join(RIG, "build", "python"))
for _candidate in (sorted(glob.glob(os.path.join(USD, "lib", "python*",
                                                 "site-packages")))
                   + [os.path.join(USD, "Lib", "site-packages"),
                      os.path.join(USD, "lib", "python")]):
    if os.path.isdir(_candidate):
        if _candidate not in sys.path:
            sys.path.insert(0, _candidate)
        break
for _candidate in (os.path.join(USD, "lib"), os.path.join(USD, "bin"),
                   os.path.join(RIG, "build")):
    if os.path.isdir(_candidate) and hasattr(os, "add_dll_directory"):
        os.add_dll_directory(_candidate)

from PIL import Image, ImageDraw, ImageFilter, ImageFont  # noqa: E402

FFMPEG = os.environ.get("FFMPEG", shutil.which("ffmpeg") or
                        r"C:\ffmpeg\ffmpeg.exe")

# --- output geometry -------------------------------------------------------
# 640 is the width the docs page displays. Authoring wider and letting the
# browser resample to 640 smeared every one-pixel guide, grid line and
# 9px label; the 3D is supersampled instead (SUPERSAMPLE) and reduced
# here, and every glyph is drawn at 1:1 on the final canvas.
VIEW_W, VIEW_H = 640, 360
SUPERSAMPLE = 2
TITLE_H = 58
CANVAS_W = VIEW_W
GIF_FPS = 12
GIF_COLORS = 128
# Target number of distinct poses in a loop. The examples are 12 frames
# long, which at 12fps is a one-second snap; sampling between the authored
# frames is what makes the motion read as motion.
TARGET_SAMPLES = 23
# Frames evenly spaced through a loop fly through the extreme pose in one
# 80ms tick, and a reader never gets to compare "before" with "after".
# Both ends are held instead: repeated frames, because a GIF's per-frame
# delay is not something ffmpeg's constant-rate encoder will vary.
HOLD_FIRST = 5
HOLD_EXTREME = 3

# --- palette ---------------------------------------------------------------
# One law across all 22 pages. ACCENT is reserved for the animated driver;
# nothing else in the picture is allowed to use it.
BG_TOP = (58, 63, 71)
BG_BOTTOM = (34, 37, 42)
TITLE_BG = (20, 22, 26)
RULE = (44, 48, 56)
TEXT = (232, 236, 242)
TEXT_DIM = (150, 160, 173)
ACCENT = (242, 161, 60)          # the control you animate
STATIC_CTRL = (150, 165, 181)    # a control that is NOT animated here
# Joints and their bones. Deliberately a desaturated TEAL rather than the
# steel-blue this used to be: the ghost is cyan, the ghost composites down
# to roughly that steel over the dark gradient, and the key's `rest` and
# `joints` swatches then read as one colour at 640 px -- a four-swatch key
# that says three things. Teal keeps the "cool, not the accent" law and
# separates from both the ghost and the neutral mesh grey.
JOINT_RGB = (0.34, 0.66, 0.62)
GHOST_RGB = (0.42, 0.82, 1.00)   # the rest pose
GHOST_ALPHA = 0.72
# A deformation that swells outwards (lattice, smooth, volume-correct)
# swallows a rest pose drawn underneath it, so the rest pose is a
# wireframe drawn OVER the live frame -- and pushed harder for the movers,
# where the departure from rest IS the operator.
GHOST_ALPHA_MOVER = 0.90
# How strongly the guides-only overlay shows through solid geometry. The
# guides are drawn ONLY here, never in the live pass, so the mesh being
# posed is never hidden behind the skeleton posing it.
GUIDE_XRAY = 0.78

# --- the influence overlay -------------------------------------------------
# A page whose documented node IS a weight object paints that field onto the
# geometry it weights, the same picture usdview's volume-weight tool draws:
# RigExecImaging_SetWeightOverlay (registry.h) hands the selection to every
# active bridge, _FillWeightOverlay (bridge.cpp) copies the weights the
# MOVER ACTUALLY CONSUMED onto the field's own target prim, and
# RigExecResultsSceneIndex turns them into a per-vertex displayColor ramp
# (sceneIndices.cpp, _WeightOverlayColor). Nothing is re-derived here: a
# plausible gradient over the wrong region looks exactly like a correct one.
WEIGHT_SCHEMAS = frozenset((
    "RigExecStaticWeight", "RigExecDynamicWeight", "RigExecSphereWeight",
    "RigExecPlaneWeight", "RigExecCurveWeight", "RigExecCombineWeight",
    "RigExecCurvenetWeight"))
# The two ends of _WeightOverlayColor's ramp in 8-bit -- grey (0.55) at
# w = 0, red (1.0, 0.05, 0.05) at w = 1 -- so the legend swatch and the
# pixels cannot drift apart.
WEIGHT_RAMP_OFF = (140, 140, 140)
WEIGHT_RAMP_ON = (255, 13, 13)
# The rest pose is a wireframe drawn OVER the live surface, and on an
# overlay page that surface carries the whole message: at the usual 0.72
# the blue wire sits on the ramp and greys the reds it crosses.
GHOST_ALPHA_OVERLAY = 0.42
# The influence ramp is DATA, and Storm shades it like a material: the
# ramp colour arrives as a displayColor primvar and is then multiplied by
# N.L, so a sheet that tips away from the key light comes back with its
# HOT end darker than its cold end and a reader who trusts the legend
# swatch reads the ramp backwards (measured on plane_weight: w=1 surface
# (85,18,22) against w=0 surface (105,108,115)). An overlay page is
# therefore lit flatter -- the diffuse terms are unchanged, the ambient
# floor is raised -- so the field keeps its own value while the geometry
# still has a lit and a shaded side.
OVERLAY_SCENE_AMBIENT = (0.45, 0.46, 0.48, 1.0)
OVERLAY_MATERIAL_AMBIENT = (0.55, 0.56, 0.58, 1.0)

# One 3/4 view for every example: yaw right, pitch down, 35mm. Straight-on
# hides every rotation about Y and flattens the guides into the geometry
# they pose.
YAW = 32.0
PITCH = -20.0
FOCAL = 35.0
H_APERTURE = 20.955
FIT_MARGIN = 1.08
# The margin for a subject whose TRAVEL is bigger than the subject: there
# the union box is mostly empty space the asset passes through, and 8% of
# breathing room on top of it is 8% of subject thrown away for nothing.
FIT_MARGIN_TRAVEL = 1.0
# How much bigger the union of every frame has to be than one frame before
# the subject counts as travelling rather than deforming in place. A card
# that slides its own width scores about 1.8; a card that scales to 2.2x
# scores 1.0, because its union IS its biggest frame.
TRAVEL_SPREAD = 1.35

# Per-example camera, where the global 3/4 view points along the motion
# instead of across it. Each entry needs a reason.
CAMERA_OVERRIDES = {
    # The eyes swivel in yaw about Y. At the global yaw the swing is
    # almost along the view axis and the pupils barely move; from above
    # the whole sweep is in the picture plane, and the eye-to-target
    # distance stops being pure depth.
    "aim_constraint": (26.0, -46.0),
    # The cage bulges in X and Z on a flat vertical card. A near-front
    # view puts the widening across the frame; at yaw 32 it is half depth.
    "lattice_mover": (18.0, -11.0),
    # Same card, same bulge, same reason.
    "volume_correct_mover": (20.0, -12.0),
    # A long thin joint chain twisting about its own axis: the roll only
    # reads when the chain runs across the frame rather than into it.
    "twist_distribution": (52.0, -22.0),
    # Twelve guides in two rows 7.6 units long in X. At the global yaw the
    # row runs into the view axis: the left-hand shapes pile up and
    # project about 55 px while the right-hand ones project about 95, so
    # "every shape at every draw mode, side by side" is measured by where
    # a guide sits in the row instead of by what it is. A shallow yaw puts
    # the whole row across the picture at one size.
    "control": (14.0, -20.0),
}

# Guides are unit-sized by default, which is right at arm scale and
# enormous on a 4-unit docs example. These are fractions of the geometry's
# diagonal, applied through the session layer so no example file changes.
JOINT_GUIDE_RADIUS = 0.019
CONTROL_GUIDE_SCALE = 0.050
CONTROL_WIRE_WIDTH = 0.010
JOINT_GUIDE_OPACITY = 1.0
# An aggregate solver draws its own sphere-and-cone chain along each
# evaluated frame's +X axis, in blue, on top of the joint guides that
# already draw the same skeleton -- plus one long cone off the last
# element, which in a still frame reads as a spike out of the asset and
# drags the camera back to frame it. Zero radius draws no solver guide at
# all (schema.usda, guide:radius). Set this True to see them.
SOLVER_GUIDES = False
# The attributes that make a NON-control prim worth watching: a mover's
# envelope, faded in and out, is as much a driven channel as an avar.
ENVELOPE_ATTRS = ("inputs:defaultWeight", "inputs:enabled")
GRID_PATH = "/DocsGrid"
# The name the imaging plugin gives a control's synthesized guide child
# (libs/rigExecImaging/sceneIndices.cpp, _controlGuideName). Selecting the
# control alone leaves the drawn shape unhighlighted.
CONTROL_GUIDE_CHILD = "rigGuideCtrl"
# Every overlay stays this far inside the viewport. Chips were reaching
# column 714 of 719 and getting clipped by the page.
SAFE = 20

FONT_DIR = os.environ.get("WINDIR", r"C:\Windows") + os.sep + "Fonts"


def _font(name, size):
    for candidate in (os.path.join(FONT_DIR, name),
                      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(candidate, size)
        except (OSError, IOError):
            continue
    return ImageFont.load_default()


def _plugin_path():
    """Our two resource directories FIRST on PXR_PLUGINPATH_NAME.

    A machine can easily have another USD's plugins on this variable
    (C:\\USD\\lib is a common one); ours have to win, and the imaging
    plugin has to be there at all or the rig never evaluates in Hydra.
    """
    ours = [os.path.join(RIG, "build", "usd", "rigExecSchema", "resources"),
            os.path.join(RIG, "build", "usd", "rigExecImaging", "resources")]
    rest = [p for p in os.environ.get("PXR_PLUGINPATH_NAME", "").split(os.pathsep)
            if p and os.path.normcase(os.path.abspath(p)) not in
            {os.path.normcase(os.path.abspath(o)) for o in ours}]
    os.environ["PXR_PLUGINPATH_NAME"] = os.pathsep.join(ours + rest)


_plugin_path()


class ImagingBridge(object):
    """Release the process-global RigExec imaging registry between stages.

    The registry activates ONE stage (registry.h, EnsureActivated: a no-op
    when already active on that stage), so a second example rendered in the
    same process draws with the first example's rig still evaluating -- the
    lattice stage came out showing the IK arm. A fresh UsdImagingGL engine
    does not clear that; only Deactivate does.
    """

    def __init__(self):
        import ctypes

        for directory in (os.path.join(RIG, "build"),
                          os.path.join(USD, "lib"), os.path.join(USD, "bin")):
            if os.path.isdir(directory) and hasattr(os, "add_dll_directory"):
                os.add_dll_directory(directory)
        self._lib = ctypes.CDLL(self._library())
        self._lib.RigExecImaging_Deactivate.argtypes = []
        self._lib.RigExecImaging_Deactivate.restype = None
        # Same signature plugin/rigExecUsdview/rigExecUsdview.py binds:
        # one path in, 0 out on success. Looked up rather than assumed,
        # because a docs run against an older build should lose the
        # overlay and say so, not die on an AttributeError.
        self._overlay = getattr(self._lib,
                                "RigExecImaging_SetWeightOverlay", None)
        if self._overlay is not None:
            self._overlay.argtypes = [ctypes.c_char_p]
            self._overlay.restype = ctypes.c_int
        self._said = False

    @staticmethod
    def _library():
        try:
            import rigExecUsdview
            return rigExecUsdview.ImagingLibraryPath()
        except ImportError:
            pass
        explicit = os.environ.get("RIGEXEC_IMAGING_DLL")
        if explicit:
            return explicit
        name = ("rigExecImaging.dll" if os.name == "nt" else
                "librigExecImaging.dylib" if sys.platform == "darwin" else
                "librigExecImaging.so")
        return os.path.join(RIG, "build", name)

    def release(self):
        self._lib.RigExecImaging_Deactivate()

    def weight_overlay(self, path):
        """Select the weight object to paint, or "" to paint none.

        A viewer mode on the process-global registry, not a property of
        one bridge: it is remembered across activations and republishes at
        the current time, so setting it BEFORE the first render is what
        makes the rig's very first generation already carry the field.
        """
        if self._overlay is None:
            if not self._said:
                self._said = True
                print("note: this build exports no "
                      "RigExecImaging_SetWeightOverlay; weight pages "
                      "render without the influence overlay")
            return False
        code = self._overlay((path or "").encode("utf-8"))
        if code != 0:
            print("note: SetWeightOverlay(%r) returned %d" % (path, code))
        return code == 0


def _schema_plugin():
    import rigexec
    rigexec.load_schema_plugin(os.path.abspath(
        os.path.join(RIG, "build", "usd", "rigExecSchema", "resources")))
    return rigexec


def bake(src, frames, dst):
    """Evaluate `src` at `frames` and export a standalone baked stage.

    Nothing in the GIF pipeline bakes any more -- the engine evaluates the
    rig live -- but a baked stage is still the quickest way to hand an
    evaluated example to a tool that has never heard of RigExec.
    """
    rigexec = _schema_plugin()
    from pxr import Usd  # noqa: E402

    stage = Usd.Stage.Open(src)
    roots = [str(p.GetPath()) for p in stage.Traverse()
             if p.GetTypeName() == "RigExecRoot"]
    rigexec.export_baked(stage, roots, frames, dst)


# ---------------------------------------------------------------------------
# Stage preparation: guide sizing, ground grid, driver discovery
# ---------------------------------------------------------------------------

class Scene(object):
    """Everything the renderer needs to know about one example."""

    def __init__(self, name, stage):
        self.name = name
        self.stage = stage
        self.frames = []
        self.samples = []
        self.pingpong = False
        self.is_mover = False
        self.diag = 1.0
        self.fit = None
        self.camera = None
        self.yaw = YAW
        self.pitch = PITCH
        self.drivers = []       # [(prim, [attrs])]
        self.driver_paths = []
        self.root_xform = None
        self.rigs = []
        self.links = []         # [(control path, joint path, rel)]
        self.cages = []         # [(prim, divisions)]
        self.controls = []      # every control path on the stage
        self.chain = ""         # "HandIK (you move this) -> ... -> Upper"
        self.overlay_weight = None   # weight prim painted onto its target
        self.overlay_target = None   # the geometry that paint lands on
        self.has_joints = False      # is there a RigExecJoint to key?
        self.driver_anchored = False  # does any driver project in frame?
        # readout index -> (preferred side, force the margin slot), decided
        # ONCE for the whole loop so a chip does not jump across the handle
        # it labels as the handle drifts past the middle of the frame.
        self.chip_sides = {}
        # Do all the drivers print the same value in every frame? Then one
        # chip, counted, rather than the same string two or three times.
        self.collapse_chips = False


def compile_rigs(stage):
    """One compiled evaluator per RigExecRoot on the stage."""
    rigexec = _schema_plugin()
    rigs = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != "RigExecRoot":
            continue
        rig = rigexec.Rig(stage, str(prim.GetPath()))
        rig.compile()
        rigs.append(rig)
    return rigs


def _moved_points(stage, rigs, times):
    """World points of the geometry the RIG moves, the prims it moves, and
    the same points split PER SAMPLED FRAME.

    A mover changes points in Hydra and leaves the USD attribute alone, so
    UsdGeomBBoxCache on a deformed mesh returns the bind pose -- the FK
    strip curled a long way out of a frame fitted that way. The evaluator
    publishes the moved points, so the frame is fitted to those instead.

    Real points rather than a box around them: a box adds eight corners
    the asset never occupies, and framing those corners is what leaves a
    diagonal layout with two empty quarters.

    The per-frame split is what tells TRAVEL apart from SIZE: the union of
    every frame says how much room the subject needs, one frame says how
    big the subject is, and the ratio decides how much breathing room the
    fit can afford (`_travel_dominates`).
    """
    from pxr import Gf, Usd, UsdGeom

    cloud, moved, per_frame = [], set(), []
    for time in times:
        here = []
        cache = UsdGeom.XformCache(Usd.TimeCode(time))
        for rig in rigs:
            pose = rig.evaluate(time)
            spans = {}
            for path, value in pose.moved_properties().items():
                prim_path, _, attribute = path.rpartition(".")
                if attribute == "points":
                    spans[prim_path] = [Gf.Vec3d(p[0], p[1], p[2])
                                        for p in value]
                elif attribute == "extent" and prim_path not in spans:
                    low, high = value[0], value[1]
                    spans[prim_path] = [
                        Gf.Vec3d(x, y, z)
                        for x in (low[0], high[0]) for y in (low[1], high[1])
                        for z in (low[2], high[2])]
            for prim_path, points in spans.items():
                prim = stage.GetPrimAtPath(prim_path)
                if not prim or not points:
                    continue
                moved.add(prim_path)
                matrix = cache.GetLocalToWorldTransform(prim)
                here += [matrix.Transform(p) for p in points]
        cloud += here
        per_frame.append(here)
    return cloud, moved, per_frame


def _guide_points(stage, rigs, times, diag, root_xform, only=None):
    """World points for the guides where the rig actually DRAWS them.

    A control's and a joint's posed frame is applied in Hydra and never
    written to the stage, so UsdGeomBBoxCache reports every guide at its
    rest position: the aim example's look-at control swings five units and
    the camera framed the one place it never is.
    """
    from pxr import Gf

    control_radius = diag * CONTROL_GUIDE_SCALE * 1.3
    joint_radius = diag * JOINT_GUIDE_RADIUS
    cloud = []
    for time in times:
        for rig in rigs:
            pose = rig.evaluate(time)
            for paths, radius, frame_of in (
                    (pose.control_paths(), control_radius, pose.control_frame),
                    (pose.joint_paths(), joint_radius, pose.joint_frame)):
                for path in paths:
                    if only is not None and path not in only:
                        continue
                    origin = frame_of(path).origin
                    point = root_xform.Transform(
                        Gf.Vec3d(origin[0], origin[1], origin[2]))
                    for axis in range(3):
                        for sign in (-radius, radius):
                            offset = Gf.Vec3d(0, 0, 0)
                            offset[axis] = sign
                            cloud.append(point + offset)
    return cloud


def _static_points(stage, times, purposes, skip, only=None):
    """World box corners of every drawn thing the rig does NOT move."""
    from pxr import Gf, Usd, UsdGeom

    prims = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path in skip or path.startswith(GRID_PATH):
            continue
        if (not prim.IsA(UsdGeom.Gprim) or
                prim.GetTypeName().startswith("RigExec")):
            continue
        if only is not None and path not in only:
            continue
        prims.append(prim)
    cloud = []
    for time in times:
        cache = UsdGeom.BBoxCache(Usd.TimeCode(time), purposes, False)
        for prim in prims:
            box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
            if box.IsEmpty():
                continue
            low, high = box.GetMin(), box.GetMax()
            cloud += [Gf.Vec3d(x, y, z) for x in (low[0], high[0])
                      for y in (low[1], high[1]) for z in (low[2], high[2])]
    return cloud


def _subject_frames(stage, times, moved_frames, skip):
    """Per sampled frame, the points the subject occupies AT that frame.

    Not every driven prim publishes moved points: a property-math mover
    writes `xformOp:transform` on an Xform and the mesh underneath keeps
    its own points, so the travelling card shows up only in a bounds
    query at that frame (the evaluator's matrix is already in the session
    layer by now, `_driven_xform_fallback`). Both sources are gathered
    here so `_travel_dominates` sees the whole subject either way.
    """
    frames = []
    for index, time in enumerate(times):
        here = list(moved_frames[index]) if index < len(moved_frames) else []
        here += _static_points(stage, [time], ["default", "render", "proxy"],
                               skip)
        frames.append(here)
    return frames


def _cage_points(stage, cages, times):
    """World points of every lattice cage, which is drawn but invisible."""
    from pxr import Usd, UsdGeom

    cloud = []
    for time in times:
        cache = UsdGeom.XformCache(Usd.TimeCode(time))
        for prim, _divisions in cages:
            attr = prim.GetAttribute("points")
            value = attr.Get(Usd.TimeCode(time)) if attr else None
            if not value:
                continue
            matrix = cache.GetLocalToWorldTransform(prim)
            cloud += [matrix.Transform((p[0], p[1], p[2])) for p in value]
    return cloud


def _neutralise_geometry(stage, skip):
    """One neutral for all geometry, so the only hue in frame is the rig.

    Every example authors its own displayColor -- a chartreuse slab, a
    violet strip, an orange upper arm that is the SAME orange as the
    accent -- and stacking guide, joint, ghost and highlight colours on
    top gives five or six hues a page. The authored luminance is kept (a
    dark pupil stays dark, a white eye stays light) and the hue is thrown
    away, with a small alternation so two adjacent parts of one asset stay
    distinguishable.
    """
    from pxr import Gf, Usd, UsdGeom, Vt

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        index = 0
        for prim in stage.Traverse():
            if (not prim.IsA(UsdGeom.Gprim) or
                    prim.GetTypeName().startswith("RigExec") or
                    str(prim.GetPath()).startswith(GRID_PATH)):
                continue
            # Backfaces are DRAWN (cullStyle is CULL_STYLE_NOTHING) but a
            # single-sided normal makes N.L negative there, so the
            # fragment falls to ambient alone -- about (0.04, 0.04, 0.05)
            # -- and any open sheet that rotates past edge-on to the fixed
            # camera turns into a featureless black wedge for half the
            # loop. doubleSided makes Storm flip the normal instead, so a
            # sheet shades from either side. Set for every drawn gprim,
            # including the ones whose colour is left alone (a driver
            # surface, a weight page's painted target).
            sided = UsdGeom.Gprim(prim).GetDoubleSidedAttr()
            if not sided:
                sided = UsdGeom.Gprim(prim).CreateDoubleSidedAttr()
            sided.Set(True)
            if str(prim.GetPath()) in skip:
                continue
            attr = prim.GetAttribute("primvars:displayColor")
            authored = attr.Get() if attr else None
            if authored:
                colour = authored[0]
                lum = (0.2126 * colour[0] + 0.7152 * colour[1] +
                       0.0722 * colour[2])
            else:
                lum = 0.5
            # Kept well below the rest-pose blue and the guide steel:
            # a near-white mesh washed both of them out.
            value = 0.21 + 0.33 * min(max(lum, 0.0), 1.0)
            value += 0.05 if index % 2 == 0 else -0.05
            value = min(max(value, 0.15), 0.58)
            index += 1
            neutral = Gf.Vec3f(value, value * 1.01, value * 1.05)
            if not attr:
                attr = UsdGeom.Gprim(prim).CreateDisplayColorAttr()
            attr.Set(Vt.Vec3fArray([neutral]))


def _apply_guide_overrides(stage, diag):
    """Size and colour the synthesized guides, in the session layer."""
    from pxr import Gf, Usd

    steel = Gf.Vec3f(*JOINT_RGB)
    static = Gf.Vec3f(STATIC_CTRL[0] / 255.0, STATIC_CTRL[1] / 255.0,
                      STATIC_CTRL[2] / 255.0)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in stage.Traverse():
            typename = prim.GetTypeName()
            if not typename.startswith("RigExec"):
                continue
            if typename == "RigExecControl":
                for axis in "XYZ":
                    attr = prim.GetAttribute("guide:scale" + axis)
                    if attr:
                        attr.Set(diag * CONTROL_GUIDE_SCALE)
                width = prim.GetAttribute("guide:wireWidth")
                if width:
                    # wireWidth is in the guide's own pre-scale units, so it
                    # is a fraction of the shape and not of the asset.
                    width.Set(CONTROL_WIRE_WIDTH / CONTROL_GUIDE_SCALE)
                colour = prim.GetAttribute("guide:displayColor")
                if colour:
                    # Every control starts desaturated. _mark_drivers gives
                    # the accent back to the ONE that is animated.
                    colour.Set(static)
                # A control keeps the default purpose so it stays pickable
                # in usdview. Here it is moved to `guide` with its joint
                # siblings, which is what lets the rest-pose ghost pass
                # (showGuides off) draw the deformed geometry ALONE.
                purpose = prim.GetAttribute("purpose")
                if purpose:
                    purpose.Set("guide")
            elif typename == "RigExecJoint":
                colour = prim.GetAttribute("guide:displayColor")
                if colour:
                    colour.Set(steel)
            radius = prim.GetAttribute("guide:radius")
            if radius:
                radius.Set(diag * JOINT_GUIDE_RADIUS
                           if typename == "RigExecJoint" or SOLVER_GUIDES
                           else 0.0)
            opacity = prim.GetAttribute("guide:displayOpacity")
            if opacity:
                opacity.Set(JOINT_GUIDE_OPACITY)


def _add_grid(stage, bounds, diag):
    """A ground grid under the asset: scale, horizon, and a floor to read
    the 3/4 camera against.

    Sized from the bounds the CAMERA frames, and deliberately a little
    larger than them, so it runs off the edges of the picture the way a
    floor does instead of sitting under the asset like a table mat.
    """
    from pxr import Gf, Usd, UsdGeom, Vt

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        grid = UsdGeom.BasisCurves.Define(stage, GRID_PATH)
        low, high = bounds.GetMin(), bounds.GetMax()
        cx = (low[0] + high[0]) / 2.0
        cz = (low[2] + high[2]) / 2.0
        floor = low[1] - diag * 0.03
        spacing = diag * 0.22
        reach = max(high[0] - low[0], high[2] - low[2]) * 0.75 + spacing
        divisions = max(4, int(math.ceil(reach / spacing)))
        half = divisions * spacing
        lines, counts = [], []
        for index in range(2 * divisions + 1):
            offset = -half + spacing * index
            lines += [Gf.Vec3f(cx + offset, floor, cz - half),
                      Gf.Vec3f(cx + offset, floor, cz + half)]
            counts.append(2)
            lines += [Gf.Vec3f(cx - half, floor, cz + offset),
                      Gf.Vec3f(cx + half, floor, cz + offset)]
            counts.append(2)
        grid.CreatePointsAttr(Vt.Vec3fArray(lines))
        grid.CreateCurveVertexCountsAttr(Vt.IntArray(counts))
        grid.CreateTypeAttr("linear")
        # Thickened: a hairline at 1280 wide survived the reduction to 640
        # as a dotted crawl rather than a line.
        grid.CreateWidthsAttr(Vt.FloatArray([diag * 0.0055] * len(lines)))
        grid.GetWidthsAttr().SetMetadata("interpolation", "vertex")
        grid.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.37, 0.40, 0.45)]))
        grid.CreatePurposeAttr("guide")


def _find_drivers(stage):
    """The prims a reader should watch: whatever is actually animated.

    Controls first, because an animator's handle is the story in most
    examples; but several movers are driven by an animated cage, curve or
    surface with no control anywhere on the stage, and there the driver
    geometry IS the control.
    """
    skip = ("visibility", "extent")
    controls, envelopes, others = [], [], []
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(GRID_PATH):
            continue
        animated = [a for a in prim.GetAttributes()
                    if a.GetName() not in skip and a.ValueMightBeTimeVarying()]
        if not animated:
            continue
        if prim.GetTypeName() == "RigExecControl":
            controls.append((prim, animated))
            continue
        others.append((prim, animated))
        if any(a.GetName() in ENVELOPE_ATTRS for a in animated):
            envelopes.append((prim, animated))
    # An animated ENVELOPE is a second story, not a second handle: on
    # parent_constraint the hand keeps swinging while `defaultWeight`
    # drops to 0 and the prop is released, and with only the control
    # collected two fifths of the loop shows the card detaching with
    # nothing on screen naming the cause. Those are appended to the
    # controls rather than replacing them, capped at the three readouts
    # `compose` draws.
    if controls:
        return controls + envelopes[:max(0, 3 - len(controls))]
    return others


def _mark_drivers(stage, drivers, diag, cage_paths):
    """Make the animated driver the thing the eye goes to.

    Storm's selection highlight is a faint tint on a wire guide -- real,
    but not something a reader spots in a 12fps loop -- so the driver is
    also given the accent colour and a slightly larger guide, and an
    invisible driver (several movers animate one) is made visible, because
    an invisible driver teaches nothing. A lattice cage is the exception:
    it is drawn as a projected cage in `compose`, because eight unconnected
    points are confetti and not a lattice.
    """
    from pxr import Gf, Usd, UsdGeom, Vt

    accent = Gf.Vec3f(ACCENT[0] / 255.0, ACCENT[1] / 255.0, ACCENT[2] / 255.0)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim, _attrs in drivers:
            if prim.GetTypeName() == "RigExecControl":
                colour = prim.GetAttribute("guide:displayColor")
                if colour:
                    colour.Set(accent)
                for axis in "XYZ":
                    attr = prim.GetAttribute("guide:scale" + axis)
                    if attr:
                        attr.Set(diag * CONTROL_GUIDE_SCALE * 1.3)
                continue
            if (prim.GetTypeName().startswith("RigExec") or
                    str(prim.GetPath()) in cage_paths):
                continue
            imageable = UsdGeom.Imageable(prim)
            if not imageable:
                continue
            if imageable.ComputeVisibility() == UsdGeom.Tokens.invisible:
                imageable.MakeVisible()
                purpose = prim.GetAttribute("purpose")
                if purpose and not purpose.HasAuthoredValue():
                    purpose.Set("guide")
            colour = prim.GetAttribute("primvars:displayColor")
            if colour:
                colour.Set(Vt.Vec3fArray([accent]))
            widths = prim.GetAttribute("widths")
            if widths and widths.Get():
                widths.Set(Vt.FloatArray([diag * 0.014] * len(widths.Get())))
            elif prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.NurbsPatch):
                # A driver SURFACE is a sheet, not a handle: painted solid
                # accent it becomes the largest and loudest thing in frame
                # (the surface mover's ground plane filled the picture)
                # and the accent stops meaning "the thing you animate".
                # Translucent, it reads as the driver the geometry is
                # draped on and the geometry stays visible through it.
                opacity = UsdGeom.Gprim(prim).CreateDisplayOpacityPrimvar(
                    UsdGeom.Tokens.constant)
                opacity.Set(Vt.FloatArray([0.40]))


_OP_MATRIX = {
    # The evaluator hands a matrix back as sixteen numbers in one flat
    # sequence, not as four rows.
    "transform": lambda v, Gf: Gf.Matrix4d(*[float(c) for c in v]),
    "scale": lambda v, Gf: Gf.Matrix4d().SetScale(Gf.Vec3d(v[0], v[1], v[2])),
    "translate": lambda v, Gf: Gf.Matrix4d().SetTranslate(
        Gf.Vec3d(v[0], v[1], v[2])),
}


def _driven_xform_fallback(stage, rigs, times):
    """Author the driven transform of an Xform whose GEOMETRY is a child.

    A property-math mover writes `xformOp:scale` or `xformOp:transform` on
    an Xform and the mesh hangs underneath it. The imaging plugin
    publishes the driven transform for the prim the mover names, but the
    scene index sits downstream of flattening, so the child mesh keeps the
    matrix it was flattened with and never moves: the value on the chip
    counts up and the picture is a still card.

    The evaluator's own matrix is authored here as one extra session-layer
    op, per sampled frame, so the picture agrees with the numbers. Only
    for a driven prim that carries no geometry itself -- where the plugin
    draws nothing, and there is therefore nothing to double up.

    Returns the gprim paths BELOW every prim it authored one for. Those
    meshes are the subject of such an example just as moved points are
    the subject elsewhere, and the framing needs to be told so: see the
    driver cloud in `prepare_stage`.
    """
    from pxr import Gf, Usd, UsdGeom

    wanted = {}
    for time in times:
        for rig in rigs:
            for path, value in rig.evaluate(time).moved_properties().items():
                prim_path, _, attribute = path.rpartition(".")
                if not attribute.startswith("xformOp:"):
                    continue
                prim = stage.GetPrimAtPath(prim_path)
                if (not prim or prim.IsA(UsdGeom.Gprim) or
                        not any(child.IsA(UsdGeom.Gprim)
                                for child in Usd.PrimRange(prim))):
                    continue
                wanted.setdefault(prim_path, {})[time] = (attribute, value)

    driven = set()
    for prim_path, samples in wanted.items():
        prim = stage.GetPrimAtPath(prim_path)
        xformable = UsdGeom.Xformable(prim)
        ops = xformable.GetOrderedXformOps()
        if len(ops) != 1:
            continue
        op = ops[0]
        kind = op.GetOpName().split(":", 1)[1].split(":", 1)[0]
        build = _OP_MATRIX.get(kind)
        if build is None or any(name != op.GetOpName()
                                for name, _ in samples.values()):
            continue
        base = op.GetOpTransform(Usd.TimeCode.Default())
        inverse = base.GetInverse()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            extra = UsdGeom.Xformable(prim).AddTransformOp(
                UsdGeom.XformOp.PrecisionDouble, "docsDriven")
            for time, (_name, value) in sorted(samples.items()):
                extra.Set(inverse * build(value, Gf), Usd.TimeCode(time))
        driven.update(str(child.GetPath()) for child in Usd.PrimRange(prim)
                      if child.IsA(UsdGeom.Gprim))
    return driven


def _drive_links(stage):
    """[(control path, joint path, rel)]: the link the operator IS.

    A constraint, an IK solve and an FK chain are all statements about a
    relationship, and a picture of two objects at rest in the same frame
    does not state it. The pairing rules are the schema's own semantics:
    `rootControl` poses the first joint of the chain, `effectorControl`
    the last, `poleControl` picks the bend at the middle one, and a
    list-valued `controls` pairs with `joints` by index.
    """
    links = []
    for prim in stage.Traverse():
        if not prim.GetTypeName().startswith("RigExec"):
            continue
        controls, joints = {}, []
        for rel in prim.GetRelationships():
            name = rel.GetName()
            if not name.startswith("rigExec:"):
                continue
            short = name.split(":", 1)[1]
            for target in rel.GetTargets():
                other = stage.GetPrimAtPath(target.GetPrimPath())
                if not other:
                    continue
                typename = other.GetTypeName()
                if typename == "RigExecControl":
                    controls.setdefault(short, []).append(
                        str(target.GetPrimPath()))
                elif (typename == "RigExecJoint" and
                        short in ("joints", "moves")):
                    joints.append(str(target.GetPrimPath()))
        if not controls or not joints:
            continue
        for short, paths in controls.items():
            if short == "rootControl":
                links.append((paths[0], joints[0], short))
            elif short == "effectorControl":
                links.append((paths[0], joints[-1], short))
            elif short == "poleControl":
                links.append((paths[0], joints[len(joints) // 2], short))
            # A spline IK names one midControl and one endControl against
            # a whole chain of joints, so both used to fall into the
            # catch-all below and draw one control against the first THREE
            # joints: six dashes across the arch interior, asserting that
            # the end control poses the root joints. It does not -- it
            # carries the last two CVs and the end twist.
            elif short == "midControl":
                links.append((paths[0], joints[len(joints) // 2], short))
            elif short == "endControl":
                links.append((paths[0], joints[-1], short))
            elif len(paths) == len(joints):
                links += [(c, j, short) for c, j in zip(paths, joints)]
            else:
                for control in paths[:3]:
                    links += [(control, j, short) for j in joints[:3]]
    return links


def _weight_overlay_prim(stage, schema):
    """The weight prim THIS PAGE documents, or None.

    One stage can carry several weight objects -- dynamic_weight authors a
    RigExecStaticWeight "Paint" that a RigExecDynamicWeight "KneeDriven"
    wraps, and the page is about the wrapper -- so the documented TYPE
    decides, not proximity or authoring order.

    Among prims of that type, one a mover actually binds through
    `rigExec:weightObject` wins: an unbound field is never resolved into
    the pose, so `pose.weightFields` has no entry for it and the overlay
    would silently paint nothing.
    """
    if schema not in WEIGHT_SCHEMAS:
        return None
    bound = set()
    for prim in stage.Traverse():
        rel = prim.GetRelationship("rigExec:weightObject")
        for target in (rel.GetTargets() if rel else []):
            bound.add(str(target.GetPrimPath()))
    candidates = [str(p.GetPath()) for p in stage.Traverse()
                  if p.GetTypeName() == schema]
    for path in candidates:
        if path in bound:
            return path
    return candidates[0] if candidates else None


def _weight_overlay_target(stage, weight_path):
    """The geometry prim a weight object paints, off its own target rel.

    The bridge paints `pose.weightFields[...].target`, which is this
    relationship resolved by the evaluator; read here only so the renderer
    knows which prim NOT to recolour.
    """
    prim = stage.GetPrimAtPath(weight_path) if weight_path else None
    rel = prim.GetRelationship("rigExec:weightTarget") if prim else None
    targets = rel.GetTargets() if rel else []
    return str(targets[0].GetPrimPath()) if targets else None


def _emitted_guide_targets(stage):
    """Prims a curve mover WRITES as its output rather than deforms.

    A `RigExecCurveMover` in `emitGuidePoints` mode owns every point of
    its target: the curve carries no authored shape of its own, it is one
    point per solver frame origin, published every frame. That is rig
    output -- the same status as a joint bone or a driver handle -- so it
    keeps whatever colour the example authored for it instead of being
    neutralised into the asset's grey, where it reads as an unexplained
    pale streak with no key entry.
    """
    out = set()
    for prim in stage.Traverse():
        if prim.GetTypeName() != "RigExecCurveMover":
            continue
        mode = prim.GetAttribute("rigExec:mode")
        if not mode or mode.Get() != "emitGuidePoints":
            continue
        rel = prim.GetRelationship("rigExec:moves")
        for target in (rel.GetTargets() if rel else []):
            out.add(str(target.GetPrimPath()))
    return out


def _cages(stage):
    """[(cage prim, divisions)] for every lattice mover on the stage."""
    out = []
    for prim in stage.Traverse():
        rel = prim.GetRelationship("rigExec:cage")
        targets = rel.GetTargets() if rel else []
        if not targets:
            continue
        cage = stage.GetPrimAtPath(targets[0].GetPrimPath())
        divisions = prim.GetAttribute("rigExec:divisions")
        value = divisions.Get() if divisions else None
        if cage:
            out.append((cage, tuple(value) if value else None))
    return out


def _chain_line(stage, scene, moved):
    """"HandIK (you move this) -> Shoulder.Elbow.Wrist -> Upper, Fore".

    The whole "which is the handle and which is the follower" question,
    answered in TEXT, for all 22, walked straight off the stage's own
    `rigExec:*` relationships. This is the one thing the node-network
    strip under the viewport carried that the viewport does not, and a
    line of text says it without a second diagram to decode.
    """
    used = set()
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            if rel.GetName().startswith("rigExec:"):
                used.update(str(t.GetPrimPath()) for t in rel.GetTargets())
    names = [p.GetName() for p, _ in scene.drivers]
    # Namespace order, not alphabetical: the line is read against the
    # picture, and "Fore, Hand, Upper" names an arm no reader can find --
    # the chain runs Upper, Fore, Hand and the stage says so.
    joints, geometry = [], []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path in used and prim.GetTypeName() == "RigExecJoint":
            joints.append(prim.GetName())
        if path in moved:
            geometry.append(prim.GetName())
    if not geometry:
        # A property-math mover revises `xformOp:transform` and publishes
        # no moved point at all, so `moved` is empty and the line used to
        # stop at the handle with no arrow and no follower. The movers
        # themselves still say what they write.
        seen = set()
        for prim in stage.Traverse():
            rel = prim.GetRelationship("rigExec:moves")
            for target in (rel.GetTargets() if rel else []):
                other = stage.GetPrimAtPath(target.GetPrimPath())
                if (not other or other.GetTypeName().startswith("RigExec") or
                        str(other.GetPath()) in seen):
                    continue
                seen.add(str(other.GetPath()))
                geometry.append(other.GetName())
    if not names:
        return ""
    # Naming the first two of twelve equal handles is worse than counting
    # them: a reader sees twelve rings and two names and looks for the
    # difference between them, and there is none.
    if len(names) > 3:
        kind = ("controls" if all(p.GetTypeName() == "RigExecControl"
                                  for p, _ in scene.drivers) else "drivers")
        parts = ["%d %s (you move these)" % (len(names), kind)]
    else:
        parts = [" + ".join(names) + " (you move this)"]
    if joints:
        parts.append(" \u00b7 ".join(joints[:4]) if len(joints) <= 4
                     else "%d joints" % len(joints))
    if geometry:
        parts.append(", ".join(geometry[:3]) +
                     (" \u2026" if len(geometry) > 3 else ""))
    return "   \u2192   ".join(parts)


def prepare_stage(src, samples=TARGET_SAMPLES, schema=None):
    """Open an example and make it ready to draw: geometry neutralised,
    guides sized to the asset, a ground grid, drivers marked, camera fit.

    `schema` is the type of the node the PAGE documents. When that is a
    weight object the field it resolves is painted onto its target
    geometry (`Scene.overlay_weight`), and that one prim is left out of
    the neutral recolour.
    """
    from pxr import Gf, Usd, UsdGeom

    _schema_plugin()
    name = os.path.splitext(os.path.basename(src))[0]
    stage = Usd.Stage.Open(src)
    scene = Scene(name, stage)
    scene.yaw, scene.pitch = CAMERA_OVERRIDES.get(name, (YAW, PITCH))
    start = int(stage.GetStartTimeCode())
    end = int(stage.GetEndTimeCode())
    scene.frames = list(range(start, end + 1))

    span = max(1, end - start)
    step = span / float(max(1, samples - 1))
    scene.samples = [start + step * i for i in range(samples)]

    # Compiled before the session-layer edits below so the measurement is
    # of the example exactly as authored, and again afterwards for the
    # per-frame control positions, which have to agree with what the
    # engine draws.
    probe = compile_rigs(stage)
    moved_cloud, moved, _per_frame = _moved_points(stage, probe, scene.frames)
    geometry = Gf.Range3d()
    for point in moved_cloud + _static_points(
            stage, scene.frames, ["default", "render", "proxy"], moved):
        geometry.UnionWith(point)
    if geometry.IsEmpty():
        raise ValueError("no authored geometry to frame")
    scene.diag = max(geometry.GetSize().GetLength(), 1e-4)
    scene.is_mover = bool(moved) and not any(
        p.GetTypeName() in ("RigExecTwoBoneIk", "RigExecFkChain")
        for p in stage.Traverse())

    scene.cages = _cages(stage)
    cage_paths = {str(p.GetPath()) for p, _ in scene.cages}
    scene.drivers = _find_drivers(stage)
    scene.driver_paths = [str(p.GetPath()) for p, _ in scene.drivers]
    scene.overlay_weight = _weight_overlay_prim(stage, schema)
    scene.overlay_target = _weight_overlay_target(stage, scene.overlay_weight)
    # The painted prim keeps whatever displayColor it has: the overlay's
    # per-vertex ramp arrives from the RESULTS SCENE INDEX, which overlays
    # the stage's own primvars container and therefore wins whatever is
    # authored here -- but authoring a second, contradictory displayColor
    # into the session layer for the one prim whose colour IS the point is
    # the kind of thing that is right until someone reorders a scene index.
    _neutralise_geometry(
        stage, cage_paths | set(scene.driver_paths) |
        _emitted_guide_targets(stage) |
        ({scene.overlay_target} if scene.overlay_target else set()))
    _apply_guide_overrides(stage, scene.diag)
    _mark_drivers(stage, scene.drivers, scene.diag, cage_paths)
    scene.links = _drive_links(stage)
    scene.controls = [str(p.GetPath()) for p in stage.Traverse()
                      if p.GetTypeName() == "RigExecControl"]
    scene.chain = _chain_line(stage, scene, moved)

    # The fit covers geometry AND guides, over every sampled frame, so
    # nothing the rig draws can leave the picture mid-loop. The ground grid
    # is excluded on purpose and authored afterwards: fitting to it would
    # pull the camera back until the asset was a detail in the middle.
    root = stage.GetDefaultPrim()
    scene.root_xform = (UsdGeom.Xformable(root).ComputeLocalToWorldTransform(
        Usd.TimeCode(start)) if root and UsdGeom.Xformable(root)
        else Gf.Matrix4d(1))

    scene.rigs = compile_rigs(stage)
    driven = _driven_xform_fallback(stage, scene.rigs, scene.samples)
    moved_cloud, moved, moved_frames = _moved_points(stage, scene.rigs,
                                                     scene.samples)
    # The subject is what moves plus the handle that moves it. A mesh
    # carried by a driven xformOp moves without publishing a single moved
    # point, so `driven` speaks for it here; without that a property-math
    # example whose one driver is a stationary control has a subject box
    # the size of that control's guide, and the coverage re-fit below
    # collapses the frame onto the guide with the card off every edge.
    subject = set(scene.driver_paths) | driven
    driver_cloud = (moved_cloud +
                    _cage_points(stage, scene.cages, scene.samples) +
                    _guide_points(stage, scene.rigs, scene.samples, scene.diag,
                                  scene.root_xform,
                                  only=set(scene.driver_paths)) +
                    _static_points(stage, scene.samples,
                                   ["default", "render", "proxy", "guide"],
                                   moved, only=subject))
    cloud = (moved_cloud +
             _guide_points(stage, scene.rigs, scene.samples, scene.diag,
                           scene.root_xform) +
             _cage_points(stage, scene.cages, scene.samples) +
             _static_points(stage, scene.samples,
                            ["default", "render", "proxy", "guide"], moved))
    if not cloud:
        raise ValueError("nothing to frame")
    union = Gf.Range3d()
    for point in cloud:
        union.UnionWith(point)
    scene.fit = union
    _add_grid(stage, union, scene.diag)
    travels = _travel_dominates(
        _subject_frames(stage, scene.samples, moved_frames, moved))
    scene.camera = fit_camera(
        cloud, VIEW_W / float(VIEW_H), scene.yaw, scene.pitch,
        margin=FIT_MARGIN_TRAVEL if travels else FIT_MARGIN)
    # Box-union framing leaves a diagonal layout -- a long joint chain with
    # a small piece of geometry on it, an eye pair up here and a target
    # down there -- at under a tenth ink coverage, and the subject is then
    # too small to read. When that happens the frame is re-fit to the
    # SUBJECT (what moves, and the handle that moves it) and the rest of
    # the rig is allowed to run off the edges.
    if driver_cloud and _coverage(scene.camera, driver_cloud) < 0.20:
        scene.camera = fit_camera(driver_cloud, VIEW_W / float(VIEW_H),
                                  scene.yaw, scene.pitch, margin=1.34)
    # And once more for the worst case, a long joint chain carrying a
    # small piece of geometry (twist distribution): even the subject box
    # is mostly the distance between the handle and the thing it moves.
    # There the geometry wins the frame and the handle is allowed off the
    # edge -- `compose` then draws its chip without a ring, rather than
    # putting a ring somewhere the control is not.
    #
    # Allowed off the edge, but not out of the example. On aim_constraint
    # this fired and left the LookAt control -- the ONE thing a reader is
    # supposed to watch, and the thing the eyes are aiming at -- outside
    # the picture for every frame of the loop: no ring anywhere, and a
    # chip pinned to the offscreen slot in the corner saying a number with
    # nothing to attach it to. So the tightened frame has to keep at least
    # one sampled driver position in shot, or it is not taken.
    if moved_cloud and _coverage(scene.camera, moved_cloud) < 0.07:
        tightened = fit_camera(moved_cloud, VIEW_W / float(VIEW_H),
                               scene.yaw, scene.pitch, margin=1.45)
        if _any_in_frame(tightened, _driver_anchors(scene)):
            scene.camera = tightened

    # The loop only needs a ping-pong when it does not already close: most
    # docs examples animate out and back, and doubling those just makes a
    # four-second GIF out of a two-second one.
    scene.pingpong = not _loop_closes(scene)
    scene.has_joints = any(p.GetTypeName() == "RigExecJoint"
                           for p in stage.Traverse())
    # Last, because it projects through the camera the fit just chose.
    (scene.chip_sides, scene.driver_anchored,
     scene.collapse_chips) = _chip_placement(scene)
    return scene


def _driver_vector(scene, time):
    """Every animated driver value at `time`, flattened, for comparisons."""
    out = []
    for _prim, attrs in scene.drivers:
        for attr in attrs:
            value = attr.Get(time)
            if value is None:
                continue
            try:
                for item in value:
                    try:
                        out += [float(c) for c in item]
                    except TypeError:
                        out.append(float(item))
            except TypeError:
                out.append(float(value))
    return out


def _loop_closes(scene):
    first = _driver_vector(scene, scene.samples[0])
    last = _driver_vector(scene, scene.samples[-1])
    if len(first) != len(last):
        return False
    return all(abs(a - b) < 1e-6 for a, b in zip(first, last))


def extreme_sample(scene):
    """Index of the sample furthest from rest: the pose worth holding."""
    rest = _driver_vector(scene, scene.samples[0])
    best, best_index = -1.0, len(scene.samples) // 2
    for index, time in enumerate(scene.samples):
        here = _driver_vector(scene, time)
        if len(here) != len(rest):
            continue
        distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(here, rest)))
        if distance > best:
            best, best_index = distance, index
    return best_index


def _coverage(camera, cloud):
    """Fraction of the viewport the projected cloud's box covers."""
    pixels = [p for p in (_project(camera, point, VIEW_W, VIEW_H)
                          for point in cloud) if p]
    if len(pixels) < 2:
        return 1.0
    xs = [p[0] for p in pixels]
    ys = [p[1] for p in pixels]
    return (((max(xs) - min(xs)) * (max(ys) - min(ys))) /
            float(VIEW_W * VIEW_H))


def _travel_dominates(per_frame):
    """True when the subject moves further than the subject is big.

    A card that slides 2.5 units while being 2 across makes a union box
    whose long axis is the TRAVEL and not the asset; fitted with the usual
    breathing room the card ends up a quarter of the frame wide with an
    empty third on either side. No camera removes that -- a vertical
    travel in a 16:9 frame IS a tall thin subject -- but the 8% margin is
    then paid for nothing, so it is spent down to zero and the subject
    gets the pixels instead.
    """
    from pxr import Gf

    union = Gf.Range3d()
    biggest = 0.0
    for points in per_frame:
        box = Gf.Range3d()
        for point in points:
            box.UnionWith(point)
        if box.IsEmpty():
            continue
        biggest = max(biggest, box.GetSize().GetLength())
        union.UnionWith(box)
    if union.IsEmpty() or biggest < 1e-6:
        return False
    return union.GetSize().GetLength() > biggest * TRAVEL_SPREAD


def _driver_anchors(scene):
    """Where the driver chips and rings go, over every sampled frame.

    The same points `compose` rings, so "can a reader see the handle?" is
    asked of the positions that are actually drawn rather than of a box
    that approximates them.
    """
    points = []
    for time in scene.samples:
        for _label, point in frame_state(scene, time).readouts:
            if point is not None:
                points.append(point)
    return points


def _chip_placement(scene):
    """(sides, anchored): where each driver's chip lives, for the LOOP.

    Both decisions `compose` used to make per frame -- which side of the
    ring the chip sits on, and whether it is shoved out to the margin
    because the ring is over the subject -- are functions of a projected x
    that MOVES. A handle that crosses the middle of the frame therefore
    flipped its own label across itself twice a loop, and a handle that
    drifted through the middle band toggled between an adjacent chip and a
    margin chip with a 300 px leader. Deciding once, from the mean of the
    projected positions the chips are actually drawn at, costs nothing in
    legibility -- the leader already exists so a non-adjacent chip still
    points at its ring -- and removes the flicker.
    """
    seen = {}
    collapse = len(scene.drivers) > 1
    for time in scene.samples:
        readouts = frame_state(scene, time).readouts
        # And, on the same walk: do ALL the drivers print the same value in
        # EVERY frame? The control page keys twelve guides off one set of
        # splines, and three chips saying the same thing three times says
        # only that the renderer has not noticed. Asked of the whole loop,
        # not of one frame, or a page whose handles merely start together
        # would collapse at rest and split mid-play.
        values = [label.partition("   ")[2] for label, _p in readouts[:3]]
        if not values or not values[0] or any(v != values[0] for v in values):
            collapse = False
        for index, (_label, point) in enumerate(readouts):
            if point is None:
                continue
            pixel = _project(scene.camera, point, VIEW_W, VIEW_H)
            if pixel:
                seen.setdefault(index, []).append(pixel[0])
    sides = {}
    for index, xs in seen.items():
        mean = sum(xs) / float(len(xs))
        sides[index] = (1 if mean < VIEW_W / 2.0 else -1,
                        abs(mean - VIEW_W / 2.0) < VIEW_W * 0.16)
    return sides, bool(seen), collapse


def _any_in_frame(camera, points):
    """Does ANY of `points` land inside the viewport through `camera`?"""
    for point in points:
        pixel = _project(camera, point, VIEW_W, VIEW_H)
        if pixel and 0 <= pixel[0] <= VIEW_W and 0 <= pixel[1] <= VIEW_H:
            return True
    return False


def fit_camera(cloud, aspect, yaw=YAW, pitch=PITCH, margin=FIT_MARGIN):
    """A 3/4 camera that frames every point in `cloud` with a fixed margin.

    `cloud` is world POINTS -- the moved points the evaluator publishes,
    the guide origins, one box per static prim -- not one box for the
    stage, whose corners are often empty. A Gf.Range3d is accepted too,
    for callers that only have bounds.

    Returns a Gf.Camera; docs/fit_cameras.py authors the same one into the
    example as MainCam, so the page's camera and the GIF's camera agree.
    """
    from pxr import Gf

    basis = (Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), pitch)) *
             Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), yaw)))
    right = Gf.Vec3d(basis[0][0], basis[0][1], basis[0][2])
    up = Gf.Vec3d(basis[1][0], basis[1][1], basis[1][2])
    back = Gf.Vec3d(basis[2][0], basis[2][1], basis[2][2])
    forward = -back

    if hasattr(cloud, "GetMin"):
        low, high = cloud.GetMin(), cloud.GetMax()
        cloud = [Gf.Vec3d(x, y, z) for x in (low[0], high[0])
                 for y in (low[1], high[1]) for z in (low[2], high[2])]
    xs = [Gf.Dot(c, right) for c in cloud]
    ys = [Gf.Dot(c, up) for c in cloud]
    zs = [Gf.Dot(c, forward) for c in cloud]

    v_aperture = H_APERTURE / aspect
    tan_h = H_APERTURE / 2.0 / FOCAL
    tan_v = v_aperture / 2.0 / FOCAL
    half_w = max((max(xs) - min(xs)) / 2.0, 1e-4)
    half_h = max((max(ys) - min(ys)) / 2.0, 1e-4)
    depth = (max(zs) - min(zs)) / 2.0
    distance = max(half_w / tan_h, half_h / tan_v) * margin + depth

    centre = (right * ((min(xs) + max(xs)) / 2.0) +
              up * ((min(ys) + max(ys)) / 2.0) +
              forward * ((min(zs) + max(zs)) / 2.0))
    eye = centre - forward * distance

    camera = Gf.Camera()
    camera.focalLength = FOCAL
    camera.horizontalAperture = H_APERTURE
    camera.verticalAperture = v_aperture
    camera.clippingRange = Gf.Range1f(max(distance * 0.01, 1e-3),
                                      distance * 8.0 + depth * 4.0)
    matrix = Gf.Matrix4d(1)
    matrix.SetRow3(0, right)
    matrix.SetRow3(1, up)
    matrix.SetRow3(2, back)
    matrix.SetTranslateOnly(eye)
    camera.transform = matrix
    return camera


def camera_ops(camera, yaw=YAW, pitch=PITCH):
    """(translate, rotateXYZ) for the same camera as USD xform ops.

    xformOpOrder ["xformOp:translate", "xformOp:rotateXYZ"] composes to a
    matrix whose upper 3x3 is the rotation and whose fourth row is the
    translation -- exactly the camera matrix fit_camera built -- so the two
    describe the same camera and docs/fit_cameras.py can author it.
    """
    translate = camera.transform.ExtractTranslation()
    return ((translate[0], translate[1], translate[2]), (pitch, yaw, 0.0))


# ---------------------------------------------------------------------------
# Offscreen Storm renderer
# ---------------------------------------------------------------------------

class Viewport(object):
    """The offscreen GL context, framebuffer and Storm engine.

    Rendered at SUPERSAMPLE times the published size and reduced in PIL:
    8x MSAA does nothing for a one-pixel wire that lands between samples,
    and the guides, the ground grid and the control rings are all made of
    those.
    """

    def __init__(self, width=VIEW_W * SUPERSAMPLE, height=VIEW_H * SUPERSAMPLE):
        from PySide6.QtCore import QSize
        from PySide6.QtGui import (QOffscreenSurface, QOpenGLContext,
                                   QSurfaceFormat)
        from PySide6.QtOpenGL import (QOpenGLFramebufferObject,
                                      QOpenGLFramebufferObjectFormat)
        from PySide6.QtWidgets import QApplication

        self.width, self.height = width, height
        self.app = QApplication.instance() or QApplication([])
        surface_format = QSurfaceFormat()
        surface_format.setSamples(8)
        surface_format.setDepthBufferSize(24)
        surface_format.setAlphaBufferSize(8)
        self.surface = QOffscreenSurface()
        self.surface.setFormat(surface_format)
        self.surface.create()
        self.context = QOpenGLContext()
        self.context.setFormat(surface_format)
        if not self.context.create():
            raise RuntimeError("no OpenGL context for offscreen rendering")
        self.context.makeCurrent(self.surface)

        fbo_format = QOpenGLFramebufferObjectFormat()
        fbo_format.setAttachment(
            QOpenGLFramebufferObject.Attachment.CombinedDepthStencil)
        fbo_format.setSamples(8)
        self.fbo = QOpenGLFramebufferObject(QSize(width, height), fbo_format)
        if not self.fbo.isValid():
            raise RuntimeError("could not create a %dx%d framebuffer"
                               % (width, height))
        self.fbo.bind()

        self.engine = None
        self.camera = None
        # Set per example by render_one: an influence-overlay page is lit
        # flatter, because there the surface colour is the data.
        self.overlay = False
        self.new_engine()

    def new_engine(self):
        """A FRESH engine for each stage.

        One engine handed a second stage keeps the first one's prims -- the
        lattice example rendered the IK arm -- so the cheap-looking reuse
        is not an option; only the GL context and the framebuffer are
        shared across examples.
        """
        from pxr import UsdImagingGL

        self.engine = None
        self.engine = UsdImagingGL.Engine()
        self.camera = None

    def set_camera(self, camera):
        from pxr import CameraUtil, Gf

        frustum = camera.frustum
        self.engine.SetRenderBufferSize(Gf.Vec2i(self.width, self.height))
        self.engine.SetFraming(CameraUtil.Framing(
            Gf.Range2f(Gf.Vec2f(0, 0), Gf.Vec2f(self.width, self.height)),
            Gf.Rect2i(Gf.Vec2i(0, 0), self.width, self.height)))
        self.engine.SetCameraState(frustum.ComputeViewMatrix(),
                                   frustum.ComputeProjectionMatrix())
        self.camera = camera
        self._light(frustum)

    def _light(self, frustum):
        """usdview's camera light plus one cool fill, so a 3/4 view has a
        lit side and a shaded side instead of one flat tone.

        An influence-overlay page raises the ambient floor: there the
        surface colour is DATA (the weight ramp, arriving as a
        displayColor primvar) and N.L shading of data reads the ramp
        backwards wherever the geometry tips away from the key.
        """
        from pxr import Glf

        material = Glf.SimpleMaterial()
        material.ambient = (OVERLAY_MATERIAL_AMBIENT if self.overlay
                            else (0.24, 0.25, 0.27, 1.0))
        material.specular = (0.10, 0.10, 0.11, 1.0)
        material.shininess = 24.0
        position = frustum.position
        view_inverse = frustum.ComputeViewInverse()

        key = Glf.SimpleLight()
        key.ambient = (0, 0, 0, 1)
        key.diffuse = (1.05, 1.02, 0.98, 1)
        key.specular = (0.35, 0.35, 0.35, 1)
        key.position = (position[0], position[1], position[2], 1)
        key.transform = view_inverse

        # A DIRECTIONAL fill (w = 0), up and to the camera's left. A
        # positional one would have to be placed at some multiple of the
        # asset size and would then fall off differently on every example.
        offset = view_inverse.TransformDir((-1.0, 0.9, 0.5))
        fill = Glf.SimpleLight()
        fill.ambient = (0, 0, 0, 1)
        fill.diffuse = (0.36, 0.40, 0.48, 1)
        fill.specular = (0, 0, 0, 1)
        fill.position = (offset[0], offset[1], offset[2], 0)
        self.engine.SetLightingState(
            [key, fill], material,
            OVERLAY_SCENE_AMBIENT if self.overlay else (0.16, 0.17, 0.20, 1.0))

    def select(self, paths):
        from pxr import Gf, Sdf

        self.engine.SetSelectionColor(Gf.Vec4f(1.0, 0.58, 0.16, 1.0))
        wanted = []
        for path in paths:
            wanted.append(Sdf.Path(path))
            wanted.append(Sdf.Path(path).AppendChild(CONTROL_GUIDE_CHILD))
        self.engine.SetSelected(wanted)

    def render(self, stage, time, ghost=False, guides=False, root=None):
        """One RGBA pass at supersampled size, background transparent.

        The dark gradient, the rest pose and the live pose are composited
        in PIL, which is the only way to put one draw over another at a
        chosen strength.

        `root` restricts what is drawn to one subtree -- pointed at the
        RigExecRoot it draws the synthesized guides and nothing else, which
        is how the guides are kept off the geometry they pose.
        """
        from OpenGL import GL
        from pxr import Gf, Usd, UsdImagingGL

        params = UsdImagingGL.RenderParams()
        params.frame = Usd.TimeCode(time)
        params.complexity = 1.1
        params.showProxy = True
        params.showRender = False
        params.gammaCorrectColors = False
        params.enableSampleAlphaToCoverage = True
        params.enableSceneMaterials = True
        params.enableSceneLights = False
        params.clearColor = Gf.Vec4f(0, 0, 0, 0)
        params.cullStyle = UsdImagingGL.CullStyle.CULL_STYLE_NOTHING
        params.showGuides = bool(guides)
        params.highlight = bool(guides)
        if ghost:
            # The rest pose is a WIREFRAME drawn over the live one. A
            # filled silhouette underneath read as a second object lying
            # on the floor in three of four pilots, and a deformation that
            # swells outwards hid it completely in the fourth.
            params.drawMode = UsdImagingGL.DrawMode.DRAW_WIREFRAME
            params.enableSceneMaterials = False
            params.overrideColor = Gf.Vec4f(GHOST_RGB[0], GHOST_RGB[1],
                                            GHOST_RGB[2], 1.0)
            params.wireframeColor = Gf.Vec4f(GHOST_RGB[0], GHOST_RGB[1],
                                             GHOST_RGB[2], 1.0)
        else:
            params.drawMode = UsdImagingGL.DrawMode.DRAW_WIREFRAME_ON_SURFACE
            params.wireframeColor = Gf.Vec4f(0.10, 0.11, 0.13, 0.85)

        # Rebinding every pass is not belt and braces: Storm leaves a
        # different framebuffer bound when it is done, and without this the
        # second and every later pass silently presents somewhere else and
        # the read-back returns the previous frame.
        self.fbo.bind()
        GL.glViewport(0, 0, self.width, self.height)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LESS)
        GL.glDepthMask(GL.GL_TRUE)
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        GL.glClearColor(0, 0, 0, 0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        subtree = root if root is not None else stage.GetPseudoRoot()
        self.engine.Render(subtree, params)
        for _ in range(64):
            if self.engine.IsConverged():
                break
            self.engine.Render(subtree, params)
        self.fbo.bind()
        return _to_pil(self.fbo.toImage())

    def close(self):
        # Order matters: Storm's context arena still holds framebuffers
        # that name Qt's FBO, so the engine has to go first and the
        # context has to still be current while it does.
        self.engine = None
        self.fbo = None
        if self.context:
            self.context.doneCurrent()
        self.context = None
        self.surface = None


def _to_pil(qimage):
    from PySide6.QtGui import QImage

    converted = qimage.convertToFormat(QImage.Format.Format_RGBA8888)
    width, height = converted.width(), converted.height()
    bits = converted.constBits()
    data = bytes(bits)[:width * height * 4]
    return Image.frombytes("RGBA", (width, height), data)


# ---------------------------------------------------------------------------
# Frame composition
# ---------------------------------------------------------------------------

def _gradient(width, height, top, bottom):
    band = Image.new("RGB", (1, height))
    pen = band.load()
    for y in range(height):
        t = y / float(max(1, height - 1))
        pen[0, y] = tuple(int(round(top[i] + (bottom[i] - top[i]) * t))
                          for i in range(3))
    return band.resize((width, height), Image.NEAREST)


def _gradient_h(width, height, left, right):
    """A left-to-right ramp, for the legend's weight swatch."""
    band = Image.new("RGB", (width, 1))
    pen = band.load()
    for x in range(width):
        t = x / float(max(1, width - 1))
        pen[x, 0] = tuple(int(round(left[i] + (right[i] - left[i]) * t))
                          for i in range(3))
    return band.resize((width, height), Image.NEAREST)


def _project(camera, point, width, height):
    """World point -> pixel in the viewport, or None if behind the eye."""
    frustum = camera.frustum
    matrix = frustum.ComputeViewMatrix() * frustum.ComputeProjectionMatrix()
    x = (point[0] * matrix[0][0] + point[1] * matrix[1][0] +
         point[2] * matrix[2][0] + matrix[3][0])
    y = (point[0] * matrix[0][1] + point[1] * matrix[1][1] +
         point[2] * matrix[2][1] + matrix[3][1])
    w = (point[0] * matrix[0][3] + point[1] * matrix[1][3] +
         point[2] * matrix[2][3] + matrix[3][3])
    if w <= 1e-9:
        return None
    return ((x / w * 0.5 + 0.5) * width, (0.5 - y / w * 0.5) * height)


def _clip_segment(a, b, width, height):
    """Liang-Barsky: the part of a-b inside (0,0)-(width,height), or None.

    PIL clips what it draws to the image it draws into, but `_dashed`
    walks the segment in dash-sized steps FIRST, and a point projected
    close to the eye plane lands hundreds of thousands of pixels away: the
    walk then costs that many no-op iterations. Clipping first makes the
    walk proportional to what is actually drawn.
    """
    x0, y0 = float(a[0]), float(a[1])
    x1, y1 = float(b[0]), float(b[1])
    dx, dy = x1 - x0, y1 - y0
    low, high = 0.0, 1.0
    for edge, distance in ((-dx, x0), (dx, width - x0),
                           (-dy, y0), (dy, height - y0)):
        if abs(edge) < 1e-12:
            if distance < 0:
                return None        # parallel to this edge and outside it
            continue
        cut = distance / edge
        if edge < 0:
            if cut > high:
                return None
            low = max(low, cut)
        else:
            if cut < low:
                return None
            high = min(high, cut)
    return ((x0 + dx * low, y0 + dy * low), (x0 + dx * high, y0 + dy * high))


def _dashed(pen, a, b, fill, width=1, dash=6, gap=5, clip=None):
    if clip is not None:
        clipped = _clip_segment(a, b, clip[0], clip[1])
        if clipped is None:
            return
        a, b = clipped
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return
    ux, uy = dx / length, dy / length
    walked = 0.0
    while walked < length:
        end = min(walked + dash, length)
        pen.line([(a[0] + ux * walked, a[1] + uy * walked),
                  (a[0] + ux * end, a[1] + uy * end)], fill=fill, width=width)
        walked = end + gap


_ANGLE_AVARS = ("rx", "ry", "rz", "rspin")


_VALUE_CHARS = 26          # how much value text a chip carries comfortably


def _is_matrix4(value):
    try:
        return len(value) == 4 and len(value[0]) == 4
    except TypeError:
        return False


def _matrix_channels(value):
    """(label, number, is_angle) for the parts of a matrix a reader reads.

    The translation is the last row, and the rotation is taken about Z
    from the first row -- atan2(m[0][1], m[0][0]) -- because USD is
    row-vector and the docs examples that publish a matrix turn in the
    view plane. A full Euler triple would print three numbers where two
    of them are always zero.
    """
    rz = math.degrees(math.atan2(float(value[0][1]), float(value[0][0])))
    return (("tx", float(value[3][0]), False),
            ("ty", float(value[3][1]), False),
            ("tz", float(value[3][2]), False),
            ("rz", rz, True))


def _vector_channels(value):
    return tuple((axis, float(value[index]), False)
                 for index, axis in enumerate("xyz"))


def _moving(attr, samples, channels, tolerance=1e-4):
    """Which channel labels actually move over the sampled frames.

    A chip that prints "tx +0.00  ty +0.00  tz +0.00  rz +44.5" spends
    three quarters of its width on numbers that are the same in every
    frame of the loop, and the one that is not gets lost among them.
    """
    if not samples:
        return set(label for label, _v, _a in channels)
    low, high = {}, {}
    for time in samples:
        value = attr.Get(time)
        if value is None:
            continue
        for label, number, _angle in channels(value):
            low[label] = min(low.get(label, number), number)
            high[label] = max(high.get(label, number), number)
    return set(label for label in low
               if high[label] - low[label] > tolerance)


def _channel_text(channels, moving, angle_format="%+.1f\u00b0",
                  number_format="%+.2f", keep_still=False):
    """Format channels, dropping the still ones when the chip gets wide."""
    def render(kept):
        return "  ".join(
            "%s %s" % (label, (angle_format if angle else number_format)
                       % number)
            for label, number, angle in kept)

    changing = [c for c in channels if c[0] in moving]
    if keep_still:
        text = render(channels)
        if len(text) <= _VALUE_CHARS or not changing:
            return text
        return render(changing)
    if changing:
        return render(changing)
    # Nothing moves: print what is at least not zero, so a static driver
    # still names a number rather than a row of +0.00.
    alive = [c for c in channels if abs(c[1]) > 1e-4]
    return render(alive) if alive else None


def _is_vector_array(value):
    """A point3f[]/vec3f[] of more than one element."""
    try:
        if len(value) < 1 or _is_matrix4(value):
            return False
        item = value[0]
        return len(item) == 3 and all(
            isinstance(float(c), float) for c in item)
    except (TypeError, ValueError):
        return False


def _moving_element(attr, samples, time=None):
    """(index, axis, value at `time`) for the element that travels most.

    The one number in an animated point array that is worth printing: the
    element with the largest travel over the sampled frames, on its own
    most-moving axis. Returns None when nothing moves or there are no
    samples to measure against.
    """
    if not samples:
        return None
    low, high = {}, {}
    for when in samples:
        value = attr.Get(when)
        if value is None:
            continue
        for index, item in enumerate(value):
            for axis in range(3):
                number = float(item[axis])
                key = (index, axis)
                low[key] = min(low.get(key, number), number)
                high[key] = max(high.get(key, number), number)
    if not low:
        return None
    key = max(low, key=lambda k: high[k] - low[k])
    if high[key] - low[key] < 1e-4:
        return None
    value = attr.Get(samples[0] if time is None else time)
    if value is None or key[0] >= len(value):
        return None
    return key[0], "xyz"[key[1]], float(value[key[0]][key[1]])


def _value_text(attr, time, divisions=None, samples=None):
    """One "attribute value" pair for the on-screen readout, or None.

    Always an AUTHORED attribute. The chip used to print a derived
    statistic for array-valued drivers ("points dmax 1.27") -- a number a
    reader cannot find anywhere in usdview or in the .usda, naming no
    parameter they could change.
    """
    name = attr.GetBaseName()
    value = attr.Get(time)
    if value is None:
        return None
    if isinstance(value, (float, int)):
        if name in _ANGLE_AVARS:
            return "%s %+.1f\u00b0" % (name, float(value))
        return "%s %+.2f" % (name, float(value))
    try:
        count = len(value)
    except TypeError:
        return None
    # A matrix channel (`posed:space`) used to fall through to the array
    # branch and print "space  4" -- the row count, which is 4 for every
    # matrix ever authored. It prints the translation and the Z rotation
    # the matrix carries instead, in the same shape as a scalar avar.
    if _is_matrix4(value):
        channels = _matrix_channels(value)
        return _channel_text(
            channels, _moving(attr, samples, _matrix_channels))
    try:
        numbers = [float(c) for c in value[:3]]
    except (TypeError, ValueError):
        numbers = None
    if numbers is not None:
        if count == 3:
            channels = _vector_channels(value)
            return _channel_text(
                channels, _moving(attr, samples, _vector_channels),
                number_format="%.2f", keep_still=True)
        return "%s (%s)" % (name, ", ".join("%+.2f" % c for c in numbers))
    if divisions:
        return "%s  %s" % (name, "\u00d7".join(str(d) for d in divisions))
    if _is_vector_array(value):
        # A driver whose animation IS an array of points -- a swept curve,
        # a ribbon's CVs -- used to print the array's LENGTH, which is the
        # same integer in every frame of the loop: a chip that labels
        # nothing, with a leader across the frame to reach it. Print the
        # element that actually travels, by name, so the number is one a
        # reader can watch move AND can find in the .usda.
        moved = _moving_element(attr, samples, time)
        if moved is not None:
            index, axis, number = moved
            return "%s[%d].%s %+.2f" % (name, index, axis, number)
        first = value[0]
        return "%s[0] (%s)" % (name, ", ".join("%+.2f" % float(c)
                                               for c in first))
    return "%s  %d" % (name, count)


class FrameState(object):
    """Everything about one sampled frame that the overlays need."""

    __slots__ = ("controls", "joints", "cages", "readouts")

    def __init__(self):
        self.controls = {}
        self.joints = {}
        self.cages = []     # [(world points, divisions)]
        self.readouts = []  # [(label, world point or None)]


def frame_state(scene, time):
    from pxr import Gf, Usd, UsdGeom

    state = FrameState()
    for rig in scene.rigs:
        pose = rig.evaluate(time)
        for path in pose.control_paths():
            origin = pose.control_frame(path).origin
            state.controls[path] = scene.root_xform.Transform(
                Gf.Vec3d(origin[0], origin[1], origin[2]))
        for path in pose.joint_paths():
            origin = pose.joint_frame(path).origin
            state.joints[path] = scene.root_xform.Transform(
                Gf.Vec3d(origin[0], origin[1], origin[2]))

    cache = UsdGeom.XformCache(Usd.TimeCode(time))
    cage_divisions = {}
    for prim, divisions in scene.cages:
        attr = prim.GetAttribute("points")
        value = attr.Get(Usd.TimeCode(time)) if attr else None
        if not value:
            continue
        matrix = cache.GetLocalToWorldTransform(prim)
        state.cages.append(([matrix.Transform((p[0], p[1], p[2]))
                             for p in value], divisions))
        cage_divisions[str(prim.GetPath())] = divisions

    for prim, attrs in scene.drivers:
        path = str(prim.GetPath())
        divisions = cage_divisions.get(path)
        bits = [t for t in (_value_text(a, time, divisions, scene.samples)
                            for a in attrs)
                if t]
        label = prim.GetName()
        if bits:
            label += "   " + "   ".join(bits[:2])
        point = state.controls.get(path)
        if point is None and path in cage_divisions:
            for points, _divisions in state.cages:
                centre = Gf.Vec3d(0, 0, 0)
                for item in points:
                    centre += item
                point = centre / float(len(points))
                break
        if point is None:
            box = UsdGeom.BBoxCache(
                Usd.TimeCode(time), ["default", "render", "proxy", "guide"],
                False).ComputeWorldBound(prim).ComputeAlignedBox()
            if not box.IsEmpty():
                point = (box.GetMin() + box.GetMax()) * 0.5
        state.readouts.append((label, point))
    return state


class Chrome(object):
    """The static furniture: gradient, title strip, legend.

    The canvas is the title strip and the viewport, and nothing else. A
    node-network strip used to sit underneath: a second picture, with its
    own vocabulary, that a reader had to decode before it said anything
    the page's prose did not already say. The one thing it carried that
    the viewport does not -- which handle drives which joint drives which
    geometry -- is now one line of TEXT in this strip (`Scene.chain`).
    """

    # The swatches ARE the colours the frame uses -- read straight off
    # the constants, so the key cannot drift away from the picture.
    LEGEND = (("driver", ACCENT),
              ("rest", tuple(int(c * 255) for c in GHOST_RGB)),
              ("joints", tuple(int(c * 255) for c in JOINT_RGB)),
              ("mesh", (118, 120, 124)))

    @staticmethod
    def _key(scene):
        """The key for THIS picture: no swatch without a referent.

        A fixed four-swatch key promises things a given page may not
        contain -- a `joints` swatch on a stage with no joint at all, an
        orange `driver` swatch on a page whose driver is a mover prim with
        no position, so the only accent in frame is the chip outline and
        the progress bar. And the `rest` swatch was drawn at full
        GHOST_RGB while the ghost itself is composited at 0.42 to 0.90
        over a dark gradient, which is a different colour: the swatch is
        now mixed down by the same alpha the frame uses.
        """
        alpha = (GHOST_ALPHA_OVERLAY if scene.overlay_weight
                 else GHOST_ALPHA_MOVER if scene.is_mover else GHOST_ALPHA)
        ghost = tuple(int(c * 255 * alpha + BG_BOTTOM[i] * (1 - alpha))
                      for i, c in enumerate(GHOST_RGB))
        key = [("driver" if scene.driver_anchored else "value", ACCENT),
               ("rest", ghost)]
        if scene.has_joints:
            key.append(("joints", tuple(int(c * 255) for c in JOINT_RGB)))
        key.append(("mesh", (118, 120, 124)))
        return key

    def __init__(self, scene, notes):
        self.title_font = _font("seguisb.ttf", 15)
        self.summary_font = _font("segoeui.ttf", 11)
        self.chain_font = _font("segoeui.ttf", 11)
        self.legend_font = _font("segoeui.ttf", 10)
        self.label_font = _font("seguisb.ttf", 11)
        self.tag_font = _font("segoeui.ttf", 10)
        self.height = TITLE_H + VIEW_H

        base = Image.new("RGB", (CANVAS_W, self.height), TITLE_BG)
        base.paste(_gradient(VIEW_W, VIEW_H, BG_TOP, BG_BOTTOM), (0, TITLE_H))
        pen = ImageDraw.Draw(base)
        pen.line([(0, TITLE_H - 1), (CANVAS_W, TITLE_H - 1)], fill=RULE)

        title = notes.get("title", scene.name)
        pen.text((14, 6), title, font=self.title_font, fill=TEXT)
        offset = 14 + pen.textlength(title, font=self.title_font) + 12
        summary = notes.get("summary", "")
        if summary:
            pen.text((offset, 9),
                     _fit_text(pen, summary, self.summary_font,
                               CANVAS_W - offset - 12),
                     font=self.summary_font, fill=TEXT_DIM)

        # The legend is the colour law, printed. A reader learns "orange =
        # the handle, blue wire = where it started, steel = joints, grey =
        # mesh" once and carries it to all 22 pages.
        legend = self._key(scene)
        if scene.overlay_weight:
            # A fifth entry only where there is a fifth colour to read: the
            # weight ramp is painted on the geometry on these pages and
            # nowhere else, and an entry for it on the other 20 would be a
            # key to something not in the picture.
            legend.append(("weight 0 → 1",
                           (WEIGHT_RAMP_OFF, WEIGHT_RAMP_ON)))
        x = CANVAS_W - 12
        for name, colour in reversed(legend):
            width = pen.textlength(name, font=self.legend_font)
            x -= width
            pen.text((x, 31), name, font=self.legend_font, fill=TEXT_DIM)
            x -= 13
            if isinstance(colour[0], tuple):
                # The ramp, not a sample of one end of it: what the reader
                # has to match against the geometry is the GRADIENT.
                x -= 14
                base.paste(_gradient_h(23, 9, colour[0], colour[1]),
                           (int(x), 33))
                pen.rectangle([x, 33, x + 22, 41], outline=RULE)
            else:
                pen.rectangle([x, 33, x + 8, 41], fill=colour)
            x -= 9
        if scene.chain:
            pen.text((14, 31),
                     _fit_text(pen, scene.chain, self.chain_font, x - 24),
                     font=self.chain_font, fill=(196, 205, 216))
        self.base = base


def _fit_text(pen, text, font, width):
    if pen.textlength(text, font=font) <= width:
        return text
    while text and pen.textlength(text + "\u2026", font=font) > width:
        text = text[:-1]
    return text + "\u2026"


def _faded(image, amount):
    out = image.copy()
    out.putalpha(out.getchannel("A").point(lambda a: int(a * amount)))
    return out


def _place_chips(pen, font, wanted, blockers=()):
    """Chips inside the safe area, off the subject, and off each other.

    `wanted` is [(anchor pixel, preferred side, text, force margin, ring)],
    where `ring` says whether an accent ring was drawn at that anchor --
    a chip with no ring gets no leader, because a leader's whole job is to
    tie a chip to a ring and one drawn from the corner slot is a stub
    leaving the frame and pointing at nothing.
    `blockers` is [(x0, y0, x1, y1)] of things already drawn that a chip
    must not land on -- the driver rings it labels and the control name
    tags -- because a chip is only a readout if the reader can read both
    it and what it sits on. Returns the boxes to draw.
    """
    taken = [tuple(box) for box in blockers]
    boxes = []
    for (ax, ay), side, text, margin, ring in wanted:
        width = pen.textlength(text, font=font) + 14
        x = ax + 14 if side >= 0 else ax - 14 - width
        # A driver near the middle of the frame is ON the asset it drives;
        # a chip 14px away from it is on the asset too. Those go out to
        # the margin and keep a leader back to the ring. The test is made
        # once per loop (`_chip_placement`) rather than per frame.
        if margin:
            x = SAFE if side >= 0 else VIEW_W - SAFE - width
        y = ay - 24
        x = min(max(x, SAFE), VIEW_W - SAFE - width)
        y = min(max(y, SAFE), VIEW_H - SAFE - 20)
        for _attempt in range(12):
            clash = None
            for other in taken:
                if (x < other[2] + 4 and x + width > other[0] - 4 and
                        y < other[3] + 3 and y + 19 > other[1] - 3):
                    clash = other
                    break
            if clash is None:
                break
            y = clash[3] + 5
            if y + 19 > VIEW_H - SAFE:
                y = max(SAFE, clash[1] - 24)
        taken.append((x, y, x + width, y + 19))
        boxes.append((x, y, x + width, y + 19, text, (ax, ay), ring))
    return boxes


def compose(chrome, passes, state, scene, index, total):
    grid, live, ghost, guides = passes
    view = Image.new("RGBA", (live.size), (0, 0, 0, 0))
    if grid is not None:
        view.alpha_composite(grid)
    view.alpha_composite(live)
    if ghost is not None:
        # On an overlay page the live surface carries the whole message,
        # so the rest-pose wire is pulled back off it.
        view.alpha_composite(_faded(
            ghost, GHOST_ALPHA_OVERLAY if scene.overlay_weight
            else GHOST_ALPHA_MOVER if scene.is_mover else GHOST_ALPHA))
    if guides is not None:
        # The guides are drawn HERE and nowhere else, at partial strength,
        # so the mesh the rig is deforming is visible through the skeleton
        # deforming it. In the live pass they were opaque and on top, and
        # the only geometry a reader could see was the rest-pose ghost.
        view.alpha_composite(_faded(guides, GUIDE_XRAY))
    view = view.resize((VIEW_W, VIEW_H), Image.LANCZOS)

    frame = chrome.base.copy()
    frame.paste(view, (0, TITLE_H), view)
    # Every overlay is drawn into a VIEW_W x VIEW_H layer and composited
    # at the viewport's origin, so a projected pixel OUTSIDE the viewport
    # -- an aim target swinging below the bottom edge, a control leader
    # running off the side -- is clipped by the layer instead of being
    # painted over the title strip or off the canvas. It used to run over
    # both: the aim example drew its dashed links down past the picture.
    overlay = Image.new("RGBA", (VIEW_W, VIEW_H), (0, 0, 0, 0))
    pen = ImageDraw.Draw(overlay, "RGBA")
    clip = (VIEW_W, VIEW_H)

    def to_pixel(point):
        """World point -> pixel in VIEWPORT coordinates (0,0 top left)."""
        return _project(scene.camera, point, VIEW_W, VIEW_H)

    # 1. the lattice cage, as a cage. Twelve disconnected points are
    #    confetti; the operator is the grid they form.
    for points, divisions in state.cages:
        pixels = [to_pixel(p) for p in points]
        if divisions and len(divisions) == 3:
            nx, ny, nz = divisions
            if nx * ny * nz == len(points):
                def at(i, j, k):
                    return pixels[i + nx * (j + ny * k)]
                for k in range(nz):
                    for j in range(ny):
                        for i in range(nx):
                            for di, dj, dk in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
                                if (i + di < nx and j + dj < ny and
                                        k + dk < nz):
                                    a, b = at(i, j, k), at(i + di, j + dj,
                                                           k + dk)
                                    if not a or not b:
                                        continue
                                    edge = _clip_segment(a, b, VIEW_W, VIEW_H)
                                    if edge:
                                        pen.line(list(edge),
                                                 fill=ACCENT + (205,), width=2)
        for pixel in pixels:
            if pixel and -4 <= pixel[0] <= VIEW_W + 4 and \
                    -4 <= pixel[1] <= VIEW_H + 4:
                pen.rectangle([pixel[0] - 2.5, pixel[1] - 2.5,
                               pixel[0] + 2.5, pixel[1] + 2.5],
                              fill=ACCENT + (255,))

    # 2. the relationship the operator IS: pole to elbow, effector to
    #    wrist, aim to target. Accent when the driver is on one end.
    for control, joint, _rel in scene.links[:8]:
        a = state.controls.get(control)
        b = state.joints.get(joint)
        if a is None or b is None:
            continue
        pa, pb = to_pixel(a), to_pixel(b)
        if not pa or not pb:
            continue
        live_link = control in scene.driver_paths
        _dashed(pen, pa, pb,
                (ACCENT + (225,)) if live_link else (STATIC_CTRL + (185,)),
                width=2 if live_link else 1, clip=clip)

    # 3. name every control in frame. An unlabelled ring reads as a second
    #    handle with a secret -- the IK pole control swings every frame and
    #    was never named.
    blockers = []
    for path in scene.controls:
        if path in scene.driver_paths:
            continue
        point = state.controls.get(path)
        pixel = to_pixel(point) if point is not None else None
        # A control the frame does not contain is not named: a label
        # clamped to the edge claims a position the control is not at,
        # which is the same lie as a ring clamped to the edge.
        if (pixel is None or not (0 <= pixel[0] <= VIEW_W) or
                not (0 <= pixel[1] <= VIEW_H)):
            continue
        name = path.rsplit("/", 1)[-1]
        span = pen.textlength(name, font=chrome.tag_font)
        x = min(max(pixel[0], SAFE), VIEW_W - SAFE)
        y = min(max(pixel[1], SAFE), VIEW_H - SAFE)
        # Flip to the left of the ring rather than let the name run out of
        # the viewport and be cut in half by the clip.
        offset = 8 if x + 8 + span <= VIEW_W - 4 else -(span + 8)
        # The same plate the value chip carries. A name drawn as bare text
        # with a one-pixel shadow is the least readable thing in the
        # frame the moment it lands on a wireframe cage or on the mesh,
        # which is exactly where a control sits.
        pen.rounded_rectangle([x + offset - 4, y - 9, x + offset + span + 3,
                               y + 6], radius=3, fill=(16, 18, 22, 228))
        pen.text((x + offset, y - 7), name, font=chrome.tag_font,
                 fill=(203, 215, 227, 245))
        blockers.append((x + offset - 4, y - 9, x + offset + span + 3, y + 6))

    # 4. the animated driver: ring, leader, and the authored value.
    #
    # A page can carry a dozen handles keyed off one animation -- the
    # control page drives all twelve guides with the same three splines --
    # and three chips printing the same string three times says only that
    # the renderer does not know they are the same. One chip, counted.
    drawn = list(state.readouts[:3])
    if scene.collapse_chips and drawn:
        kind = ("controls" if all(p.GetTypeName() == "RigExecControl"
                                  for p, _a in scene.drivers) else "drivers")
        drawn = [("%d %s   %s" % (len(scene.drivers), kind,
                                  drawn[0][0].partition("   ")[2]),
                  drawn[0][1])]
    wanted = []
    for index, (label, point) in enumerate(drawn):
        pixel = to_pixel(point) if point is not None else None
        # A driver the frame does not contain gets a chip and no ring: a
        # ring clamped to the edge claims a position the control is not at.
        offscreen = (pixel is None or not (0 <= pixel[0] <= VIEW_W) or
                     not (0 <= pixel[1] <= VIEW_H))
        if offscreen:
            # The fixed corner slot, and no ring -- so no leader either.
            wanted.append(((SAFE + 8, VIEW_H - 46), 1, label, False, False))
            continue
        x = min(max(pixel[0], SAFE), VIEW_W - SAFE)
        y = min(max(pixel[1], SAFE), VIEW_H - SAFE)
        pen.ellipse([x - 9, y - 9, x + 9, y + 9], outline=ACCENT + (240,),
                    width=2)
        pen.ellipse([x - 2, y - 2, x + 2, y + 2], fill=ACCENT + (255,))
        # Chips go AWAY from the subject, not over it -- and never over the
        # ring they label.
        blockers.append((x - 11, y - 11, x + 11, y + 11))
        side, margin = scene.chip_sides.get(
            index, (1 if x < VIEW_W / 2 else -1,
                    abs(x - VIEW_W / 2.0) < VIEW_W * 0.16))
        wanted.append(((x, y), side, label, margin, True))
    placed = _place_chips(pen, chrome.label_font, wanted, blockers)
    # Leaders first, then every chip, so a later chip's leader cannot be
    # drawn across an earlier chip's text.
    for x0, y0, _x1, _y1, _text, (ax, ay), ring in placed:
        if ring and math.hypot(ax - x0 - 6, ay - y0 - 9) > 12:
            leader = _clip_segment((ax, ay), (x0 + 6, y0 + 9), VIEW_W, VIEW_H)
            if leader:
                pen.line(list(leader), fill=ACCENT + (150,), width=1)
    for x0, y0, x1, y1, text, _anchor, _ring in placed:
        pen.rounded_rectangle([x0, y0, x1, y1], radius=4,
                              fill=(16, 18, 22, 228), outline=ACCENT + (215,),
                              width=1)
        pen.text((x0 + 7, y0 + 3), text, font=chrome.label_font,
                 fill=(255, 226, 190))

    # 5. where in the loop we are. The old "f 1006 (1001-1012)" counter was
    #    a debugging artifact that snapped at the wrap and told a docs
    #    reader nothing.
    bar_y = VIEW_H - 9
    pen.rectangle([SAFE, bar_y, VIEW_W - SAFE, bar_y + 3],
                  fill=(255, 255, 255, 38))
    span = VIEW_W - 2 * SAFE
    filled = span * (index / float(max(1, total - 1)))
    pen.rectangle([SAFE, bar_y, SAFE + filled, bar_y + 3],
                  fill=ACCENT + (230,))

    frame.paste(overlay, (0, TITLE_H), overlay)
    return frame.convert("RGB")


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def _write_gif(frames, path, fps=GIF_FPS):
    """Assemble the GIF with ffmpeg's palettegen/paletteuse.

    PIL can write a GIF too, and writes a visibly worse one: a per-frame
    median-cut palette banded the viewport gradient and crawled between
    frames. Rather than quietly producing that, a missing ffmpeg is an
    error the run reports.
    """
    if not FFMPEG or not os.path.exists(FFMPEG):
        raise RuntimeError("ffmpeg not found (looked at %r); put it on PATH "
                           "or set FFMPEG" % FFMPEG)
    workdir = tempfile.mkdtemp(prefix="rigexec-gif-")
    try:
        for index, frame in enumerate(frames):
            frame.save(os.path.join(workdir, "f%04d.png" % index))
        # reserve_transparent=0 is what actually keeps a transparent entry
        # out of the palette. Without it palettegen spends one of its
        # colours on a transparent index whether or not any pixel uses it,
        # every reader reports the GIF as "has transparency", and a
        # converter that honours the entry can bring back the
        # pastel-shapes-on-a-white-page bug this pipeline replaced.
        # -gifflags -transdiff additionally turns off transparent-index
        # delta encoding between frames.
        filters = ("split[a][b];[a]palettegen=max_colors=%d:"
                   "stats_mode=diff:reserve_transparent=0[p];"
                   "[b][p]paletteuse=dither=sierra2_4a:diff_mode=rectangle"
                   % GIF_COLORS)
        done = subprocess.run(
            [FFMPEG, "-y", "-v", "error", "-framerate", str(fps),
             "-i", os.path.join(workdir, "f%04d.png"),
             "-vf", filters, "-gifflags", "-transdiff",
             "-loop", "0", path],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if done.returncode != 0:
            # ffmpeg says exactly what it disliked on stderr, and a bare
            # CalledProcessError throws that away: the run then reports
            # "returned non-zero exit status 1" and nothing else.
            tail = (done.stderr or b"").decode("utf-8", "replace").strip()
            tail = " | ".join(tail.splitlines()[-4:]) or "(no stderr)"
            raise RuntimeError("ffmpeg exit %d writing %s: %s"
                               % (done.returncode, os.path.basename(path),
                                  tail))
    finally:
        shutil.rmtree(workdir, ignore_errors=True)
    _assert_opaque(path)


def _assert_opaque(path):
    """No frame declares a transparent index at all, or the build says so.

    The previous pipeline asserted opacity in prose and was wrong: frames
    1..22 used the transparent index on tens of thousands of pixels and
    only looked opaque because frame 0 was complete and the disposal
    method happened to be 1. The check is now the stronger one PIL can
    make -- no `transparency` key in any frame's info -- because that is
    the thing every downstream reader looks at.
    """
    with Image.open(path) as gif:
        index = 0
        while True:
            if gif.info.get("transparency") is not None:
                raise RuntimeError(
                    "%s frame %d declares a transparent palette index"
                    % (os.path.basename(path), index))
            index += 1
            try:
                gif.seek(index)
            except EOFError:
                break


def _write_sheet(frames, path, across=6, down=1):
    picks = across * down
    chosen = [frames[int(round(i * (len(frames) - 1) / float(picks - 1)))]
              for i in range(picks)]
    width, height = chosen[0].size
    sheet = Image.new("RGB", (width * across, height * down), (0, 0, 0))
    for index, frame in enumerate(chosen):
        sheet.paste(frame, ((index % across) * width,
                            (index // across) * height))
    sheet.save(path)


def _sequence(scene, frames):
    """The frame order the GIF plays, with the two ends held.

    Evenly spaced samples fly through the extreme in one tick; a reader
    needs to see "before" and "after" stand still long enough to compare
    them. Repeated frames rather than per-frame delays, because ffmpeg's
    constant-rate encoder does not vary them.
    """
    order = []
    extreme = extreme_sample(scene)
    if not scene.pingpong and len(frames) > 2:
        # A loop that closes ends where it started, so the last sample IS
        # the first one: the GIF wrapped onto a duplicate frame, which
        # HOLD_FIRST then dwelt on for five more ticks and which read as a
        # hitch at the seam. Dropping it is the only way to have both a
        # closed loop and no repeat -- ending the stage one frame early
        # instead makes `_loop_closes` false and doubles the whole GIF.
        frames = frames[:-1]
    for index, frame in enumerate(frames):
        repeat = 1
        if index == 0:
            repeat = HOLD_FIRST
        elif index == extreme:
            repeat = HOLD_EXTREME
        order += [frame] * repeat
    if scene.pingpong and len(frames) > 2:
        order += frames[-2:0:-1]
    return order


def page_notes(name, page=None):
    """The OPERATORS entry for the PAGE being rendered, and its key.

    Keyed on the page rather than on the stage, because an entry may name
    a shared `example_key`: several pages then document different nodes of
    one stage, and "which node is this picture about?" is a question only
    the page can answer. With no page given the stage's own key is the
    page, and an entry that merely borrows this stage is the fallback.
    """
    import operator_notes

    table = operator_notes.OPERATORS
    key = page or name
    if key in table:
        return key, table[key]
    for candidate, notes in table.items():
        if notes.get("example_key") == key:
            return candidate, notes
    return key, {"title": key, "summary": ""}


def render_one(src, viewport, bridge, out_dir, sheet=False,
               samples=TARGET_SAMPLES, page=None, out_name=None):
    """Render one example to <out_dir>/<out_name or stage name>.gif.

    `out_name` exists because a page may borrow another page's stage: the
    picture is about the borrowing page and has to be written under *its*
    key, or the second page would overwrite the first one's GIF.
    """
    # Before anything opens the stage: the imaging registry still holds
    # whichever stage was rendered last, and would keep evaluating that one.
    bridge.release()
    name = os.path.splitext(os.path.basename(src))[0]
    _key, notes = page_notes(name, page)
    name = out_name or name
    scene = prepare_stage(src, samples=samples, schema=notes.get("schema"))

    roots = [str(p.GetPath()) for p in scene.stage.Traverse()
             if p.GetTypeName() == "RigExecRoot"]

    chrome = Chrome(scene, notes)
    viewport.overlay = bool(scene.overlay_weight)
    viewport.new_engine()
    viewport.set_camera(scene.camera)
    viewport.select(scene.driver_paths)

    # Before ANY render, because the first one is what activates the rig
    # and the registry hands the pending selection to each new session so
    # its very first generation already carries the field.
    if scene.overlay_weight:
        bridge.weight_overlay(scene.overlay_weight)

    rig_prim = (scene.stage.GetPrimAtPath(roots[0]) if roots else None)
    grid_prim = scene.stage.GetPrimAtPath(GRID_PATH)
    # The grid is guide-purpose and the live pass has guides off, so it
    # gets its own pass -- once, since it never moves -- composited under
    # everything.
    grid = (viewport.render(scene.stage, scene.samples[0], guides=True,
                            root=grid_prim) if grid_prim else None)
    ghost = viewport.render(scene.stage, scene.samples[0], ghost=True)
    # Storm draws a one-pixel wireframe, which at the supersampled size is
    # half a pixel once reduced and vanishes over a dark gradient. The
    # rest pose is the single most important cue on every mover page, so
    # it is dilated before it is composited.
    ghost = ghost.filter(ImageFilter.MaxFilter(3))
    frames = []
    try:
        for index, time in enumerate(scene.samples):
            live = viewport.render(scene.stage, time)
            guides = (viewport.render(scene.stage, time, guides=True,
                                      root=rig_prim) if rig_prim else None)
            frames.append(compose(chrome, (grid, live, ghost, guides),
                                  frame_state(scene, time), scene, index,
                                  len(scene.samples)))
    finally:
        # The selection outlives this stage -- it is remembered across
        # activations on purpose -- so it is cleared here rather than at
        # the start of the next example, which may not be a weight page
        # and would otherwise render with this one's overlay still armed.
        if scene.overlay_weight:
            bridge.weight_overlay("")

    os.makedirs(out_dir, exist_ok=True)
    gif = os.path.join(out_dir, name + ".gif")
    _write_gif(_sequence(scene, frames), gif)
    if sheet:
        _write_sheet(frames, os.path.join(out_dir, name + "_sheet.png"))
    return gif


def page_stage(key, notes=None):
    """The stage a page's picture is rendered from.

    A page normally owns `docs/examples/<key>.usda`; `example_key` points
    at another page's stage, which is then rendered once per page that
    borrows it rather than once per file.
    """
    if notes is None:
        import operator_notes
        notes = operator_notes.OPERATORS.get(key, {})
    stem = notes.get("example_key", key)
    return os.path.join(RIG, "docs", "examples", stem + ".usda")


def page_jobs():
    """(page key, stage) for every page with a GIF, in CATEGORIES order."""
    import operator_notes

    jobs = []
    seen = set()
    ordered = [k for _title, keys in operator_notes.CATEGORIES for k in keys]
    ordered += [k for k in operator_notes.OPERATORS if k not in ordered]
    for key in ordered:
        notes = operator_notes.OPERATORS.get(key)
        if notes is None or notes.get("no_gif") or key in seen:
            continue
        seen.add(key)
        jobs.append((key, page_stage(key, notes)))
    return jobs


def _short_path(path):
    """`path` relative to the repo, or as given when it is not under it.

    os.path.relpath raises on Windows when the two paths sit on different
    drives, and a stage rendered from a scratch directory on another
    volume is a legitimate thing to ask for -- the report line at the end
    of a successful render should not be what kills the run.
    """
    try:
        return os.path.relpath(path, RIG)
    except ValueError:
        return path


def main(argv):
    import argparse

    parser = argparse.ArgumentParser(prog="render_media.py")
    parser.add_argument("stages", nargs="*")
    parser.add_argument("--page", help="render as this docs page: the GIF is "
                        "named after the page and titled from its "
                        "OPERATORS entry, whatever stage it comes from")
    parser.add_argument("--out", default=os.path.join(RIG, "docs", "gifs"))
    parser.add_argument("--samples", type=int, default=TARGET_SAMPLES)
    parser.add_argument("--sheet", action="store_true",
                        help="also write <name>_sheet.png, 6 frames across")
    parser.add_argument("--list", action="store_true", dest="list_only",
                        help="print the pages a bare run would render, "
                             "as \"<key> <- <stage>\", and render nothing")
    args = parser.parse_args(argv[1:])

    files = [a if os.path.isabs(a) else os.path.join(RIG, a)
             for a in args.stages]
    # (page key or None, stage, output name or None)
    if args.page:
        jobs = [(args.page, src, args.page) for src in
                (files or [page_stage(args.page)])]
    elif files:
        # An explicit stage path keeps the old contract: the GIF is named
        # after the stage, and page_notes falls back to the stage's key.
        jobs = [(None, src, None) for src in files]
    else:
        jobs = [(key, src, key) for key, src in page_jobs()]

    if args.list_only:
        for key, src, _name in jobs:
            label = key or os.path.splitext(os.path.basename(src))[0]
            print("%s <- %s" % (label, _short_path(src)))
        return 0

    bridge = ImagingBridge()
    viewport = Viewport()
    failures = 0
    try:
        for key, src, out_name in jobs:
            label = out_name or os.path.splitext(os.path.basename(src))[0]
            try:
                gif = render_one(src, viewport, bridge, args.out,
                                 sheet=args.sheet, samples=args.samples,
                                 page=key, out_name=out_name)
            except Exception as failure:  # noqa: BLE001 - report, don't stop
                failures += 1
                print("FAIL %s: %s: %s" % (label, type(failure).__name__,
                                           failure))
            else:
                size = os.path.getsize(gif) // 1024
                print("ok   %s <- %s (%d KB)"
                      % (os.path.basename(gif), _short_path(src),
                         size))
    finally:
        viewport.close()
    print("%d/%d gifs rendered" % (len(jobs) - failures, len(jobs)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
