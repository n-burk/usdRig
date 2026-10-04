#
# RigExec usdview plugin: feeds the stage and the timeline into the
# rigExecImaging registry so the RigExec scene indices publish live
# OpenExec-evaluated results into the viewport.
#
# The C++ side is reached through the exported C surface of
# rigExecImaging.dll (ctypes) with the stage handed over in-process via
# UsdUtils.StageCache — no custom Python bindings are required.
#
import atexit
import ctypes
import os
import sys

from pxr import Gf, Tf, Usd, UsdUtils
from pxr.Usdviewq.plugin import PluginContainer

try:
    import imagingHandle
    import sessionRegistry
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import imagingHandle
    import sessionRegistry



def _LibraryFileName():
    # The build directory's library name is the platform's, not Windows':
    # ctypes gets no search-path help here, so the default must name the
    # exact file CMake produced.
    if os.name == "nt":
        return "rigExecImaging.dll"
    if sys.platform == "darwin":
        return "librigExecImaging.dylib"
    return "librigExecImaging.so"


def ImagingLibraryPath():
    """Where rigExecImaging is, for this platform and this layout.

    Public because the testusdview scripts load the same library directly to
    read the generation counter and drive the overlay; sharing this keeps the
    platform naming and the search order in one place instead of letting each
    caller guess at a path.

    RIGEXEC_IMAGING_DLL overrides the search entirely.
    """
    explicit = os.environ.get("RIGEXEC_IMAGING_DLL")
    if explicit:
        return explicit

    moduleDir = os.path.dirname(os.path.abspath(__file__))
    # Installed layout:
    #   <prefix>/lib/python/rigExecUsdview/rigExecUsdview.py
    #   <prefix>/lib/librigExecImaging.*
    # Source/build layout remains a supported developer fallback:
    #   <repo>/plugin/rigExecUsdview/rigExecUsdview.py
    #   <repo>/build/librigExecImaging.*
    candidates = [
        os.path.normpath(os.path.join(
            moduleDir, "..", "..", _LibraryFileName())),
        os.path.normpath(os.path.join(
            moduleDir, "..", "..", "build", _LibraryFileName())),
    ]
    return next((path for path in candidates if os.path.isfile(path)),
                candidates[0])


def _LoadRigExecImaging():
    lib = ctypes.CDLL(ImagingLibraryPath())
    lib.RigExecImaging_Activate.argtypes = [
        ctypes.c_longlong, ctypes.c_char_p, ctypes.c_double]
    lib.RigExecImaging_Activate.restype = ctypes.c_int
    lib.RigExecImaging_SetTime.argtypes = [ctypes.c_double]
    lib.RigExecImaging_SetTime.restype = ctypes.c_int
    # Frame-cache warming is optional like the preview below: a session
    # without it evaluates live on every frame change exactly as before.
    try:
        lib.RigExecImaging_OnEditCommitted.argtypes = []
        lib.RigExecImaging_OnEditCommitted.restype = ctypes.c_int
        lib.RigExecImaging_OnIdle.argtypes = []
        lib.RigExecImaging_OnIdle.restype = ctypes.c_int
    except AttributeError:
        pass
    # The frame-cache strip and its driver are optional the same way:
    # WarmRange sets the persistent warm range, GetFrameStates reports
    # per-frame cached/warming/dirty/uncached, and ClearFrameCache
    # drops one rig's cached frames. An older library without them
    # warms on frame changes only, exactly as before.
    try:
        lib.RigExecImaging_WarmRange.argtypes = [
            ctypes.c_char_p, ctypes.POINTER(ctypes.c_double),
            ctypes.c_int]
        lib.RigExecImaging_WarmRange.restype = ctypes.c_int
        lib.RigExecImaging_GetFrameStates.argtypes = [
            ctypes.c_char_p, ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_int), ctypes.c_int]
        lib.RigExecImaging_GetFrameStates.restype = ctypes.c_int
        lib.RigExecImaging_ClearFrameCache.argtypes = [
            ctypes.c_char_p]
        lib.RigExecImaging_ClearFrameCache.restype = ctypes.c_int
    except AttributeError:
        pass
    # The strip's repaint gate is optional with them: without the
    # completions counter the panel repaints on states alone.
    try:
        lib.RigExecImaging_GetWarmingCompletedCount.argtypes = []
        lib.RigExecImaging_GetWarmingCompletedCount.restype = (
            ctypes.c_longlong)
    except AttributeError:
        pass
    try:
        lib.RigExecImaging_GetWarmingProgressCount.argtypes = []
        lib.RigExecImaging_GetWarmingProgressCount.restype = ctypes.c_longlong
    except AttributeError:
        pass
    lib.RigExecImaging_Deactivate.argtypes = []
    lib.RigExecImaging_Deactivate.restype = None
    try:
        lib.RigExecImaging_GetControlFrameAssetSpace.argtypes = [
            ctypes.c_longlong, ctypes.c_char_p, ctypes.c_double, ctypes.c_int,
            ctypes.POINTER(ctypes.c_double)]
        lib.RigExecImaging_GetControlFrameAssetSpace.restype = ctypes.c_int
    except AttributeError:
        pass  # Older libraries cannot supply deformation-relative gizmos.

    # The manipulation preview is optional in exactly the same way, and its
    # absence is not worth a warning: a session without it authors on release
    # as usual, and simply does not redraw until then.
    try:
        lib.RigExecImaging_BeginPreview.argtypes = [ctypes.c_char_p]
        lib.RigExecImaging_BeginPreview.restype = ctypes.c_int
        lib.RigExecImaging_UpdatePreview.argtypes = [
            ctypes.POINTER(ctypes.c_double), ctypes.c_int]
        lib.RigExecImaging_UpdatePreview.restype = ctypes.c_int
        lib.RigExecImaging_EndPreview.argtypes = []
        lib.RigExecImaging_EndPreview.restype = ctypes.c_int
    except AttributeError:
        pass

    # The influence overlay is optional: an older rigExecImaging.dll
    # does not export it, and touching a missing symbol on a CDLL raises
    # AttributeError at *bind* time. Binding it here rather than at the
    # call site means the whole plugin's load path stays a single
    # try/except, and the panel just disables its checkbox.
    try:
        lib.RigExecImaging_SetWeightOverlay.argtypes = [ctypes.c_char_p]
        lib.RigExecImaging_SetWeightOverlay.restype = ctypes.c_int
    except AttributeError:
        Tf.Warn("rigExecUsdview: rigExecImaging does not export "
                "RigExecImaging_SetWeightOverlay; the volume weight "
                "influence overlay will be unavailable.")

    # The per-stage (...ForStage) spellings of all of the above, where the
    # library has them. Every call goes through an imagingHandle bound to
    # this session's stage, which picks the spelling.
    return imagingHandle.BindLibrary(lib)


# The RigExec menu layout. Items sit at the top level or in one of these
# submenus, and every entry carries a rank so that the order holds no
# matter which plugin container's configureView runs first: usdview's
# PluginMenu only appends, and plugin/touchPose and plugin/shapeEditor add
# their items to the same submenus from their own containers. Those two
# carry a copy of this helper rather than importing it, so the directories
# stay independent; keep the ranks in step.
#
#   Reactivate RigExec Evaluation   0
#   Viewport            10: Viewport Tools 10, View Cube 20, View Axis 30
#   General Editors     20: Avar Editor 10, Layer Stack 15,
#                           Layer Opinions 20, Execution Stack 30,
#                           Deformer Toggles 35, Profiler 40,
#                           Cache Strip 50
#   Animation Editors   30: Graph Editor 10, Shape Editor 20,
#                           Control Picker 30, TouchPose 40,
#                           Volume Weight Editor 50
_SUBMENU_RANKS = {"Viewport": 10, "General Editors": 20,
                  "Animation Editors": 30}
_MENU_RANK = "rigExecMenuRank"

# How long a notice burst must be quiet before the pending edit-commit
# warms. The flush samples the whole neighbor+sweep band synchronously on
# the UI thread (benchCommitLag: ~136 ms on the biped, ~121 ms of it the
# per-frame sampling), so firing it on the next event-loop turn put that
# freeze inside every gizmo release and every undo/redo. A short idle
# delay keeps the gesture responsive -- the release's own repaint lands
# first -- while a pause in editing still warms the held playhead with no
# scrub. Must stay positive: zero puts the burst back inside the gesture.
_WARMING_COMMIT_DELAY_MS = 120


# How often the recurring idle driver re-queries the warm range at a held
# playhead. Same 120 ms discipline as the commit burst: each tick is one
# batched state query plus at most one budgeted OnIdle sweep, and the
# driver sleeps as soon as every frame reads cached.
_WARMING_IDLE_TICK_MS = 120

# Native sampling can defer a declined frame before choosing its live
# evaluation fallback. Allow that retry window before parking the driver.
_WARMING_STALL_TICKS = 4


def _PlaceByRank(qMenu, action, rank):
    # Move `action` ahead of the first entry ranked above it, unless it
    # is already there. Unranked entries (another plugin's) stay put.
    action.setProperty(_MENU_RANK, rank)
    for other in qMenu.actions():
        if other == action:
            return
        otherRank = other.property(_MENU_RANK)
        if otherRank is not None and otherRank > rank:
            qMenu.removeAction(action)
            qMenu.insertAction(other, action)
            return


def AddToRigExecMenu(plugUIBuilder, submenu, commandPlugin, rank):
    """Add `commandPlugin` to RigExec (or RigExec -> `submenu`) at `rank`.

    findOrCreateMenu and findOrCreateSubmenu hand every container the same
    menu objects, so no load order can make a second RigExec menu or a
    second copy of a submenu.
    """
    menu = plugUIBuilder.findOrCreateMenu("RigExec")
    if submenu is not None:
        menu = menu.findOrCreateSubmenu(submenu)
    action = menu.addItem(commandPlugin)
    qMenu = action.parent()
    _PlaceByRank(qMenu, action, rank)
    if submenu is not None:
        _PlaceByRank(qMenu.parent(), qMenu.menuAction(),
                     _SUBMENU_RANKS[submenu])
    return action


# Every session's container, filed under its main window. usdview's plugin
# loader does not retain container instances that register no commands;
# without a strong reference the container is garbage collected and Qt
# disconnects its signals. The registry is that reference, one per session
# (a host runs several usdview sessions in one process), and it is how the
# other plugin modules find THEIR session's container. It lets go when the
# session's window is destroyed.
_containers = sessionRegistry.SessionRegistry("rigExecUsdview containers")


def ContainerFor(usdviewApi=None):
    """
    The RigExec container of `usdviewApi`'s session, or None.

    Without an api: the session owning the active window, else the only
    session (see sessionRegistry.SessionRegistry.Current).
    """
    if usdviewApi is None:
        return _containers.Current()
    return _containers.Get(usdviewApi)


def __getattr__(name):
    # `_container` was the process's single container. It is now a view
    # onto the per-session registry, for scripts that read it (the
    # testusdview harness): in a plain usdview it is the one container.
    if name == "_container":
        return _containers.Current()
    raise AttributeError("module %r has no attribute %r" % (__name__, name))


class RigExecUsdviewContainer(PluginContainer):

    def registerPlugins(self, plugRegistry, plugCtx):
        _containers.Set(plugCtx, self)

        self._api = plugCtx
        self._lib = None
        self._imaging = None
        self._active = False
        self._warmingCommitPending = False
        self._warmingFlushArmed = False
        self._warmingTimer = None
        self._warmingIdleTimer = None
        self._cacheDisplayTimer = None
        self._warmingIdleLastStates = None
        self._warmRangeFrames = []
        self._stripPanel = None
        self._timelineOverlay = None
        self._rigPaths = []
        self._cachedStage = None
        self._stageNoticeKey = None
        self._activating = False
        self._undoStack = None
        self._viewportTools = None
        self._viewportToolsFailed = False
        self._viewCube = None
        self._viewCubeFailed = False
        self._viewAxis = None
        self._viewAxisFailed = False

        # Release the stage BEFORE the interpreter finalizes.
        #
        # Nothing guarantees signalStageReplaced fires on quit, and a
        # StageCache entry that survives into finalization is released
        # after the GIL is gone -- which surfaces as a fatal
        # PyThreadState_Get during shutdown rather than as anything
        # visible while the app runs. atexit runs while Python is still
        # alive, which is the whole point.
        atexit.register(self._Shutdown)

        # A manual re-activation command (also anchors this container via
        # its bound-method callback).
        self._reactivate = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.reactivate",
            "Reactivate RigExec Evaluation",
            lambda api: self._OnStageReplaced())

        # The volumetric weight authoring window. Its module pulls in Qt
        # and is imported lazily inside the callback so that this
        # container stays importable in the headless contexts the C++
        # tests use.
        self._volumeWeights = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.volumeWeights",
            "Volume Weight Editor",
            lambda api: self._OpenVolumeWeightPanel(api))

        # The conventional animation graph editor. Same lazy-import
        # reasoning as the panels above: graphEditorUI pulls in Qt.
        self._graphEditor = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.graphEditor",
            "Graph Editor",
            lambda api: self._OpenGraphEditor(api))

        # The per-layer opinion editor. Same lazy-import reasoning as
        # the panels above: layerOpinionsUI pulls in Qt.
        self._layerOpinions = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.layerOpinions",
            "Layer Opinions",
            lambda api: self._OpenLayerOpinionsPanel(api))

        # The layer stack: the stage's sublayer tree, with a checkbox to
        # mute any layer and switch that branch of the character off.
        # Same lazy-import reasoning as the panels above.
        self._layerStack = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.layerStack",
            "Layer Stack",
            lambda api: self._OpenLayerStackPanel(api))

        # The execution stack: what runs, in what order, and what it
        # writes. Same lazy-import reasoning as the panels above.
        self._execStack = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.execStack",
            "Execution Stack",
            lambda api: self._OpenExecStackPanel(api))

        # Deformer toggles: every mover that writes a mesh, with a
        # checkbox, so "the face breaks when I move the head" can be
        # bisected in the viewport instead of by editing the asset.
        self._deformerToggles = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.deformerToggles",
            "Deformer Toggles",
            lambda api: self._OpenDeformerTogglesPanel(api))

        # The avar editor: the selected control's avar channels as
        # sliders and number fields, writing through the shared undo
        # stack. Same lazy-import reasoning as the panels above.
        self._avarEditor = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.avarEditor",
            "Avar Editor",
            lambda api: self._OpenAvarEditorPanel(api))

        # The control picker: a picker layout baked to JSON beside
        # the rig, driving usdview's selection. Same lazy sibling
        # import as the panels above.
        self._picker = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.picker",
            "Control Picker",
            lambda api: self._OpenPickerPanel(api))

        # The viewport manipulator toolbar. Same lazy-import reasoning
        # again; the menu item toggles it rather than opening a window,
        # because the toolbar lives inside the viewport frame.
        self._viewportToolsCommand = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.viewportTools",
            "Viewport Tools",
            lambda api: self._ToggleViewportTools())

        # The conventional view cube. Same lazy-import reasoning again;
        # the menu item toggles it rather than opening a window,
        # because the cube lives inside the viewport.
        self._viewCubeCommand = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.viewCube",
            "View Cube",
            lambda api: self._ToggleViewCube())

        # The navigation axis gizmo, bottom-left. Same lazy-import and
        # toggle reasoning as the view cube.
        self._viewAxisCommand = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.viewAxis",
            "View Axis",
            lambda api: self._ToggleViewAxis())

        # The profiler: where the open rig's time goes, how deep its
        # schedule is, and how many threads any of it actually ran on.
        # Same lazy-import reasoning as the panels above.
        self._profiler = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.profiler",
            "Profiler",
            lambda api: self._OpenProfilerPanel(api))

        # The cache strip: per-frame warming states over the stage
        # range, with clear-cache and warm-range actions. Same
        # lazy-import reasoning as the panels above.
        self._cacheStrip = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.cacheStrip",
            "Cache Strip",
            lambda api: self._OpenCacheStripPanel(api))

        dataModel = self._api.dataModel
        # Plugins load before the stage opens: bind stage observation on
        # replacement, discover roots added later by authoring, and
        # re-evaluate on every timeline change.
        dataModel.signalStageReplaced.connect(self._OnStageReplaced)
        dataModel.currentFrameChanged.connect(self._OnFrameChanged)
        # The stage may already be present when the plugin loads late.
        if dataModel.stage:
            self._OnStageReplaced()

    def configureView(self, plugRegistry, plugUIBuilder):
        AddToRigExecMenu(plugUIBuilder, None, self._reactivate, 0)
        AddToRigExecMenu(plugUIBuilder, "Viewport",
                         self._viewportToolsCommand, 10)
        AddToRigExecMenu(plugUIBuilder, "Viewport",
                         self._viewCubeCommand, 20)
        AddToRigExecMenu(plugUIBuilder, "Viewport",
                         self._viewAxisCommand, 30)
        AddToRigExecMenu(plugUIBuilder, "General Editors",
                         self._avarEditor, 10)
        AddToRigExecMenu(plugUIBuilder, "General Editors",
                         self._layerStack, 15)
        AddToRigExecMenu(plugUIBuilder, "General Editors",
                         self._layerOpinions, 20)
        AddToRigExecMenu(plugUIBuilder, "General Editors",
                         self._execStack, 30)
        AddToRigExecMenu(plugUIBuilder, "General Editors",
                         self._deformerToggles, 35)
        AddToRigExecMenu(plugUIBuilder, "General Editors",
                         self._profiler, 40)
        AddToRigExecMenu(plugUIBuilder, "General Editors",
                         self._cacheStrip, 50)
        AddToRigExecMenu(plugUIBuilder, "Animation Editors",
                         self._graphEditor, 10)
        # Shape Editor (20) and TouchPose (40) come from their own
        # plugin containers and slot in between by rank.
        AddToRigExecMenu(plugUIBuilder, "Animation Editors",
                         self._picker, 30)
        AddToRigExecMenu(plugUIBuilder, "Animation Editors",
                         self._volumeWeights, 50)
        self._InstallOutlinerArcMenu()

    def _InstallOutlinerArcMenu(self):
        """
        Put "Add Composition Arc" on usdview's own prim menu (hierarchy
        and viewport right-click), on the shared undo stack. Best effort:
        a usdview this cannot hook keeps its menu as it was.
        """
        try:
            try:
                import outlinerArcMenu
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import outlinerArcMenu
            outlinerArcMenu.InstallPrimContextMenuHook(
                self._api, self._UndoStack())
        except Exception as error:
            Tf.Warn("rigExecUsdview: prim menu arcs unavailable: %s"
                    % error)

    def _EnsureLibrary(self):
        # The library is loaded on stage replacement, but the authoring
        # panel can be opened on a stage that carries no RigExecRoot yet
        # (that is how a rig gets built), so it must be able to force
        # the load itself. Returns the library or None.
        if self._lib is not None:
            return self._lib
        try:
            self._lib = _LoadRigExecImaging()
        except OSError as error:
            Tf.Warn("rigExecUsdview: failed to load rigExecImaging.dll: %s"
                    % error)
            self._lib = None
        return self._lib

    def _Imaging(self):
        """
        This session's handle on rigExecImaging, or None without a library.

        Bound to the stage this container activated (or holds in the
        StageCache), so every call reaches that stage's imaging context and
        no other session's. getattr-tolerant like the rest of this class:
        bare test containers carry only `_lib` (and maybe `_cachedStage`),
        and get a handle over whatever they carry.
        """
        lib = getattr(self, "_lib", None)
        if lib is None:
            return None
        handle = getattr(self, "_imaging", None)
        stage = getattr(self, "_cachedStage", None)
        if (handle is None or handle.lib is not lib or
                handle.stage is not stage):
            handle = imagingHandle.ImagingHandle(lib, stage)
            self._imaging = handle
        return handle

    def ImagingHandle(self):
        """This session's imaging handle, or None before a library loads."""
        return self._Imaging()

    def IsRigActive(self):
        """Whether RigExec evaluates this session's stage right now."""
        return bool(getattr(self, "_active", False))

    def _DeactivateImaging(self):
        """
        Stop evaluating THIS session's stage -- never another session's.

        A per-stage library deactivates only the stage this container
        activated (and nothing when it activated none). An older library
        has one process-wide context, and gets the unconditional global
        Deactivate it always got.
        """
        lib = getattr(self, "_lib", None)
        if not lib:
            return
        if imagingHandle.SupportsMultiStage(lib):
            if getattr(self, "_cachedStage", None) is None:
                return
        self._Imaging().Deactivate()

    def _HasWeightOverlay(self):
        """
        True when rigExecImaging exports RigExecImaging_SetWeightOverlay.
        """
        lib = self._EnsureLibrary()
        if lib is None:
            return False
        return self._Imaging().Has("SetWeightOverlay")

    def _SetWeightOverlay(self, primPath):
        """
        Paints the named weight object's influence onto its target
        geometry; the empty string turns the overlay off.
        """
        lib = self._EnsureLibrary()
        if lib is None:
            return False
        if (imagingHandle.SupportsMultiStage(lib) and
                getattr(self, "_cachedStage", None) is None):
            # The overlay can be chosen before the stage has a rig (that is
            # how a rig gets built), and a per-stage library remembers it on
            # THIS stage's context -- which it can only find through the
            # cache. Cache the stage the way activation does; replacement
            # and _Shutdown release it the same way.
            stage = self._api.dataModel.stage
            if stage:
                UsdUtils.StageCache.Get().Insert(stage)
                self._cachedStage = stage
        entryPoint = getattr(self._Imaging(),
                             "RigExecImaging_SetWeightOverlay", None)
        if entryPoint is None:
            return False
        status = entryPoint(primPath.encode("utf-8"))
        if status != 0:
            Tf.Warn("rigExecUsdview: SetWeightOverlay(%s) failed (%d)"
                    % (primPath, status))
            return False
        self._WakeWarmingDriver()
        return True

    def _OpenVolumeWeightPanel(self, usdviewApi):
        # Imported here rather than at module scope: volumeWeightUI
        # imports Qt, and this file must stay importable without it.
        # The plugin directory is what usdview put on sys.path to find
        # this module, so a plain top-level import resolves the sibling;
        # the fallback covers a loader that used a different mechanism.
        try:
            import volumeWeightUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import volumeWeightUI

        return volumeWeightUI.OpenVolumeWeightPanel(
            usdviewApi,
            setWeightOverlay=self._SetWeightOverlay,
            hasWeightOverlay=self._HasWeightOverlay,
            undoStack=self._UndoStack())

    def _OpenLayerOpinionsPanel(self, usdviewApi):
        # Same lazy sibling import as _OpenVolumeWeightPanel. Shares the
        # undo stack, so Ctrl+Z spans an opinion delete and a gizmo drag.
        try:
            import layerOpinionsUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import layerOpinionsUI

        return layerOpinionsUI.OpenLayerOpinionsPanel(
            usdviewApi or self._api, self._UndoStack())

    def _OpenLayerStackPanel(self, usdviewApi=None):
        # Same lazy sibling import as _OpenLayerOpinionsPanel.
        try:
            import layerStackUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import layerStackUI

        return layerStackUI.OpenLayerStackPanel(
            usdviewApi or self._api, self.MuteLayers, self._UndoStack())

    def MuteLayers(self, change):
        """Run `change(stage)`, a layer mute or unmute, with RigExec
        evaluation released around it.

        Muting a layer resyncs the whole stage, and OpenExec's stage
        listener does not take a resync of the pseudo-root: it fails a
        verification and leaves the exec network as it was. So the rig is
        deactivated first, the layers change with no exec system
        listening, and the rig is activated again on the recomposed stage,
        which compiles it fresh. Returns what `change` returned.
        """
        stage = self._api.dataModel.stage
        if not stage:
            return change(stage)
        wasActive = self._active
        self._activating = True
        try:
            if self._lib:
                self._lib.RigExecImaging_Deactivate()
            self._active = False
            result = change(stage)
        finally:
            self._activating = False
        if wasActive or self._FindRigPaths(stage):
            self._ActivateCurrentStage()
        return result

    def _OpenProfilerPanel(self, usdviewApi=None):
        # Same lazy sibling import as _OpenExecStackPanel: profilerUI
        # pulls in Qt, and this container must stay importable headless.
        # profilerModel beside it does not, which is what lets the CLI
        # run the same measurement with no Qt at all.
        try:
            import profilerUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import profilerUI

        return profilerUI.OpenProfilerPanel(usdviewApi or self._api)

    def _OpenDeformerTogglesPanel(self, usdviewApi=None):
        # Same lazy sibling import as the other panels: this module pulls
        # in Qt and the container must stay importable headless.
        try:
            import deformerTogglesUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import deformerTogglesUI

        return deformerTogglesUI.OpenDeformerTogglesPanel(
            usdviewApi or self._api)

    def _OpenCacheStripPanel(self, usdviewApi=None):
        # Same lazy sibling import as _OpenProfilerPanel: cacheStripUI
        # pulls in Qt, and this container must stay importable headless.
        # cacheStripPanel beside it does not, which is what lets the
        # headless tests drive the same poll/skip/action logic.
        try:
            import cacheStripUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import cacheStripUI

        panel = cacheStripUI.OpenCacheStripPanel(
            usdviewApi or self._api, self)
        self._stripPanel = panel
        return panel

    def StripLibrary(self):
        """
        This session's imaging handle, or None before the library loads.

        A handle, not the raw library: it answers to the same
        RigExecImaging_* names (cacheStripModel looks them up by name) and
        routes every one of them to THIS session's stage.
        """
        return self._Imaging()

    def StripRigs(self):
        """Active rig paths as strings, for the strip chooser."""
        return [str(path) for path in self._rigPaths]

    def StripWakeDriver(self):
        """Restart the recurring driver after a strip action."""
        # Older native libraries clear the configured range with the cache.
        self._PushWarmRangeFromStage()
        self._WakeWarmingDriver()

    def _OpenExecStackPanel(self, usdviewApi=None):
        # Same lazy sibling import as _OpenVolumeWeightPanel: execStackUI
        # pulls in Qt, and this container must stay importable headless.
        try:
            import execStackUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import execStackUI

        return execStackUI.OpenExecStackPanel(usdviewApi or self._api)

    def _OpenAvarEditorPanel(self, usdviewApi=None):
        # Same lazy sibling import as _OpenVolumeWeightPanel: avarEditorUI
        # pulls in Qt, and this container must stay importable headless.
        # Shares the undo stack, so Ctrl+Z spans a slider drag and a
        # gizmo drag alike.
        try:
            import avarEditorUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import avarEditorUI

        # A slider drag previews through the viewport tools' Hydra channel
        # and authors once on release; that channel is installed with the
        # tools, so make sure they exist even if the toolbar was never shown.
        self._EnsureViewportTools()
        return avarEditorUI.OpenAvarEditorPanel(
            usdviewApi or self._api, self._UndoStack())

    def _OpenPickerPanel(self, usdviewApi=None):
        # Same lazy sibling import as _OpenAvarEditorPanel: pickerUI pulls
        # in Qt and this container must stay importable headless.
        try:
            import pickerUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import pickerUI

        return pickerUI.OpenPickerPanel(
            usdviewApi or self._api, self._UndoStack())

    def _OpenGraphEditor(self, usdviewApi=None):
        """
        Open the graph editor on the SHARED undo stack, so Ctrl+Z spans
        graph edits and viewport gizmo drags alike.

        Takes no argument from the toolbar button, which has no
        usdviewApi of its own to pass.
        """
        # Same lazy sibling import as _OpenVolumeWeightPanel.
        try:
            import graphEditorUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import graphEditorUI

        return graphEditorUI.OpenGraphEditor(usdviewApi or self._api,
                                             self._UndoStack())

    def _UndoStack(self):
        """
        The undo stack the viewport gizmos push onto, created once.

        Lazy for the same reason the panels are: rigExecUndo is Qt-free,
        but nothing needs a stack until something authors through it,
        and the two existing panels can adopt this one later.
        """
        if self._undoStack is None:
            try:
                import rigExecUndo
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import rigExecUndo
            self._undoStack = rigExecUndo.UndoStack()
        return self._undoStack

    def _EnsureViewportTools(self):
        """
        Install the gizmo toolbar on the stage view, once it exists.

        Plugins load BEFORE the stage view is built, so this is driven
        off stage replacement rather than off registerPlugins. Returns
        None in any context without Qt or without a viewport, which is
        how the headless tests get away with loading this container.
        """
        # getattr, like _ActivateCurrentStage's _activating guard: a container
        # built by __new__ for one branch of a test (testUsdviewRigExec.py's
        # activation-failure case) has only the attributes that branch needs,
        # and reaching this far through _OnStageReplaced must not depend on
        # the rest of registerPlugins having run.
        if getattr(self, "_viewportTools", None) is not None:
            return self._viewportTools
        if getattr(self, "_viewportToolsFailed", False):
            # A headless or Qt-less session fails identically on every
            # stage replacement; warning each time would bury the one
            # message that mattered.
            return None
        try:
            try:
                import gizmoUI
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import gizmoUI
            import gizmoMath
            import gizmoPreview
            # Both are filed under THIS session: another window's gizmo
            # reads its own session's frames and previews into its own
            # stage's imaging context.
            gizmoMath.SetPublishedControlFrameReader(
                self._ReadPublishedControlFrame, session=self._api)
            gizmoPreview.SetSink(self._PreviewSink(), session=self._api)
            self._viewportTools = gizmoUI.InstallViewportTools(
                self._api, self._UndoStack(),
                openGraphEditor=self._OpenGraphEditor)
        except Exception as error:
            Tf.Warn("rigExecUsdview: viewport tools unavailable: %s"
                    % error)
            self._viewportTools = None
            self._viewportToolsFailed = True
        return self._viewportTools

    def _PreviewSink(self):
        """
        The manipulation preview's route into rigExecImaging.

        Three thin calls over the C entry points, and only Update runs per
        mouse sample: the attribute paths are marshalled once per drag and
        every sample after that is an array of doubles, which is the shape the
        C side asked for so that dragging costs what dragging costs.

        Every call resolves the library FRESH. The viewport tools are built
        the first time they are shown, which can be before any rig is
        activated and is certainly before the stage is replaced the next time;
        a sink that captured the library at construction would preview against
        whichever one happened to be loaded then, or against none at all.

        The same goes for the stage: every call goes through this session's
        imaging handle, so a preview reaches the chains of the stage this
        window shows and nothing in any other session's viewport.
        """
        container = self

        class _Sink(object):
            def _Entry(self, name):
                lib = container._lib
                if lib is None or not container._active:
                    return None
                return getattr(container._Imaging(), name, None)

            def Begin(self, packedPaths):
                entry = self._Entry("RigExecImaging_BeginPreview")
                if entry is None:
                    return -1
                result = entry(packedPaths.encode("utf-8"))
                if result >= 0:
                    container._previewWarmingPaused = True
                    container._SleepWarmingDriver()
                return result

            def Update(self, values):
                entry = self._Entry("RigExecImaging_UpdatePreview")
                if entry is None:
                    return False
                buffer = (ctypes.c_double * len(values))(*values)
                return entry(buffer, len(values)) == 0

            def End(self, publish=True):
                # Without the republish when a commit authors next (see
                # gizmoPreview.End); a library predating that entry point
                # takes the plain end, which is correct and evaluates twice.
                entry = None if publish else self._Entry(
                    "RigExecImaging_EndPreviewWithoutPublish")
                if entry is None:
                    entry = self._Entry("RigExecImaging_EndPreview")
                if entry is None:
                    return False
                ended = entry() == 0
                if ended:
                    container._previewWarmingPaused = False
                    if getattr(container, "_warmingCommitPending", False):
                        container._ScheduleWarmingFlush()
                    else:
                        container._WakeWarmingDriver()
                return ended

        return _Sink()

    def _ReadPublishedControlFrame(self, stage, path, time):
        # Answers for THIS session's stage only; gizmoMath asks every
        # session's reader and takes the one that answers.
        if not self._active or self._lib is None or stage != self._cachedStage:
            return None
        read = getattr(self._lib, "RigExecImaging_GetControlFrameAssetSpace", None)
        if read is None:
            return None
        values = (ctypes.c_double * 16)()
        cacheId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        isDefault = time.IsDefault()
        if not read(cacheId, str(path).encode("utf-8"),
                    0.0 if isDefault else time.GetValue(), int(isDefault), values):
            return None
        return Gf.Matrix4d(*[tuple(values[r*4:(r+1)*4]) for r in range(4)])

    def _ToggleViewportTools(self):
        """Menu item: show or hide the toolbar and its manipulators."""
        controller = self._EnsureViewportTools()
        if controller is None:
            return None
        controller.SetVisible(not controller.IsVisible())
        return controller

    def _EnsureViewCube(self):
        """
        Install the view cube on the stage view, once it exists.

        Plugins load BEFORE the stage view is built, so this is driven
        off stage replacement rather than off registerPlugins. Returns
        None in any context without Qt or without a viewport, which is
        how the headless tests get away with loading this container.
        """
        # Same tolerance as _EnsureViewportTools above, for the same reason.
        if getattr(self, "_viewCube", None) is not None:
            return self._viewCube
        if getattr(self, "_viewCubeFailed", False):
            # A headless or Qt-less session fails identically on every
            # stage replacement; warning each time would bury the one
            # message that mattered.
            return None
        try:
            try:
                import viewCubeUI
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import viewCubeUI
            self._viewCube = viewCubeUI.InstallViewCube(self._api)
        except Exception as error:
            Tf.Warn("rigExecUsdview: view cube unavailable: %s"
                    % error)
            self._viewCube = None
            self._viewCubeFailed = True
        return self._viewCube

    def _ToggleViewCube(self):
        """Menu item: show or hide the view cube."""
        controller = self._EnsureViewCube()
        if controller is None:
            return None
        controller.SetVisible(not controller.IsVisible())
        return controller

    def _EnsureViewAxis(self):
        """Install the view axis gizmo; see _EnsureViewCube."""
        if getattr(self, "_viewAxis", None) is not None:
            return self._viewAxis
        if getattr(self, "_viewAxisFailed", False):
            return None
        try:
            try:
                import viewAxisUI
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import viewAxisUI
            self._viewAxis = viewAxisUI.InstallViewAxis(self._api)
        except Exception as error:
            Tf.Warn("rigExecUsdview: view axis unavailable: %s" % error)
            self._viewAxis = None
            self._viewAxisFailed = True
        return self._viewAxis

    def _ToggleViewAxis(self):
        """Menu item: show or hide the view axis gizmo."""
        controller = self._EnsureViewAxis()
        if controller is None:
            return None
        controller.SetVisible(not controller.IsVisible())
        return controller

    def _FrameValue(self, frame=None):
        """
        \\p frame as a plain double, defaulting to the data model's
        current frame.

        The argument exists because the data model's current frame is NOT
        readable from inside its own change signal. RootDataModel's
        setter emits currentFrameChanged(value) and only THEN assigns
        self._currentFrame (rootDataModel.py:158-160), so a handler that
        re-reads the property is delivered the frame the artist just left.
        Every consumer of this frame -- the pose, the guides, and the
        influence overlay -- would then be one scrub behind.
        """
        if frame is None:
            frame = self._api.dataModel.currentFrame
        if isinstance(frame, Usd.TimeCode):
            if frame.IsDefault():
                return 0.0
            return frame.GetValue()
        return float(frame)

    def _Shutdown(self):
        """
        Orderly teardown at interpreter exit: stop the imaging engine and
        drop our stage reference while Python is still able to run.

        Deliberately swallows everything. This runs during atexit, where
        an exception is printed but cannot be handled, and where the Qt
        and USD state it would touch is already half gone -- a noisy
        traceback on quit teaches an artist nothing.
        """
        try:
            self._DeactivateImaging()
        except Exception:
            pass
        try:
            for _name in ("_warmingTimer", "_warmingIdleTimer",
                          "_cacheDisplayTimer"):
                timer = getattr(self, _name, None)
                if timer is not None:
                    timer.stop()
        except Exception:
            pass
        self._active = False
        try:
            self._ClearTimelineOverlay()
        except Exception:
            pass
        self._RevokeStageNotice()
        try:
            self._ReleaseCachedStage()
        except Exception:
            pass

    def _RevokeStageNotice(self):
        """Stop observing edits on the previously opened stage."""
        key = getattr(self, "_stageNoticeKey", None)
        if key is not None:
            try:
                key.Revoke()
            except Exception:
                pass
        self._stageNoticeKey = None

    def _ObserveStage(self, stage):
        """Observe the stage even when it does not contain a rig yet."""
        self._RevokeStageNotice()
        if stage:
            self._stageNoticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged,
                self._OnStageObjectsChanged,
                stage)

    @staticmethod
    def _FindRigPaths(stage):
        if not stage:
            return []
        return sorted(
            (prim.GetPath() for prim in stage.Traverse()
             if prim.GetTypeName() == "RigExecRoot"),
            key=lambda path: str(path))

    @staticmethod
    def _RootsHaveActivationCandidates(stage, rigPaths):
        """Whether every live-authored root has something it can publish.

        Defining a root, its grouping scopes, and its first output produces
        separate synchronous USD notices.  Waiting through the first two
        avoids knowingly compiling an empty intermediate rig (and warning the
        artist about the perfectly ordinary act of building it).  This is
        only a readiness hint: native Compile remains the authority and still
        validates the complete composed contract.
        """
        outputTypes = {
            "RigExecControl", "RigExecJoint",
            "RigExecSphereWeight", "RigExecPlaneWeight",
            "RigExecCurveWeight",
        }
        for rigPath in rigPaths:
            root = stage.GetPrimAtPath(rigPath)
            found = False
            for prim in Usd.PrimRange(root):
                if prim.GetTypeName() in outputTypes:
                    found = True
                    break
                # A mover anywhere under the root -- carrying rigExec:moves
                # is what makes it one, as it is to the native compiler.
                if prim.GetRelationship("rigExec:moves"):
                    found = True
                    break
            if not found:
                return False
        return True

    def _OnStageObjectsChanged(self, notice, stage):
        """Activate when a rig is authored into an already-open stage.

        The no-argument launcher opens a rootless blank layer.  A stage-
        replacement-only plugin never sees the RigExecRoot subsequently
        created in that layer, so its joints cannot enter the Hydra chain
        until the file is reopened or the user manually reactivates.

        Retry while roots exist but activation is still down.  Creating the
        root itself is normally one edit and creating its first output is a
        later edit: the first compile is correctly rejected as empty, and the
        second notice is what makes the now-valid rig start drawing.
        """
        if getattr(self, "_activating", False):
            return
        # Any edit re-centers warming: an idle delay after the burst commits
        # (neighbors plus sweep around the playhead) instead of idling, so
        # an edit at a held playhead warms without waiting for a scrub. Set
        # for every notice -- gizmo releases, undo/redo, panel commits --
        # and consumed by the flush (or the next frame tick without Qt), so
        # a burst of notices warms once, not per edit. The delay keeps the
        # synchronous sampling burst off the gesture (see
        # _WARMING_COMMIT_DELAY_MS).
        self._warmingCommitPending = True
        # An already-running idle timer must observe the same debounce as
        # the commit timer; otherwise a drag still samples its changing rig
        # every idle tick before the delayed commit has a chance to fire.
        self._SleepWarmingDriver()
        self._ScheduleWarmingFlush()
        self._ScheduleCacheDisplayRefresh()
        rigPaths = self._FindRigPaths(stage)
        if not (rigPaths != self._rigPaths or
                (rigPaths and not self._active)):
            return
        if rigPaths and not self._RootsHaveActivationCandidates(
                stage, rigPaths):
            return
        self._ActivateCurrentStage()

    def _ReleaseCachedStage(self, liveStage=None):
        """
        Drops our StageCache reference unless usdview is still using it.

        Insert() takes a reference the cache holds for the life of the
        PROCESS unless someone erases it, and the cache outlives usdview's
        own release of the stage. Leaving it pinned means the stage, its
        layers, and everything Python built inside them are still alive at
        interpreter finalization, where releasing them touches Python
        state after the GIL is gone -- a fatal PyThreadState_Get on exit
        rather than anything visible while the app runs.

        In this OpenUSD Python binding, StageCache.Insert transfers the strong
        stage ownership into the cache and leaves Python holding a weak stage
        wrapper. Erasing the entry while usdview still exposes that wrapper
        turns its current stage into an invalid null stage. Keep the current
        live stage cached through activation failures and manual reactivation;
        an actual stage replacement or _Shutdown releases it safely.
        """
        if self._cachedStage is None:
            return
        if self._cachedStage is liveStage:
            return
        try:
            UsdUtils.StageCache.Get().Erase(self._cachedStage)
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not release the cached stage: %s"
                    % error)
        self._cachedStage = None

    def _OnStageReplaced(self):
        self._ObserveStage(self._api.dataModel.stage)
        self._ActivateCurrentStage()
        # After activation, so the first gizmo target is resolved
        # against a stage the evaluator has already published.
        self._EnsureViewportTools()
        self._EnsureViewCube()
        self._EnsureViewAxis()

    def _ActivateCurrentStage(self):
        stage = self._api.dataModel.stage
        if getattr(self, "_activating", False):
            return
        self._activating = True
        try:
            self._ResetWarmingDriver()
            self._active = False
            self._rigPaths = []
            # A scene-index store outlives a stage replacement.  Clear the old
            # activation before inspecting or attempting the new stage so a
            # no-rig stage, load failure, or compile failure cannot inherit a
            # previous stage's path-addressed deformation.
            #
            # Deactivate BEFORE erasing: the imaging registry resolves the
            # stage out of the cache, so pulling it first would leave the
            # registry holding a handle to something it can no longer look up.
            #
            # Only THIS session's previous stage: with several sessions in
            # one process, each keeps its own imaging context evaluating.
            self._DeactivateImaging()
            self._ReleaseCachedStage(stage)
            if not stage:
                return
            # An empty path passed to the C surface means every RigExecRoot on
            # the stage. Keep the paths here for diagnostics/UI rather than
            # silently selecting the first character.
            self._rigPaths = self._FindRigPaths(stage)
            if not self._rigPaths:
                return
            if self._EnsureLibrary() is None:
                return

            UsdUtils.StageCache.Get().Insert(stage)
            self._cachedStage = stage
            status = self._Imaging().Activate(b"", self._FrameValue())
            if status == 0:
                self._active = True
                # Activate() installs the native evaluator's stage notice.
                # Re-register ours afterwards so root removal/addition reaches
                # this root-set guard before the evaluator tries to recompile
                # a session whose root has just disappeared. Ordinary edits
                # fall through here and are then handled by the native notice.
                self._ObserveStage(stage)
                # RigExec controls/joints/guides are purpose=guide — Storm only
                # draws them when the viewer has guide purpose enabled.
                try:
                    vs = self._api.dataModel.viewSettings
                    if not vs.displayGuide:
                        vs.displayGuide = True
                except Exception:
                    pass
                self._PushWarmRangeFromStage()
                self._EnsureTimelineOverlay()
                self._WakeWarmingDriver()
            else:
                Tf.Warn("rigExecUsdview: activation failed (%d)" % status)
                # The cache is the current stage's strong owner. Retain it so
                # a malformed rig disables RigExec without invalidating
                # usdview's stage; replacement or _Shutdown eventually erases
                # it. A later edit notice retries the compile.
        finally:
            self._activating = False
            self._NotifyStripTick(False)

    def _ScheduleWarmingFlush(self):
        # Coalesced like the pending flag, and deferred past the gesture.
        # The timer restarts on every notice, so a burst -- or a sustained
        # notice stream, whose single-shots would otherwise fire mid-stream
        # -- warms once, when it stops. Without Qt (or without usdview's
        # shim) this degrades to the pending flag, which the next frame
        # change consumes as before.
        timer = getattr(self, "_warmingTimer", None)
        if getattr(self, "_warmingFlushArmed", False):  # bare test containers skip __init__
            if timer is not None:
                try:
                    timer.start(_WARMING_COMMIT_DELAY_MS)
                except Exception:
                    pass
            return
        try:
            from pxr.Usdviewq.qt import QtCore
        except Exception:
            return
        self._warmingFlushArmed = True
        try:
            if timer is None:
                timer = QtCore.QTimer()
                timer.setSingleShot(True)
                timer.timeout.connect(self._FlushWarmingCommit)
                self._warmingTimer = timer
            timer.start(_WARMING_COMMIT_DELAY_MS)
        except Exception:
            self._warmingFlushArmed = False

    def _ResetWarmingDriver(self):
        """Discard the previous activation's delayed work and display."""
        self._SleepWarmingDriver()
        timer = getattr(self, "_warmingTimer", None)
        if timer is not None:
            timer.stop()
        displayTimer = getattr(self, "_cacheDisplayTimer", None)
        if displayTimer is not None:
            displayTimer.stop()
        self._warmingCommitPending = False
        self._warmingFlushArmed = False
        self._warmRangeFrames = []
        self._previewWarmingPaused = False
        self._ClearTimelineOverlay()

    def _ScheduleCacheDisplayRefresh(self):
        # The native notice handlers retire entries after this listener.
        # Refresh on the next Qt turn, independently of the warming debounce,
        # so a sustained edit displays dirty frames while warming is paused.
        try:
            from pxr.Usdviewq.qt import QtCore
            timer = getattr(self, "_cacheDisplayTimer", None)
            if timer is None:
                timer = QtCore.QTimer()
                timer.setSingleShot(True)
                timer.timeout.connect(self._RefreshCacheDisplay)
                self._cacheDisplayTimer = timer
            if not timer.isActive():
                timer.start(0)
        except Exception:
            pass

    def _RefreshCacheDisplay(self):
        # Display only: sampling and warming wait for _FlushWarmingCommit.
        if not (self._active and self._lib):
            return
        panel = getattr(self, "_stripPanel", None)
        if panel is not None:
            try:
                panel.PollAfterEdit()
            except Exception:
                self._stripPanel = None
        frames = self._WarmRangeFrames(getattr(self, "_cachedStage", None))
        if frames != getattr(self, "_warmRangeFrames", None):
            # The delayed commit will push a changed range to native warming.
            # Until then, old frame positions must not retain green cells.
            self._ClearTimelineOverlay()
            return
        model = self._CacheStripModel()
        states = []
        for rigPath in self._rigPaths:
            one = model.FetchFrameStates(self._Imaging(), str(rigPath), frames)
            if one is None:
                self._ClearTimelineOverlay()
                return
            states.extend(one)
        self._UpdateTimelineOverlay(states)

    def _FlushWarmingCommit(self):
        # The release path: consume a pending edit-commit at the held
        # playhead. Fires from the idle timer, so the synchronous sampling
        # burst runs after the gesture, not inside it. AttributeError-
        # tolerant like _OnFrameChanged: older shims may name the trigger
        # differently.
        self._warmingFlushArmed = False
        if not self._warmingCommitPending:
            return
        if not (self._active and self._lib):
            return
        if getattr(self, "_previewWarmingPaused", False):
            return
        self._warmingCommitPending = False
        try:
            self._Imaging().OnEditCommitted()
        except AttributeError:
            pass
        self._WakeWarmingDriver()

    @staticmethod
    def _CacheStripModel():
        # Lazy sibling import like the panels: cacheStripModel is Qt-free
        # and pxr-free, but the plugin directory reaches sys.path only
        # through the loader that found this module.
        try:
            import cacheStripModel
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import cacheStripModel
        return cacheStripModel

    def _HasFrameStates(self):
        # The recurring driver needs the per-frame state query: without
        # it there is nothing to sleep on, so an older library stays on
        # frame-change-only warming rather than spinning a blind timer.
        if self._lib is None:
            return False
        return getattr(self._Imaging(), "RigExecImaging_GetFrameStates",
                        None) is not None

    def _WarmRangeFrames(self, stage):
        try:
            model = self._CacheStripModel()
        except Exception:
            return []
        try:
            start = stage.GetStartTimeCode()
            end = stage.GetEndTimeCode()
        except Exception:
            return []
        return model.FrameListForRange(start, end)

    def _PushWarmRangeFromStage(self):
        # SetWarmRange from the stage range on activation, so the
        # full-range cursor visits the timeline instead of the default
        # playhead-relative sweep. Empty ranges must also be pushed to
        # replace any range left over from earlier stage metadata.
        stage = self._cachedStage
        if stage is None or self._lib is None:
            return
        frames = self._WarmRangeFrames(stage)
        self._warmRangeFrames = frames
        try:
            model = self._CacheStripModel()
        except Exception:
            return
        for rigPath in self._rigPaths:
            model.PushWarmRange(self._Imaging(), str(rigPath), frames)

    def _RepushWarmRangeIfChanged(self):
        # A stage-range edit leaves the pushed warm range stale: the
        # full-range cursor would sweep the old timeline while new
        # frames stay uncached. Every driver wake re-pushes when the
        # stage range drifted, so activation, edit-commit, frame-change,
        # and strip-action wakes all pick up range changes. Runs before
        # the Qt/timer checks below on purpose: without Qt there is no
        # recurring tick, but the wake still refreshes the native range
        # for the synchronous frame-change sweeps. getattr-tolerant:
        # bare test containers skip __init__ and carry no stage at all.
        stage = getattr(self, "_cachedStage", None)
        if stage is None or getattr(self, "_lib", None) is None:
            return
        try:
            start = stage.GetStartTimeCode()
            end = stage.GetEndTimeCode()
            model = self._CacheStripModel()
            drifted = model.WarmRangeDrifted(
                getattr(self, "_warmRangeFrames", None), start, end)
        except Exception:
            return
        if drifted:
            self._PushWarmRangeFromStage()

    def _WakeWarmingDriver(self):
        # Start (or keep) the recurring idle driver: it ticks OnIdle
        # while unwarm frames remain in range and sleeps otherwise.
        # Wakes on SetTime, edit commit, and range change; the range
        # re-push runs first so a stale timeline never starts the tick.
        # Without Qt there is no recurring tick at all -- frame changes
        # warm synchronously as before.
        if not (self._active and self._lib):
            return
        self._RepushWarmRangeIfChanged()
        if (getattr(self, "_warmingCommitPending", False)
                or getattr(self, "_previewWarmingPaused", False)):
            return
        if not self._HasFrameStates():
            return
        try:
            from pxr.Usdviewq.qt import QtCore
        except Exception:
            return
        try:
            timer = getattr(self, "_warmingIdleTimer", None)
            if timer is None:
                timer = QtCore.QTimer()
                timer.setSingleShot(False)
                timer.timeout.connect(self._TickWarmingDriver)
                self._warmingIdleTimer = timer
            self._warmingIdleLastStates = None
            self._warmingIdleLastProgress = None
            self._warmingIdleStallTicks = 0
            if not timer.isActive():
                timer.start(_WARMING_IDLE_TICK_MS)
        except Exception:
            pass

    def _SleepWarmingDriver(self):
        try:
            timer = getattr(self, "_warmingIdleTimer", None)
            if timer is not None:
                timer.stop()
        except Exception:
            pass
        self._warmingIdleLastStates = None
        self._warmingIdleLastProgress = None
        self._warmingIdleStallTicks = 0

    def _NotifyStripTick(self, mayHaveEnqueued, states=None):
        # Forward one recurring tick to the open strip panel, with
        # the tick's own enqueue knowledge: sleep paths pass False
        # (no sweep ran, so no enqueue could have flipped states
        # behind the completions counter), sweeps pass True. A dead
        # panel detaches rather than breaking the driver tick. Ticks
        # that queried states also paint the timeline overlay with
        # them -- no second C call.
        panel = getattr(self, "_stripPanel", None)
        if panel is not None:
            try:
                panel.PollTick(mayHaveEnqueued)
            except Exception:
                self._stripPanel = None
        if states is not None:
            self._UpdateTimelineOverlay(states)

    def _EnsureTimelineOverlay(self):
        # The timeline overlay, installed once per window over
        # usdview's own slider. Headless-safe: without Qt, without a
        # main window (as in bare test containers), or without a
        # findable slider, there is no overlay and every update below
        # is a no-op.
        if getattr(self, "_timelineOverlay", None) is not None:
            return
        try:
            try:
                import cacheTimelineOverlay
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import cacheTimelineOverlay
            self._timelineOverlay =                 cacheTimelineOverlay.InstallTimelineOverlay(
                    self._api.qMainWindow)
        except Exception:
            self._timelineOverlay = None

    def _UpdateTimelineOverlay(self, states):
        # Paints one driver tick's states over the timeline slider.
        # The band shows the first rig's frames; multi-rig sessions
        # still sleep and wake on every rig, only the overlay is
        # single-rig. A dead widget detaches like the strip panel.
        widget = getattr(self, "_timelineOverlay", None)
        if widget is None:
            return
        frames = getattr(self, "_warmRangeFrames", None) or []
        rigStates = list(states)
        if len(rigStates) > len(frames):
            rigStates = rigStates[:len(frames)]
        try:
            widget.Update(rigStates, frames)
        except Exception:
            self._timelineOverlay = None

    def _ClearTimelineOverlay(self):
        widget = getattr(self, "_timelineOverlay", None)
        if widget is None:
            return
        try:
            widget.Clear()
        except Exception:
            self._timelineOverlay = None

    def _TickWarmingDriver(self):
        # One recurring tick at a held playhead: query every rig's
        # states over the warm range and idle-sweep while any frame
        # is unwarm, else sleep until the next wake. Several unchanged
        # ticks with nothing in flight allow native sampling retries and
        # its live fallback before parking uncacheable work.
        if not (self._active and self._lib):
            self._SleepWarmingDriver()
            self._ClearTimelineOverlay()
            self._NotifyStripTick(False)
            return
        if (getattr(self, "_warmingCommitPending", False)
                or getattr(self, "_previewWarmingPaused", False)):
            self._SleepWarmingDriver()
            return
        if self._IsPlaying():
            # Keep the timer armed: RootDataModel has no playingChanged
            # signal, so the next idle tick resumes after playback stops.
            return
        # A range edit that lands mid-warm re-centers the sweep from
        # here, so the tick below queries the timeline the stage has,
        # not the one activation pushed.
        self._RepushWarmRangeIfChanged()
        try:
            model = self._CacheStripModel()
        except Exception:
            self._SleepWarmingDriver()
            self._NotifyStripTick(False)
            return
        frames = getattr(self, "_warmRangeFrames", None) or []
        if not frames:
            self._SleepWarmingDriver()
            self._NotifyStripTick(False)
            return
        states = []
        for rigPath in self._rigPaths:
            one = model.FetchFrameStates(self._Imaging(), str(rigPath),
                                       frames)
            if one is None:
                self._SleepWarmingDriver()
                self._NotifyStripTick(False)
                return
            states.extend(one)
        if not model.AnyUnwarm(states):
            self._SleepWarmingDriver()
            self._NotifyStripTick(False, states)
            return
        last = getattr(self, "_warmingIdleLastStates", None)
        progressEntry = getattr(self._Imaging(),
                               "RigExecImaging_GetWarmingProgressCount", None)
        progress = int(progressEntry()) if progressEntry is not None else None
        lastProgress = getattr(self, "_warmingIdleLastProgress", None)
        if (last == states and lastProgress == progress
                and not model.AnyWarming(states)):
            self._warmingIdleStallTicks = (
                getattr(self, "_warmingIdleStallTicks", 0) + 1)
            if self._warmingIdleStallTicks >= _WARMING_STALL_TICKS:
                self._SleepWarmingDriver()
                self._NotifyStripTick(False, states)
                return
        else:
            self._warmingIdleStallTicks = 0
        self._warmingIdleLastStates = states
        self._warmingIdleLastProgress = progress
        try:
            self._Imaging().OnIdle()
        except AttributeError:
            self._SleepWarmingDriver()
            self._NotifyStripTick(False)
            return
        self._NotifyStripTick(True, states)

    def _WarmingDriverTicking(self):
        # Whether the recurring driver timer is armed right now. While
        # it ticks, frame changes skip their synchronous idle sweep
        # below: the tick sweeps the same budgeted band within 120 ms,
        # so a second sweep on the frame would serialize its sampling
        # into the scrub for jobs already about to enqueue. Bare
        # containers (no timer attribute at all) answer False --
        # without a driver the synchronous sweep is the whole story.
        try:
            timer = getattr(self, "_warmingIdleTimer", None)
            return timer is not None and bool(timer.isActive())
        except Exception:
            return False

    def _IsPlaying(self):
        api = getattr(self, "_api", None)
        return bool(getattr(getattr(api, "dataModel", None),
                            "playing", False))

    def _OnFrameChanged(self, frame):
        # The SIGNAL's frame, never dataModel.currentFrame -- see
        # _FrameValue. Reading the property here published the previous
        # frame's pose, which a drag hides almost perfectly (each step
        # draws the step before it) and which turns every other edit into
        # a no-op: the authoring panel writes a spline knot at the frame
        # the artist is LOOKING at, so an engine sitting one frame back
        # resolves the untouched value and the overlay appears frozen.
        if not (self._active and self._lib):
            return
        imaging = self._Imaging()
        imaging.SetTime(self._FrameValue(frame))
        if getattr(self, "_previewWarmingPaused", False):
            return
        # Warming, from the frame loop: an edit since the last tick commits
        # (neighbors plus sweep re-center on the playhead), otherwise the
        # tick is an idle sweep -- unless the recurring driver is already
        # ticking, in which case the frame yields to it instead of
        # serializing a second budgeted sweep into the scrub. Commits
        # never yield: they re-center warming on the playhead the artist
        # just chose. Optional in an older library; a drag release with
        # no following frame change normally warms on the notice flush
        # already, and this tick is its fallback without Qt.
        try:
            if self._warmingCommitPending:
                self._warmingCommitPending = False
                imaging.OnEditCommitted()
            elif not self._WarmingDriverTicking() and not self._IsPlaying():
                imaging.OnIdle()
        except AttributeError:
            pass
        # The recurring driver continues at the held playhead from here:
        # it re-ticks OnIdle while unwarm frames remain and sleeps once
        # the range reads cached. Without Qt (or an older library) this
        # is a no-op and the synchronous tick above is the whole story.
        self._WakeWarmingDriver()


Tf.Type.Define(RigExecUsdviewContainer)
