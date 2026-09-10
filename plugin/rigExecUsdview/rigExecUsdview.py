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
    lib.RigExecImaging_Deactivate.argtypes = []
    lib.RigExecImaging_Deactivate.restype = None
    try:
        lib.RigExecImaging_GetControlFrameAssetSpace.argtypes = [
            ctypes.c_longlong, ctypes.c_char_p, ctypes.c_double, ctypes.c_int,
            ctypes.POINTER(ctypes.c_double)]
        lib.RigExecImaging_GetControlFrameAssetSpace.restype = ctypes.c_int
    except AttributeError:
        pass  # Older libraries cannot supply deformation-relative gizmos.

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

    return lib


# usdview's plugin loader does not retain container instances that
# register no commands; without a strong reference the container is
# garbage collected and Qt disconnects its signals. Keep it alive here.
_container = None


class RigExecUsdviewContainer(PluginContainer):

    def registerPlugins(self, plugRegistry, plugCtx):
        global _container
        _container = self

        self._api = plugCtx
        self._lib = None
        self._active = False
        self._rigPaths = []
        self._cachedStage = None
        self._stageNoticeKey = None
        self._activating = False
        self._undoStack = None
        self._viewportTools = None
        self._viewportToolsFailed = False
        self._viewCube = None
        self._viewCubeFailed = False

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
            "Volume Weight Authoring",
            lambda api: self._OpenVolumeWeightPanel(api))

        # The curvenet authoring window. Same lazy-import reasoning as the
        # volume weight panel: its module pulls in Qt.
        self._curvenets = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.curvenets",
            "Curvenet Authoring",
            lambda api: self._OpenCurvenetPanel(api))

        # The Maya-style animation graph editor. Same lazy-import
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

        # The viewport manipulator toolbar. Same lazy-import reasoning
        # again; the menu item toggles it rather than opening a window,
        # because the toolbar lives inside the viewport frame.
        self._viewportToolsCommand = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.viewportTools",
            "Viewport Tools",
            lambda api: self._ToggleViewportTools())

        # The Maya-style view cube. Same lazy-import reasoning again;
        # the menu item toggles it rather than opening a window,
        # because the cube lives inside the viewport.
        self._viewCubeCommand = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.viewCube",
            "View Cube",
            lambda api: self._ToggleViewCube())

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
        menu = plugUIBuilder.findOrCreateMenu("RigExec")
        menu.addItem(self._reactivate)
        menu.addItem(self._volumeWeights)
        menu.addItem(self._curvenets)
        menu.addItem(self._graphEditor)
        menu.addItem(self._layerOpinions)
        menu.addItem(self._viewportToolsCommand)
        menu.addItem(self._viewCubeCommand)

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

    def _HasWeightOverlay(self):
        """
        True when rigExecImaging exports RigExecImaging_SetWeightOverlay.
        """
        lib = self._EnsureLibrary()
        if lib is None:
            return False
        return hasattr(lib, "RigExecImaging_SetWeightOverlay")

    def _SetWeightOverlay(self, primPath):
        """
        Paints the named weight object's influence onto its target
        geometry; the empty string turns the overlay off.
        """
        lib = self._EnsureLibrary()
        if lib is None:
            return False
        entryPoint = getattr(lib, "RigExecImaging_SetWeightOverlay", None)
        if entryPoint is None:
            return False
        status = entryPoint(primPath.encode("utf-8"))
        if status != 0:
            Tf.Warn("rigExecUsdview: SetWeightOverlay(%s) failed (%d)"
                    % (primPath, status))
            return False
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
            hasWeightOverlay=self._HasWeightOverlay)

    def _OpenCurvenetPanel(self, usdviewApi):
        # Same lazy sibling import as _OpenVolumeWeightPanel.
        try:
            import curvenetUI
        except ImportError:
            sys.path.insert(
                0, os.path.dirname(os.path.abspath(__file__)))
            import curvenetUI

        return curvenetUI.OpenCurvenetPanel(usdviewApi)

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
            gizmoMath.SetPublishedControlFrameReader(self._ReadPublishedControlFrame)
            self._viewportTools = gizmoUI.InstallViewportTools(
                self._api, self._UndoStack(),
                openGraphEditor=self._OpenGraphEditor)
        except Exception as error:
            Tf.Warn("rigExecUsdview: viewport tools unavailable: %s"
                    % error)
            self._viewportTools = None
            self._viewportToolsFailed = True
        return self._viewportTools

    def _ReadPublishedControlFrame(self, stage, path, time):
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
            if self._lib:
                self._lib.RigExecImaging_Deactivate()
        except Exception:
            pass
        self._active = False
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
            moversPath = rigPath.AppendChild("Movers")
            found = False
            for prim in Usd.PrimRange(root):
                if prim.GetTypeName() in outputTypes:
                    found = True
                    break
                if (prim.GetPath().HasPrefix(moversPath) and
                        prim.GetRelationship("rigExec:moves")):
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

    def _ActivateCurrentStage(self):
        stage = self._api.dataModel.stage
        if getattr(self, "_activating", False):
            return
        self._activating = True
        try:
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
            if self._lib:
                self._lib.RigExecImaging_Deactivate()
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

            cacheId = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
            self._cachedStage = stage
            status = self._lib.RigExecImaging_Activate(
                cacheId, b"", self._FrameValue())
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
            else:
                Tf.Warn("rigExecUsdview: activation failed (%d)" % status)
                # The cache is the current stage's strong owner. Retain it so
                # a malformed rig disables RigExec without invalidating
                # usdview's stage; replacement or _Shutdown eventually erases
                # it. A later edit notice retries the compile.
        finally:
            self._activating = False

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
        self._lib.RigExecImaging_SetTime(self._FrameValue(frame))


Tf.Type.Define(RigExecUsdviewContainer)
