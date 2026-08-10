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

from pxr import Tf, Usd, UsdUtils
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


def _LoadRigExecImaging():
    explicit = os.environ.get("RIGEXEC_IMAGING_DLL")
    if explicit:
        dllPath = explicit
    else:
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
        dllPath = next((path for path in candidates if os.path.isfile(path)),
                       candidates[0])
    lib = ctypes.CDLL(dllPath)
    lib.RigExecImaging_Activate.argtypes = [
        ctypes.c_longlong, ctypes.c_char_p, ctypes.c_double]
    lib.RigExecImaging_Activate.restype = ctypes.c_int
    lib.RigExecImaging_SetTime.argtypes = [ctypes.c_double]
    lib.RigExecImaging_SetTime.restype = ctypes.c_int
    lib.RigExecImaging_Deactivate.argtypes = []
    lib.RigExecImaging_Deactivate.restype = None

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

        dataModel = self._api.dataModel
        # Plugins load before the stage opens: activate on stage
        # replacement and re-evaluate on every timeline change.
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

    def _EnsureLibrary(self):
        # The library is loaded on stage replacement, but the authoring
        # panel can be opened on a stage that carries no RigExecRig yet
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
        try:
            self._ReleaseCachedStage()
        except Exception:
            pass

    def _ReleaseCachedStage(self):
        """
        Drops our StageCache reference to the previously activated stage.

        Insert() takes a reference the cache holds for the life of the
        PROCESS unless someone erases it, and the cache outlives usdview's
        own release of the stage. Leaving it pinned means the stage, its
        layers, and everything Python built inside them are still alive at
        interpreter finalization, where releasing them touches Python
        state after the GIL is gone -- a fatal PyThreadState_Get on exit
        rather than anything visible while the app runs.

        Erasing is safe even if usdview still holds the stage: the cache
        reference is ours, not theirs.
        """
        if self._cachedStage is None:
            return
        try:
            UsdUtils.StageCache.Get().Erase(self._cachedStage)
        except Exception as error:
            Tf.Warn("rigExecUsdview: could not release the cached stage: %s"
                    % error)
        self._cachedStage = None

    def _OnStageReplaced(self):
        stage = self._api.dataModel.stage
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
        self._ReleaseCachedStage()
        if not stage:
            return
        # An empty path passed to the C surface means every RigExecRig on the
        # stage.  Keep the paths here for diagnostics/UI rather than silently
        # selecting the first character.
        for prim in stage.Traverse():
            if prim.GetTypeName() == "RigExecRig":
                self._rigPaths.append(prim.GetPath())
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
        else:
            Tf.Warn("rigExecUsdview: activation failed (%d)" % status)
            self._ReleaseCachedStage()

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
