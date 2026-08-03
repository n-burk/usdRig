#
# RigExec usdview plugin: feeds the stage and the timeline into the
# rigExecImaging registry so the RigExec scene indices publish live
# OpenExec-evaluated results into the viewport.
#
# The C++ side is reached through the exported C surface of
# rigExecImaging.dll (ctypes) with the stage handed over in-process via
# UsdUtils.StageCache — no custom Python bindings are required.
#
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
    dllPath = os.environ.get(
        "RIGEXEC_IMAGING_DLL",
        os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "..", "build", _LibraryFileName())))
    lib = ctypes.CDLL(dllPath)
    lib.RigExecImaging_Activate.argtypes = [
        ctypes.c_longlong, ctypes.c_char_p, ctypes.c_double]
    lib.RigExecImaging_Activate.restype = ctypes.c_int
    lib.RigExecImaging_SetTime.argtypes = [ctypes.c_double]
    lib.RigExecImaging_SetTime.restype = ctypes.c_int
    lib.RigExecImaging_Deactivate.argtypes = []
    lib.RigExecImaging_Deactivate.restype = None
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
        self._rigPath = None

        # A manual re-activation command (also anchors this container via
        # its bound-method callback).
        self._reactivate = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.reactivate",
            "Reactivate RigExec Evaluation",
            lambda api: self._OnStageReplaced())

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

    def _FrameValue(self):
        frame = self._api.dataModel.currentFrame
        if isinstance(frame, Usd.TimeCode):
            if frame.IsDefault():
                return 0.0
            return frame.GetValue()
        return float(frame)

    def _OnStageReplaced(self):
        stage = self._api.dataModel.stage
        self._active = False
        if not stage:
            if self._lib:
                self._lib.RigExecImaging_Deactivate()
            return
        # Only engage for stages that actually carry a RigExec rig.
        self._rigPath = None
        for prim in stage.Traverse():
            if prim.GetTypeName() == "RigExecRig":
                self._rigPath = prim.GetPath()
                break
        if self._rigPath is None:
            return
        try:
            if self._lib is None:
                self._lib = _LoadRigExecImaging()
        except OSError as error:
            Tf.Warn("rigExecUsdview: failed to load rigExecImaging.dll: %s"
                    % error)
            return

        cacheId = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
        status = self._lib.RigExecImaging_Activate(
            cacheId, b"", self._FrameValue())
        if status == 0:
            self._active = True
        else:
            Tf.Warn("rigExecUsdview: activation failed (%d)" % status)

    def _OnFrameChanged(self, frame):
        if not (self._active and self._lib):
            return
        self._lib.RigExecImaging_SetTime(self._FrameValue())


Tf.Type.Define(RigExecUsdviewContainer)
