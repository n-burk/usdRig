#
# RigExec usdview plugin: one stage's handle on rigExecImaging.
#
# rigExecImaging keeps one imaging CONTEXT per UsdStage (docs/
# multistage-imaging.md): its own snapshot store, chains, previews, warming
# and published generation. Its C surface has two spellings of every
# per-stage entry point:
#
#   RigExecImaging_<Name>(args...)                       the CURRENT context:
#       the stage most recently activated through RigExecImaging_Activate
#   RigExecImaging_<Name>ForStage(stageCacheId, args...) THAT stage's context
#
# A usdview session must only ever touch its own stage, and several sessions
# share one process in a host, so everything the plugins do goes through an
# ImagingHandle bound to the session's stage. The handle calls the ForStage
# symbol when the library exports it and falls back to the legacy symbol when
# it does not -- an older library is exactly the old single-stage behavior,
# said once in the log.
#
# The handle answers to the legacy NAMES in two ways, so callers written
# against the library keep working unchanged:
#
#   handle.SetTime(frame), handle.OnIdle(), handle.GetGeneration(), ...
#   handle.RigExecImaging_SetTime(frame)  -- the same, via getattr, which is
#       what cacheStripModel and the strip panel already look up.
#
# A missing entry point (neither spelling exported) raises AttributeError from
# both, exactly like a CDLL does, so `getattr(handle, name, None)` and the
# existing `except AttributeError` fallbacks behave as they did on the library.
#
# Activation is NOT per-stage-suffixed: RigExecImaging_Activate already takes
# the stage's cache id. Deactivation is: a handle only ever deactivates its
# own stage (DeactivateForStage), never another session's.
#
import ctypes

from pxr import Tf, UsdUtils


_PREFIX = "RigExecImaging_"
_SUFFIX = "ForStage"

_ll = ctypes.c_longlong
_i = ctypes.c_int
_d = ctypes.c_double
_s = ctypes.c_char_p
_dp = ctypes.POINTER(ctypes.c_double)
_ip = ctypes.POINTER(ctypes.c_int)
_fp = ctypes.POINTER(ctypes.c_float)

# name -> (restype, argtypes of the LEGACY spelling). The ForStage spelling
# is the same with the stage cache id (long long) in front. Only names that
# are per-stage appear here; Activate and GetControlFrameAssetSpace already
# take the id and are bound separately below.
_PER_STAGE = {
    "SetTime": (_i, [_d]),
    "OnEditCommitted": (_i, []),
    "OnIdle": (_i, []),
    "WriteProfileSummary": (_i, [_s]),
    "Deactivate": (None, []),
    "GetGeneration": (_ll, []),
    "GetMovedFloats": (_i, [_s, _fp, _i]),
    "SetWeightOverlay": (_i, [_s]),
    "GetFrameStates": (_i, [_s, _dp, _ip, _i]),
    "ClearFrameCache": (_i, [_s]),
    "WarmRange": (_i, [_s, _dp, _i]),
    "GetWarmingCompletedCount": (_ll, []),
    "BeginPreview": (_i, [_s]),
    "UpdatePreview": (_i, [_dp, _i]),
    "EndPreview": (_i, []),
    "GetGuideBoundsAssetSpace": (_i, [_s, _dp]),
    "GetAllGuideBoundsAssetSpace": (_i, [_dp]),
}

# Entry points that take the stage cache id in every library.
_STAGE_ID = {
    "Activate": (_i, [_ll, _s, _d]),
    "GetControlFrameAssetSpace": (_i, [_ll, _s, _d, _i, _dp]),
    "IsActiveForStage": (_i, [_ll]),
    "ContextCount": (_i, []),
}

# The symbol whose presence says the library has per-stage contexts.
_MULTI_STAGE_PROBE = _PREFIX + "DeactivateForStage"

_BOUND_MARK = "_rigExecImagingHandleBound"

# Whether the "library predates multi-stage support" warning was given. A
# log-once flag, not state any session reads.
_warnedLegacy = [False]


def _Bind(function, restype, argtypes):
    try:
        function.restype = restype
        function.argtypes = argtypes
    except (AttributeError, TypeError):
        pass    # a test double: plain Python callables take no signature


def BindLibrary(lib):
    """
    Give every per-stage symbol `lib` exports its ctypes signature, once.

    Idempotent and tolerant: a symbol the library does not export is
    skipped, and a double that is not a CDLL is left alone.
    """
    if lib is None or getattr(lib, _BOUND_MARK, False):
        return lib
    for name, (restype, argtypes) in _PER_STAGE.items():
        legacy = getattr(lib, _PREFIX + name, None)
        if legacy is not None:
            _Bind(legacy, restype, argtypes)
        scoped = getattr(lib, _PREFIX + name + _SUFFIX, None)
        if scoped is not None:
            _Bind(scoped, restype, [_ll] + list(argtypes))
    for name, (restype, argtypes) in _STAGE_ID.items():
        function = getattr(lib, _PREFIX + name, None)
        if function is not None:
            _Bind(function, restype, argtypes)
    try:
        setattr(lib, _BOUND_MARK, True)
    except Exception:
        pass
    return lib


def SupportsMultiStage(lib):
    """Whether `lib` keeps one imaging context per stage."""
    return lib is not None and getattr(lib, _MULTI_STAGE_PROBE,
                                       None) is not None


def _WarnLegacyOnce():
    if _warnedLegacy[0]:
        return
    _warnedLegacy[0] = True
    Tf.Warn("rigExecUsdview: rigExecImaging predates multi-stage support "
            "(no RigExecImaging_*ForStage entry points); RigExec evaluates "
            "one stage per process, the most recently activated.")


def StageCacheId(stage, insert=False):
    """
    `stage`'s UsdUtils.StageCache id as an int, or -1.

    With `insert`, a stage not in the cache is inserted -- which transfers
    ownership to the cache in this binding, so only the container that
    also erases it (rigExecUsdview._ReleaseCachedStage) passes True.
    """
    if not stage:
        return -1
    try:
        cache = UsdUtils.StageCache.Get()
        stageId = cache.GetId(stage)
        if not stageId.IsValid():
            if not insert:
                return -1
            stageId = cache.Insert(stage)
        return stageId.ToLongInt() if stageId.IsValid() else -1
    except Exception:
        return -1


class _Scoped(object):
    """A per-stage entry point with the legacy call signature."""

    __slots__ = ("_handle", "_function", "_scoped")

    def __init__(self, handle, function, scoped):
        self._handle = handle
        self._function = function
        self._scoped = scoped

    def __call__(self, *args):
        if self._scoped:
            return self._function(self._handle.CacheId(), *args)
        return self._function(*args)


class ImagingHandle(object):
    """
    One stage's view of rigExecImaging.

    `lib` is the loaded library (or a test double carrying the same
    symbols); `stage` the session's UsdStage. With `insert`, the stage is
    put in UsdUtils.StageCache if it is not there yet (see StageCacheId);
    without, a stage that is not cached has id -1 until someone caches it,
    and the id is looked up again on the next call.
    """

    def __init__(self, lib, stage, insert=False):
        self._lib = BindLibrary(lib)
        self._stage = stage
        self._insert = bool(insert)
        self._cacheId = StageCacheId(stage, self._insert)
        self._multiStage = SupportsMultiStage(lib)
        if lib is not None and not self._multiStage:
            _WarnLegacyOnce()

    def __repr__(self):
        return "<ImagingHandle stage=%s id=%d %s>" % (
            getattr(self._stage, "GetRootLayer", lambda: None)()
            if self._stage else None,
            self._cacheId,
            "per-stage" if self._multiStage else "legacy")

    # -- identity --------------------------------------------------------

    @property
    def lib(self):
        return self._lib

    @property
    def stage(self):
        return self._stage

    def IsMultiStage(self):
        """Whether calls reach this stage's own context (else the current)."""
        return self._multiStage

    def CacheId(self):
        """The stage's cache id, looked up again while it is still -1."""
        if self._cacheId < 0 and self._stage:
            self._cacheId = StageCacheId(self._stage, self._insert)
        return self._cacheId

    # -- dispatch --------------------------------------------------------

    def _Entry(self, name):
        """The callable for per-stage `name`, legacy signature; raises
        AttributeError like a CDLL when neither spelling is exported."""
        lib = self._lib
        if lib is None:
            raise AttributeError(_PREFIX + name)
        scoped = getattr(lib, _PREFIX + name + _SUFFIX, None)
        if scoped is not None:
            return _Scoped(self, scoped, True)
        legacy = getattr(lib, _PREFIX + name, None)
        if legacy is None:
            raise AttributeError(_PREFIX + name)
        return _Scoped(self, legacy, False)

    def Has(self, name):
        """Whether `name` (legacy spelling, prefix optional) is callable."""
        if name.startswith(_PREFIX):
            name = name[len(_PREFIX):]
        try:
            if name in _PER_STAGE:
                self._Entry(name)
                return True
            return getattr(self._lib, _PREFIX + name, None) is not None
        except AttributeError:
            return False

    def __getattr__(self, name):
        # Only reached for names the class does not define: the legacy
        # C spellings, so a handle stands in for the library wherever a
        # caller looks symbols up by name (cacheStripModel does). Entry
        # points that already take the cache id pass through with
        # their own (identical) signature.
        if name.startswith(_PREFIX):
            short = name[len(_PREFIX):]
            if short in _PER_STAGE:
                return self._Entry(short)
            if short in _STAGE_ID and self._lib is not None:
                return getattr(self._lib, name)
        raise AttributeError(name)

    # -- activation --------------------------------------------------------

    def Activate(self, rigPath=b"", frame=0.0):
        """RigExecImaging_Activate on this stage. 0 on success."""
        if isinstance(rigPath, str):
            rigPath = rigPath.encode("utf-8")
        activate = getattr(self._lib, _PREFIX + "Activate")
        return activate(self.CacheId(), rigPath, frame)

    def Deactivate(self):
        """Stop evaluating THIS stage (legacy library: the current one)."""
        return self._Entry("Deactivate")()

    def IsActive(self):
        """True/False from a per-stage library, None when it cannot say."""
        probe = getattr(self._lib, _PREFIX + "IsActiveForStage", None)
        if probe is None:
            return None
        return bool(probe(self.CacheId()))

    # -- the per-stage surface, by its legacy names --------------------------

    def SetTime(self, frame):
        return self._Entry("SetTime")(frame)

    def OnEditCommitted(self):
        return self._Entry("OnEditCommitted")()

    def OnIdle(self):
        return self._Entry("OnIdle")()

    def WriteProfileSummary(self, path):
        if isinstance(path, str):
            path = path.encode("utf-8")
        return self._Entry("WriteProfileSummary")(path)

    def GetGeneration(self):
        return self._Entry("GetGeneration")()

    def GetMovedFloats(self, packedPaths, out, count):
        return self._Entry("GetMovedFloats")(packedPaths, out, count)

    def SetWeightOverlay(self, primPath):
        if isinstance(primPath, str):
            primPath = primPath.encode("utf-8")
        return self._Entry("SetWeightOverlay")(primPath)

    def GetFrameStates(self, rigPath, frames, statesOut, count):
        return self._Entry("GetFrameStates")(rigPath, frames, statesOut,
                                             count)

    def ClearFrameCache(self, rigPath):
        return self._Entry("ClearFrameCache")(rigPath)

    def WarmRange(self, rigPath, frames, count):
        return self._Entry("WarmRange")(rigPath, frames, count)

    def GetWarmingCompletedCount(self):
        return self._Entry("GetWarmingCompletedCount")()

    def BeginPreview(self, packedPaths):
        if isinstance(packedPaths, str):
            packedPaths = packedPaths.encode("utf-8")
        return self._Entry("BeginPreview")(packedPaths)

    def UpdatePreview(self, values, count):
        return self._Entry("UpdatePreview")(values, count)

    def EndPreview(self):
        return self._Entry("EndPreview")()

    def GetGuideBoundsAssetSpace(self, primPath, outMinMax):
        if isinstance(primPath, str):
            primPath = primPath.encode("utf-8")
        return self._Entry("GetGuideBoundsAssetSpace")(primPath, outMinMax)

    def GetAllGuideBoundsAssetSpace(self, outMinMax):
        return self._Entry("GetAllGuideBoundsAssetSpace")(outMinMax)

    def GetControlFrameAssetSpace(self, primPath, frame, isDefault,
                                  outMatrix):
        """Already stage-scoped in every library: it takes the cache id."""
        if isinstance(primPath, str):
            primPath = primPath.encode("utf-8")
        read = getattr(self._lib, _PREFIX + "GetControlFrameAssetSpace")
        return read(self.CacheId(), primPath, frame, isDefault, outMatrix)


def ContextCount(lib):
    """RigExecImaging_ContextCount(), or None from an older library."""
    count = getattr(lib, _PREFIX + "ContextCount", None)
    return None if count is None else int(count())
