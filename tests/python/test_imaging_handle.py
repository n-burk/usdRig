#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/imagingHandle.py and the RigExec
container's use of it: every per-stage call reaches THIS session's stage.

rigExecImaging keeps one imaging context per UsdStage and exports every
per-stage entry point twice (docs/multistage-imaging.md): the legacy
spelling, which drives the CURRENT context, and `...ForStage(stageCacheId,
...)`, which drives that stage's own. The handle picks the ForStage
spelling when the library has it and falls back to the legacy one when it
does not. The libraries are doubles here, so this runs with no
rigExecImaging, no display and no usdview.

Two stages opened from the SAME layer -- identical prim paths -- are the
case that matters: a call keyed by path alone could not tell them apart.

Usage: test_imaging_handle.py
"""
import ctypes
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd, UsdUtils  # noqa: E402

import imagingHandle  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


class _LegacyLibrary(object):
    """A library from before per-stage contexts: legacy spellings only."""

    def __init__(self):
        self.calls = []

    def RigExecImaging_Activate(self, cacheId, rigPath, frame):
        self.calls.append(("Activate", cacheId, rigPath, frame))
        return 0

    def RigExecImaging_SetTime(self, frame):
        self.calls.append(("SetTime", frame))
        return 0

    def RigExecImaging_OnIdle(self):
        self.calls.append(("OnIdle",))
        return 0

    def RigExecImaging_OnEditCommitted(self):
        self.calls.append(("OnEditCommitted",))
        return 0

    def RigExecImaging_Deactivate(self):
        self.calls.append(("Deactivate",))

    def RigExecImaging_GetGeneration(self):
        self.calls.append(("GetGeneration",))
        return 7

    def RigExecImaging_WarmRange(self, rigPath, frames, count):
        self.calls.append(("WarmRange", bytes(rigPath), count))
        return 0


class _MultiStageLibrary(_LegacyLibrary):
    """A per-stage library: the legacy spellings AND the ForStage ones.

    Every ForStage call records the cache id it was given, so the test can
    say which stage's context a call reached.
    """

    def __init__(self):
        super(_MultiStageLibrary, self).__init__()
        self.generations = {}
        self.active = set()

    def RigExecImaging_Activate(self, cacheId, rigPath, frame):
        self.active.add(cacheId)
        return super(_MultiStageLibrary, self).RigExecImaging_Activate(
            cacheId, rigPath, frame)

    def RigExecImaging_SetTimeForStage(self, cacheId, frame):
        self.calls.append(("SetTimeForStage", cacheId, frame))
        self.generations[cacheId] = self.generations.get(cacheId, 0) + 1
        return 0

    def RigExecImaging_OnIdleForStage(self, cacheId):
        self.calls.append(("OnIdleForStage", cacheId))
        return 0

    def RigExecImaging_OnEditCommittedForStage(self, cacheId):
        self.calls.append(("OnEditCommittedForStage", cacheId))
        return 0

    def RigExecImaging_DeactivateForStage(self, cacheId):
        self.calls.append(("DeactivateForStage", cacheId))
        self.active.discard(cacheId)

    def RigExecImaging_GetGenerationForStage(self, cacheId):
        self.calls.append(("GetGenerationForStage", cacheId))
        return self.generations.get(cacheId, 0)

    def RigExecImaging_GetFrameStatesForStage(
            self, cacheId, rigPath, frames, statesOut, count):
        self.calls.append(("GetFrameStatesForStage", cacheId, bytes(rigPath),
                           count))
        for i in range(count):
            statesOut[i] = 2
        return count

    def RigExecImaging_IsActiveForStage(self, cacheId):
        return int(cacheId in self.active)


def _TwinStages():
    """Two stages over ONE layer: identical prim paths, distinct stages."""
    layer = Sdf.Layer.CreateAnonymous("shared.usda")
    Usd.Stage.Open(layer).DefinePrim("/Rig", "RigExecRoot")
    first = Usd.Stage.Open(layer, Sdf.Layer.CreateAnonymous("sessionA"))
    second = Usd.Stage.Open(layer, Sdf.Layer.CreateAnonymous("sessionB"))
    return first, second


def _Erase(*stages):
    cache = UsdUtils.StageCache.Get()
    for stage in stages:
        if cache.Contains(stage):
            cache.Erase(stage)


def TestForStageRouting():
    """Each handle reaches its own stage's context, by cache id."""
    lib = _MultiStageLibrary()
    first, second = _TwinStages()
    try:
        a = imagingHandle.ImagingHandle(lib, first, insert=True)
        b = imagingHandle.ImagingHandle(lib, second, insert=True)
        _Check(a.IsMultiStage() and b.IsMultiStage(),
               "a library with DeactivateForStage is per-stage")
        _Check(a.CacheId() >= 0 and b.CacheId() >= 0,
               "insert=True caches both stages")
        _Check(a.CacheId() != b.CacheId(),
               "two stages over one layer are two cache entries")
        _Check(first.GetPrimAtPath("/Rig") and second.GetPrimAtPath("/Rig"),
               "and they do share the prim path")

        a.SetTime(3.0)
        a.SetTime(4.0)
        b.SetTime(1.0)
        _Check(("SetTimeForStage", a.CacheId(), 3.0) in lib.calls,
               "A's frame went to A's context: %r" % lib.calls)
        _Check(("SetTimeForStage", b.CacheId(), 1.0) in lib.calls,
               "B's frame went to B's context")
        _Check(not [c for c in lib.calls if c[0] == "SetTime"],
               "nothing reached the legacy (current-context) SetTime")
        _Check(a.GetGeneration() == 2 and b.GetGeneration() == 1,
               "each stage's generation moved with its own frames only")

        # The legacy names, looked up by getattr the way cacheStripModel
        # looks them up on the library.
        entry = getattr(b, "RigExecImaging_GetFrameStates", None)
        _Check(entry is not None, "the handle answers to the C spelling")
        frames = (ctypes.c_double * 3)(1.0, 2.0, 3.0)
        states = (ctypes.c_int * 3)()
        _Check(entry(b"/Rig", frames, states, 3) == 3,
               "the C-spelled call returns the library's answer")
        _Check(lib.calls[-1] == ("GetFrameStatesForStage", b.CacheId(),
                                 b"/Rig", 3),
               "and was routed to B: %r" % (lib.calls[-1],))

        a.OnEditCommitted()
        a.OnIdle()
        _Check(lib.calls[-2:] == [("OnEditCommittedForStage", a.CacheId()),
                                  ("OnIdleForStage", a.CacheId())],
               "warming triggers reach A only")

        # Activation already takes the id; deactivation is per stage.
        _Check(a.Activate(b"", 1.0) == 0 and b.Activate("", 1.0) == 0,
               "both stages activate")
        _Check(a.IsActive() and b.IsActive(), "both are active at once")
        a.Deactivate()
        _Check(lib.calls[-1] == ("DeactivateForStage", a.CacheId()),
               "A's deactivation names A")
        _Check(not a.IsActive() and b.IsActive(),
               "deactivating A leaves B active")
        _Check(not [c for c in lib.calls if c[0] == "Deactivate"],
               "the legacy global Deactivate was never called")
    finally:
        _Erase(first, second)
    print("  ok: ForStage routing keeps two same-path stages apart")


def TestLegacyFallback():
    """An older library gets exactly the old single-stage calls."""
    lib = _LegacyLibrary()
    first, _second = _TwinStages()
    try:
        handle = imagingHandle.ImagingHandle(lib, first, insert=True)
        _Check(not handle.IsMultiStage(), "no ForStage symbols: legacy")
        handle.SetTime(5.0)
        handle.OnIdle()
        handle.Deactivate()
        _Check(handle.GetGeneration() == 7, "legacy generation read")
        _Check(lib.calls == [("SetTime", 5.0), ("OnIdle",), ("Deactivate",),
                             ("GetGeneration",)],
               "legacy symbols, legacy arguments: %r" % lib.calls)
        _Check(handle.IsActive() is None,
               "a legacy library cannot say whether ONE stage is active")
        # The id-taking entry point keeps its own signature either way.
        _Check(handle.Activate("", 2.0) == 0, "activation")
        _Check(lib.calls[-1] == ("Activate", handle.CacheId(), b"", 2.0),
               "Activate carries the stage's cache id")
    finally:
        _Erase(first)
    print("  ok: legacy library falls back to the legacy symbols")


def TestMissingEntryPoints():
    """Neither spelling exported: AttributeError, like a CDLL."""
    lib = _LegacyLibrary()
    handle = imagingHandle.ImagingHandle(lib, None)
    _Check(getattr(handle, "RigExecImaging_BeginPreview", None) is None,
           "getattr with a default answers None for a missing entry point")
    _Check(not handle.Has("BeginPreview"), "Has() says so too")
    _Check(handle.Has("SetTime") and handle.Has("RigExecImaging_WarmRange"),
           "Has() accepts both spellings")
    try:
        handle.EndPreview()
    except AttributeError:
        pass
    else:
        raise AssertionError("a missing entry point must raise AttributeError")
    try:
        handle.RigExecImaging_NotAThing
    except AttributeError:
        pass
    else:
        raise AssertionError("an unknown name must raise AttributeError")
    _Check(imagingHandle.ImagingHandle(None, None).lib is None,
           "a handle over no library is inert")
    print("  ok: missing entry points raise AttributeError")


def TestLazyCacheId():
    """insert=False waits for someone else to cache the stage."""
    lib = _MultiStageLibrary()
    stage, _other = _TwinStages()
    try:
        handle = imagingHandle.ImagingHandle(lib, stage)
        _Check(handle.CacheId() == -1, "not cached, not inserted: -1")
        _Check(not UsdUtils.StageCache.Get().Contains(stage),
               "insert=False never inserts")
        cacheId = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
        _Check(handle.CacheId() == cacheId,
               "the id is looked up again once the stage is cached")
        handle.SetTime(2.0)
        _Check(lib.calls[-1] == ("SetTimeForStage", cacheId, 2.0),
               "and calls carry it")
    finally:
        _Erase(stage)
    print("  ok: cache id resolves lazily without inserting")


def _BareContainer(lib, stage):
    """A RigExec container with only what activation reads, as the
    testusdview harness builds them (testUsdviewRigExec.py)."""
    from rigExecUsdview import RigExecUsdviewContainer

    dataModel = type("DataModel", (), {})()
    dataModel.stage = stage
    dataModel.currentFrame = Usd.TimeCode(1)
    dataModel.viewSettings = type("ViewSettings", (), {})()
    dataModel.viewSettings.displayGuide = True
    api = type("UsdviewApi", (), {})()
    api.dataModel = dataModel

    container = RigExecUsdviewContainer.__new__(RigExecUsdviewContainer)
    container._api = api
    container._lib = lib
    container._active = False
    container._rigPaths = []
    container._cachedStage = None
    container._stageNoticeKey = None
    container._activating = False
    container._warmingCommitPending = False
    return container


def TestContainersNeverTouchEachOther():
    """Two sessions' containers in one process, one per-stage library."""
    lib = _MultiStageLibrary()
    first, second = _TwinStages()
    a = _BareContainer(lib, first)
    b = _BareContainer(lib, second)
    try:
        a._ActivateCurrentStage()
        b._ActivateCurrentStage()
        idA = a._Imaging().CacheId()
        idB = b._Imaging().CacheId()
        _Check(a._active and b._active, "both sessions activated")
        _Check(idA != idB and lib.active == {idA, idB},
               "both stages active at once: %r" % lib.active)
        _Check(not [c for c in lib.calls if c[0] == "Deactivate"],
               "no container ran the global Deactivate")

        # Frame changes drive only their own stage.
        a._OnFrameChanged(3.0)
        b._OnFrameChanged(1.0)
        _Check(("SetTimeForStage", idA, 3.0) in lib.calls and
               ("SetTimeForStage", idB, 1.0) in lib.calls,
               "each session's frame reached its own stage")
        _Check(lib.generations == {idA: 1, idB: 1},
               "one publication each: %r" % lib.generations)

        # A re-activation of A (the Reactivate command) deactivates A only.
        del lib.calls[:]
        a._ActivateCurrentStage()
        _Check(("DeactivateForStage", idA) in lib.calls,
               "A's reactivation deactivated A first")
        _Check(("DeactivateForStage", idB) not in lib.calls,
               "and never B")
        _Check(idB in lib.active, "B stayed active throughout")

        # A's shutdown leaves B evaluating.
        a._Shutdown()
        _Check(("DeactivateForStage", idA) in lib.calls and
               ("DeactivateForStage", idB) not in lib.calls,
               "A's shutdown deactivated A and not B")
        _Check(lib.active == {idB}, "B is still active: %r" % lib.active)
        b._OnFrameChanged(2.0)
        _Check(lib.generations[idB] == 2, "and B still evaluates")

        # A container that never activated deactivates nothing at all.
        idle = _BareContainer(lib, Usd.Stage.CreateInMemory())
        del lib.calls[:]
        idle._Shutdown()
        _Check(not [c for c in lib.calls if c[0].startswith("Deactivate")],
               "a never-activated container deactivates nothing: %r"
               % lib.calls)
    finally:
        b._Shutdown()
        _Erase(first, second)
    print("  ok: containers of two sessions never deactivate each other")


def TestContainerLegacyLibrary():
    """With an older library the container behaves exactly as before."""
    lib = _LegacyLibrary()
    stage, _other = _TwinStages()
    container = _BareContainer(lib, stage)
    try:
        container._ActivateCurrentStage()
        _Check(lib.calls[0] == ("Deactivate",),
               "activation starts with the global Deactivate, as before")
        _Check(lib.calls[1][0] == "Activate", "then activates")
        container._OnFrameChanged(2.0)
        _Check(("SetTime", 2.0) in lib.calls, "legacy SetTime per frame")
    finally:
        container._Shutdown()
        _Erase(stage)
    _Check(lib.calls[-1] == ("Deactivate",), "shutdown: global Deactivate")
    print("  ok: a legacy library keeps the single-stage behavior")


def main():
    TestForStageRouting()
    TestLegacyFallback()
    TestMissingEntryPoints()
    TestLazyCacheId()
    TestContainersNeverTouchEachOther()
    TestContainerLegacyLibrary()
    print("IMAGING_HANDLE_OK (6 groups)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
