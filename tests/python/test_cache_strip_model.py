#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/cacheStripModel.py: the C-API
state fetch, the driver sleep/wake check, the state-to-role mapping,
and the once-defined palette the Stream 3 strip will reuse.

Everything here runs with neither Qt nor pxr -- no usdview, no stage.
The C entry points are test doubles with the C signatures; the one
native check is that the state ints match registry.h.

Usage: test_cache_strip_model.py
"""
import re
import sys

# Sibling module: this script's own directory is sys.path[0].
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

import cacheStripModel as model  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


class _FakeLib(object):
    """A rigExecImaging stand-in with the C signatures.

    statesFor maps rig-path bytes to {frame: state}; missing rigs
    answer -1 like the C unknown-rig path. Every call is recorded.
    """

    def __init__(self, statesFor=None, warmStatus=0, clearStatus=0):
        self._statesFor = statesFor or {}
        self._warmStatus = warmStatus
        self._clearStatus = clearStatus
        self.stateCalls = []
        self.warmCalls = []
        self.clearCalls = []

    def RigExecImaging_GetFrameStates(self, rigPath, frames, statesOut,
                                     count):
        self.stateCalls.append((bytes(rigPath), [frames[i] for i in
                                                 range(count)]))
        table = self._statesFor.get(bytes(rigPath))
        if table is None:
            return -1
        for i in range(count):
            statesOut[i] = table.get(frames[i], model.UNCACHED)
        return count

    def RigExecImaging_WarmRange(self, rigPath, frames, count):
        self.warmCalls.append((bytes(rigPath), [frames[i] for i in
                                                range(count)]
                               if frames is not None else []))
        return self._warmStatus

    def RigExecImaging_ClearFrameCache(self, rigPath):
        self.clearCalls.append(bytes(rigPath))
        return self._clearStatus


def TestStateIntsMatchTheCApi():
    # registry.h: 0 uncached, 1 warming, 2 cached, 3 dirty.
    _Check((model.UNCACHED, model.WARMING, model.CACHED, model.DIRTY)
           == (0, 1, 2, 3),
           "state ints must match RigExecImaging_GetFrameStates")
    _Check(set(model.STATES) == {0, 1, 2, 3}, "STATES names every state")


def TestRoleMapping():
    _Check(model.RoleForState(model.UNCACHED) == "uncached", "uncached")
    _Check(model.RoleForState(model.WARMING) == "warming", "warming")
    _Check(model.RoleForState(model.CACHED) == "cached", "cached")
    _Check(model.RoleForState(model.DIRTY) == "dirty", "dirty")
    # A newer library may report states this model predates; the strip
    # still paints them instead of dropping the repaint.
    _Check(model.RoleForState(99) == "uncached",
           "unknown states read as uncached")


def TestPaletteIsDefinedOnce():
    roles = {model.RoleForState(s) for s in model.STATES}
    _Check(set(model.PALETTE.keys()) == roles,
           "the palette names every role exactly once: %s"
           % sorted(model.PALETTE.keys()))
    colors = list(model.PALETTE.values())
    _Check(len(set(colors)) == len(colors),
           "every role reads distinctly: %s" % colors)
    for role, color in sorted(model.PALETTE.items()):
        _Check(re.match(r"^#[0-9a-fA-F]{6}$", color) is not None,
               "%s is a #rrggbb string: %r" % (role, color))
    _Check(model.ColorForState(model.CACHED) == model.PALETTE["cached"],
           "ColorForState comes from the palette, not a second copy")
    _Check(model.ColorForState(99) == model.PALETTE["uncached"],
           "unknown states take the uncached color")


def TestAnyUnwarm():
    _Check(not model.AnyUnwarm([]), "an empty range is settled")
    _Check(not model.AnyUnwarm([model.CACHED] * 64), "all cached sleeps")
    for state in (model.UNCACHED, model.WARMING, model.DIRTY, 99):
        _Check(model.AnyUnwarm([model.CACHED, state]),
               "state %r keeps the driver ticking" % (state,))
    _Check(model.AnyWarming([model.CACHED, model.WARMING]),
           "in-flight work is visible")
    _Check(not model.AnyWarming([model.CACHED, model.DIRTY]),
           "stale is unwarm but not in flight")


def TestFrameListForRange():
    _Check(model.FrameListForRange(1.0, 4.0) == [1.0, 2.0, 3.0, 4.0],
           "whole bounds stay inclusive")
    _Check(model.FrameListForRange(1.5, 4.2) == [2.0, 3.0, 4.0],
           "fractional bounds round inward")
    _Check(model.FrameListForRange(3.0, 3.0) == [3.0],
           "a single frame is still a range")
    _Check(model.FrameListForRange(4.0, 1.0) == [],
           "an inverted range warms nothing")
    _Check(model.FrameListForRange(1.2, 1.8) == [],
           "a range with no whole frame warms nothing")
    _Check(model.FrameListForRange(None, 4.0) == [],
           "a missing timecode warms nothing, not an exception")


def TestFetchFrameStates():
    lib = _FakeLib({b"/Asset/Rig": {1.0: model.CACHED,
                                    2.0: model.WARMING,
                                    3.0: model.DIRTY}})
    got = model.FetchFrameStates(lib, "/Asset/Rig", [1.0, 2.0, 3.0, 4.0])
    _Check(got == [model.CACHED, model.WARMING, model.DIRTY,
                   model.UNCACHED],
           "one batched call, states in order: %s" % (got,))
    _Check(len(lib.stateCalls) == 1, "a repaint costs one C call")
    rigPath, frames = lib.stateCalls[0]
    _Check(rigPath == b"/Asset/Rig", "the rig path crosses as bytes")
    _Check(frames == [1.0, 2.0, 3.0, 4.0], "the frame list crosses whole")
    # Empty ranges never reach the C side.
    _Check(model.FetchFrameStates(lib, "/Asset/Rig", []) == [],
           "an empty range answers empty")
    _Check(len(lib.stateCalls) == 1, "without a C call")
    # Unknown rigs and missing bindings degrade to None, never a guess.
    _Check(model.FetchFrameStates(lib, "/Nope", [1.0]) is None,
           "an unknown rig answers None")
    _Check(model.FetchFrameStates(object(), "/Asset/Rig", [1.0]) is None,
           "an older library without the binding answers None")


def TestPushWarmRange():
    lib = _FakeLib()
    _Check(model.PushWarmRange(lib, "/Asset/Rig", [1.0, 2.0, 3.0]),
           "a 0 status pushes")
    _Check(lib.warmCalls == [(b"/Asset/Rig", [1.0, 2.0, 3.0])],
           "path as bytes, frames whole: %s" % (lib.warmCalls,))
    _Check(model.PushWarmRange(lib, "/Asset/Rig", []),
           "an empty range still pushes (it warms nothing)")
    _Check(lib.warmCalls[-1] == (b"/Asset/Rig", []),
           "with no frames across: %s" % (lib.warmCalls[-1],))
    failing = _FakeLib(warmStatus=-1)
    _Check(not model.PushWarmRange(failing, "/Nope", [1.0]),
           "a -1 status does not push")
    _Check(not model.PushWarmRange(object(), "/Asset/Rig", [1.0]),
           "an older library without the binding does not push")


def TestClearFrameCache():
    lib = _FakeLib()
    _Check(model.ClearFrameCache(lib, "/Asset/Rig"), "a 0 status clears")
    _Check(lib.clearCalls == [b"/Asset/Rig"],
           "the rig path crosses as bytes")
    _Check(not model.ClearFrameCache(object(), "/Asset/Rig"),
           "an older library without the binding does not clear")


def TestWarmRangeDrifted():
    _Check(not model.WarmRangeDrifted([1.0, 2.0, 3.0], 1.0, 3.0),
           "an identical range has not drifted")
    _Check(model.WarmRangeDrifted([1.0, 2.0], 1.0, 3.0),
           "an extended stage range has drifted")
    _Check(model.WarmRangeDrifted([1.0, 2.0, 3.0], 1.0, 2.0),
           "a shrunk stage range has drifted")
    _Check(model.WarmRangeDrifted(None, 1.0, 3.0),
           "no pushed range yet always drifts")
    _Check(not model.WarmRangeDrifted([], 4.2, 4.1),
           "an empty stage range matches an empty push")
    _Check(not model.WarmRangeDrifted([2.0, 3.0, 4.0], 1.5, 4.2),
           "fractional bounds round inward before comparing")
    _Check(not model.WarmRangeDrifted(None, 4.2, 4.1),
           "never pushed plus an empty stage range needs no push")
    _Check(model.WarmRangeDrifted([1.0], "junk", "junk"),
           "an unreadable stage range clears a stale push")


def TestCachedCount():
    _Check(model.CachedCount([]) == 0, "no states, none cached")
    _Check(model.CachedCount([model.CACHED] * 4) == 4,
           "all cached counts all")
    _Check(model.CachedCount([model.CACHED, model.WARMING, model.DIRTY,
                              model.UNCACHED, 99]) == 1,
           "only cached counts -- warming, dirty, uncached, unknown")


def TestPlayheadFraction():
    frames = [1.0, 2.0, 3.0, 4.0]
    _Check(model.PlayheadFraction(frames, 1.0) == 0.125,
           "the first cell's center: %s"
           % (model.PlayheadFraction(frames, 1.0),))
    _Check(model.PlayheadFraction(frames, 4.0) == 0.875,
           "the last cell's center")
    _Check(model.PlayheadFraction(frames, 2.4) == 0.375,
           "a fractional playhead marks its cell's center")
    _Check(model.PlayheadFraction(frames, 0.9) is None,
           "before the range marks nothing")
    _Check(model.PlayheadFraction(frames, 4.1) is None,
           "past the range marks nothing")
    _Check(model.PlayheadFraction(frames, None) is None,
           "no playhead marks nothing")
    _Check(model.PlayheadFraction(frames, "junk") is None,
           "an unreadable playhead marks nothing")
    _Check(model.PlayheadFraction(frames, float("nan")) is None,
           "NaN marks nothing")
    _Check(model.PlayheadFraction([], 2.0) is None,
           "no frames mark nothing")
    _Check(model.PlayheadFraction([3.0], 3.0) == 0.5,
           "a one-frame strip centers its only cell")
    _Check(model.PlayheadFraction([3.0], 9.0) is None,
           "outside a one-frame strip marks nothing")


def TestImportsWithNeitherQtNorPxr():
    for loaded in ("pxr", "PySide", "PyQt5", "PyQt6"):
        _Check(loaded not in sys.modules,
               "cacheStripModel must not pull in %s" % loaded)
    with open(model.__file__, "r") as handle:
        source = handle.read()
    forbidden = re.findall(r"(?m)^\s*(?:import|from)\s+(\S+)", source)
    for name in forbidden:
        top = name.split(".")[0]
        _Check(top not in ("pxr", "PySide2", "PySide6", "PyQt5", "PyQt6"),
               "cacheStripModel.py must not import %s" % name)


def main():
    groups = [
        ("state ints match the C API", TestStateIntsMatchTheCApi),
        ("state-to-role mapping", TestRoleMapping),
        ("the palette is defined once", TestPaletteIsDefinedOnce),
        ("the sleep/wake check", TestAnyUnwarm),
        ("stage ranges become frame lists", TestFrameListForRange),
        ("batched state fetch", TestFetchFrameStates),
        ("warm-range push", TestPushWarmRange),
        ("warm-range drift", TestWarmRangeDrifted),
        ("cache clear", TestClearFrameCache),
        ("cached counting", TestCachedCount),
        ("playhead fraction", TestPlayheadFraction),
        ("headless import", TestImportsWithNeitherQtNorPxr),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("CACHE_STRIP_MODEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
