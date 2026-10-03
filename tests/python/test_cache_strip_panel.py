#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/cacheStripPanel.py: the Stream 3
strip's range tracking, poll/repaint gate, and clear/warm actions.

Everything here runs with neither Qt nor pxr -- no usdview, no stage.
The C entry points are test doubles with the C signatures, mirroring
test_cache_strip_model.py. The Qt dialog in cacheStripUI.py drives this
same model, so the skips and actions asserted here are the ones the
artist sees.

Usage: test_cache_strip_panel.py
"""
import re
import sys

# Sibling module: this script's own directory is sys.path[0].
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

import cacheStripModel as strip  # noqa: E402
import cacheStripPanel as panel  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


class _FakeLib(object):
    """A rigExecImaging stand-in with the C signatures.

    statesFor maps rig-path bytes to {frame: state}; missing rigs
    answer -1 like the C unknown-rig path. completedCount is the
    lifetime completions counter the test moves by hand. Every call is
    recorded.
    """

    def __init__(self, statesFor=None, completedCount=0):
        self._statesFor = statesFor or {}
        self.completedCount = completedCount
        self.stateCalls = []
        self.counterCalls = 0
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
            statesOut[i] = table.get(frames[i], strip.UNCACHED)
        return count

    def RigExecImaging_GetWarmingCompletedCount(self):
        self.counterCalls += 1
        return self.completedCount

    def RigExecImaging_WarmRange(self, rigPath, frames, count):
        self.warmCalls.append((bytes(rigPath), [frames[i] for i in
                                                range(count)]
                               if frames is not None else []))
        return 0

    def RigExecImaging_ClearFrameCache(self, rigPath):
        self.clearCalls.append(bytes(rigPath))
        return 0


def _Model(rig="/Asset/Rig", start=1.0, end=4.0, playhead=1.0):
    model = panel.CacheStripPanelModel(rig)
    model.SetRange(start, end)
    model.SetPlayhead(playhead)
    return model


def TestRangeFromStageTimecodes():
    model = panel.CacheStripPanelModel("/Asset/Rig")
    _Check(model.SetRange(1.0, 4.0), "a new range takes")
    _Check(model.frames == [1.0, 2.0, 3.0, 4.0],
           "whole timecodes stay inclusive: %s" % (model.frames,))
    _Check(not model.SetRange(1.0, 4.0),
           "the same range is not a change")
    _Check(model.SetRange(1.5, 4.2), "a moved range takes")
    _Check(model.frames == [2.0, 3.0, 4.0],
           "fractional timecodes round inward: %s" % (model.frames,))
    _Check(model.SetRange(4.0, 1.0), "an inverted range takes")
    _Check(model.frames == [], "and warms nothing")


def TestRigSwitching():
    model = _Model()
    _Check(not model.SetRig("/Asset/Rig"), "the same rig is not a change")
    _Check(model.SetRig("/Asset/Other"), "a new rig takes")
    _Check(model.rigPath == "/Asset/Other", "and sticks")


def TestFirstPollRepaints():
    lib = _FakeLib({b"/Asset/Rig": {1.0: strip.CACHED}})
    model = _Model()
    _Check(model.Poll(lib), "the first poll repaints")
    _Check(lib.counterCalls == 1, "after reading the counter once")
    _Check(len(lib.stateCalls) == 1, "and fetching states once")
    _Check(model.states == [strip.CACHED, strip.UNCACHED, strip.UNCACHED,
                             strip.UNCACHED],
           "states in frames order: %s" % (model.states,))
    _Check(model.completedCount == 0, "the counter is remembered")


def TestStillCounterSkipsFetchAndRepaint():
    lib = _FakeLib({b"/Asset/Rig": {1.0: strip.CACHED}})
    model = _Model()
    _Check(model.Poll(lib), "the first poll repaints")
    calls = (lib.counterCalls, len(lib.stateCalls))
    _Check(not model.Poll(lib, panel.REASON_TICK, False),
           "a still counter with no enqueue skips the repaint")
    _Check((lib.counterCalls, len(lib.stateCalls)) == (calls[0] + 1,
                                                       calls[1]),
           "reading only the counter, never the states")
    # ...but the same tick WITH a possible enqueue refetches.
    _Check(not model.Poll(lib, panel.REASON_TICK, True),
           "unchanged states still skip the repaint")
    _Check(len(lib.stateCalls) == calls[1] + 1,
           "after refetching, since a sweep may have enqueued")


def TestMovedCounterRefetchesAndRepaints():
    lib = _FakeLib({b"/Asset/Rig": {1.0: strip.CACHED}})
    model = _Model()
    model.Poll(lib)
    lib.completedCount = 7
    lib._statesFor[b"/Asset/Rig"][2.0] = strip.CACHED
    _Check(model.Poll(lib, panel.REASON_TICK, False),
           "a moved counter repaints even with no enqueue")
    _Check(model.completedCount == 7, "and remembers the new counter")
    _Check(model.states[1] == strip.CACHED,
           "with the newly completed frame: %s" % (model.states,))


def TestEnqueueFlipRepaintsWithCounterStill():
    lib = _FakeLib()
    model = _Model()
    model.Poll(lib)
    # The tick's own sweep enqueued frame 1: uncached reads warming
    # with no completion anywhere, so the counter is still.
    lib._statesFor[b"/Asset/Rig"] = {1.0: strip.WARMING}
    _Check(model.Poll(lib, panel.REASON_TICK, True),
           "an enqueue flip repaints with the counter still")
    _Check(model.states[0] == strip.WARMING,
           "showing warming: %s" % (model.states,))


def TestPlayheadMoveRepaints():
    lib = _FakeLib({b"/Asset/Rig": {1.0: strip.CACHED}})
    model = _Model()
    model.Poll(lib)
    model.SetPlayhead(2.0)
    _Check(model.Poll(lib, panel.REASON_SETTIME, True),
           "a SetTime repaints (the marker moved)")
    model.SetPlayhead(2.0)
    _Check(not model.Poll(lib, panel.REASON_TICK, False),
           "a settled playhead skips again")


def TestReasonsForceTheFetch():
    lib = _FakeLib({b"/Asset/Rig": {1.0: strip.CACHED}})
    model = _Model()
    model.Poll(lib)
    for reason in (panel.REASON_SETTIME, panel.REASON_COMMIT,
                   panel.REASON_RANGE, panel.REASON_CLEAR,
                   panel.REASON_WARM):
        before = len(lib.stateCalls)
        model.Poll(lib, reason, False)
        _Check(len(lib.stateCalls) == before + 1,
               "%s forces a refetch past a still counter" % reason)


def TestOlderLibraryDegradesToStatesAlone():
    states = {1.0: strip.CACHED}

    class _OldLib(_FakeLib):
        # A library predating the counter binding: no
        # RigExecImaging_GetWarmingCompletedCount at all.
        RigExecImaging_GetWarmingCompletedCount = None

    lib = _OldLib({b"/Asset/Rig": states})
    model = _Model()
    _Check(model.Poll(lib), "the first poll repaints")
    _Check(model.completedCount is None, "with no counter to remember")
    _Check(not model.Poll(lib, panel.REASON_TICK, False),
           "equal states still skip the repaint")
    _Check(len(lib.stateCalls) == 2,
           "but only after fetching: no counter, no fetch skip")
    states[2.0] = strip.CACHED
    _Check(model.Poll(lib, panel.REASON_TICK, False),
           "changed states repaint")


def TestFetchCompletedCount():
    lib = _FakeLib(completedCount=41)
    _Check(strip.FetchCompletedCount(lib) == 41, "the counter crosses")
    _Check(strip.FetchCompletedCount(object()) is None,
           "an older library without the binding answers None")


def TestClearAction():
    lib = _FakeLib({b"/Asset/Rig": {1.0: strip.CACHED,
                                    2.0: strip.CACHED}})
    model = _Model()
    model.Poll(lib)
    lib._statesFor[b"/Asset/Rig"] = {}
    _Check(model.Clear(lib), "clear drives the C binding")
    _Check(lib.clearCalls == [b"/Asset/Rig"],
           "with the rig path as bytes")
    _Check(model.Poll(lib, panel.REASON_TICK, False),
           "the next tick refetches past the still counter")
    _Check(model.states == [strip.UNCACHED] * 4,
           "showing the emptied cache: %s" % (model.states,))
    _Check(not model.Clear(object()), "without the binding it fails")


def TestClearFailureDoesNotForce():
    lib = _FakeLib({b"/Asset/Rig": {1.0: strip.CACHED}})
    model = _Model()
    model.Poll(lib)
    _Check(not model.Clear(object()), "a missing binding fails")
    _Check(not model.Poll(lib, panel.REASON_TICK, False),
           "and a failed clear forces nothing")


def TestWarmAction():
    lib = _FakeLib()
    model = _Model()
    model.Poll(lib)
    _Check(model.WarmRange(lib), "warm drives the C binding")
    _Check(lib.warmCalls == [(b"/Asset/Rig", [1.0, 2.0, 3.0, 4.0])],
           "with the strip's own frames: %s" % (lib.warmCalls,))
    lib._statesFor[b"/Asset/Rig"] = {1.0: strip.WARMING}
    _Check(model.Poll(lib, panel.REASON_TICK, False),
           "the next tick refetches the enqueued warming")
    _Check(not model.WarmRange(object()), "without the binding it fails")


def TestUnknownRigPaintsUnavailableOnce():
    lib = _FakeLib()
    model = _Model(rig="/Nope")
    _Check(model.Poll(lib), "an unknown rig paints once (unavailable)")
    _Check(model.states is None, "holding no states: %s" % (model.states,))
    _Check(not model.Poll(lib), "then holds the paint")


def TestCounts():
    lib = _FakeLib({b"/Asset/Rig": {1.0: strip.CACHED,
                                    2.0: strip.WARMING,
                                    3.0: strip.DIRTY}})
    model = _Model()
    model.Poll(lib)
    _Check(model.Counts() == {"cached": 1, "warming": 1, "dirty": 1,
                              "uncached": 1},
           "tallied by display role: %s" % (model.Counts(),))


def TestHeadlessAndSinglePalette():
    for loaded in ("pxr", "PySide", "PyQt5", "PyQt6"):
        _Check(loaded not in sys.modules,
               "cacheStripPanel must not pull in %s" % loaded)
    with open(panel.__file__, "r") as handle:
        source = handle.read()
    forbidden = re.findall(r"(?m)^\s*(?:import|from)\s+(\S+)", source)
    for name in forbidden:
        top = name.split(".")[0]
        _Check(top not in ("pxr", "PySide2", "PySide6", "PyQt5", "PyQt6"),
               "cacheStripPanel.py must not import %s" % name)
    # The panel names no colors of its own: every role color comes from
    # the once-defined palette in cacheStripModel.
    _Check(re.search(r"#[0-9a-fA-F]{6}", source) is None,
           "cacheStripPanel.py must name no hex color of its own")
    import os
    uiPath = os.path.join(os.path.dirname(panel.__file__),
                          "cacheStripUI.py")
    with open(uiPath, "r") as handle:
        uiSource = handle.read()
    _Check("cacheStripModel.PALETTE" in uiSource,
           "the Qt dialog reads the shared palette")
    _Check("cacheStripModel.ColorForState" in uiSource,
           "and paints cells with the shared state colors")
    _Check(re.search(r"#[0-9a-fA-F]{6}", uiSource) is None,
           "naming no hex color of its own")


def main():
    groups = [
        ("stage ranges become frame lists", TestRangeFromStageTimecodes),
        ("rig switching", TestRigSwitching),
        ("first poll repaints", TestFirstPollRepaints),
        ("still counter skips fetch and repaint",
         TestStillCounterSkipsFetchAndRepaint),
        ("moved counter refetches and repaints",
         TestMovedCounterRefetchesAndRepaints),
        ("enqueue flip repaints with counter still",
         TestEnqueueFlipRepaintsWithCounterStill),
        ("playhead move repaints", TestPlayheadMoveRepaints),
        ("reasons force the fetch", TestReasonsForceTheFetch),
        ("older library degrades to states alone",
         TestOlderLibraryDegradesToStatesAlone),
        ("completed-count fetch", TestFetchCompletedCount),
        ("clear action", TestClearAction),
        ("failed clear forces nothing", TestClearFailureDoesNotForce),
        ("warm action", TestWarmAction),
        ("unknown rig paints unavailable once",
         TestUnknownRigPaintsUnavailableOnce),
        ("counts tally by role", TestCounts),
        ("headless import, single palette", TestHeadlessAndSinglePalette),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("CACHE_STRIP_PANEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
