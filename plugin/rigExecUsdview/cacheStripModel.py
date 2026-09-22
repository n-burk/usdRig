#
# Cache-strip model: per-frame warming states without Qt or usdview.
#
# Headless by contract: this module imports neither Qt nor pxr, so the
# Stream 3 strip dialog, the recurring warming driver in
# rigExecUsdview.py, and the ctest suite below all share one state fetch,
# one sleep/wake check, and one palette, and cannot drift into three
# different answers.
#
# State ints mirror the C API exactly (registry.h:
# RigExecImaging_GetFrameStates writes 0 uncached, 1 warming, 2 cached,
# 3 dirty). The palette is the Maya-style coloring, defined once here;
# the Stream 3 view reuses it rather than naming its own colors.
#
import ctypes


# Per-frame states, in C-API int order.
UNCACHED = 0
WARMING = 1
CACHED = 2
DIRTY = 3

STATES = (UNCACHED, WARMING, CACHED, DIRTY)

# Display roles, one per state: what the strip legend names.
ROLE_UNCACHED = "uncached"
ROLE_WARMING = "warming"
ROLE_CACHED = "cached"
ROLE_DIRTY = "dirty"

STATE_ROLES = {
    UNCACHED: ROLE_UNCACHED,
    WARMING: ROLE_WARMING,
    CACHED: ROLE_CACHED,
    DIRTY: ROLE_DIRTY,
}

# Maya-style timeline palette, defined once: cached reads done (green),
# warming reads in-flight (blue), dirty reads stale (red), uncached
# reads untouched (dark grey, near the timeline's own background).
PALETTE = {
    ROLE_CACHED: "#4CAF50",
    ROLE_WARMING: "#42A5F5",
    ROLE_DIRTY: "#EF5350",
    ROLE_UNCACHED: "#616161",
}


def RoleForState(state):
    """The display role for a C-API state int.

    Unknown ints read as uncached rather than raising: a newer library
    may report states this model predates, and the strip must still
    paint them instead of dropping the repaint.
    """
    return STATE_ROLES.get(state, ROLE_UNCACHED)


def ColorForState(state):
    """The strip color for a C-API state int, as a #rrggbb string."""
    return PALETTE[RoleForState(state)]


def AnyUnwarm(states):
    """Whether any frame still needs warming work.

    The recurring driver's sleep/wake check: cached is the only settled
    state, so warming (in flight), dirty (stale), uncached, and unknown
    all keep the driver ticking. An empty range is settled.
    """
    return any(state != CACHED for state in states)


def AnyWarming(states):
    """Whether any frame has work in flight right now."""
    return any(state == WARMING for state in states)


def CachedCount(states):
    """How many of `states` read cached (a progress readout's numerator)."""
    return sum(1 for state in states if state == CACHED)


def PlayheadFraction(frames, playhead):
    """Horizontal fraction [0, 1] of the playhead cell's center, or None.

    A strip paints one cell per frame; the playhead line drops through
    the center of the cell whose frame the playhead rounds to. None when
    there is no strip to mark (no frames), no playhead, or the playhead
    sits outside the range -- every caller paints no line then. The one
    playhead-cell rounding both strip painters share.
    """
    frames = list(frames) if frames else []
    if not frames:
        return None
    try:
        value = float(playhead)
    except (TypeError, ValueError):
        return None
    if value != value:  # NaN marks nothing.
        return None
    count = len(frames)
    first = float(frames[0])
    last = float(frames[-1])
    if value < first or value > last:
        return None
    step = (last - first) / (count - 1) if count > 1 else 1.0
    if step <= 0:
        step = 1.0
    index = int(round((value - first) / step))
    index = max(0, min(count - 1, index))
    return (index + 0.5) / count


def FrameListForRange(startTime, endTime):
    """Whole frames covering [startTime, endTime], inclusive.

    The strip and the warm range speak whole frames while stage
    timecodes are fractional, so a fractional bound rounds inward:
    [1.5, 4.2] warms frames 2, 3, 4. Empty when nothing whole fits.
    """
    try:
        start = int(startTime)
        end = int(endTime)
    except (TypeError, ValueError):
        return []
    if float(start) < float(startTime):
        start += 1
    if float(end) > float(endTime):
        end -= 1
    if end < start:
        return []
    return [float(frame) for frame in range(start, end + 1)]


def WarmRangeDrifted(cachedFrames, startTime, endTime):
    """Whether the stage range no longer matches the pushed warm range.

    The recurring driver's range-change check: an extended, shrunk, or
    never-pushed range needs a fresh PushWarmRange, while an identical
    range (including empty versus empty) needs none. Fractional bounds
    round inward exactly as the push does, so the comparison never
    re-pushes a range the stage did not actually change.
    """
    if cachedFrames is None:
        cachedFrames = []
    return [float(frame) for frame in cachedFrames] != FrameListForRange(
        startTime, endTime)


def FetchFrameStates(lib, rigPath, frames):
    """Batched per-frame states over `frames`, in order.

    One C call per strip repaint, no sampling on the query path. `lib`
    is the loaded rigExecImaging library (or anything carrying
    RigExecImaging_GetFrameStates with the C signature -- the tests pass
    a double). Returns the state ints, or None when the binding is
    missing, the rig is unknown, or the query fails: the caller degrades
    to frame-change-only warming rather than guessing at colors.
    """
    frames = list(frames)
    if not frames:
        return []
    entry = getattr(lib, "RigExecImaging_GetFrameStates", None)
    if entry is None:
        return None
    if isinstance(rigPath, str):
        rigPath = rigPath.encode("utf-8")
    inBuffer = (ctypes.c_double * len(frames))(*frames)
    outBuffer = (ctypes.c_int * len(frames))()
    written = entry(rigPath, inBuffer, outBuffer, len(frames))
    if written != len(frames):
        return None
    return [int(outBuffer[i]) for i in range(len(frames))]


def PushWarmRange(lib, rigPath, frames):
    """Sets the rig's persistent warm range to `frames`.

    The full-range cursor visits these closest-first from the playhead
    across idle triggers instead of the default playhead-relative sweep.
    An empty list warms nothing. True on success, False when the binding
    is missing or the rig is unknown.
    """
    frames = list(frames)
    if isinstance(rigPath, str):
        rigPath = rigPath.encode("utf-8")
    entry = getattr(lib, "RigExecImaging_WarmRange", None)
    if entry is None:
        return False
    if frames:
        inBuffer = (ctypes.c_double * len(frames))(*frames)
    else:
        inBuffer = None
    return entry(rigPath, inBuffer, len(frames)) == 0


def FetchCompletedCount(lib):
    """Lifetime warming completions, the strip's repaint gate.

    A poll whose counter is still, with unchanged states and playhead,
    skips the repaint. None when the binding is missing (an older
    library): the caller degrades to repainting on states alone rather
    than skipping repaints it cannot prove needless.
    """
    entry = getattr(lib, "RigExecImaging_GetWarmingCompletedCount", None)
    if entry is None:
        return None
    return int(entry())


def ClearFrameCache(lib, rigPath):
    """Drops one rig's cached frames (the strip's clear action).

    True on success, False when the binding is missing. Unknown rigs
    answer success: nothing to drop is still dropped.
    """
    if isinstance(rigPath, str):
        rigPath = rigPath.encode("utf-8")
    entry = getattr(lib, "RigExecImaging_ClearFrameCache", None)
    if entry is None:
        return False
    return entry(rigPath) == 0
