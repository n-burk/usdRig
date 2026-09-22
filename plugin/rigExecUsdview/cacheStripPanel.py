#
# Cache-strip panel model: the Stream 3 strip's poll, skip, and actions.
#
# Headless by contract: this module imports neither Qt nor pxr, so the
# ctest suite below exercises the exact poll/skip/action logic the Qt
# dialog in cacheStripUI.py drives. State fetch, the sleep/wake check,
# and the Maya palette all live in cacheStripModel.py; this module adds
# the panel's own state on top: the rig, the stage range, the playhead,
# the repaint gate, and the clear/warm actions.
#
# The repaint gate: a driver-tick poll whose completions counter is
# still, with an unchanged playhead and range and no enqueue since the
# last paint, skips the state fetch AND the repaint. Anything else --
# a moved counter, moved playhead, changed states, a SetTime, an edit
# commit, a range change, a clear, a warm push -- refetches and repaints
# when the snapshot differs. An older library without the counter
# binding degrades to fetching every tick and repainting on states
# alone, never to skipping repaints it cannot prove needless.
#
import os
import sys

try:
    import cacheStripModel
except ImportError:  # pragma: no cover - plugin path, not test path
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import cacheStripModel


# Poll reasons. Only "tick" is counter-gated; every other reason forces
# a refetch, because each names something the completions counter
# cannot see: SetTime memoizes live results, an edit commit dirties,
# a range change reframes, a clear empties, a warm push enqueues.
REASON_TICK = "tick"
REASON_SETTIME = "settime"
REASON_COMMIT = "commit"
REASON_RANGE = "range"
REASON_CLEAR = "clear"
REASON_WARM = "warm"


class CacheStripPanelModel(object):
    """One strip's rig, range, playhead, and painted snapshot."""

    def __init__(self, rigPath=""):
        self._rigPath = rigPath
        self._frames = []
        self._playhead = None
        self._states = None
        self._completed = None
        # The last painted snapshot: what the view currently shows.
        # A poll repaints only when the fresh snapshot differs from it.
        self._paintedOnce = False
        self._paintedRig = None
        self._paintedFrames = None
        self._paintedPlayhead = None
        self._paintedStates = None
        self._paintedCompleted = None
        # Set by range/rig/action changes: the next tick refetches even
        # when the counter is still, because our own change moved states
        # no counter observes (a clear empties, a warm push enqueues).
        self._forced = False

    @property
    def rigPath(self):
        return self._rigPath

    @property
    def frames(self):
        return list(self._frames)

    @property
    def playhead(self):
        return self._playhead

    @property
    def states(self):
        """Last fetched states, in frames order; None before first fetch."""
        return list(self._states) if self._states is not None else None

    @property
    def completedCount(self):
        """Last read completions counter; None without the binding."""
        return self._completed

    def SetRig(self, rigPath):
        """Follow a new rig; True when the rig actually changed."""
        if rigPath == self._rigPath:
            return False
        self._rigPath = rigPath
        self._forced = True
        return True

    def SetRange(self, startTime, endTime):
        """Follow a stage range; True when the frame list actually changed.

        The range is the stage's start/end timecodes, rounded inward to
        whole frames by cacheStripModel.FrameListForRange.
        """
        frames = cacheStripModel.FrameListForRange(startTime, endTime)
        if frames == self._frames:
            return False
        self._frames = frames
        self._forced = True
        return True

    def SetPlayhead(self, frame):
        """Follow the playhead; a move alone repaints (the marker moves)."""
        self._playhead = frame

    def Counts(self):
        """Painted states tallied by display role, for the status line."""
        counts = {}
        for state in self._paintedStates or []:
            role = cacheStripModel.RoleForState(state)
            counts[role] = counts.get(role, 0) + 1
        return counts

    def Poll(self, lib, reason=REASON_TICK, mayHaveEnqueued=True):
        """Poll the C side; True when the view must repaint.

        `mayHaveEnqueued` is the recurring driver's own knowledge:
        False only when no OnIdle sweep ran since the last poll, so no
        enqueue could have flipped uncached to warming behind the
        counter's back. The driver passes False on its sleep paths and
        True everywhere else; SetTime always passes True.
        """
        forced = (reason != REASON_TICK or self._forced
                  or not self._paintedOnce)
        completed = cacheStripModel.FetchCompletedCount(lib)
        if (not forced and not mayHaveEnqueued and completed is not None
                and completed == self._paintedCompleted
                and self._playhead == self._paintedPlayhead
                and self._frames == self._paintedFrames
                and self._rigPath == self._paintedRig):
            # The counter is still and nothing could have moved: skip
            # the fetch and the repaint both.
            return False
        if self._frames:
            states = cacheStripModel.FetchFrameStates(
                lib, self._rigPath, self._frames)
        else:
            states = []
        if states is None:
            # Unknown rig or missing binding: hold the last paint, which
            # degrades to frame-change-only warming upstream. Paint once
            # so the view shows the unavailable state instead of nothing.
            if not self._paintedOnce:
                self._paintedOnce = True
                self._paintedStates = []
                self._paintedCompleted = completed
                self._paintedPlayhead = self._playhead
                self._paintedFrames = list(self._frames)
                self._paintedRig = self._rigPath
                self._forced = False
                return True
            return False
        repaint = (
            not self._paintedOnce
            or states != self._paintedStates
            or completed != self._paintedCompleted
            or self._playhead != self._paintedPlayhead
            or self._frames != self._paintedFrames
            or self._rigPath != self._paintedRig)
        self._states = states
        self._completed = completed
        self._paintedStates = states
        self._paintedCompleted = completed
        self._paintedPlayhead = self._playhead
        self._paintedFrames = list(self._frames)
        self._paintedRig = self._rigPath
        self._paintedOnce = True
        self._forced = False
        return repaint

    def Clear(self, lib):
        """Drop this rig's cached frames (the strip's clear action).

        True on success; the next poll refetches, because the fenced
        clear emptied states the completions counter never observes.
        """
        ok = cacheStripModel.ClearFrameCache(lib, self._rigPath)
        if ok:
            self._forced = True
        return ok

    def WarmRange(self, lib):
        """Push the strip's frames as the rig's warm range.

        True on success; the next poll refetches, because the push
        enqueues warming work no counter observes yet.
        """
        ok = cacheStripModel.PushWarmRange(
            lib, self._rigPath, self._frames)
        if ok:
            self._forced = True
        return ok
