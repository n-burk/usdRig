#!/usr/bin/env python
"""
Evaluate gives the GIL up itself, so a caller that enters it holding the GIL
neither hangs nor gets a different answer.

The Python binding's `evaluate` releases the GIL before it calls in, but a
C++ host need not: Hydra's Render is entered from Python with the GIL held
and reaches RigExecRigEvaluator::Evaluate through the imaging bridge. Every
Evaluate joins TBB workers, and a worker that needs the GIL -- the first
exec definition lookup in a process loads its plugin through
TfScriptModuleLoader, which takes it -- would then wait on the very thread
that waits on it. `Rig._evaluate_holding_gil` is that host's entry point: it
calls Evaluate with the GIL still held.

The process starts cold on purpose. The first held call is the first thing
this interpreter asks of rigExec, so the epoch's compile, its plugin loads
and its first frame all happen under a caller holding the GIL. A second
Python thread keeps asking for the GIL throughout, which is the shape of the
deadlock, and a watchdog turns a hang into a traceback and a failure instead
of a ctest timeout.

Usage: test_rigexec_gil.py [<generated schema resources dir>]
"""
import faulthandler
import os
import sys
import threading

# The suite's shared bootstrap. It fixes sys.path (including the build tree's
# python directory, found from the resources argument the add_test entry
# passes), the Windows DLL search directories and PXR_PLUGINPATH_NAME.
from test_rigexec_python import _setup_environment  # noqa: E402
_setup_environment()

from pxr import Plug, Usd  # noqa: E402

import rigexec  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_STAGE = os.path.join(_REPO, "examples", "biped", "Biped_anim.usda")
_RIG = "/Biped/Rig"
_POINTS = "/Biped/Geom/body_geo.points"
_FRAMES = (1.0, 2.0, 3.0, 7.0, 12.0)
_DRAG = ("hips_ctl", "avars:rz", 11.5)
# Under ctest's 120 s TIMEOUT, so a hang is reported by the dump below, which
# names the frame every thread is stuck in, rather than by ctest's kill.
_HANG_SECONDS = 90


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _RequireHook():
    if not hasattr(rigexec.Rig, "_evaluate_holding_gil"):
        raise AssertionError(
            "this build's _rigexec is older than this test: Rig is missing "
            "_evaluate_holding_gil. Rebuild and re-run; the module in use is "
            "%s." % getattr(rigexec, "__file__", "?"))


def _Snapshot(pose):
    joints = {}
    for path in pose.joint_paths():
        frame = pose.joint_frame(path, True)
        joints[path] = (tuple(frame.origin), tuple(frame.x_axis),
                        tuple(frame.y_axis), tuple(frame.z_axis))
    controls = {p: tuple(pose.control_frame(p).origin)
                for p in pose.control_paths()}
    return joints, controls, tuple(tuple(p) for p in
                                   pose.moved_property(_POINTS))


class _Contender(threading.Thread):
    """A Python thread that wants the GIL for as long as it runs."""

    def __init__(self):
        super().__init__(daemon=True)
        self.ticks = 0
        self._done = threading.Event()

    def run(self):
        while not self._done.is_set():
            self.ticks += 1

    def stop(self):
        self._done.set()
        self.join()


def _Run(mode, holding, contender):
    """Every frame of one cold rig in `mode`, then a drag and its release.

    Returns the snapshots, and for every generation after the first -- the
    ones that do not compile -- whether the contender ran while this thread
    was inside the call.
    """
    stage = Usd.Stage.Open(_STAGE)
    rig = rigexec.Rig(stage, _RIG)
    rig.evaluation_mode = mode
    control = next(p.GetPath().pathString for p in stage.Traverse()
                   if p.GetTypeName() == "RigExecControl"
                   and p.GetName() == _DRAG[0])
    evaluate = rig._evaluate_holding_gil if holding else rig.evaluate
    snaps = []
    ranInside = []

    def one(time):
        before = contender.ticks
        pose = evaluate(time)
        ranInside.append(contender.ticks > before)
        assert pose.valid, "%s %s t=%g: %s" % (
            mode, "held" if holding else "released", time,
            "; ".join(pose.diagnostics))
        if mode == "parity":
            assert not pose.baked_parity_mismatches, (
                "t=%g: %d baked parity mismatch(es)"
                % (time, pose.baked_parity_mismatches))
        snaps.append(_Snapshot(pose))

    for time in _FRAMES:
        one(time)
    rig.set_interactive_overrides([(control, _DRAG[1], _DRAG[2])])
    one(_FRAMES[1])
    rig.clear_interactive_overrides()
    one(_FRAMES[1])
    return snaps, ranInside[1:]


def main():
    _RegisterSchema()
    _RequireHook()
    if not os.path.exists(_STAGE):
        print("SKIP: %s not present" % _STAGE)
        return 0
    faulthandler.dump_traceback_later(_HANG_SECONDS, exit=True)
    contender = _Contender()
    contender.start()
    try:
        for mode in ("baked", "dynamic", "parity"):
            # Held first: in the first mode that is the process's first
            # compile, plugin loads included.
            held, ranInside = _Run(mode, True, contender)
            released, _ = _Run(mode, False, contender)
            assert held == released, (
                "%s: a caller holding the GIL got a different pose" % mode)
            # Evaluate released the GIL: the contender, which only runs
            # when it has it, ran while this thread was inside a held call.
            # The first generation is left out, because Compile gives the
            # GIL up on its own. Most rather than all of the rest, because
            # a released GIL is an opportunity and the scheduler may still
            # hand it back before the contender wakes. A GIL held through
            # the call gives it none: a baked steady frame, which reaches no
            # inner guard, then scores 0 of 6.
            ran = sum(ranInside)
            assert 2 * ran >= len(ranInside), (
                "%s: another Python thread ran inside only %d of %d held "
                "steady evaluate(s)" % (mode, ran, len(ranInside)))
            print("  ok: %s (%d generations held; another thread ran inside "
                  "%d of the %d that did not compile)"
                  % (mode, len(held), ran, len(ranInside)))
    finally:
        contender.stop()
        faulthandler.cancel_dump_traceback_later()
    print("RIGEXEC_GIL_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
