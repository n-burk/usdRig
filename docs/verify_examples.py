#!/usr/bin/env python
"""Verify every docs/examples/*.usda stage compiles and evaluates.

Usage: python docs/verify_examples.py [example.usda ...]

Opens each stage, compiles its RigExecRoot(s), and evaluates at the
stage's start, middle, and end frames. Reports failures with the
evaluator diagnostics; exits nonzero if any example is invalid.
"""
import glob
import os
import sys

RIG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.environ.get("USD", os.path.join(os.path.dirname(RIG), "usd-install"))

sys.path.insert(0, os.path.join(RIG, "build", "python"))
# Same order bin/_env.sh probes: the POSIX lib/python<X.Y>/site-packages
# layout is globbed rather than pinned, then the Windows Lib/site-packages,
# then the old lib/python. Pinning a minor version breaks on a USD rebuild.
for candidate in (sorted(glob.glob(os.path.join(USD, "lib", "python*",
                                                "site-packages")))
                  + [os.path.join(USD, "Lib", "site-packages"),
                     os.path.join(USD, "lib", "python")]):
    if os.path.isdir(candidate):
        if candidate not in sys.path:
            sys.path.insert(0, candidate)
        break

import rigexec  # noqa: E402
from pxr import Usd  # noqa: E402

rigexec.load_schema_plugin(os.path.join(RIG, "build", "usd", "rigExecSchema", "resources"))


def _diagnostics(pose):
    """The evaluator's messages for an invalid pose, ready to append."""
    messages = [str(m) for m in (getattr(pose, "diagnostics", None) or [])]
    if not messages:
        return " (no evaluator diagnostics)"
    return ": " + "; ".join(messages)


def verify(path):
    stage = Usd.Stage.Open(path)
    roots = [str(p.GetPath()) for p in stage.Traverse()
             if p.GetTypeName() == "RigExecRoot"]
    if not roots:
        return "no RigExecRoot found"
    start = stage.GetStartTimeCode()
    end = stage.GetEndTimeCode()
    times = sorted({start, (start + end) / 2.0, end})
    for root in roots:
        rig = rigexec.Rig(stage, root)
        # compile() has no diagnostics accessor of its own: it raises
        # ValueError carrying every message, so name the phase and pass the
        # messages through instead of letting a bare traceback stand in.
        try:
            rig.compile()
        except Exception as failure:  # noqa: BLE001 - report, don't crash
            return "%s failed to compile: %s" % (root, failure)
        for time in times:
            pose = rig.evaluate(time)
            if not pose.valid:
                return "%s invalid at t=%s%s" % (root, time,
                                                 _diagnostics(pose))
    return None


def main(argv):
    if len(argv) > 1:
        files = [a if os.path.isabs(a) else os.path.join(RIG, a) for a in argv[1:]]
    else:
        files = sorted(glob.glob(os.path.join(RIG, "docs", "examples", "*.usda")))
    if not files:
        print("no example stages found")
        return 1
    failures = 0
    for path in files:
        try:
            error = verify(path)
        except Exception as failure:  # noqa: BLE001 - report, don't crash
            error = "%s: %s" % (type(failure).__name__, failure)
        name = os.path.basename(path)
        if error:
            failures += 1
            print("FAIL %s: %s" % (name, error))
        else:
            print("ok   %s" % name)
    print("%d/%d examples verified" % (len(files) - failures, len(files)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
