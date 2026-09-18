#!/usr/bin/env python
"""Report a rig's evaluation schedule, its parallelism, and its cost.

The command-line face of `plugin/rigExecUsdview/profilerModel.py`, which
is also what the usdview panel (RigExec -> Profiler) runs -- one
measurement, so the two cannot drift into different answers. See that
module's docstring for what each number means and how to read it.

Usage:
    rigexec_schedule.py <stage> [--rig-root /Biped/Rig] [--samples 40]
                        [--control hips_ctl] [--avar avars:ry]
                        [--json report.json] [--trace trace.json]
                        [--top 12]

`--json` writes the whole report as data, so two runs can be diffed.
`--trace` writes Chrome Trace Event JSON; open it at ui.perfetto.dev or
chrome://tracing to see the spans laid out per thread.

Run it through the plugin environment so `rigexec` imports:

    bin/run_probe.sh tools/rigexec_schedule.py examples/biped/Biped_stack.usda
"""
import argparse
import json
import os
import sys

from pxr import Usd

# profilerModel lives with the usdview plugin because the panel is its
# other caller; it imports no Qt, so this works headless.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "plugin", "rigExecUsdview"))
import profilerModel  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage")
    ap.add_argument("--rig-root", default=None,
                    help="default: the first RigExecRoot on the stage")
    ap.add_argument("--samples", type=int, default=40)
    ap.add_argument("--control", default=None,
                    help="control to drive; default: hips_ctl if present")
    ap.add_argument("--avar", default="avars:ry")
    ap.add_argument("--json", dest="json_path", default=None)
    ap.add_argument("--trace", dest="trace_path", default=None)
    ap.add_argument("--top", type=int, default=12)
    args = ap.parse_args(argv)

    stage = Usd.Stage.Open(args.stage)
    if stage is None:
        raise SystemExit("cannot open %s" % args.stage)

    rig_root = args.rig_root
    if rig_root is None:
        roots = profilerModel.rig_roots(stage)
        if not roots:
            raise SystemExit("no RigExecRoot on %s" % args.stage)
        rig_root = roots[0]

    try:
        report, rig = profilerModel.build_report(
            stage, rig_root, args.samples, args.control, args.avar)
    except ValueError as error:
        raise SystemExit(str(error))

    print(profilerModel.render(report, args.top))
    print("  compile          : %.0f ms, %d threads"
          % (report["compile"]["ms"],
             report["compile"]["threads"]["distinct_threads"]))
    print("  driven           : %s.%s"
          % (report["driven"]["control"], report["driven"]["avar"]))
    print("")

    if args.json_path:
        with open(args.json_path, "w") as fp:
            json.dump(report, fp, indent=2, sort_keys=True)
        print("  wrote %s" % args.json_path)
    if args.trace_path:
        profilerModel.write_trace(rig, report["driven"]["control"],
                                  report["driven"]["avar"], args.trace_path)
        print("  wrote %s  (open at ui.perfetto.dev)" % args.trace_path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
