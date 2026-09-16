"""Evaluates probe stages in dynamic / baked / parity modes and prints the
full frames of J, C, K (origin + X-axis heading in degrees), moved
properties, diagnostics, and baked accounting.

usage: python run.py stage.usda [stage.usda ...]
"""
import math
import os
import sys

import rigexec  # noqa: F401  (registers dll dirs)
rigexec.load_schema_plugin()
from pxr import Usd  # noqa: E402
_rigexec = rigexec  # Rig lives on the package

JOINTS = ["/Asset/Rig/Joints/J", "/Asset/Rig/Joints/J/C",
          "/Asset/Rig/Joints/J/C/K", "/Asset/Rig/Joints/Q"]


def heading(frame):
    # x_axis is the X LANDMARK POINT (points[1]), not a direction.
    x = frame.x_axis
    o = frame.origin
    return math.degrees(math.atan2(x[1] - o[1], x[0] - o[0]))


def run(path, mode):
    stage = Usd.Stage.Open(path)
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    rig.evaluation_mode = mode
    rig.solver_guides_enabled = False
    try:
        rig.compile()
    except Exception as exc:  # compile failure
        print("  [%s] COMPILE FAILED: %s" % (mode, exc))
        return
    reasons = rig.bakeability_reasons() if mode != "dynamic" else []
    pose = rig.evaluate(-1.0)
    print("  [%s] valid=%s bakeable=%s baked_parity_mismatches=%s" % (
        mode, pose.valid, rig.is_bakeable() if mode != "dynamic" else "-",
        pose.baked_parity_mismatches))
    for r in reasons:
        print("      bake refusal: %s" % r)
    paths = set(pose.joint_paths())
    for j in JOINTS:
        if j not in paths:
            continue
        f = pose.joint_frame(j)
        o = f.origin
        print("      %-26s origin=(% .4f % .4f % .4f) xHeading=% 8.3f deg" % (
            j.replace("/Asset/Rig/Joints/", "J:"), o[0], o[1], o[2],
            heading(f)))
    for k, v in sorted(pose.moved_properties().items()):
        print("      moved %s = %r" % (k, v))
    for d in pose.diagnostics:
        if d.startswith("mover graph:"):
            continue
        print("      diagnostic: %s" % d)


def main():
    modes = os.environ.get("MODES", "dynamic,baked,parity").split(",")
    for path in sys.argv[1:]:
        print("== %s" % os.path.basename(path))
        for mode in modes:
            run(path, mode)


if __name__ == "__main__":
    main()
