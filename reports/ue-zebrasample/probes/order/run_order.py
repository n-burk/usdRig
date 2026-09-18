"""Compile and evaluate the order_*.usda stages and print where the
Tracker (reader of the ankle), the FootIK effector and the solved ankle end
up, or the compile error. Run under bin/_env.sh after gen_order.py."""
import os
import sys

import rigexec  # noqa: F401  (registers dll dirs)
rigexec.load_schema_plugin()
from pxr import Usd  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
STAGES = ["order_read", "order_read_first", "order_move_first"]


def origin(frame):
    o = frame.origin
    return "(%.4f, %.4f, %.4f)" % (o[0], o[1], o[2])


for name in STAGES:
    stage = Usd.Stage.Open(os.path.join(HERE, name + ".usda"))
    for mode in ("dynamic", "baked"):
        rig = rigexec.Rig(stage, "/Asset/Rig")
        rig.evaluation_mode = mode
        try:
            rig.compile()
        except Exception as exc:
            print("%-18s %-7s COMPILE FAILED: %s" % (name, mode, exc))
            continue
        pose = rig.evaluate(-1.0)
        print("%-18s %-7s valid=%s  Tracker=%s  FootIK=%s  Ankle=%s" % (
            name, mode, pose.valid,
            origin(pose.control_frame("/Asset/Rig/Controls/Tracker")),
            origin(pose.control_frame("/Asset/Rig/Controls/FootIK")),
            origin(pose.joint_frame("/Asset/Rig/Joints/Hip/Knee/Ankle"))))
        for d in pose.diagnostics:
            if not d.startswith("mover graph:"):
                print("    diagnostic:", d)
sys.exit(0)
