"""Emit RigExecSplineIk probe stages: a 3-control baseline and several ways
of handing the solver a 4th control.

Chain J0..J6 along +X at x = i. Controls: Root (0,0,0), Mid (3,0,0),
End (6,0,0), Extra (4.5,0,0). At frame 1 Extra is pulled +2 in Y (Mid stays
put), so any influence of Extra on the chain is visible in joint Y.
At frame 2 Mid is pulled +1 in Y (Extra stays put).
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

VARIANTS = {
    # baseline: exactly the three schema slots
    "spline_3ctl": dict(root="<{R}>", mid="<{M}>", end="<{E}>", extra=""),
    # 4 controls: two targets on midControl, schema slot order [Mid, Extra]
    "spline_4ctl_mid_two_targets": dict(
        root="<{R}>", mid="[<{M}>, <{X}>]", end="<{E}>", extra=""),
    # same, reversed [Extra, Mid]
    "spline_4ctl_mid_two_targets_rev": dict(
        root="<{R}>", mid="[<{X}>, <{M}>]", end="<{E}>", extra=""),
    # 4 controls: two targets on endControl
    "spline_4ctl_end_two_targets": dict(
        root="<{R}>", mid="<{M}>", end="[<{E}>, <{X}>]", extra=""),
    # 4 controls: an extra non-schema relationship naming a 4th control
    "spline_4ctl_extra_rel": dict(
        root="<{R}>", mid="<{M}>", end="<{E}>",
        extra="                rel rigExec:mid2Control = <{X}>\n"),
    # 4 controls: an extra array-valued 'controls' relationship
    "spline_4ctl_controls_rel": dict(
        root="<{R}>", mid="<{M}>", end="<{E}>",
        extra="                rel rigExec:controls = [<{R}>, <{M}>, <{X}>, <{E}>]\n"),
}

PATHS = dict(R="/Asset/Rig/Controls/Root", M="/Asset/Rig/Controls/Mid",
             E="/Asset/Rig/Controls/End", X="/Asset/Rig/Controls/Extra")

HEAD = """#usda 1.0
(
    defaultPrim = "Asset"
    startTimeCode = 1
    endTimeCode = 2
    upAxis = "Y"
)

def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        uniform token rigExec:partition = "Asset"
        custom int rigExec:restFrameVersion = 2

        def Scope "Controls"
        {
"""

CONTROL = """            def RigExecControl "%s" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
%s                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (%g, 0, 0, 1) )
            }
"""

JOINT = """            def RigExecJoint "J%d"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (%d, 0, 0, 1) )
            }
"""


def ty(samples):
    body = ",\n".join("                    %d: %g" % kv for kv in samples)
    return ("                double avars:ty = 0\n"
            "                double avars:ty.timeSamples = {\n%s\n                }\n" % body)


def main():
    for name, v in VARIANTS.items():
        s = HEAD
        s += CONTROL % ("Root", "", 0)
        s += CONTROL % ("Mid", ty([(1, 0), (2, 1)]), 3)
        s += CONTROL % ("End", "", 6)
        s += CONTROL % ("Extra", ty([(1, 2), (2, 0)]), 4.5)
        s += "        }\n\n        def Scope \"Solvers\"\n        {\n"
        s += "            def RigExecSplineIk \"Spine\"\n            {\n"
        s += "                rel rigExec:rootControl = %s\n" % v["root"].format(**PATHS)
        s += "                rel rigExec:midControl = %s\n" % v["mid"].format(**PATHS)
        s += "                rel rigExec:endControl = %s\n" % v["end"].format(**PATHS)
        s += v["extra"].format(**PATHS)
        s += "                uniform token rigExec:restLength = \"curve\"\n"
        s += "                rel rigExec:joints = [\n"
        s += "".join("                    </Asset/Rig/Joints/J%d>,\n" % i for i in range(7))
        s += "                ]\n            }\n        }\n\n"
        s += "        def Scope \"Joints\"\n        {\n"
        s += "".join(JOINT % (i, i) for i in range(7))
        s += "        }\n    }\n}\n"
        with open(os.path.join(HERE, name + ".usda"), "w") as f:
            f.write(s)
        print(name)


if __name__ == "__main__":
    main()
