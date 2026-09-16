"""Emit RigExecTwoBoneIk probe stages (plain text, no pxr needed).

Chain: Hip at origin, Knee at +1 X, Ankle at +2 X (chain length 2).
Effector control FootIK sits at (ratio * 2, 0, 0) via avars:tx time samples,
one ratio per frame (frame k = RATIOS[k-1]). Pole above the knee.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CHAIN = 2.0
RATIOS = [0.90, 0.95, 0.99, 0.999, 1.0, 1.001, 1.01, 1.05, 1.2, 3.0, 5.0]

VARIANTS = {
    "tb_soft0.1_stretch1": (0.1, 1.0),
    "tb_soft0.1_stretch0.5": (0.1, 0.5),
    "tb_soft0.1_stretch0": (0.1, 0.0),
    "tb_soft0_stretch1": (0.0, 1.0),
}

TEMPLATE = """#usda 1.0
(
    defaultPrim = "Asset"
    startTimeCode = 1
    endTimeCode = {end}
    upAxis = "Y"
)

def Xform "Asset"
{{
    def RigExecRoot "Rig"
    {{
        uniform token rigExec:partition = "Asset"
        custom int rigExec:restFrameVersion = 2

        def Scope "Controls"
        {{
            def RigExecControl "HipRoot" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {{
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }}

            def RigExecControl "FootIK" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {{
                double avars:tx = 0
                double avars:tx.timeSamples = {{
{samples}
                }}
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }}

            def RigExecControl "KneePole" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {{
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (1, 1, 0, 1) )
            }}
        }}

        def Scope "Solvers"
        {{
            def RigExecTwoBoneIk "LegIK"
            {{
                float inputs:softness = {softness}
                float inputs:stretch = {stretch}
                rel rigExec:effectorControl = </Asset/Rig/Controls/FootIK>
                rel rigExec:joints = [
                    </Asset/Rig/Joints/Hip>,
                    </Asset/Rig/Joints/Hip/Knee>,
                    </Asset/Rig/Joints/Hip/Knee/Ankle>,
                ]
                rel rigExec:poleControl = </Asset/Rig/Controls/KneePole>
                rel rigExec:rootControl = </Asset/Rig/Controls/HipRoot>
                uniform token rigExec:stretchPolicy = "uniformSegments"
                uniform token rigExec:unreachablePolicy = "clampWithSoftness"
            }}
        }}

        def Scope "Joints"
        {{
            def RigExecJoint "Hip"
            {{
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )

                def RigExecJoint "Knee"
                {{
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (1, 0, 0, 1) )

                    def RigExecJoint "Ankle"
                    {{
                        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (1, 0, 0, 1) )
                    }}
                }}
            }}
        }}
    }}
}}
"""


def main():
    for name, (softness, stretch) in VARIANTS.items():
        samples = ",\n".join(
            "                    %d: %.17g" % (k + 1, r * CHAIN)
            for k, r in enumerate(RATIOS))
        text = TEMPLATE.format(end=len(RATIOS), samples=samples,
                               softness=softness, stretch=stretch)
        with open(os.path.join(HERE, name + ".usda"), "w") as f:
            f.write(text)
        print(name)


if __name__ == "__main__":
    main()
