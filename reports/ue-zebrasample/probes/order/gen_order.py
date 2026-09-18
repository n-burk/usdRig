"""Mover order vs solver order.

Writes three stages that share one two-bone leg (Hip at (0,8,0), bones of
length 4, rest pose straight down to the ankle at the origin):

  order_read.usda        ReadAnkle: Tracker follows the IK ankle. The IK
                         effector is FootIK at (0,1,3); the solved ankle
                         lands there, the rest ankle is at (0,0,0).
  order_read_first.usda  ReadAnkle fires FIRST (bottom sibling), then
                         MoveEffector moves FootIK onto Driver (0,2,2).
                         Under spec 4.2 ordering ReadAnkle reads the ankle
                         before the IK has run for this generation.
  order_move_first.usda  The same two movers, MoveEffector first.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

HEAD = """#usda 1.0
(
    defaultPrim = "Asset"
    metersPerUnit = 0.01
    upAxis = "Y"
)

def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        uniform token rigExec:partition = "Asset"

        def Scope "Controls"
        {
            def RigExecControl "HipRoot" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                matrix4d rest:space = ( (0, -1, 0, 0), (0, 0, -1, 0), (1, 0, 0, 0), (0, 8, 0, 1) )
            }
            def RigExecControl "FootIK" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:ty = 1
                double avars:tz = 3
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            }
            def RigExecControl "KneePole" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 4, 3, 1) )
            }
            def RigExecControl "Driver" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 2, 2, 1) )
            }
            def RigExecControl "Tracker" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (10, 0, 0, 1) )
            }
        }

        def Scope "Solvers"
        {
            def RigExecTwoBoneIk "LegIK"
            {
                float inputs:softness = 0
                float inputs:stretch = 0
                rel rigExec:effectorControl = </Asset/Rig/Controls/FootIK>
                rel rigExec:joints = [
                    </Asset/Rig/Joints/Hip>,
                    </Asset/Rig/Joints/Hip/Knee>,
                    </Asset/Rig/Joints/Hip/Knee/Ankle>,
                ]
                rel rigExec:poleControl = </Asset/Rig/Controls/KneePole>
                rel rigExec:rootControl = </Asset/Rig/Controls/HipRoot>
            }
        }

        def Scope "Joints"
        {
            def RigExecJoint "Hip"
            {
                matrix4d rest:space = ( (0, -1, 0, 0), (0, 0, -1, 0), (1, 0, 0, 0), (0, 8, 0, 1) )
                def RigExecJoint "Knee"
                {
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (4, 0, 0, 1) )
                    def RigExecJoint "Ankle"
                    {
                        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (4, 0, 0, 1) )
                    }
                }
            }
        }

        def Scope "Movers"
        {
"""

READ = """            def RigExecPositionConstraint "ReadAnkle" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Rig/Controls/Tracker>
                rel rigExec:sources = </Asset/Rig/Joints/Hip/Knee/Ankle>
            }
"""

MOVE = """            def RigExecPositionConstraint "MoveEffector" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Rig/Controls/FootIK>
                rel rigExec:sources = </Asset/Rig/Controls/Driver>
            }
"""

TAIL = """        }
    }
}
"""

# Siblings execute bottom first, so the LAST child written fires FIRST.
STAGES = {
    "order_read": [READ],
    "order_read_first": [MOVE, READ],
    "order_move_first": [READ, MOVE],
}

for name, movers in STAGES.items():
    with open(os.path.join(HERE, name + ".usda"), "w", newline="\n") as f:
        f.write(HEAD + "".join(movers) + TAIL)
    print("wrote", name)
