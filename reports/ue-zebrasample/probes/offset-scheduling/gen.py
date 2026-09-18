"""Generates the offset-scheduling probe stages.

Rig: FK chain Ctl -> J. Ctl avars:rz = 90, so J (rest translate (0,1,0))
sits at (0,1,0) with its local X axis along world +Y. J has an UNCLAIMED
child C (rest local translate (2,0,0)), so C sits at (0,3,0) before any
constraint. A second unclaimed joint K under C (local (1,0,0)) sits at
(0,4,0).

Movers are self-sourced RigExecParentConstraints on J.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

HEADER = '''#usda 1.0
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
            def RigExecControl "Ctl" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:rz = 90
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1) )
            }
%(extra_controls)s
        }

        def Scope "Solvers"
        {
            def RigExecFkChain "Chain"
            {
                rel rigExec:controls = </Asset/Rig/Controls/Ctl>
                rel rigExec:joints = </Asset/Rig/Joints/J>
            }
%(extra_solvers)s
        }

        def Scope "Joints"
        {
            def RigExecJoint "J"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1) )

                def RigExecJoint "C"
                {
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (2, 0, 0, 1) )

                    def RigExecJoint "K"
                    {
                        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (1, 0, 0, 1) )
                    }
                }
            }
%(extra_joints)s
        }

%(extra_scopes)s

        def Scope "Movers"
        {
%(movers)s
        }
    }
}
'''


def fmt_vec(v):
    return "(%s)" % ", ".join(repr(float(x)) for x in v)


def parent_offset(name, t=(0, 0, 0), r=(0, 0, 0), weight=None,
                  weight_conn=None, joint="/Asset/Rig/Joints/J",
                  sources=None):
    sources = sources or [joint]
    lines = [
        '            def RigExecParentConstraint "%s" (' % name,
        '                prepend apiSchemas = ["RigExecMoverAPI"]',
        '            )',
        '            {',
        '                rel rigExec:moves = <%s>' % joint,
        '                rel rigExec:sources = [%s]' % ", ".join(
            "<%s>" % s for s in sources),
        '                double3[] inputs:translationOffsets = [%s]' %
        ", ".join(fmt_vec(t) for _ in sources),
        '                double3[] inputs:rotationOffsets = [%s]' %
        ", ".join(fmt_vec(r) for _ in sources),
    ]
    if weight is not None:
        lines.append('                float inputs:defaultWeight = %r' %
                     float(weight))
    if weight_conn is not None:
        lines.append('                float inputs:defaultWeight.connect = <%s>'
                     % weight_conn)
    lines.append('            }')
    return "\n".join(lines)


def write(name, movers, reorder=None, extra_controls="", extra_solvers="",
          extra_joints="", extra_scopes=""):
    body = ""
    if reorder:
        body += '            reorder nameChildren = [%s]\n\n' % ", ".join(
            '"%s"' % n for n in reorder)
    body += "\n\n".join(movers)
    text = HEADER % dict(movers=body, extra_controls=extra_controls,
                         extra_solvers=extra_solvers,
                         extra_joints=extra_joints,
                         extra_scopes=extra_scopes)
    path = os.path.join(HERE, name)
    with open(path, "w", newline="\n") as f:
        f.write(text)
    print("wrote", path)


# A mover the base needs so the rig has something to compile besides the
# solver: a zero-weight self offset (pass-through).
def main():
    # ---- C1a: single offset, local frame, weight sweep ---------------------
    write("c1a_base.usda", [parent_offset("Off", t=(1, 0, 0), weight=0.0)])
    for tag, w in (("w0", 0.0), ("w05", 0.5), ("w1", 1.0), ("w15", 1.5),
                   ("wm05", -0.5)):
        # translation only
        write("c1a_t_%s.usda" % tag,
              [parent_offset("Off", t=(1, 0, 0), weight=w)])
        # translation + rotation, which is what distinguishes local vs world
        write("c1a_tr_%s.usda" % tag,
              [parent_offset("Off", t=(1, 0, 0), r=(0, 0, 30), weight=w)])

    # ---- C1a: stacking in mover order --------------------------------------
    # Sibling rows execute bottom-to-top. With nameChildren [B, A], A runs
    # first. A = translate (1,0,0), B = rotate Z 90.
    a = parent_offset("A", t=(1, 0, 0), weight=1.0)
    b = parent_offset("B", r=(0, 0, 90), weight=1.0)
    write("c1a_stack_AthenB.usda", [a, b], reorder=["B", "A"])
    write("c1a_stack_BthenA.usda", [a, b], reorder=["A", "B"])
    # the same, both at half weight
    a5 = parent_offset("A", t=(1, 0, 0), weight=0.5)
    b5 = parent_offset("B", r=(0, 0, 90), weight=0.5)
    write("c1a_stack_AthenB_half.usda", [a5, b5], reorder=["B", "A"])

    # ---- C1c: defaultWeight connected to a dial revised by FloatMathMovers --
    # Dial authored 0.1; chain: multiply by 4 (0.4), then add 0.35 (0.75).
    dial_scope = '''        def Scope "Channels"
        {
            def Scope "Dials"
            {
                custom float rigExec:offsetDial = 0.1
            }
        }
'''
    dial = "/Asset/Rig/Channels/Dials.rigExec:offsetDial"
    math_mul = '''            def RigExecFloatMathMover "DialMul" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "multiply"
                float inputs:value = 4
                rel rigExec:moves = <%s>
            }''' % dial
    math_add = '''            def RigExecFloatMathMover "DialAdd" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "add"
                float inputs:value = 0.35
                rel rigExec:moves = <%s>
            }''' % dial
    off = parent_offset("Off", t=(1, 0, 0), r=(0, 0, 30), weight=1.0,
                        weight_conn=dial)
    # Mover order bottom-to-top: DialMul, DialAdd, then Off (Off is top).
    write("c1c_dial_chain.usda", [off, math_add, math_mul],
          reorder=["Off", "DialAdd", "DialMul"], extra_scopes=dial_scope)
    # Reference: the dial with NO math movers (constraint should read 0.1)
    write("c1c_dial_nochain.usda", [off], extra_scopes=dial_scope)
    # Reference: plain authored weight 0.75
    write("c1c_ref_w075.usda",
          [parent_offset("Off", t=(1, 0, 0), r=(0, 0, 30), weight=0.75)])
    # Math movers placed ABOVE the constraint in the stack (run after it)
    write("c1c_dial_chain_after.usda", [math_add, math_mul, off],
          reorder=["DialAdd", "DialMul", "Off"], extra_scopes=dial_scope)

    # ---- C2: defaultWeight connected to a RigExecPose.outputs:weight --------
    # Driver = a second control (Drv) with avars:rz = 45: poses neutral(0),
    # Forward(+45), Back(-45), whole-rotation, radius 45deg. At rz=45 the
    # Forward weight is 1.0 (exactly on the pose). The pose attribute is
    # authored 0 (default) -- or 0.25 in the "authored" variant, so a silent
    # read of the authored value is distinguishable from a zero weight.
    import math
    def quat_z(deg):
        h = math.radians(deg) * 0.5
        return "(%r, 0, 0, %r)" % (math.cos(h), math.sin(h))
    spacing = 0.7853981633974483

    def interp_scope(authored_weight):
        poses = []
        for name, deg in (("neutral", 0.0), ("Forward", 45.0),
                          ("Back", -45.0)):
            aw = ""
            if authored_weight is not None and name == "Forward":
                aw = "\n                    float outputs:weight = %r" % \
                    authored_weight
            poses.append('''                def RigExecPose "%s"
                {
                    uniform token rigExec:poseType = "whole"
                    quatf rigExec:rotation = %s
                    float rigExec:rotationRadius = %r%s
                }''' % (name, quat_z(deg), spacing, aw))
        return '''        def Scope "PoseInterpolators"
        {
            def RigExecPoseInterpolator "Swing"
            {
                rel rigExec:driver = </Asset/Rig/Controls/Drv>
                uniform token rigExec:kernel = "gaussian"
                uniform token rigExec:twistAxis = "Z"

%s
            }
        }
''' % "\n\n".join(poses)

    drv = '''            def RigExecControl "Drv" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:rz = 45
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (5, 0, 0, 1) )
            }'''
    pose_w = "/Asset/Rig/PoseInterpolators/Swing/Forward.outputs:weight"
    off_pose = parent_offset("Off", t=(1, 0, 0), r=(0, 0, 30), weight=1.0,
                             weight_conn=pose_w)
    write("c2_pose_weight.usda", [off_pose], extra_controls=drv,
          extra_scopes=interp_scope(None))
    write("c2_pose_weight_authored025.usda", [off_pose], extra_controls=drv,
          extra_scopes=interp_scope(0.25))
    # Control: the interpolator alone, no consumer, to read the weights.
    write("c2_pose_only.usda",
          [parent_offset("Off", t=(1, 0, 0), weight=0.0)],
          extra_controls=drv, extra_scopes=interp_scope(None))

    # C2 solver variant: an FK chain's ... use a TwoBoneIk-free float input:
    # RigExecBlendPointFrames inputs:weight connected to the pose weight.
    # A second chain Drv2 joint pair: FK A (rz 0) and FK B (rz 90) on joint
    # Q, blended by inputs:weight -> pose weight.
    extra_solvers = '''            def RigExecFkChain "QA"
            {
                rel rigExec:controls = </Asset/Rig/Controls/QA>
                rel rigExec:joints = </Asset/Rig/Joints/Q>
            }

            def RigExecFkChain "QB"
            {
                rel rigExec:controls = </Asset/Rig/Controls/QB>
                rel rigExec:joints = </Asset/Rig/Joints/Q>
            }

            def RigExecBlendPointFrames "QBlend"
            {
                float inputs:weight = %(w)s
                float inputs:weight.connect = <%(conn)s>
                rel rigExec:inputA = </Asset/Rig/Solvers/QA>
                rel rigExec:inputB = </Asset/Rig/Solvers/QB>
                rel rigExec:joints = </Asset/Rig/Joints/Q>
            }'''
    extra_controls_q = drv + '''

            def RigExecControl "QA" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:tx = 0
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 10, 1) )
            }

            def RigExecControl "QB" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:tx = 4
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 10, 1) )
            }'''
    extra_joints_q = '''            def RigExecJoint "Q"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 10, 1) )
            }'''
    write("c2_solver_pose_weight.usda",
          [parent_offset("Off", t=(1, 0, 0), weight=0.0)],
          extra_controls=extra_controls_q,
          extra_solvers=extra_solvers % dict(w="0.25", conn=pose_w),
          extra_joints=extra_joints_q, extra_scopes=interp_scope(None))
    # Reference: solver weight connected to a float math dial = 1.0 instead
    write("c2_solver_ref_w1.usda",
          [parent_offset("Off", t=(1, 0, 0), weight=0.0)],
          extra_controls=extra_controls_q,
          extra_solvers=(extra_solvers % dict(w="1.0", conn="X")).replace(
              '                float inputs:weight.connect = <X>\n', ''),
          extra_joints=extra_joints_q, extra_scopes=interp_scope(None))


if __name__ == "__main__":
    main()
