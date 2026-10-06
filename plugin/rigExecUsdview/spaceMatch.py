"""
Space matching: change a control's space without moving it.

A RigExecSpaceSwitch re-expresses its control's avars in the newly selected
space, so switching mid-pose moves the control -- the schema leaves keeping
it still to tools. This is that tool, as a picker's space button does it:
note where the control is, change the space, and solve the control's
translate and rotate channels so its posed frame lands back on the note.

The solve uses the frame the evaluator reports for the control in the new
space (posed = avars * P, so P = avars^-1 * posed), which makes rotation-only
spaces, rotation orders and `avars:rotationSign` need no special cases. Two
passes, because a control can carry something its own frame depends on.
Scale is never written, and neither is a channel the rig itself writes or
drives (ikfkMatch._Writable): a rotation-only space keeps its position, so
it needs only the rotation, and a breathing-driven translate stays the
rig's.

Conventions: USD row vectors (a child's world frame is local * parent);
frames are asset space, as rigexec.Pose reports them.
"""

from pxr import Sdf

import gizmoMath
import ikfkMatch

# How close a pass must land, in rig units and degrees-ish matrix entries,
# before another pass is not worth evaluating.
_SETTLED = 1e-6
_PASSES = 3


def SwitchTarget(stage, attrPath):
    """The control whose space `attrPath` selects -- the target of the
    RigExecSpaceSwitch reading it -- or None when no switch reads it."""
    attrPath = Sdf.Path(str(attrPath))
    for prim in stage.Traverse():
        if prim.GetTypeName() != "RigExecSpaceSwitch":
            continue
        rel = prim.GetRelationship("rigExec:activeSpaceAttribute")
        if rel and attrPath in rel.GetTargets():
            targets = prim.GetRelationship("rigExec:target").GetTargets()
            return targets[0] if targets else None
    return None


def _Error(a, b):
    return max(abs(a[i][j] - b[i][j]) for i in range(4) for j in range(4))


def Plan(stage, control, spaceAttr, value, time, evaluate, written=None,
         current=None):
    """{attribute path: value} that sets `spaceAttr` to `value` and keeps
    `control`'s posed frame where it is at `time`. The space value itself
    is in the result, so the caller writes everything as one edit.
    `current` ({attribute path: value}) is a pose not authored on the
    stage, which `evaluate` already applies -- the channels start there."""
    control = Sdf.Path(str(control))
    spaceAttr = Sdf.Path(str(spaceAttr))
    if written is None:
        written = ikfkMatch.RigWritten(stage)
    target = ikfkMatch.ControlFrame(evaluate({}, time), control)
    channels = ikfkMatch._Channels(stage, control, time, current)
    names = ikfkMatch._Writable(
        stage, control, ikfkMatch._TRANSLATE + ikfkMatch._ROTATE, written)
    values = {}
    plan = {spaceAttr: float(value)}
    for _ in range(_PASSES):
        overrides = dict(plan)
        posed = ikfkMatch.ControlFrame(evaluate(overrides, time), control)
        if _Error(posed, target) < _SETTLED:
            break
        parent = channels.Matrix(values).GetInverse() * posed
        solved = channels.Solve(target, parent)
        values.update({n: solved[n] for n in names})
        for name in names:
            plan[control.AppendProperty("avars:" + name)] = values[name]
    return plan


def Apply(stage, plan, time, mode, undoStack=None, label="Switch space"):
    """Author `plan` as one edit and one undo entry, landing the way a drag
    does (`mode` is a gizmoMath write mode: keyed in animation mode)."""
    scope = None
    if undoStack is not None:
        import rigExecUndo
        scope = rigExecUndo.SpecScope(stage, list(plan), undoStack, label)
        scope.__enter__()
    try:
        writer = gizmoMath.Writer(stage, time, mode)
        for path, value in plan.items():
            attr = stage.GetAttributeAtPath(path)
            if attr:
                writer.Set(attr, value)
        writer.CommitToStage()
    except Exception as error:
        if scope is not None:
            scope.__exit__(type(error), error, None)
        raise
    if scope is not None:
        scope.__exit__(None, None, None)
