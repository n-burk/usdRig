#
# RigExec usdview gizmo: native box (marquee) selection of controls.
#
# Drag a rectangle anywhere in the viewport and every RigExec control
# whose EVALUATED origin falls inside it is selected. This is a property
# of the viewport, not of a panel: it works with the Select tool and with
# Move/Rotate/Scale, and it does not care whether TouchPose is loaded.
#
# Qt-free by design (the same banner rule gizmoMath and gizmoScreen carry):
# everything here is arithmetic over a stage, a Gf.Camera and a rectangle,
# so tests/python/test_gizmo_marquee.py can assert the exact control set a
# band catches with no display. gizmoUI owns the Qt half -- the press,
# the drag slop, the rubber band, and the write into dataModel.selection.
#
import sys
import os

from pxr import Tf, Usd

try:
    import gizmoMath
    import gizmoScreen
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoMath
    import gizmoScreen


# The three selection modes, spelled exactly as the Control Picker
# (pickerUI's "replace"/"toggle"/"remove") and TouchPose already spell
# them. A third vocabulary for the same three gestures would be the
# worst outcome here, so the strings are shared verbatim rather than
# re-derived.
MODE_REPLACE = "replace"
MODE_TOGGLE = "toggle"
MODE_REMOVE = "remove"

# LOGICAL pixels of travel before a press becomes a marquee rather than a
# click. The same 3.0 touchPoseUI uses, and for the same reason: without
# it every click is a one-pixel band and usdview's own single-prim pick
# never runs at all.
DRAG_SLOP = 3.0

# Only controls are box-selectable. RigExecJoint is deliberately NOT here:
# the biped carries 126 controls against several hundred bind joints, and
# a band drawn over the character would hand the animator a selection of
# joints that nothing poses. The picker draws the same line.
CONTROL_TYPE_NAME = "RigExecControl"


def Mode(ctrl=False, shift=False):
    """The selection mode a modifier state asks for.

    Ctrl is tested FIRST so Ctrl+Shift can only ever subtract: the promise
    is that there is one gesture that never adds, and a Shift-wins
    ordering would break it for the one combination an animator is most
    likely to hit by accident. Same order as PickerView.ModeFor and
    TouchPoseController.ModeFor -- if this disagrees with either, the
    three tools disagree about what Shift means.
    """
    if ctrl:
        return MODE_REMOVE
    if shift:
        return MODE_TOGGLE
    return MODE_REPLACE


def PastSlop(press, point, ratio=1.0):
    """True once a press has travelled far enough to be a band.

    `press` and `point` are PHYSICAL pixels and `ratio` the display's
    device pixel ratio, so the slop is the same distance on the glass on
    a HiDPI screen as on a 1.0 one -- a 3-pixel threshold measured in
    physical pixels would be 1.5 logical pixels on a retina display and
    the shakiest click would turn into a marquee.
    """
    slop = DRAG_SLOP * ratio
    return (abs(point[0] - press[0]) > slop
            or abs(point[1] - press[1]) > slop)


def IsControl(prim):
    """True for a RigExecControl, schema registered or not.

    The type-name fallback mirrors gizmoMath._IsRigTargetType: a session
    without the schema plugin still has the type name in the layer, and
    refusing to box-select there would make the feature look broken
    rather than make it safe.
    """
    if not prim or not prim.IsValid():
        return False
    schemaType = Tf.Type.FindByName(CONTROL_TYPE_NAME)
    if schemaType.isUnknown:
        return str(prim.GetTypeName()) == CONTROL_TYPE_NAME
    return prim.IsA(schemaType)


def ControlPrims(stage):
    """Every RigExecControl on the stage, in traversal order."""
    if stage is None:
        return []
    return [prim for prim in stage.Traverse() if IsControl(prim)]


def _PosedPaths(prim, solverPosed, byRoot):
    """The rig-overwritten paths for this prim's rig root, memoised."""
    root = gizmoMath.FindRigRoot(prim)
    key = root.GetPath() if root else None
    if key not in byRoot:
        if not root:
            byRoot[key] = set()
        elif solverPosed is not None:
            byRoot[key] = solverPosed.For(root)
        else:
            byRoot[key] = gizmoMath.SolverPosedPaths(root)
    return byRoot[key]


def Selectable(stage, solverPosed=None):
    """[(prim, the rig-overwritten path set for its root)], box-selectable.

    A control the rig poses OUTRIGHT is dropped: the evaluator injects its
    frame as a value override, so its avars are inert and the gizmo
    already refuses to build a target for it. Selecting one is a trap --
    the animator gets a manipulator that will not move -- and there are
    15 of them on examples/biped/Biped.usda (the spine and finger-follow
    RigExecParentConstraint targets plus the four *_params controls),
    measured against 126 controls in all.

    `solverPosed` is an optional gizmoMath.SolverPosedCache, shared with
    the controller so a marquee reuses the rig walk the target build
    already paid for.
    """
    byRoot = {}
    out = []
    for prim in ControlPrims(stage):
        posed = _PosedPaths(prim, solverPosed, byRoot)
        if prim.GetPath() in posed:
            continue
        out.append((prim, posed))
    return out


def SelectablePaths(stage, solverPosed=None):
    """Selectable(), as sorted path strings. For tests and status text."""
    return sorted(str(prim.GetPath())
                  for prim, _posed in Selectable(stage, solverPosed))


def ScreenPositions(stage, camera, viewport, time, solverPosed=None):
    """{control path: (x, y)} in PHYSICAL pixels, for every selectable control.

    The projected point is the EVALUATED origin -- `frames.posed` carried
    to world by `frames.assetToWorld` -- never the authored rest. The two
    differ by up to 42 cm on the biped once an arm is posed (see
    gizmoMath.SolverPosedPaths), and a band drawn around the hand that
    selected the rest-pose control would be indefensible.

    A control behind the eye projects to None and is simply absent, which
    is the same answer a band over it would give.
    """
    if stage is None or camera is None or viewport is None:
        return {}
    try:
        viewProj = gizmoScreen.ViewProjection(camera)
    except Exception:
        return {}
    positions = {}
    # ONE frame cache and ONE memo scope for the whole pass. Measured on
    # examples/biped/Biped.usda, the 111 selectable controls cost 0.844 s
    # with a memo per ComputeRigFrames call and 0.262 s inside one scope;
    # this runs on the mouse release that ends the drag, so the 0.58 s is
    # latency the animator feels.
    frameCache = {}
    with gizmoMath.MemoScope():
        for prim, posed in Selectable(stage, solverPosed):
            try:
                frames = gizmoMath.ComputeRigFrames(stage, prim, time,
                                                    posed, frameCache)
            except Exception:
                # A cyclic posed:space or a broken composition refuses one
                # control, not the whole marquee.
                continue
            world = (frames.posed * frames.assetToWorld).ExtractTranslation()
            try:
                screen = gizmoScreen.ProjectPoint(viewProj, viewport, world)
            except Exception:
                screen = None
            if screen is None:
                continue
            positions[str(prim.GetPath())] = (screen[0], screen[1])
    return positions


def PathsInBand(positions, x0, y0, x1, y1):
    """The paths in `positions` inside the band, sorted.

    The rectangle is normalised here rather than at the call site: a drag
    that ends up and to the left of where it started is the same band as
    one that ends down and to the right.
    """
    left, right = min(x0, x1), max(x0, x1)
    top, bottom = min(y0, y1), max(y0, y1)
    return sorted(path for path, (x, y) in positions.items()
                  if left <= x <= right and top <= y <= bottom)


def ControlsInBand(stage, camera, viewport, time, band, solverPosed=None):
    """The selectable controls a band catches, as sorted path strings."""
    positions = ScreenPositions(stage, camera, viewport, time, solverPosed)
    return PathsInBand(positions, band[0], band[1], band[2], band[3])


def Resolve(current, wanted, mode):
    """The selection `mode` produces, as an ordered list of path strings.

    Pure path algebra, identical in meaning to pickerUI's block and
    TouchPoseController.Pick: plain replaces, Shift toggles each member
    independently, Ctrl only ever removes. Order is preserved so the LAST
    entry -- usdview's focus prim, and the gizmo's group lead -- stays
    where the gesture put it.
    """
    have = set(current)
    if mode == MODE_REPLACE:
        return list(wanted)
    if mode == MODE_REMOVE:
        drop = set(wanted)
        return [path for path in current if path not in drop]
    keep = list(current)
    for path in wanted:
        if path in have:
            keep = [p for p in keep if p != path]
        else:
            keep.append(path)
    return keep
