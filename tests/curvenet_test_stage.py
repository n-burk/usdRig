#
# What the curvenet viewport tests need from whatever stage they are given.
#
# These four tests (draw, move, puppet, visible) were written against one
# particular character and named its prims: a body mesh seven levels down
# and a model root beside it. That character is not in this repository, so
# the tests could not run in a fresh checkout at all -- usdview failed to
# open the stage, and the runner reported it as a test failure rather than
# as a missing asset.
#
# Nothing about what they assert needs that character. They need a surface
# to draw a curvenet on and the model root to frame, so they ask the stage
# for them here. The one that counts a net's rings asks the net.
#

from pxr import Usd, UsdGeom


def Model(stage):
    """The mesh a curvenet test should draw on, and its model root.

    The biggest default-purpose PointBased prim on the stage: a curvenet
    is drawn on the model, and the model is the thing that fills the frame.
    Guides and proxies are skipped -- a rig's decorations are not surfaces
    to draw on -- and so is anything under a RigExecRoot, which is the rig
    rather than the character.

    Returns (mesh prim, model root prim). Raises with what it did see, so
    a stage that cannot serve the test says so instead of failing an
    assertion about something else.
    """
    best, bestCount = None, 0
    seen = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.PointBased):
            continue
        if _UnderRig(prim):
            continue
        imageable = UsdGeom.Imageable(prim)
        if imageable and imageable.ComputePurpose() != UsdGeom.Tokens.default_:
            continue
        points = prim.GetAttribute("points")
        count = len(points.Get() or []) if points else 0
        seen.append("%s (%d points)" % (prim.GetPath(), count))
        if count > bestCount:
            best, bestCount = prim, count
    if best is None:
        raise AssertionError(
            "no default-purpose PointBased prim to draw on; saw: %s"
            % (", ".join(seen) or "nothing"))
    return best, _ModelRoot(best)


def _UnderRig(prim):
    for ancestor in prim.GetPath().GetAncestorsRange():
        at = prim.GetStage().GetPrimAtPath(ancestor)
        if at and at.GetTypeName() == "RigExecRoot":
            return True
    return False


def _ModelRoot(prim):
    """The outermost ancestor under the pseudo-root."""
    path = prim.GetPath()
    while path.GetParentPath() != path.GetParentPath().GetParentPath():
        path = path.GetParentPath()
    root = prim.GetStage().GetPrimAtPath(path)
    return root if root else prim
