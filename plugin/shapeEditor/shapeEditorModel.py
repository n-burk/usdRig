#
# What the Shape Editor knows, with no Qt in it.
#
# Split from the UI for the reason every other panel here is: the model is
# unit-testable headlessly, and a Qt-free module can be exercised by a
# behavioural test without a display. `pickerModel.py` and
# `touchPoseModel.py` are the same shape.
#
# WHAT THIS PANEL IS FOR. Pose-space deformation is invisible until it goes
# wrong. A corrective either fires or it does not, and when it does not the
# question is always the same: is the driver not moving, is the
# interpolator not resolving, or is the shape not bound? Reading that off a
# prim tree means navigating to
# `/Biped/Rig/PoseInterpolators/<driver>/<pose>.outputs:weight` for each of
# 121 poses in turn. This shows all of them at once, live.
#
import re

from pxr import Sdf, Usd


INTERPOLATOR = "RigExecPoseInterpolator"
POSE = "RigExecPose"
BLEND_INPUT = "RigExecBlendInput"

WEIGHT = "outputs:weight"
ENABLED = "inputs:enabled"

# A weight below this reads as "not firing" in the UI. Not a tolerance on
# correctness: the engine publishes exact zeros for an inactive pose, and
# the neutral of an undriven interpolator sits at exactly 1. This is only
# the threshold at which a bar is worth drawing.
LIVE = 1e-4


class Pose(object):
    """One authored pose of an interpolator, and its live weight."""

    def __init__(self, prim):
        self.prim = prim
        self.path = prim.GetPath()
        self.name = prim.GetName()
        self.weight = 0.0
        # The corrective this pose drives, resolved once: the connection
        # runs pose.outputs:weight -> blendInput.inputs:weight, so it is
        # found by asking who points AT us rather than by name matching.
        self.target = None

    @property
    def is_neutral(self):
        """The rest pose, which weighs 1 when nothing else does.

        Named rather than inferred from the weight: at rest EVERY
        interpolator's neutral reads 1.000000, so a "weight == 1 means
        neutral" test would be true for the wrong reason.
        """
        return self.name == "neutral"

    @property
    def enabled(self):
        attr = self.prim.GetAttribute(ENABLED)
        value = attr.Get() if attr and attr.IsValid() else None
        return True if value is None else bool(value)


class Interpolator(object):
    """One driver joint's pose set."""

    def __init__(self, prim):
        self.prim = prim
        self.path = prim.GetPath()
        self.name = prim.GetName()
        self.poses = []
        self.driver = None
        self.kernel = None

    @property
    def enabled(self):
        attr = self.prim.GetAttribute(ENABLED)
        value = attr.Get() if attr and attr.IsValid() else None
        return True if value is None else bool(value)

    @property
    def firing(self):
        """Poses other than the neutral carrying a weight worth drawing.

        The neutral is excluded on purpose. It is 1 at rest on every
        interpolator, so counting it would report all 29 as "firing" on an
        untouched rig, which is the opposite of useful.
        """
        return [p for p in self.poses
                if not p.is_neutral and abs(p.weight) > LIVE]


def Discover(stage, rigPath="/Biped/Rig"):
    """Every interpolator on the stage, with its poses and their targets.

    Pure stage reads: no evaluation, no Qt, no side effects. Returns [] on
    a stage with no PSD rather than raising, because the panel is expected
    to open on rigs that have none.
    """
    if stage is None:
        return []
    root = stage.GetPrimAtPath(Sdf.Path(rigPath))
    if not root or not root.IsValid():
        root = stage.GetPseudoRoot()

    # pose path -> the blend channel it drives. Built by walking the
    # channels and reading their connections, because the arrow points
    # that way: a BlendInput names its pose, a pose names nothing.
    driven = {}
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != BLEND_INPUT:
            continue
        attr = prim.GetAttribute("inputs:weight")
        if not attr or not attr.IsValid():
            continue
        for source in attr.GetConnections():
            driven[source.GetPrimPath()] = prim.GetName()

    found = []
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != INTERPOLATOR:
            continue
        interp = Interpolator(prim)
        rel = prim.GetRelationship("rigExec:driver")
        targets = rel.GetTargets() if rel else []
        interp.driver = targets[0] if targets else None
        kernel = prim.GetAttribute("rigExec:kernel")
        interp.kernel = kernel.Get() if kernel and kernel.IsValid() else None
        for child in prim.GetChildren():
            if child.GetTypeName() != POSE:
                continue
            pose = Pose(child)
            pose.target = driven.get(child.GetPath())
            interp.poses.append(pose)
        interp.poses.sort(key=lambda p: (not p.is_neutral, p.name))
        found.append(interp)
    found.sort(key=lambda i: i.name)
    return found


def ReadWeights(interpolators, pose):
    """Copy the evaluated weights off a RigExecRigPose onto the model.

    `pose.moved_property` is the published value -- the number the engine
    actually handed the blend channels this generation -- not a re-read of
    the stage, which would show the authored default and always be 0.
    Returns how many poses were found, so a caller can tell "everything is
    zero" from "nothing was published".
    """
    if pose is None:
        return 0
    published = {}
    for path in pose.moved_properties():
        text = str(path)
        if text.endswith("." + WEIGHT):
            published[Sdf.Path(text).GetPrimPath()] = path

    seen = 0
    for interp in interpolators:
        for entry in interp.poses:
            path = published.get(entry.path)
            if path is None:
                entry.weight = 0.0
                continue
            value = pose.moved_property(path)
            entry.weight = float(value if value is not None else 0.0)
            seen += 1
    return seen


def SetEnabled(prim, value):
    """Author `inputs:enabled`, creating it if the schema left it absent.

    Returns the attribute so a caller can record it for undo.
    """
    attr = prim.GetAttribute(ENABLED)
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute(ENABLED, Sdf.ValueTypeNames.Bool)
    attr.Set(bool(value))
    return attr


def Summarise(interpolators):
    """One line for the panel's status bar."""
    poses = sum(len(i.poses) for i in interpolators)
    firing = sum(len(i.firing) for i in interpolators)
    driven = sum(1 for i in interpolators for p in i.poses if p.target)
    off = sum(1 for i in interpolators if not i.enabled)
    text = ("%d interpolators, %d poses, %d driving a corrective; "
            "%d firing" % (len(interpolators), poses, driven, firing))
    return text + (", %d disabled" % off if off else "")
