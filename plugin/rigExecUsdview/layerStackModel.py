#
# RigExec usdview plugin: the Layer Stack panel's rules, with no Qt.
#
# The stage's sublayer tree, read from the root layer down, and the one
# thing the panel does to it: mute or unmute a layer. Muting is how a
# branch of a character (the face, the correctives, the picker) is
# switched off and on while the stage is open. It is session state, not
# an edit: no layer is written, the file on disk is untouched, and the
# stage recomposes without the muted layer and everything it sublayers.
#
# Tested headlessly by tests/python/test_layer_stack_model.py.
#
import os

from pxr import Sdf


class LayerNode(object):
    """One sublayer in the tree.

    `identifier` is what the stage mutes by. `muted` is the layer's own
    state; `hidden` is true when an ancestor is muted, so the layer is off
    whatever its own state says. The root layer cannot be muted.
    """

    def __init__(self, identifier, layer, muted, hidden, canMute):
        self.identifier = identifier
        self.layer = layer
        self.muted = muted
        self.hidden = hidden
        self.canMute = canMute
        self.children = []

    @property
    def displayName(self):
        return os.path.basename(self.identifier.replace("\\", "/"))

    @property
    def missing(self):
        return self.layer is None

    @property
    def active(self):
        return not (self.muted or self.hidden)

    def Walk(self):
        yield self
        for child in self.children:
            for node in child.Walk():
                yield node


def _SubLayerIdentifier(layer, subLayerPath):
    return Sdf.ComputeAssetPathRelativeToLayer(layer, subLayerPath)


def BuildTree(stage):
    """The root layer's sublayer tree, or None with no stage.

    A muted layer is not loaded by the stage, so its children are read by
    opening the layer on its own; that shows what unmuting would bring
    back without composing it.
    """
    if stage is None:
        return None
    root = stage.GetRootLayer()
    node = LayerNode(root.identifier, root, False, False, False)
    _AddChildren(stage, node, set([root.identifier]))
    return node


def _AddChildren(stage, node, seen):
    if node.layer is None:
        return
    for subLayerPath in node.layer.subLayerPaths:
        identifier = _SubLayerIdentifier(node.layer, subLayerPath)
        if identifier in seen:
            continue
        layer = Sdf.Layer.FindOrOpen(identifier)
        child = LayerNode(identifier, layer,
                          stage.IsLayerMuted(identifier),
                          not node.active, True)
        node.children.append(child)
        _AddChildren(stage, child, seen | set([identifier]))


def MutedSet(stage):
    """Which layers are muted right now, as a set of identifiers."""
    return set() if stage is None else set(stage.GetMutedLayers())


def ApplyMutedSet(stage, wanted):
    """Make the muted set exactly . True when anything moved.

    One MuteAndUnmuteLayers call, not a loop: each one resyncs the whole
    stage and the container deactivates the rig around the change, so two
    calls is two recompiles of the character.
    """
    if stage is None:
        return False
    current = MutedSet(stage)
    wanted = set(wanted)
    toMute = sorted(wanted - current)
    toUnmute = sorted(current - wanted)
    if not toMute and not toUnmute:
        return False
    stage.MuteAndUnmuteLayers(toMute, toUnmute)
    return True


class MuteEdit(object):
    """One mute or unmute, undoable.

    Shaped to sit on rigExecUndo's stack beside a gizmo drag, but it
    restores no Sdf spec: muting is stage state, not scene description,
    so there is nothing for a snapshot to copy and the before and after
    are just two sets of identifiers.

    It goes back through the SAME  callable the original
    change used, which is the whole reason this is not two lines. The
    container releases RigExec evaluation around a mute because OpenExec
    will not take a resync of the pseudo-root -- see
    rigExecUsdview.MuteLayers -- and an undo that muted directly would
    leave the exec network describing a stage that no longer exists.
    """

    def __init__(self, muteLayers, before, after, label):
        self._muteLayers = muteLayers
        self._before = set(before)
        self._after = set(after)
        self.label = label

    def Undo(self):
        self._muteLayers(lambda s: ApplyMutedSet(s, self._before))

    def Redo(self):
        self._muteLayers(lambda s: ApplyMutedSet(s, self._after))


def SetMuted(stage, identifier, muted):
    """Mute or unmute one layer. True when the state changed."""
    if stage is None or identifier == stage.GetRootLayer().identifier:
        return False
    if bool(stage.IsLayerMuted(identifier)) == bool(muted):
        return False
    if muted:
        stage.MuteLayer(identifier)
    else:
        stage.UnmuteLayer(identifier)
    return True


def UnmuteAll(stage):
    """Unmute every muted layer. The count unmuted."""
    if stage is None:
        return 0
    muted = list(stage.GetMutedLayers())
    if muted:
        stage.MuteAndUnmuteLayers([], muted)
    return len(muted)
