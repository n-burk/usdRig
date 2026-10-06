#
# RigExec usdview undo: spec snapshots and a bounded undo stack.
#
# Deliberately Qt-free so the gizmo's undo semantics can be tested
# headlessly (the same rule volumeWeightUI.py applies above its Qt
# banner). The stack records what an edit did to the EDIT TARGET LAYER,
# not what it did to the composed stage: usdview starts with the session
# layer as its edit target (volumeWeightUI.py:1676-1690), so the value an
# artist sees can be a session override of a file value, and "undo" must
# put the session spec back exactly -- including removing it when it did
# not exist -- rather than authoring the file value into the session.
#
# A spline knot outranks a default in value resolution (the trap
# volumeWeightUI.SetVisibleAtTime documents), so a snapshot captures all
# three arms of an attribute spec: default, spline, and time samples.
#
# TWO SNAPSHOT KINDS, because the two jobs have different costs.
# AttributeSnapshot reads the three arms of one attribute spec into plain
# Python and is what a DRAG uses: a gizmo or a channel scrub captures on
# press and compares on release, sixty times a second in between, and it
# must not allocate a layer to do it. SpecSnapshot copies a whole spec
# aside with Sdf.CopySpec and is what a STRUCTURAL edit uses -- a prim
# created or deleted, a relationship retargeted, metadata changed -- none
# of which an attribute snapshot can see at all. Structural edits happen
# on a button press, so a scratch layer each is free.
#
from pxr import Sdf, Ts


class AttributeSnapshot(object):
    """
    The complete authored state of one attribute spec in one layer.

    Captured before and after a drag; Restore() puts the layer back to
    exactly the captured state. `exists` False means the spec was not
    authored in that layer at all, and Restore() removes it.
    """

    def __init__(self, layer, specPath):
        self.layer = layer
        self.specPath = Sdf.Path(specPath)
        self.exists = False
        self.typeName = None
        self.variability = Sdf.VariabilityVarying
        self.hasDefault = False
        self.default = None
        self.spline = None
        self.timeSamples = {}

    @classmethod
    def Capture(cls, layer, specPath):
        snap = cls(layer, specPath)
        spec = layer.GetAttributeAtPath(snap.specPath)
        if spec is None:
            return snap
        snap.exists = True
        snap.typeName = spec.typeName
        snap.variability = spec.variability
        snap.hasDefault = spec.HasDefaultValue()
        if snap.hasDefault:
            snap.default = spec.default
        if spec.HasSpline():
            # Copy: the live spline object is owned by the spec and would
            # change under us as the drag writes new knots.
            snap.spline = Ts.Spline(spec.GetSpline())
        snap.timeSamples = {
            t: layer.QueryTimeSample(snap.specPath, t)
            for t in layer.ListTimeSamplesForPath(snap.specPath)}
        return snap

    def _EnsureSpec(self):
        spec = self.layer.GetAttributeAtPath(self.specPath)
        if spec is not None:
            return spec
        primPath = self.specPath.GetPrimPath()
        primSpec = Sdf.CreatePrimInLayer(self.layer, primPath)
        return Sdf.AttributeSpec(
            primSpec, self.specPath.name, self.typeName, self.variability)

    def Restore(self):
        with Sdf.ChangeBlock():
            spec = self.layer.GetAttributeAtPath(self.specPath)
            if not self.exists:
                if spec is not None:
                    primSpec = spec.owner
                    primSpec.RemoveProperty(spec)
                return
            spec = self._EnsureSpec()
            if self.hasDefault:
                spec.default = self.default
            elif spec.HasDefaultValue():
                spec.ClearDefaultValue()
            if self.spline is not None:
                spec.SetSpline(Ts.Spline(self.spline))
            elif spec.HasSpline():
                spec.ClearSpline()
            for t in list(self.layer.ListTimeSamplesForPath(self.specPath)):
                if t not in self.timeSamples:
                    self.layer.EraseTimeSample(self.specPath, t)
            for t, value in self.timeSamples.items():
                self.layer.SetTimeSample(self.specPath, t, value)

    def __eq__(self, other):
        if not isinstance(other, AttributeSnapshot):
            return NotImplemented
        return (self.exists == other.exists
                and self.hasDefault == other.hasDefault
                and self.default == other.default
                and self.spline == other.spline
                and self.timeSamples == other.timeSamples)

    def __ne__(self, other):
        result = self.__eq__(other)
        return result if result is NotImplemented else not result


def _ScratchLayer():
    return Sdf.Layer.CreateAnonymous("rigExecUndoScratch.usda")


def _EnsureAncestors(layer, specPath):
    """
    Create the spec's parent chain in `layer`.

    Sdf.CopySpec will not invent ancestors: copying </Rig/IK> into a
    layer with no </Rig> fails inside SdfData rather than returning
    false, so the parent has to exist before the copy either way.
    """
    parent = (specPath.GetPrimPath() if specPath.IsPropertyPath()
              else specPath.GetParentPath())
    if parent and not parent.IsAbsoluteRootPath():
        Sdf.CreatePrimInLayer(layer, parent)


def _SiblingNames(layer, primPath):
    """The names of primPath's parent's children, in layer order."""
    parent = primPath.GetParentPath()
    if parent.IsAbsoluteRootPath():
        return list(layer.rootPrims.keys())
    spec = layer.GetPrimAtPath(parent)
    return list(spec.nameChildren.keys()) if spec is not None else []


def _FollowingSiblingNames(layer, specPath):
    """
    The prim siblings that come after specPath, or [] for a property.

    Property order carries no composition meaning, so it is not worth
    the copies it would cost to preserve.
    """
    if specPath.IsPropertyPath() or not specPath.IsPrimPath():
        return []
    names = _SiblingNames(layer, specPath)
    if specPath.name not in names:
        return []
    return names[names.index(specPath.name) + 1:]


def _MoveToEnd(layer, primPath):
    """Re-append one prim spec, unchanged, so it sits last again."""
    if layer.GetPrimAtPath(primPath) is None:
        return
    scratch = _ScratchLayer()
    _EnsureAncestors(scratch, primPath)
    Sdf.CopySpec(layer, primPath, scratch, primPath)
    _RemoveSpec(layer, primPath)
    Sdf.CopySpec(scratch, primPath, layer, primPath)


def _RemoveSpec(layer, specPath):
    if specPath.IsPropertyPath():
        prop = layer.GetPropertyAtPath(specPath)
        if prop is not None:
            prop.owner.RemoveProperty(prop)
        return
    spec = layer.GetPrimAtPath(specPath)
    if spec is None:
        return
    parent = spec.nameParent
    if parent is None:
        del layer.rootPrims[spec.name]
    else:
        del parent.nameChildren[spec.name]


class SpecSnapshot(object):
    """
    A whole spec (prim or property), copied aside verbatim.

    Sdf.CopySpec is what makes this general: it carries an attribute's
    default, time samples and metadata, a relationship's target list op,
    and a prim spec's entire subtree, without this module having to
    enumerate any of them. That is the difference that matters --
    AttributeSnapshot cannot see a prim being created, and cannot see a
    relationship at all, so the panels that build rig structure need
    this one.
    """

    def __init__(self, layer, specPath):
        self.layer = layer
        self.specPath = Sdf.Path(specPath)
        self.existed = False
        # Names of the siblings that followed this prim spec, so its
        # position among them can be put back. See _RestoreSiblingOrder.
        self.following = []
        self._scratch = None

    @classmethod
    def Capture(cls, layer, specPath):
        snap = cls(layer, specPath)
        if layer is None or layer.GetObjectAtPath(snap.specPath) is None:
            return snap
        snap.existed = True
        snap.following = _FollowingSiblingNames(layer, snap.specPath)
        snap._scratch = _ScratchLayer()
        _EnsureAncestors(snap._scratch, snap.specPath)
        Sdf.CopySpec(layer, snap.specPath, snap._scratch, snap.specPath)
        return snap

    def Restore(self):
        _RemoveSpec(self.layer, self.specPath)
        if not self.existed:
            return
        _EnsureAncestors(self.layer, self.specPath)
        Sdf.CopySpec(self._scratch, self.specPath, self.layer, self.specPath)
        self._RestoreSiblingOrder()

    def _RestoreSiblingOrder(self):
        """
        Put the spec back at the position it held among its siblings.

        Sdf.CopySpec APPENDS, so a spec restored into the middle of its
        parent lands at the end: undoing an edit to /A/C turns B,C,D
        into B,D,C. In RigExec that is not cosmetic -- mover evaluation
        order IS sibling order (README, "the bottom sibling fires first")
        -- so an undo would silently change which mover runs when.

        The children proxy cannot move a live spec (insert() rejects a
        handle that is already parented, and rejects a dead one after a
        delete), so the order is fixed by pushing everything that used to
        follow this spec back behind it, in its original order.
        """
        for name in self.following:
            _MoveToEnd(self.layer,
                       self.specPath.GetParentPath().AppendChild(name))

    def _Text(self):
        """The captured spec as usda text, for comparison."""
        if not self.existed:
            return None
        return self._scratch.ExportToString()

    def __eq__(self, other):
        if not isinstance(other, SpecSnapshot):
            return NotImplemented
        if self.existed != other.existed:
            return False
        # Two copies of the same spec are different layer objects, so
        # they are compared by what they SAY. Exporting is the only
        # total equality Sdf offers for a spec, and these are small.
        return self._Text() == other._Text()

    def __ne__(self, other):
        result = self.__eq__(other)
        return result if result is NotImplemented else not result


class EditEntry(object):
    def __init__(self, layer, specPath, before, after):
        self.layer = layer
        self.specPath = Sdf.Path(specPath)
        self.before = before
        self.after = after


class Edit(object):
    """One undoable unit: a label and the per-attribute before/after."""

    def __init__(self, label, entries):
        self.label = label
        self.entries = list(entries)

    def _Apply(self, useAfter):
        # ONE change block around the whole edit, not one per attribute.
        # An xform edit writes the op attributes and xformOpOrder as
        # separate specs; restoring them in separate blocks lets the
        # stage see an xformOpOrder naming ops that do not exist yet,
        # and anything that recomposes on every notice (the gizmo
        # controller, outside a drag) reads that half-applied state and
        # warns. The nested blocks inside Restore() are harmless: USD
        # sends the notices when the OUTERMOST block closes.
        with Sdf.ChangeBlock():
            for entry in self.entries:
                if entry.layer is None or entry.layer.expired:
                    continue
                (entry.after if useAfter else entry.before).Restore()

    def Undo(self):
        self._Apply(useAfter=False)

    def Redo(self):
        self._Apply(useAfter=True)


class UndoStack(object):
    """A bounded linear undo stack; a push discards the redo branch."""

    LIMIT = 200

    def __init__(self):
        self._undo = []
        self._redo = []
        self._listeners = []

    def AddListener(self, fn):
        self._listeners.append(fn)

    def _Notify(self):
        for fn in list(self._listeners):
            fn()

    def Push(self, edit):
        self._undo.append(edit)
        del self._undo[:-self.LIMIT]
        self._redo = []
        self._Notify()

    def CanUndo(self):
        return bool(self._undo)

    def CanRedo(self):
        return bool(self._redo)

    def UndoText(self):
        return self._undo[-1].label if self._undo else ""

    def RedoText(self):
        return self._redo[-1].label if self._redo else ""

    def Undo(self):
        if not self._undo:
            return False
        edit = self._undo.pop()
        edit.Undo()
        self._redo.append(edit)
        self._Notify()
        return True

    def Redo(self):
        if not self._redo:
            return False
        edit = self._redo.pop()
        edit.Redo()
        self._undo.append(edit)
        self._Notify()
        return True

    def Clear(self):
        self._undo = []
        self._redo = []
        self._Notify()


class EditRecorder(object):
    """
    Brackets one drag: Begin() snapshots the named attributes in the
    current edit target's layer, Commit() returns the Edit (None when
    nothing changed), Abort() restores the snapshots.
    """

    def __init__(self, stage, attrPaths):
        self._stage = stage
        self._paths = [Sdf.Path(p) for p in attrPaths]
        self._layer = None
        self._before = {}

    def Begin(self):
        target = self._stage.GetEditTarget()
        self._layer = target.GetLayer()
        self._before = {}
        for path in self._paths:
            specPath = target.MapToSpecPath(path)
            self._before[path] = AttributeSnapshot.Capture(
                self._layer, specPath)

    def Commit(self, label):
        entries = []
        for path, before in self._before.items():
            after = AttributeSnapshot.Capture(self._layer, before.specPath)
            if after != before:
                entries.append(
                    EditEntry(self._layer, before.specPath, before, after))
        self._before = {}
        if not entries:
            return None
        return Edit(label, entries)

    def Abort(self):
        # One change block for the same reason as Edit._Apply: the drag
        # being rolled back is one edit, so it must un-happen as one.
        with Sdf.ChangeBlock():
            for before in self._before.values():
                before.Restore()
        self._before = {}


class SpecRecorder(object):
    """
    Brackets one STRUCTURAL edit, with the same Begin/Commit/Abort
    contract EditRecorder has.

    The difference is what it can see. EditRecorder snapshots attribute
    specs, which is everything a pose edit touches and nothing a panel
    that BUILDS rig does: creating a weight, binding a mover, deleting
    a prim and retargeting a relationship are all invisible to it --
    it records no entry at all and the undo stack stays empty while the
    stage changes underneath. This one copies whole specs, so a prim
    that did not exist is removed again on undo and one that did comes
    back with its subtree, its metadata and its place among its
    siblings.

    Paths may be prim or property paths, and need not exist yet: a path
    that is absent at Begin() and present at Commit() is exactly the
    creation case, and its snapshot says "was not here".
    """

    def __init__(self, stage, specPaths):
        self._stage = stage
        self._paths = [Sdf.Path(p) for p in specPaths]
        self._layer = None
        self._before = {}

    def Begin(self):
        target = self._stage.GetEditTarget()
        self._layer = target.GetLayer()
        self._before = {}
        for path in self._paths:
            specPath = target.MapToSpecPath(path)
            self._before[path] = SpecSnapshot.Capture(self._layer, specPath)

    def Commit(self, label):
        entries = []
        for before in self._before.values():
            after = SpecSnapshot.Capture(self._layer, before.specPath)
            if after != before:
                entries.append(
                    EditEntry(self._layer, before.specPath, before, after))
        self._before = {}
        if not entries:
            return None
        return Edit(label, entries)

    def Abort(self):
        with Sdf.ChangeBlock():
            for before in self._before.values():
                before.Restore()
        self._before = {}


class SpecScope(object):
    """
    `with SpecScope(stage, paths, undoStack, "Label"):` around a
    structural edit. Pushes one entry on exit, or nothing when the edit
    turned out to change nothing; an exception rolls it back instead.
    """

    def __init__(self, stage, specPaths, undoStack, label):
        self._recorder = SpecRecorder(stage, specPaths)
        self._undo = undoStack
        self._label = label
        self.edit = None

    def __enter__(self):
        self._recorder.Begin()
        return self

    def __exit__(self, excType, exc, tb):
        if excType is not None:
            self._recorder.Abort()
            return False
        self.edit = self._recorder.Commit(self._label)
        if self.edit is not None and self._undo is not None:
            self._undo.Push(self.edit)
        return False
