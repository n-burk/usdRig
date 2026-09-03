#
# RigExec usdview undo: attribute-spec snapshots and a bounded undo stack.
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
