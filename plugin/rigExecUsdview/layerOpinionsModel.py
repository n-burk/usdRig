#
# Per-layer opinion enumeration and editing for one prim.
#
# This module is the headless half of the Layer Opinions panel: it knows
# how to find every opinion a prim has in every layer that contributes
# one, how to turn a value into the usda text the inline editor shows,
# how to parse that text back, and how to apply an edit or a delete as
# an undoable rigExecUndo.Edit. It imports no Qt, so the whole of it is
# testable without a display (tests/python/test_layer_opinions_model.py).
#
# Composition arcs need no special case here. references, payloads,
# inherits, specializes, variantSetNames and variantSelection are all
# FIELDS on the prim spec, reachable through ListInfoKeys/GetInfo/
# SetInfo/ClearInfo exactly like active, kind, typeName and specifier.
# So "metadata" and "arcs" are one row kind ("info"), not two.
#
from pxr import Sdf

import rigExecUndo


class ValueParseError(Exception):
    """The text in the value editor is not a value of the row's type."""


# Info fields that describe the spec's own existence rather than an
# opinion you would edit or clear on its own. Removing a specifier is
# what DeletePrimSpec is for.
_STRUCTURAL_INFO = ("specifier",)


def _ScratchLayer():
    return Sdf.Layer.CreateAnonymous("layerOpinionsScratch.usda")


def _TypeAlias(typeName):
    aliases = typeName.aliasesAsStrings
    return aliases[0] if aliases else str(typeName)


def FormatValue(value, typeName=None):
    """
    The usda text for `value`, as it would appear to the right of `=`.

    Written by USD rather than by str()/repr(): a bool is `true`, an
    asset path is `@...@`, a string is quoted, and a matrix keeps the
    nested-tuple form the parser accepts back.

    `typeName` is the SPEC's declared type, and is passed whenever there
    is one. Inferring it from the python value instead promotes a float
    to a double -- a `float softness = 0.15` would display as
    0.15000000596046448, and editing that row would author the
    double-rounded text back onto a float attribute.
    """
    if value is None:
        return ""
    layer = _ScratchLayer()
    prim = Sdf.CreatePrimInLayer(layer, "/P")
    prim.specifier = Sdf.SpecifierDef
    if typeName is None:
        typeName = Sdf.GetValueTypeNameForValue(value)
    if not typeName:
        # Not an Sdf value type (an enum, a list op, a dictionary). Those
        # rows are not text-editable; show python's rendering read-only.
        return str(value)
    attr = Sdf.AttributeSpec(prim, "v", typeName)
    attr.default = value
    return _ExtractValueText(layer.ExportToString())


def _ExtractValueText(usda):
    """
    The right-hand side of the single `v = ...` assignment in `usda`.

    Joined across continuation lines: a long array wraps, and taking
    only the first line would silently truncate the value.
    """
    lines = usda.split("\n")
    for i, line in enumerate(lines):
        if " v = " not in line:
            continue
        chunks = [line.split(" v = ", 1)[1]]
        # Keep consuming while brackets/parens are unbalanced.
        for tail in lines[i + 1:]:
            text = "".join(chunks)
            if (text.count("[") == text.count("]")
                    and text.count("(") == text.count(")")):
                break
            chunks.append(tail.strip())
        return "".join(chunks).strip()
    return ""


def ParseValue(typeName, text):
    """
    `text` parsed as a value of `typeName`, via usda syntax.

    Deliberately a layer import and never eval(): the value field takes
    whatever a user types, and typing a python expression into it must
    produce a parse error, not execute.
    """
    source = '#usda 1.0\ndef "P"\n{\n    %s v = %s\n}\n' % (
        _TypeAlias(typeName), text)
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a valid %s: %r" % (typeName, text))
    except Exception as error:
        raise ValueParseError("not a valid %s: %r (%s)"
                              % (typeName, text, error))
    attr = layer.GetAttributeAtPath("/P.v")
    if attr is None or not attr.HasDefaultValue():
        raise ValueParseError("not a valid %s: %r" % (typeName, text))
    return attr.default


def ParseTargets(text):
    """
    `text` parsed as a relationship target list.

    Same usda round trip as ParseValue -- the paths are validated by
    Sdf's own parser, so `[ not_a_path ]` is a parse error rather than
    a relationship that silently targets nothing.
    """
    source = '#usda 1.0\ndef "P"\n{\n    rel r = %s\n}\n' % text
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a target list: %r" % text)
    except ValueParseError:
        raise
    except Exception as error:
        raise ValueParseError("not a target list: %r (%s)" % (text, error))
    rel = layer.GetRelationshipAtPath("/P.r")
    if rel is None:
        raise ValueParseError("not a target list: %r" % text)
    return list(rel.targetPathList.explicitItems)


def ParseInfoValue(current, text):
    """
    `text` parsed as a replacement for the metadata value `current`.

    Only the scalar metadata kinds are text-editable; a list op, a
    dictionary or an enum is shown read-only and can be deleted but not
    retyped, which is what IsInfoEditable reports.
    """
    if isinstance(current, bool):
        lowered = text.strip().lower()
        if lowered in ("true", "1"):
            return True
        if lowered in ("false", "0"):
            return False
        raise ValueParseError("not a boolean: %r" % text)
    if isinstance(current, str):
        stripped = text.strip()
        if len(stripped) >= 2 and stripped[0] == '"' and stripped[-1] == '"':
            return stripped[1:-1]
        return stripped
    raise ValueParseError("%s metadata is not editable as text"
                          % type(current).__name__)


def IsInfoEditable(key, value):
    if key in _STRUCTURAL_INFO:
        return False
    return isinstance(value, (bool, str))


class OpinionRow(object):
    """
    One opinion: a field or property spec in one layer, for one prim.

    `specPath` is the path of the spec ITSELF, which is not the prim's
    path when the opinion arrives through a reference -- authoring at
    the prim path there would create a new override in the referencing
    layer instead of changing the opinion the row is showing.
    """

    def __init__(self, layer, specPath, kind, key, valueText,
                 editable, winning, typeName=None, infoValue=None):
        self.layer = layer
        self.specPath = Sdf.Path(specPath)
        self.kind = kind          # "info" | "attribute" | "relationship"
        self.key = key
        self.valueText = valueText
        self.editable = editable
        self.winning = winning
        self.typeName = typeName
        self.infoValue = infoValue

    @property
    def primPath(self):
        return self.specPath.GetPrimPath()

    def __repr__(self):
        return "<OpinionRow %s %s %s in %s>" % (
            self.kind, self.key, self.valueText, self.layer.identifier)


class OpinionGroup(object):
    """Every opinion one layer holds for the prim."""

    def __init__(self, layer, editable):
        self.layer = layer
        self.editable = editable
        self.rows = []
        # Set by the UI once it knows usdview's edit target.
        self.isEditTarget = False

    @property
    def displayName(self):
        return self.layer.GetDisplayName()


def _LayerEditable(layer):
    return bool(layer.permissionToEdit) and not layer.expired


def OpinionGroups(prim):
    """
    The prim's opinions, grouped by layer, strongest layer first.

    Only layers that actually carry a spec appear. The first row seen
    for a given (kind, key) in stack order is the one that wins, so the
    panel can show which layer an override is coming from rather than
    leaving you to guess why an edit had no effect.
    """
    groups = []
    byLayer = {}
    winners = set()
    for spec in prim.GetPrimStack():
        layer = spec.layer
        group = byLayer.get(layer)
        if group is None:
            group = OpinionGroup(layer, _LayerEditable(layer))
            byLayer[layer] = group
            groups.append(group)
        group.rows.extend(_RowsForSpec(spec, group, winners))
    return groups


def _RowsForSpec(spec, group, winners):
    rows = []
    for key in sorted(spec.ListInfoKeys()):
        value = spec.GetInfo(key)
        rows.append(OpinionRow(
            layer=spec.layer,
            specPath=spec.path,
            kind="info",
            key=key,
            valueText=FormatValue(value),
            editable=group.editable and IsInfoEditable(key, value),
            winning=_ClaimWinner(winners, "info", key),
            infoValue=value))
    for prop in spec.properties:
        if isinstance(prop, Sdf.AttributeSpec):
            kind = "attribute"
            typeName = prop.typeName
            text = (FormatValue(prop.default, typeName)
                    if prop.HasDefaultValue() else "")
            editable = group.editable
        else:
            kind = "relationship"
            text = _FormatTargets(prop)
            typeName = None
            editable = group.editable
        rows.append(OpinionRow(
            layer=spec.layer,
            specPath=prop.path,
            kind=kind,
            key=prop.name,
            valueText=text,
            editable=editable,
            winning=_ClaimWinner(winners, kind, prop.name),
            typeName=typeName))
    return rows


def _ClaimWinner(winners, kind, key):
    token = (kind, key)
    if token in winners:
        return False
    winners.add(token)
    return True


def _FormatTargets(relSpec):
    targets = relSpec.targetPathList
    items = list(targets.explicitItems) or list(targets.prependedItems)
    return "[ %s ]" % ", ".join(str(p) for p in items)


def FindRow(groups, layer, kind, key):
    for group in groups:
        if group.layer != layer:
            continue
        for row in group.rows:
            if row.kind == kind and row.key == key:
                return row
    raise KeyError("no %s row %r in %s" % (kind, key, layer.identifier))


# --------------------------------------------------------------------
# Snapshots. Each restores exactly one opinion, so an undo entry is as
# narrow as the edit that made it.
# --------------------------------------------------------------------

class InfoSnapshot(object):
    """The authored state of one info field on one prim spec."""

    def __init__(self, layer, primPath, key):
        self.layer = layer
        self.primPath = Sdf.Path(primPath)
        self.key = key
        self.had = False
        self.value = None

    @classmethod
    def Capture(cls, layer, primPath, key):
        snap = cls(layer, primPath, key)
        spec = layer.GetPrimAtPath(snap.primPath)
        if spec is not None and spec.HasInfo(key):
            snap.had = True
            snap.value = spec.GetInfo(key)
        return snap

    def Restore(self):
        spec = self.layer.GetPrimAtPath(self.primPath)
        if spec is None:
            if not self.had:
                return
            spec = Sdf.CreatePrimInLayer(self.layer, self.primPath)
        if self.had:
            spec.SetInfo(self.key, self.value)
        elif spec.HasInfo(self.key):
            spec.ClearInfo(self.key)


class SpecCopySnapshot(object):
    """
    A whole spec (property or prim), copied aside verbatim.

    Sdf.CopySpec is what makes this general: it carries an attribute's
    default, time samples and metadata, a relationship's target list op,
    and a prim spec's entire subtree, without this module having to
    enumerate any of them.
    """

    def __init__(self, layer, specPath):
        self.layer = layer
        self.specPath = Sdf.Path(specPath)
        self.existed = False
        self._scratch = None

    @classmethod
    def Capture(cls, layer, specPath):
        snap = cls(layer, specPath)
        if layer.GetObjectAtPath(snap.specPath) is None:
            return snap
        snap.existed = True
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


def _Commit(label, layer, specPath, before, after):
    return rigExecUndo.Edit(
        label, [rigExecUndo.EditEntry(layer, specPath, before, after)])


# --------------------------------------------------------------------
# Operations. Each applies the change and returns the undoable Edit.
# --------------------------------------------------------------------

def SetRowValue(row, text):
    """Author `text` as the row's new value. Returns an undoable Edit."""
    if not row.editable:
        raise ValueParseError("%s is not editable" % row.key)
    if row.kind == "info":
        return _SetInfo(row, text)
    if row.kind == "attribute":
        return _SetAttribute(row, text)
    if row.kind == "relationship":
        return _SetTargets(row, text)
    raise ValueParseError("%s rows are not editable" % row.kind)


def _SetInfo(row, text):
    value = ParseInfoValue(row.infoValue, text)
    primPath = row.specPath
    before = InfoSnapshot.Capture(row.layer, primPath, row.key)
    with Sdf.ChangeBlock():
        row.layer.GetPrimAtPath(primPath).SetInfo(row.key, value)
    after = InfoSnapshot.Capture(row.layer, primPath, row.key)
    return _Commit("Set %s" % row.key, row.layer, primPath, before, after)


def _SetAttribute(row, text):
    value = ParseValue(row.typeName, text)
    before = SpecCopySnapshot.Capture(row.layer, row.specPath)
    with Sdf.ChangeBlock():
        row.layer.GetAttributeAtPath(row.specPath).default = value
    after = SpecCopySnapshot.Capture(row.layer, row.specPath)
    return _Commit("Set %s" % row.key, row.layer, row.specPath, before, after)


def _SetTargets(row, text):
    targets = ParseTargets(text)
    before = SpecCopySnapshot.Capture(row.layer, row.specPath)
    with Sdf.ChangeBlock():
        spec = row.layer.GetRelationshipAtPath(row.specPath)
        spec.targetPathList.ClearEdits()
        spec.targetPathList.explicitItems = targets
    after = SpecCopySnapshot.Capture(row.layer, row.specPath)
    return _Commit("Set %s" % row.key, row.layer, row.specPath, before, after)


def DeleteRow(row):
    """Remove the row's opinion from its layer. Returns an undoable Edit."""
    if row.kind == "info":
        before = InfoSnapshot.Capture(row.layer, row.specPath, row.key)
        with Sdf.ChangeBlock():
            spec = row.layer.GetPrimAtPath(row.specPath)
            if spec is not None and spec.HasInfo(row.key):
                spec.ClearInfo(row.key)
        after = InfoSnapshot.Capture(row.layer, row.specPath, row.key)
    else:
        before = SpecCopySnapshot.Capture(row.layer, row.specPath)
        with Sdf.ChangeBlock():
            _RemoveSpec(row.layer, row.specPath)
        after = SpecCopySnapshot.Capture(row.layer, row.specPath)
    return _Commit("Delete %s" % row.key, row.layer, row.specPath,
                   before, after)


def DeletePrimSpec(group):
    """
    Remove the prim's whole spec from one layer -- every opinion in that
    group at once. Returns an undoable Edit.
    """
    primPath = group.rows[0].primPath if group.rows else None
    if primPath is None:
        raise KeyError("layer holds no spec for this prim")
    before = SpecCopySnapshot.Capture(group.layer, primPath)
    with Sdf.ChangeBlock():
        _RemoveSpec(group.layer, primPath)
    after = SpecCopySnapshot.Capture(group.layer, primPath)
    return _Commit("Delete %s opinions" % group.layer.GetDisplayName(),
                   group.layer, primPath, before, after)
