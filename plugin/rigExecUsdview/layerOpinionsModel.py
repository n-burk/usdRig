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
# Composition arcs need no special case to be FOUND here. references,
# payloads, inherits, specializes, variantSetNames and variantSelection
# are all fields on the prim spec, reachable through ListInfoKeys/
# GetInfo/SetInfo/ClearInfo exactly like active, kind, typeName and
# specifier. So "metadata" and "arcs" are one row kind ("info"), not two.
#
# They do need one to be EDITED. An arc field holds a list of arcs, not
# a value: `references` is an SdfReferenceListOp with up to six arms,
# and showing it as one row means the only edit the panel can offer is
# "delete every reference on this prim in this layer". So a field whose
# value is a list op is expanded into one child row per item -- carrying
# the arm and index it sits at -- and those rows are what set, delete and
# reorder operate on. The expansion is duck-typed on the arms rather
# than keyed off a list of arc names: it then also covers apiSchemas and
# any list-op field USD adds later, and cannot go stale.
#
# The same goes for the two arcs that are LAYER metadata rather than
# prim metadata -- subLayerPaths and relocates. They are listed under the
# group of the layer that holds them, so the panel that can add them can
# also see and change them.
#
# EVERY value shown here round-trips through usda, in both directions:
# an item's text is written by exporting a scratch spec that holds just
# that item, and the text typed back is parsed by importing it. Nothing
# in this module renders or parses a Reference, a layer offset or a
# relocate by hand, so what the panel shows is what the file would say,
# and a typo is a parse error rather than a value that quietly differs.
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

    Only the scalar metadata kinds are text-editable as a whole, which
    is what IsInfoEditable reports. An enum is shown read-only; a list
    op or a dictionary is shown as a heading over its entries, and it is
    those that are retyped, one at a time.
    """
    if isinstance(current, bool):
        lowered = text.strip().lower()
        if lowered in ("true", "1"):
            return True
        if lowered in ("false", "0"):
            return False
        raise ValueParseError("not a boolean: %r" % text)
    if isinstance(current, str):
        return _Unquote(text)
    raise ValueParseError("%s metadata is not editable as text"
                          % type(current).__name__)


def _Unquote(text):
    """
    `text` with the quotes the panel SHOWS a string in taken back off.

    The value column renders a string metadatum as usda writes it, so
    retyping it comes back quoted; typing it bare is just as clearly
    meant. Both are the same string.
    """
    stripped = text.strip()
    if len(stripped) >= 2 and stripped[0] == '"' and stripped[-1] == '"':
        return stripped[1:-1]
    return stripped


def IsInfoEditable(key, value):
    if key in _STRUCTURAL_INFO:
        return False
    return isinstance(value, (bool, str))


# --------------------------------------------------------------------
# List-op fields: the shape every composition arc on a prim spec has.
# --------------------------------------------------------------------

# The arms of an SdfListOp, in the order usda writes them. An opinion
# uses either `explicit` on its own or some of the other five.
LIST_OP_ARMS = ("explicit", "deleted", "added", "prepended",
                "appended", "ordered")

# What each arm is called in front of the field name in usda. The
# explicit arm has no prefix -- `references = @a@` IS the explicit form.
_ARM_PREFIX = {"explicit": "", "deleted": "delete", "added": "add",
               "prepended": "prepend", "appended": "append",
               "ordered": "reorder"}


def IsListOp(value):
    """
    Whether `value` is one of Sdf's list ops.

    Duck-typed on the arms rather than matched against a list of class
    names. There are a dozen ListOp types (Path, Reference, Payload,
    String, Token, Int, UInt, Int64, UInt64, Unregistered...) and USD
    adds more; a name list that misses one would silently fall back to
    showing a python repr, which is exactly the row nobody can edit.
    """
    return all(hasattr(value, arm + "Items") for arm in LIST_OP_ARMS)


def ListOpItems(value):
    """[(arm, index, item)] for every item in `value`, in usda order."""
    out = []
    for arm in LIST_OP_ARMS:
        for index, item in enumerate(getattr(value, arm + "Items")):
            out.append((arm, index, item))
    return out


def _SoleMetadataAssignment(usda):
    """
    (name, value) of the single metadata assignment in `usda`.

    Joined across lines: a long asset path or a nested value wraps, and
    taking only the first line would truncate it.
    """
    lines = usda.split("\n")
    for i, line in enumerate(lines):
        if not line.rstrip().endswith("("):
            continue
        chunks = []
        for tail in lines[i + 1:]:
            if tail.strip() == ")":
                break
            chunks.append(tail.strip())
        text = " ".join(chunks)
        name, sep, rest = text.partition(" = ")
        return (name.strip(), rest.strip()) if sep else ("", text)
    return "", ""


def FormatListOpItem(key, value, item):
    """
    (keyword, text) for one item of the list op `value`, which is held
    under info key `key`.

    Both halves come from USD rather than from this module, and both
    have to:

      * the keyword is NOT the info key. `inheritPaths` is written
        `inherits`, `variantSetNames` is written `variantSets`, and a
        row labelled with the info key would not match the file;
      * the item's text carries quoting, the `@...@` of an asset path,
        the `</...>` of a prim path and the `(offset = ...; scale = ...)`
        of a layer offset.

    So a scratch spec is given exactly this one item, as an explicit
    list, and asked to export itself. Same reason as FormatValue.
    """
    layer = _ScratchLayer()
    prim = Sdf.CreatePrimInLayer(layer, "/P")
    prim.specifier = Sdf.SpecifierDef
    single = type(value)()
    single.explicitItems = [item]
    prim.SetInfo(key, single)
    return _SoleMetadataAssignment(layer.ExportToString())


def ParseListOpItem(key, keyword, text):
    """
    `text` parsed as ONE item of the list-op field written `keyword`.

    The mirror of FormatListOpItem, and a layer import for the same
    reason ParseValue is one: the value field takes whatever is typed,
    so a python expression must be a parse error rather than something
    that runs.

    A text holding two items is refused rather than taking the first.
    The row addresses one position in one arm; writing two arcs into it
    would silently drop one of them.
    """
    source = ('#usda 1.0\ndef "P" (\n    %s = %s\n)\n{\n}\n'
              % (keyword, text))
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a %s: %r" % (keyword, text))
    except ValueParseError:
        raise
    except Exception as error:
        raise ValueParseError("not a %s: %r (%s)" % (keyword, text, error))
    spec = layer.GetPrimAtPath("/P")
    value = spec.GetInfo(key) if spec is not None and spec.HasInfo(key) \
        else None
    if value is None or not IsListOp(value):
        raise ValueParseError("not a %s: %r" % (keyword, text))
    items = list(value.explicitItems)
    if len(items) != 1:
        raise ValueParseError(
            "%r is %d items; this row is one %s"
            % (text, len(items), keyword))
    return items[0]


def _WriteListOp(spec, key, value):
    """
    Put `value` back on the spec, or clear the field when it is empty.

    An emptied list op is NOT the absence of one. Emptying an explicit
    opinion leaves `references = None`, which is an authored opinion
    that BLOCKS every weaker reference; emptying the other arms leaves a
    field that exports as nothing but still answers HasInfo, so the panel
    would go on listing a row with no items in it. Removing the last item
    of an arc means "this layer no longer says anything about references"
    in both cases, so the field goes.
    """
    if ListOpItems(value):
        spec.SetInfo(key, value)
    elif spec.HasInfo(key):
        spec.ClearInfo(key)


def SetListOpItem(spec, key, arm, index, item):
    """Replace one item of a list-op field in place."""
    value = spec.GetInfo(key)
    items = list(getattr(value, arm + "Items"))
    items[index] = item
    setattr(value, arm + "Items", items)
    spec.SetInfo(key, value)


def RemoveListOpItem(spec, key, arm, index):
    """Drop one item of a list-op field."""
    value = spec.GetInfo(key)
    items = list(getattr(value, arm + "Items"))
    del items[index]
    setattr(value, arm + "Items", items)
    _WriteListOp(spec, key, value)


def MoveListOpItem(spec, key, arm, index, delta):
    """
    Move one item `delta` places within its own arm.

    Within the arm only, never across arms. The arms are different
    opinions -- moving an arc from `prepend` to `append` flips it from
    winning over the existing arcs to losing to them -- and that is a
    decision, not a nudge. Out-of-range is a no-op rather than a wrap,
    so holding the key down at the end of a list does nothing.
    """
    value = spec.GetInfo(key)
    items = list(getattr(value, arm + "Items"))
    target = index + delta
    if not (0 <= index < len(items) and 0 <= target < len(items)):
        return False
    items.insert(target, items.pop(index))
    setattr(value, arm + "Items", items)
    spec.SetInfo(key, value)
    return True


# --------------------------------------------------------------------
# The two arcs that are LAYER metadata: sublayers and relocates.
#
# Neither is a field on a prim spec, so neither can go through the
# list-op path above -- but both round-trip through usda the same way,
# by exporting a scratch layer holding just the one entry and importing
# the text back.
# --------------------------------------------------------------------

def FormatSublayer(path, offset):
    """
    One `subLayers` entry as usda writes it: `@path@`, plus the
    `(offset = ...; scale = ...)` when the offset is not the identity.

    Round-tripped through a scratch layer rather than formatted here, so
    the offset's numbers are spelled the way the file spells them.
    """
    scratch = _ScratchLayer()
    scratch.subLayerPaths = [path]
    scratch.subLayerOffsets[0] = offset
    _, text = _SoleMetadataAssignment(scratch.ExportToString())
    return text.strip().strip("[]").strip()


def ParseSublayer(text):
    """(path, offset) from one `subLayers` entry."""
    source = '#usda 1.0\n(\n    subLayers = [\n        %s\n    ]\n)\n' % text
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a sublayer: %r" % text)
    except ValueParseError:
        raise
    except Exception as error:
        raise ValueParseError("not a sublayer: %r (%s)" % (text, error))
    if len(layer.subLayerPaths) != 1:
        raise ValueParseError(
            "%r is %d sublayers; this row is one"
            % (text, len(layer.subLayerPaths)))
    return layer.subLayerPaths[0], Sdf.LayerOffset(layer.subLayerOffsets[0])


def FormatRelocate(source, target):
    """One `relocates` entry as usda writes it: `</A/B>: </A/C>`."""
    scratch = _ScratchLayer()
    scratch.relocates = [(source, target)]
    _, text = _SoleMetadataAssignment(scratch.ExportToString())
    return text.strip().strip("{}").strip().rstrip(",").strip()


def ParseRelocate(text):
    """(source, target) from one `relocates` entry."""
    source = '#usda 1.0\n(\n    relocates = {\n        %s\n    }\n)\n' % text
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a relocate: %r" % text)
    except ValueParseError:
        raise
    except Exception as error:
        raise ValueParseError("not a relocate: %r (%s)" % (text, error))
    entries = list(layer.relocates)
    if len(entries) != 1:
        raise ValueParseError("%r is %d relocates; this row is one"
                              % (text, len(entries)))
    return entries[0]


def SetSublayer(layer, index, path, offset):
    """
    Replace one sublayer in place, path and offset together.

    Assigned by index rather than by rebuilding subLayerPaths: assigning
    the whole list resets EVERY offset to the identity, so a rebuild
    would silently un-retime the sublayers either side of this one.
    """
    layer.subLayerPaths[index] = path
    layer.subLayerOffsets[index] = offset


def RemoveSublayer(layer, index):
    """Drop one sublayer, taking its offset with it."""
    del layer.subLayerPaths[index]


def MoveSublayer(layer, index, delta):
    """
    Move one sublayer `delta` places, keeping every offset with its path.

    The offsets are read out, reordered alongside the paths and written
    back, because assigning subLayerPaths resets them all.
    """
    paths = list(layer.subLayerPaths)
    offsets = [Sdf.LayerOffset(o) for o in layer.subLayerOffsets]
    target = index + delta
    if not (0 <= index < len(paths) and 0 <= target < len(paths)):
        return False
    paths.insert(target, paths.pop(index))
    offsets.insert(target, offsets.pop(index))
    layer.subLayerPaths = paths
    for i, offset in enumerate(offsets):
        layer.subLayerOffsets[i] = offset
    return True


def SetRelocate(layer, index, source, target):
    entries = list(layer.relocates)
    entries[index] = (source, target)
    layer.relocates = entries


def RemoveRelocate(layer, index):
    entries = list(layer.relocates)
    del entries[index]
    if entries:
        layer.relocates = entries
    else:
        # An empty relocates map is not the same as none: assigning []
        # leaves `relocates = {}` in the file. ClearRelocates removes it.
        layer.ClearRelocates()


class OpinionRow(object):
    """
    One opinion: a field or property spec in one layer, for one prim.

    `specPath` is the path of the spec ITSELF, which is not the prim's
    path when the opinion arrives through a reference -- authoring at
    the prim path there would create a new override in the referencing
    layer instead of changing the opinion the row is showing.

    A row may have `children`: an arc field holds a list of arcs, and
    each item gets a row of its own underneath the field's. A child
    carries the `arm` and `index` that address it, which is what makes
    it separately editable, removable and movable.
    """

    def __init__(self, layer, specPath, kind, key, valueText,
                 editable, winning, typeName=None, infoValue=None,
                 infoKey=None, keyword=None, arm=None, index=None,
                 item=None, count=0):
        self.layer = layer
        self.specPath = Sdf.Path(specPath)
        # "info" | "attribute" | "relationship" | "arcItem"
        # | "variantSelection" | "layerInfo" | "sublayer" | "relocate"
        self.kind = kind
        self.key = key
        self.valueText = valueText
        self.editable = editable
        self.winning = winning
        self.typeName = typeName
        self.infoValue = infoValue
        # Set on the rows that address one entry of a list: the info
        # field it belongs to, the keyword usda writes it under, and
        # where in that field it sits.
        self.infoKey = infoKey
        self.keyword = keyword
        self.arm = arm
        self.index = index
        self.item = item
        # How many entries share this row's list, so the panel can tell
        # whether there is anywhere to move it.
        self.count = count
        self.children = []

    @property
    def primPath(self):
        return self.specPath.GetPrimPath()

    def __repr__(self):
        return "<OpinionRow %s %s %s in %s>" % (
            self.kind, self.key, self.valueText, self.layer.identifier)


class OpinionGroup(object):
    """Every opinion one layer holds for the prim."""

    def __init__(self, layer, editable, local=False):
        self.layer = layer
        self.editable = editable
        # Whether the layer is in the stage's own local layer stack --
        # session, root and its sublayers -- rather than reached through
        # a reference or a payload. Composition arcs may only be changed
        # in a local layer: an arc inside a referenced asset belongs to
        # that asset and restructuring it changes every place it is used.
        self.local = local
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

    A local layer's own sublayers and relocates are listed first, ahead
    of the prim's opinions in that layer. They are not opinions ABOUT
    this prim -- they are the layer's composition, which is why they are
    marked out as layer metadata -- but the panel that can add them is
    the only place they can be seen, and the layer group is where they
    belong.
    """
    groups = []
    byLayer = {}
    winners = set()
    local = {layer.identifier for layer in prim.GetStage().GetLayerStack()}
    for spec in prim.GetPrimStack():
        layer = spec.layer
        group = byLayer.get(layer)
        if group is None:
            group = OpinionGroup(layer, _LayerEditable(layer),
                                 layer.identifier in local)
            byLayer[layer] = group
            groups.append(group)
        group.rows.extend(_RowsForSpec(spec, group, winners))
    for group in groups:
        if group.local:
            group.rows[:0] = _LayerArcRows(group)
    return groups


def _RowsForSpec(spec, group, winners):
    rows = []
    for key in sorted(spec.ListInfoKeys()):
        value = spec.GetInfo(key)
        row = OpinionRow(
            layer=spec.layer,
            specPath=spec.path,
            kind="info",
            key=key,
            valueText=FormatValue(value),
            editable=group.editable and IsInfoEditable(key, value),
            winning=_InfoWinning(winners, key, value),
            infoValue=value)
        if IsListOp(value):
            row.valueText = _ListOpSummary(value)
            row.children = _ListOpRows(spec, group, key, value)
        elif key == "variantSelection":
            row.valueText = _CountSummary(len(value), "selection")
            row.children = _VariantSelectionRows(spec, group, value)
        rows.append(row)
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


def _InfoWinning(winners, key, value):
    """
    Whether this layer's opinion of `key` is the one that takes effect.

    A list op that is NOT explicit does not shadow the weaker layers'
    opinions of the same field -- it composes with them. A stronger
    `prepend references` and a weaker one both contribute, in that
    order, so striking the weaker row through would claim an override
    that is not happening: it neither loses to what came before nor
    claims the field against what comes after.

    An explicit list (`references = [...]`) DOES replace what is
    underneath, and claims the field like any other opinion -- which is
    what makes the weaker rows below it read as overridden.
    """
    if ("info", key) in winners:
        return False
    if IsListOp(value) and not value.isExplicit:
        return True
    return _ClaimWinner(winners, "info", key)


def _CountSummary(count, noun):
    return "%d %s%s" % (count, noun, "" if count == 1 else "s")


def _ListOpSummary(value):
    """
    The one line a collapsed arc field shows.

    An EMPTY explicit list is called out rather than shown as nothing:
    `references = None` is not the absence of an opinion, it is an
    opinion that blocks every weaker reference, and a row reading
    "0 items" would be the most misleading thing the panel could say.
    """
    count = len(ListOpItems(value))
    if count:
        return _CountSummary(count, "item")
    if value.isExplicit:
        return "none  (blocks weaker opinions)"
    return "empty"


def _ArcRowsEditable(group):
    """
    Whether the arc rows in `group` may be changed.

    Composition arcs are only editable in a LOCAL layer. The panel lets
    you retype an attribute in a referenced asset's layer -- that is one
    value in one place -- but restructuring that asset's composition
    changes every shot it appears in, and the panel is not the place to
    do it by accident. AuthoringLayers() states the same rule for the
    guided flows.
    """
    return group.editable and group.local


def _ListOpRows(spec, group, key, value):
    """One row per item of a list-op field, in the order usda writes it."""
    rows = []
    editable = _ArcRowsEditable(group)
    for arm, index, item in ListOpItems(value):
        keyword, text = FormatListOpItem(key, value, item)
        prefix = _ARM_PREFIX[arm]
        count = len(getattr(value, arm + "Items"))
        rows.append(OpinionRow(
            layer=spec.layer,
            specPath=spec.path,
            kind="arcItem",
            key="%s%s [%d]" % (prefix + " " if prefix else "",
                               keyword, index),
            valueText=text,
            editable=editable,
            # An item of a composing list op is never shadowed: the arms
            # of a weaker layer's `prepend references` compose WITH the
            # stronger layer's, they are not replaced by them.
            winning=True,
            infoKey=key,
            keyword=keyword,
            arm=arm,
            index=index,
            item=item,
            count=count))
    return rows


def _VariantSelectionRows(spec, group, selections):
    rows = []
    editable = _ArcRowsEditable(group)
    for name in sorted(selections):
        rows.append(OpinionRow(
            layer=spec.layer,
            specPath=spec.path,
            kind="variantSelection",
            key=name,
            valueText='"%s"' % selections[name],
            editable=editable,
            winning=True,
            infoKey="variantSelection",
            item=selections[name]))
    return rows


def _LayerArcRows(group):
    """
    The layer's own composition: its sublayers and its relocates.

    Returned as two parent rows with a child each, or nothing at all
    when the layer has neither -- most layers have no relocates and only
    the root layer has sublayers, so the common case adds no rows.
    """
    layer = group.layer
    editable = _ArcRowsEditable(group)
    rows = []
    paths = list(layer.subLayerPaths)
    if paths:
        offsets = [Sdf.LayerOffset(o) for o in layer.subLayerOffsets]
        parent = OpinionRow(
            layer=layer, specPath=Sdf.Path.absoluteRootPath,
            kind="layerInfo", key="subLayers",
            valueText=_CountSummary(len(paths), "layer"),
            editable=False, winning=True, infoKey="subLayerPaths")
        for index, path in enumerate(paths):
            parent.children.append(OpinionRow(
                layer=layer, specPath=Sdf.Path.absoluteRootPath,
                kind="sublayer", key="subLayer [%d]" % index,
                valueText=FormatSublayer(path, offsets[index]),
                editable=editable, winning=True,
                infoKey="subLayerPaths", index=index,
                item=(path, offsets[index]), count=len(paths)))
        rows.append(parent)
    relocates = list(layer.relocates)
    if relocates:
        parent = OpinionRow(
            layer=layer, specPath=Sdf.Path.absoluteRootPath,
            kind="layerInfo", key="relocates",
            valueText=_CountSummary(len(relocates), "relocate"),
            editable=False, winning=True, infoKey="relocates")
        for index, (source, target) in enumerate(relocates):
            parent.children.append(OpinionRow(
                layer=layer, specPath=Sdf.Path.absoluteRootPath,
                kind="relocate", key="relocate [%d]" % index,
                valueText=FormatRelocate(source, target),
                editable=editable, winning=True,
                infoKey="relocates", index=index,
                item=(source, target), count=len(relocates)))
        rows.append(parent)
    return rows


def _FormatTargets(relSpec):
    targets = relSpec.targetPathList
    items = list(targets.explicitItems) or list(targets.prependedItems)
    return "[ %s ]" % ", ".join(str(p) for p in items)


def FindRow(groups, layer, kind, key):
    for group in groups:
        if group.layer != layer:
            continue
        for row in WalkRows(group.rows):
            if row.kind == kind and row.key == key:
                return row
    raise KeyError("no %s row %r in %s" % (kind, key, layer.identifier))


def WalkRows(rows):
    """Every row in `rows`, and every row underneath them."""
    for row in rows:
        yield row
        for child in WalkRows(row.children):
            yield child


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


class SublayerSnapshot(object):
    """
    A layer's subLayerPaths and their offsets, captured together.

    Lives here rather than beside the sublayer arc because both halves
    of the panel need it: the guided flow authors a sublayer, and the
    tree edits, removes and reorders the ones already there.
    """

    def __init__(self, layer):
        self.layer = layer
        self.paths = []
        self.offsets = []

    @classmethod
    def Capture(cls, layer):
        snap = cls(layer)
        snap.paths = list(layer.subLayerPaths)
        snap.offsets = [Sdf.LayerOffset(o) for o in layer.subLayerOffsets]
        return snap

    def Restore(self):
        # Assigning subLayerPaths resets every offset to identity, so
        # the offsets have to be written back afterwards -- restoring
        # the paths alone would silently drop a retimed sublayer.
        self.layer.subLayerPaths = list(self.paths)
        for i, offset in enumerate(self.offsets):
            self.layer.subLayerOffsets[i] = offset


class RelocatesSnapshot(object):
    """A layer's relocates map."""

    def __init__(self, layer):
        self.layer = layer
        self.relocates = []

    @classmethod
    def Capture(cls, layer):
        snap = cls(layer)
        snap.relocates = list(layer.relocates)
        return snap

    def Restore(self):
        if self.relocates:
            self.layer.relocates = list(self.relocates)
        else:
            self.layer.ClearRelocates()


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
        # Names of the siblings that followed this prim spec, so its
        # position among them can be put back. See _RestoreSiblingOrder.
        self.following = []
        self._scratch = None

    @classmethod
    def Capture(cls, layer, specPath):
        snap = cls(layer, specPath)
        if layer.GetObjectAtPath(snap.specPath) is None:
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
    if row.kind == "arcItem":
        return _SetArcItem(row, text)
    if row.kind == "variantSelection":
        return _SetVariantSelection(row, text)
    if row.kind == "sublayer":
        return _SetSublayerRow(row, text)
    if row.kind == "relocate":
        return _SetRelocateRow(row, text)
    raise ValueParseError("%s rows are not editable" % row.kind)


def _SetArcItem(row, text):
    """
    Replace one arc of a list-op field with the arc `text` spells.

    The item is written back into the arm and at the index it already
    occupied, so retyping a reference's asset path does not also change
    which arcs it wins over.
    """
    item = ParseListOpItem(row.infoKey, row.keyword, text)
    before = InfoSnapshot.Capture(row.layer, row.specPath, row.infoKey)
    with Sdf.ChangeBlock():
        SetListOpItem(row.layer.GetPrimAtPath(row.specPath), row.infoKey,
                      row.arm, row.index, item)
    after = InfoSnapshot.Capture(row.layer, row.specPath, row.infoKey)
    return _Commit("Set %s" % row.key, row.layer, row.specPath,
                   before, after)


def _SetVariantSelection(row, text):
    """Select a different variant of one set, in this layer."""
    name = _Unquote(text)
    if not name:
        raise ValueParseError(
            "a variant selection cannot be empty; delete the row to stop "
            "this layer selecting %s" % row.key)
    before = InfoSnapshot.Capture(row.layer, row.specPath, "variantSelection")
    with Sdf.ChangeBlock():
        row.layer.GetPrimAtPath(row.specPath).variantSelections[
            row.key] = name
    after = InfoSnapshot.Capture(row.layer, row.specPath, "variantSelection")
    return _Commit("Select %s = %s" % (row.key, name), row.layer,
                   row.specPath, before, after)


def _SetSublayerRow(row, text):
    path, offset = ParseSublayer(text)
    before = SublayerSnapshot.Capture(row.layer)
    with Sdf.ChangeBlock():
        SetSublayer(row.layer, row.index, path, offset)
    after = SublayerSnapshot.Capture(row.layer)
    return _Commit("Set %s" % row.key, row.layer,
                   Sdf.Path.absoluteRootPath, before, after)


def _SetRelocateRow(row, text):
    source, target = ParseRelocate(text)
    before = RelocatesSnapshot.Capture(row.layer)
    with Sdf.ChangeBlock():
        SetRelocate(row.layer, row.index, source, target)
    after = RelocatesSnapshot.Capture(row.layer)
    return _Commit("Set %s" % row.key, row.layer,
                   Sdf.Path.absoluteRootPath, before, after)


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
    if row.kind in ("arcItem", "variantSelection", "sublayer", "relocate"):
        return _DeleteEntryRow(row)
    if row.kind == "layerInfo":
        raise ValueParseError(
            "%s is the layer's own composition; delete its entries one at "
            "a time" % row.key)
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


def _DeleteEntryRow(row):
    """Remove one entry of a list of arcs, leaving the rest alone."""
    if row.kind == "sublayer":
        before = SublayerSnapshot.Capture(row.layer)
        with Sdf.ChangeBlock():
            RemoveSublayer(row.layer, row.index)
        after = SublayerSnapshot.Capture(row.layer)
    elif row.kind == "relocate":
        before = RelocatesSnapshot.Capture(row.layer)
        with Sdf.ChangeBlock():
            RemoveRelocate(row.layer, row.index)
        after = RelocatesSnapshot.Capture(row.layer)
    else:
        before = InfoSnapshot.Capture(row.layer, row.specPath, row.infoKey)
        with Sdf.ChangeBlock():
            spec = row.layer.GetPrimAtPath(row.specPath)
            if row.kind == "variantSelection":
                del spec.variantSelections[row.key]
            else:
                RemoveListOpItem(spec, row.infoKey, row.arm, row.index)
        after = InfoSnapshot.Capture(row.layer, row.specPath, row.infoKey)
    return _Commit("Delete %s" % row.key, row.layer,
                   row.specPath, before, after)


def MoveRow(row, delta):
    """
    Move an arc `delta` places within its own list, strongest first.
    Returns an undoable Edit, or None when the row is already at the end.

    Order is what an arc field means: `prepend references = [@a@, @b@]`
    says a's opinions beat b's. Retyping the two rows to swap them is
    two edits with a half-swapped state in between, which composes as
    two arcs pointing at the same asset -- so moving is its own
    operation rather than something to be done by hand.
    """
    if not row.editable:
        raise ValueParseError("%s is not editable" % row.key)
    if row.kind == "sublayer":
        before = SublayerSnapshot.Capture(row.layer)
        with Sdf.ChangeBlock():
            moved = MoveSublayer(row.layer, row.index, delta)
        after = SublayerSnapshot.Capture(row.layer)
    elif row.kind == "arcItem":
        before = InfoSnapshot.Capture(row.layer, row.specPath, row.infoKey)
        with Sdf.ChangeBlock():
            moved = MoveListOpItem(row.layer.GetPrimAtPath(row.specPath),
                                   row.infoKey, row.arm, row.index, delta)
        after = InfoSnapshot.Capture(row.layer, row.specPath, row.infoKey)
    else:
        # A relocates map is unordered and a variant selection is one
        # value per set, so neither has a position to move.
        raise ValueParseError("%s rows have no order" % row.kind)
    if not moved:
        return None
    return _Commit("Move %s" % row.key, row.layer, row.specPath,
                   before, after)


# The row kinds that are one entry of a list rather than a whole
# opinion. Deleting one of these takes out that entry alone.
ENTRY_KINDS = ("arcItem", "variantSelection", "sublayer", "relocate")


def CanMove(row, delta):
    """
    Whether moving `row` by `delta` would do anything.

    Asked before the menu is built rather than discovered by calling
    MoveRow: an action that is offered, taken, and then reports that it
    did nothing is worse than one that is greyed out.

    Only an ordered list has this. A relocates map is unordered, and a
    variant selection is one value per set.
    """
    if not row.editable or row.kind not in ("arcItem", "sublayer"):
        return False
    return 0 <= row.index + delta < row.count


def CanDeleteRow(row, group):
    """
    Whether the panel should offer to remove what `row` shows.

    An entry of a list carries its own answer -- composition arcs are
    only editable in a local layer -- while an ordinary opinion goes by
    the layer, as it always has. The parent row of a layer's OWN arcs is
    not deletable at all: `subLayers` is not an opinion, it is the
    heading over a list, and its entries go one at a time.
    """
    if row.kind == "layerInfo":
        return False
    if row.kind in ENTRY_KINDS:
        return row.editable
    return bool(group is not None and group.editable)


def DeletePrimSpec(group):
    """
    Remove the prim's whole spec from one layer -- every opinion in that
    group at once. Returns an undoable Edit.
    """
    # The layer's OWN rows -- its sublayers and relocates -- sit at the
    # front of the group and their specPath is the pseudo-root. Taking
    # rows[0] blindly would hand the pseudo-root to _RemoveSpec and empty
    # the layer instead of removing one prim from it.
    primPath = next((row.primPath for row in group.rows
                     if row.kind in ("info", "attribute", "relationship")),
                    None)
    if primPath is None:
        raise KeyError("layer holds no spec for this prim")
    before = SpecCopySnapshot.Capture(group.layer, primPath)
    with Sdf.ChangeBlock():
        _RemoveSpec(group.layer, primPath)
    after = SpecCopySnapshot.Capture(group.layer, primPath)
    return _Commit("Delete %s opinions" % group.layer.GetDisplayName(),
                   group.layer, primPath, before, after)
