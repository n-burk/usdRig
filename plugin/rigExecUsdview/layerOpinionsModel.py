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
# SHORTHAND IS ACCEPTED ON THE WAY IN. Exact usda is the first thing
# tried, but a value column that only takes exact usda refuses what
# anyone would type: `ik` for a token, `bar.png` for an asset, `4 5 6`
# for a float3, `/Rig/Arm` for a target. When the exact text does not
# parse, it is rewritten ONCE into the usda it clearly means -- quoted,
# wrapped in @@ or <> or () or [] -- and that is parsed by the same
# import. Nothing is guessed that Sdf then cannot check: a bare prim
# path must be absolute and a bare layer must carry a file extension, so
# `not_a_path` and `not a reference at all` are still refused.
#
# PATHS READ AS WRITTEN. A relationship's targets, an attribute's
# connections and a prim's inherits and specializes are shown the way
# they are spelled -- `<../../Material/Cloth>` if that is what was typed
# or what the file says, `</Material/Cloth>` if that is -- and the panel's
# Save Layer writes them back the same way. Nothing is rewritten in
# either direction. Sdf keeps no spelling (it anchors a relative path
# the moment it is authored, and again on every read), so pathSpelling
# keeps it beside the layer; see that module for what it can and cannot
# keep. Relative text is anchored at the prim that owns it -- its path
# with any variant selection stripped (TargetAnchor), because a target
# path may not carry one -- which is the prim the usda reader anchors
# the spelling at when the file is read back.
#
# EVERY PATH TYPES RELATIVE. The same anchoring applies wherever a prim
# path is typed into the panel -- an inherit, a specialize, an internal
# reference, a relocate -- not only to targets (AnchorPathLiterals).
# Sdf refuses a relative path in most of those places, or anchors it to
# whatever prim the scratch parse happened to use, so the panel anchors
# it first, against the prim it will be authored on. Nothing checks the
# path exists: a target or class authored before the prim it names is
# legal, and is how a rig is often built. What IS refused is a relative
# path with no absolute form -- one that climbs above the root -- which
# Sdf would otherwise turn into an empty path without a word. Where usda
# cannot keep a relative spelling -- a reference, a relocate, anything
# inside a variant -- the row shows the absolute path it was anchored
# to, because that is what the file will say.
#
import os
import re

from pxr import Sdf

import pathSpelling
import rigExecUndo


class ValueParseError(Exception):
    """
    The text in the value editor is not a value of the row's type.

    The message is one line fit for a status bar; Sdf's own parser
    diagnostics, which run to several lines of source locations, are
    kept in `detail` for a tooltip.
    """

    def __init__(self, message, detail=""):
        super(ValueParseError, self).__init__(message)
        self.detail = detail


class _PathAnchorError(ValueParseError):
    """
    A relative path that has no absolute form where it was typed.

    Its own message already says exactly what is wrong, so _FirstParse
    reports it as is rather than as the generic "not a ...".
    """


def _FirstParse(parse, candidates, message):
    """
    The first of `candidates` that `parse` accepts.

    Tried in order and each only once: the exact text first, so any
    valid usda keeps meaning exactly what it says, and the shorthand
    rewrite after it. Raises one ValueParseError carrying the exact
    text's diagnostics -- the rewrite's would describe text the user
    never typed -- unless a path in it could not be anchored, which is
    the more useful thing to say.
    """
    first = None
    anchorError = None
    seen = set()
    for candidate in candidates:
        if candidate is None or candidate in seen:
            continue
        seen.add(candidate)
        try:
            return parse(candidate)
        except _PathAnchorError as error:
            anchorError = anchorError or error
        except ValueParseError as error:
            if first is None:
                first = error
    if anchorError is not None:
        raise anchorError
    raise ValueParseError(message, first.detail if first else "")


def _SplitTopLevel(text):
    """
    `text` split at the commas that are not inside (), [], quotes or
    an @asset@ -- so `(1, 2), (3, 4)` is two items, not four.
    """
    items, depth, quote, start = [], 0, None, 0
    for i, char in enumerate(text):
        if quote:
            if char == quote and text[i - 1] != "\\":
                quote = None
        elif char in "\"'@":
            quote = char
        elif char in "([":
            depth += 1
        elif char in ")]":
            depth -= 1
        elif char == "," and depth == 0:
            items.append(text[start:i])
            start = i + 1
    items.append(text[start:])
    return [item.strip() for item in items if item.strip()]


def _Quote(text):
    """`text` as a usda string literal."""
    return '"%s"' % (text.replace("\\", "\\\\").replace('"', '\\"')
                     .replace("\n", "\\n"))


def _AssetLiteral(text):
    """`text` as a usda asset literal: @...@, or @@@...@@@ if it has an @."""
    return ("@@@%s@@@" if "@" in text else "@%s@") % text


def _LooksLikeLayerPath(text):
    """
    Whether bare `text` is plainly a layer's path: a file extension at
    the end (Sdf picks the file format from it, so every layer has one),
    optionally followed by Sdf's format arguments.
    """
    return bool(re.search(r"\.\w+(:SDF_FORMAT_ARGS:.*)?$", text))


def _BareNumbers(text):
    """`1, 2, 3` or `1 2 3` as a list of the numbers' texts."""
    return [n for n in re.split(r"[\s,]+", text.strip()) if n]


def _ScalarShorthand(typeName, text):
    """The usda one scalar of `typeName` is plainly meant by `text`."""
    text = text.strip()
    default = typeName.defaultValue
    if isinstance(default, bool):
        return {"true": "1", "yes": "1", "on": "1",
                "false": "0", "no": "0", "off": "0"}.get(text.lower(), text)
    if isinstance(default, str):
        return text if text[:1] in "\"'" else _Quote(text)
    if isinstance(default, Sdf.AssetPath):
        return text if text.startswith("@") else _AssetLiteral(text)
    if text.startswith("("):
        return text
    dimension = getattr(default, "dimension", None)
    if isinstance(dimension, tuple):
        # A matrix typed flat, row by row.
        rows, cols = dimension
        numbers = _BareNumbers(text)
        if len(numbers) != rows * cols:
            return text
        return "(%s)" % ", ".join(
            "(%s)" % ", ".join(numbers[r * cols:(r + 1) * cols])
            for r in range(rows))
    if isinstance(dimension, int) or hasattr(default, "imaginary"):
        # A vector, color or quaternion typed without its parentheses.
        return "(%s)" % ", ".join(_BareNumbers(text))
    return text


def _ValueShorthand(typeName, text):
    """The usda a value of `typeName` is plainly meant by `text`."""
    if not typeName.isArray:
        return _ScalarShorthand(typeName, text)
    body = text.strip()
    if body.startswith("[") and body.endswith("]"):
        body = body[1:-1]
    return "[%s]" % ", ".join(
        _ScalarShorthand(typeName.scalarType, item)
        for item in _SplitTopLevel(body))


def AnchoredAssetPath(layer, assetPath):
    """
    `assetPath` as an arc authored into `layer` should carry it:
    relative to `layer`'s own file (`./rig.usda`, `../shared/hand.usda`)
    whenever it can be.

    A file browser hands back an absolute path, and authoring that
    verbatim pins the arc to one machine's disk -- the asset stops
    composing the moment the directory is moved, copied or opened from
    another checkout. A relative path moves with the layers it joins.

    Spelled with a leading `./` rather than bare: `rig.usda` is a
    SEARCH path to Ar, which may resolve somewhere other than beside the
    layer, while `./rig.usda` is anchored to it. The repo's own layers
    use the same spelling.

    Left as given when there is nothing to anchor to (an anonymous or
    unsaved layer), when the path is not an absolute file path (already
    relative, a search path, a URI, an anonymous identifier), or when no
    relative form exists (another drive on Windows).
    """
    assetPath = (assetPath or "").strip()
    if not assetPath or layer is None or layer.anonymous:
        return assetPath
    anchor = layer.realPath
    if not anchor or "://" in assetPath or not os.path.isabs(assetPath):
        return assetPath
    try:
        relative = os.path.relpath(os.path.normpath(assetPath),
                                   os.path.dirname(os.path.normpath(anchor)))
    except ValueError:
        return assetPath
    relative = relative.replace(os.sep, "/")
    if relative.startswith("../"):
        return relative
    return "./" + relative


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


# How much of a value a row formats. A mesh's points are a few hundred
# thousand elements, and formatting them whole costs twice: usda export
# of megabytes of text on every rebuild, and then Qt laying that text
# out on every paint and every column measure -- seconds per frame for
# a line nobody can read. Past these sizes a row shows a SUMMARY, the
# element count and the first few items, and is not edited inline; the
# full text is still one right-click away (FullValueText).
MAX_INLINE_ITEMS = 1000
MAX_INLINE_CHARS = 4000
_PREVIEW_ITEMS = 4
_PREVIEW_CHARS = 200


def SummarizedValueText(value, typeName=None):
    """
    (text, summarized) for a row's value column.

    An array past MAX_INLINE_ITEMS is never formatted whole -- only its
    first _PREVIEW_ITEMS are, which is what makes a heavy prim cheap.
    Any other text past MAX_INLINE_CHARS is clipped after formatting.
    """
    if value is None:
        return "", False
    arrayType = typeName
    if arrayType is None and hasattr(value, "__len__") \
            and not isinstance(value, str):
        arrayType = Sdf.GetValueTypeNameForValue(value) or None
    if (arrayType is not None and arrayType.isArray
            and len(value) > MAX_INLINE_ITEMS):
        head = FormatValue(value[:_PREVIEW_ITEMS], arrayType)
        return ("%s[%s]  %s, ...]" % (
            _TypeAlias(arrayType.scalarType), "{:,}".format(len(value)),
            head.rstrip().rstrip("]")), True)
    return _Clipped(FormatValue(value, typeName))


def _Clipped(text):
    if len(text) <= MAX_INLINE_CHARS:
        return text, False
    return text[:_PREVIEW_CHARS] + " ...", True


def FullValueText(row):
    """
    The whole value a summarized row stands for, formatted on demand --
    for "Copy Full Value". Slow for a heavy value by nature, which is
    why it is only ever asked for, never shown.
    """
    if row.kind == "attribute":
        spec = row.layer.GetAttributeAtPath(row.specPath)
        return FormatValue(spec.default, row.typeName) if spec is not None \
            and spec.HasDefaultValue() else ""
    if row.kind == "relationship":
        spec = row.layer.GetRelationshipAtPath(row.specPath)
        return _FormatTargets(spec.targetPathList, _Speller(
            pathSpelling.Spellings(row.layer), row.specPath,
            "targetPaths")) if spec is not None else ""
    if row.kind == "connection":
        spec = row.layer.GetAttributeAtPath(row.specPath)
        return _FormatTargets(spec.connectionPathList, _Speller(
            pathSpelling.Spellings(row.layer), row.specPath,
            "connectionPaths")) if spec is not None else ""
    if row.kind == "info":
        spec = row.layer.GetPrimAtPath(row.specPath)
        return FormatValue(spec.GetInfo(row.key)) if spec is not None \
            else ""
    return row.valueText


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
    `text` parsed as a value of `typeName`, via usda syntax -- exactly
    as typed, or failing that as the shorthand it plainly means (see the
    header: `ik` for a token, `4 5 6` for a float3).

    Deliberately a layer import and never eval(): the value field takes
    whatever a user types, and typing a python expression into it must
    produce a parse error, not execute.
    """
    return _FirstParse(
        lambda candidate: _ParseValueText(typeName, candidate),
        [text, _ValueShorthand(typeName, text)],
        "not a valid %s: %r" % (_TypeAlias(typeName), text))


def _ParseValueText(typeName, text):
    source = '#usda 1.0\ndef "P"\n{\n    %s v = %s\n}\n' % (
        _TypeAlias(typeName), text)
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a valid %s" % typeName)
    except ValueParseError:
        raise
    except Exception as error:
        raise ValueParseError("not a valid %s" % typeName, str(error))
    attr = layer.GetAttributeAtPath("/P.v")
    if attr is None or not attr.HasDefaultValue():
        raise ValueParseError("not a valid %s" % typeName)
    return attr.default


def ParseTargets(text, owner=None):
    """
    `text` parsed as a relationship target list (or a connection list,
    which is spelled the same), anchored at the prim path `owner`.

    Same usda round trip as ParseValue -- the paths are validated by
    Sdf's own parser, so `[ not_a_path ]` is a parse error rather than
    a relationship that silently targets nothing. The shorthand is bare
    ABSOLUTE paths, with or without the brackets: `/Rig/A, /Rig/B`,
    or explicitly relative ones: `../B`, `./C`, `.x`. A bare relative
    name is not rewritten, because `not_a_path` is one.

    Relative paths -- bare or in <> -- resolve against `owner`, the
    prim the relationship or attribute belongs to, which is what they
    mean in a usda file. Returned absolute, as Sdf stores them.
    """
    return [path for path, _ in ParseSpelledTargets(text, owner)]


def ParseSpelledTargets(text, owner=None):
    """
    ParseTargets, with each path paired with the text it was typed as:
    [(path, spelling)] -- `(/Rig/Wrist, "../Wrist")` for a relative one,
    `(/Rig/Wrist, "/Rig/Wrist")` for an absolute one. The spelling is
    what the row goes on showing, and what a save writes.
    """
    def _Parse(candidate):
        spelled = {}
        paths = _ParseTargetsText(
            AnchorPathLiterals(candidate, owner, spelled), owner)
        return [(path, spelled.get(path, str(path))) for path in paths]

    return _FirstParse(_Parse, [text, _TargetsShorthand(text)],
                       "not a target list: %r" % text)


def _TargetsShorthand(text):
    body = text.strip()
    if body.startswith("[") and body.endswith("]"):
        body = body[1:-1]
    items = [_TargetShorthand(item)
             for item in re.split(r"[\s,]+", body.strip()) if item]
    return "[ %s ]" % ", ".join(items)


def _TargetShorthand(item):
    """
    One bare target in <>. `./Child` is how a file path says "child" but
    not how Sdf does -- to Sdf that is just `Child` -- so both spellings
    are taken.
    """
    if item.startswith("./") and len(item) > 2:
        return "<%s>" % item[2:]
    if item.startswith(("/", ".")):
        return "<%s>" % item
    return item


def TargetAnchor(specPath):
    """
    The prim a relative target or connection on `specPath` resolves
    against: the owning prim, with any variant selection stripped.

    Stripped because a target may not name a variant -- Sdf refuses
    `</Asset{look=a}Material>` even from its own usda parser -- so an
    opinion authored INSIDE a variant still targets `/Asset/Material`,
    and `../../Material` from `/Asset{look=a}Geo/Body` has to mean that.
    """
    return Sdf.Path(specPath).GetPrimPath().StripAllVariantSelections()


def AnchorPath(text, owner):
    """
    The path `text` spells, made absolute against the prim `owner`:
    `../Wrist` typed on /Rig/Arm/IK is /Rig/Arm/Wrist, `.x` is
    /Rig/Arm/IK.x, and an absolute path comes back as it is. `./Child`
    is taken for `Child`, as everywhere else in the panel.

    Raises ValueParseError for text that is not a path at all, and for
    a relative path with no absolute form -- one that climbs above the
    root, like `../../../X` from /Rig/Arm. Sdf answers that with an
    EMPTY path, and an empty path is not an error once authored: a
    relocate to <> removes its prim.
    """
    text = text.strip()
    if text.startswith("./") and len(text) > 2:
        text = text[2:]
    if not Sdf.Path.IsValidPathString(text):
        raise ValueParseError("%r is not a path" % text)
    path = Sdf.Path(text)
    if path.IsAbsolutePath():
        return path
    anchor = TargetAnchor(owner)
    absolute = path.MakeAbsolutePath(anchor)
    if absolute.isEmpty:
        raise _PathAnchorError(
            "<%s> climbs above the root from %s; there is no such path"
            % (text, anchor))
    return absolute


def AnchorPathLiterals(text, owner, spelled=None):
    """
    usda `text` with every relative <path> in it anchored at the prim
    `owner` (AnchorPath), ready for a scratch parse that knows nothing
    about where it will be authored.

    Needed wherever the panel parses a path. Sdf's own handling of a
    relative path depends on the field: a target is anchored to the
    prim the scratch parse used -- a stand-in, not the real owner -- an
    inherit likewise, a reference prim path or a relocate is refused or
    silently emptied. Anchoring first makes them all mean one thing.

    The one relative path left alone is the prim path of an EXTERNAL
    arc (`@./hand.usda@<../Hand>`): it names a prim in another layer,
    where this prim is nothing to be relative to, so it is refused with
    that reason. `text` comes back unchanged when there is no `owner`.

    `spelled`, when given, is filled with {absolute path: the relative
    text it was typed as}, which is what pathSpelling.Record keeps.
    """
    if owner is None:
        return text
    out = []
    end = 0
    afterAsset = False
    for match in pathSpelling.PATH_LITERALS.finditer(text):
        literal = match.group(0)
        between = text[end:match.start()]
        out.append(between)
        if literal.startswith("<"):
            body = literal[1:-1].strip()
            external = afterAsset and not between.strip()
            if body and not body.startswith("/") \
                    and Sdf.Path.IsValidPathString(
                        body[2:] if body.startswith("./") else body):
                if external:
                    raise _PathAnchorError(
                        "%s names a prim in another layer, so it must be "
                        "absolute, like </Hand>" % literal)
                absolute = AnchorPath(body, owner)
                if spelled is not None:
                    spelled[absolute] = (body[2:] if body.startswith("./")
                                         else body)
                literal = "<%s>" % absolute
        out.append(literal)
        afterAsset = literal.startswith("@")
        end = match.end()
    out.append(text[end:])
    return "".join(out)


def _OwnerSource(owner, body):
    """
    usda holding `body` inside `over`s spelling out the prim `owner`,
    so relative paths in it anchor exactly where they will be authored.
    Only a missing owner falls back to a stand-in prim, /P; a variant
    selection is stripped (TargetAnchor), never replaced.
    """
    owner = TargetAnchor(owner) if owner else Sdf.Path("/P")
    if not owner.IsAbsolutePath() or not owner.IsPrimPath():
        owner = Sdf.Path("/P")
    names = [prefix.name for prefix in owner.GetPrefixes()]
    head = "".join('over "%s"\n{\n' % name for name in names)
    return owner, "#usda 1.0\n%s%s\n%s" % (head, body, "}\n" * len(names))


def _ParseTargetsText(text, owner=None):
    owner, source = _OwnerSource(owner, "    rel r = %s" % text)
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a target list")
    except ValueParseError:
        raise
    except Exception as error:
        raise ValueParseError("not a target list", str(error))
    rel = layer.GetRelationshipAtPath(owner.AppendProperty("r"))
    if rel is None:
        raise ValueParseError("not a target list")
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


# List-op fields: the shape every composition arc on a prim spec has.

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


def ParseListOpItem(key, keyword, text, current=None, owner=None,
                    spelled=None):
    """
    `text` parsed as ONE item of the list-op field written `keyword`.

    The mirror of FormatListOpItem, and a layer import for the same
    reason ParseValue is one: the value field takes whatever is typed,
    so a python expression must be a parse error rather than something
    that runs.

    A text holding two items is refused rather than taking the first.
    The row addresses one position in one arm; writing two arcs into it
    would silently drop one of them.

    `current` is the item the row holds now. It decides the shorthand:
    a bare layer path for a reference or payload, a bare `/Prim` or
    `../Prim` for a path, a bare word for a string or token list.

    `owner` is the prim the field is on. A relative prim path -- an
    inherit of `<../_class_Arm>`, an internal reference to `<../Leg>` --
    is anchored there (AnchorPathLiterals), not at the scratch prim the
    text is parsed on.

    `spelled`, when given, is filled with {absolute path: relative text}
    for the relative path the item was typed with, if any.
    """
    found = {}

    def _Parse(candidate):
        found.clear()
        return _ParseListOpText(
            key, keyword, AnchorPathLiterals(candidate, owner, found))

    items = _FirstParse(
        _Parse, [text, _ListOpItemShorthand(keyword, text, current)],
        "not a %s: %r" % (keyword, text))
    if len(items) != 1:
        raise ValueParseError(
            "%r is %d items; this row is one %s"
            % (text, len(items), keyword))
    if spelled is not None:
        spelled.update(found)
    return items[0]


def _ParseListOpText(key, keyword, text):
    source = ('#usda 1.0\ndef "P" (\n    %s = %s\n)\n{\n}\n'
              % (keyword, text))
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a %s" % keyword)
    except ValueParseError:
        raise
    except Exception as error:
        raise ValueParseError("not a %s" % keyword, str(error))
    spec = layer.GetPrimAtPath("/P")
    value = spec.GetInfo(key) if spec is not None and spec.HasInfo(key) \
        else None
    if value is None or not IsListOp(value):
        raise ValueParseError("not a %s" % keyword)
    return list(value.explicitItems)


# `layer.usda</Prim> (offset = 5)`, every part but the first optional.
_BARE_ARC = re.compile(
    r"^(?P<asset>[^<(]*?)\s*(?P<prim><[^>]*>)?\s*(?P<offset>\(.*\))?\s*$")


def _ListOpItemShorthand(keyword, text, current):
    text = text.strip()
    if not text or text[0] in "@<\"'[":
        return text
    if isinstance(current, (Sdf.Reference, Sdf.Payload)) or (
            current is None and keyword in ("references", "payload")):
        return _ArcShorthand(text)
    if isinstance(current, Sdf.Path) or (
            current is None and keyword in ("inherits", "specializes")):
        return _TargetShorthand(text)
    if isinstance(current, str):
        return _Quote(text)
    return text


def _ArcShorthand(text):
    """
    A reference or payload typed without its @@: `./hand.usda</Hand>`
    is external, and a lone `/Class` or `../Class` is internal. Anything
    else is left to fail, so a sentence is not taken for a file name.

    A layer is told from a relative prim path by its file extension,
    which every layer has (_LooksLikeLayerPath): `../hand.usda` is a
    file, `../Hand` a sibling prim. A bare `./Name` is not taken for a
    prim: `./` is how an arc spells a FILE beside its layer, and the
    prim it would name -- this prim's own child -- is a composition
    cycle as an arc target anyway.
    """
    match = _BARE_ARC.match(text)
    if match is None:
        return text
    asset = match.group("asset").strip()
    prim = match.group("prim") or ""
    offset = match.group("offset") or ""
    if _LooksLikeLayerPath(asset):
        return _AssetLiteral(asset) + prim + (" " + offset if offset else "")
    if asset.startswith(("/", "../")) and not prim:
        return "<%s>%s" % (asset, " " + offset if offset else "")
    return text


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


# The two arcs that are LAYER metadata: sublayers and relocates.
#
# Neither is a field on a prim spec, so neither can go through the
# list-op path above -- but both round-trip through usda the same way,
# by exporting a scratch layer holding just the one entry and importing
# the text back.

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
    """
    (path, offset) from one `subLayers` entry -- `@./a.usda@`, or the
    bare `./a.usda` shorthand when it plainly names a layer.
    """
    return _FirstParse(_ParseSublayerText,
                       [text, _SublayerShorthand(text)],
                       "not a sublayer: %r" % text)


def _SublayerShorthand(text):
    match = _BARE_ARC.match(text.strip())
    if match is None or match.group("prim"):
        return text
    asset = match.group("asset").strip()
    if asset.startswith("@") or not _LooksLikeLayerPath(asset):
        return text
    offset = match.group("offset")
    return _AssetLiteral(asset) + (" " + offset if offset else "")


def _ParseSublayerText(text):
    source = '#usda 1.0\n(\n    subLayers = [\n        %s\n    ]\n)\n' % text
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a sublayer")
    except ValueParseError:
        raise
    except Exception as error:
        raise ValueParseError("not a sublayer", str(error))
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


def ParseRelocate(text, owner=None):
    """
    (source, target) from one `relocates` entry -- `</A>: </B>`, or the
    bare `/A: /B` shorthand.

    Relocates are layer metadata, and usda anchors a relative path in
    them at the layer's root -- where `<../B>` is no path at all and
    parses to an EMPTY target, which is a relocate that deletes its
    prim. So a relative path is anchored at `owner`, the prim the panel
    is showing, like every other path typed into it.
    """
    return _FirstParse(
        lambda candidate: _ParseRelocateText(
            AnchorPathLiterals(candidate, owner)),
        [text, _RelocateShorthand(text)],
        "not a relocate: %r" % text)


# One side of a relocate: a path in <>, or a bare one that plainly is a
# path -- absolute, or explicitly relative with a leading dot.
_RELOCATE_SIDE = r"(<[^<>]*>|[/.][^<>:\s]*)"


def _RelocateShorthand(text):
    match = re.match(r"^\s*%s\s*:\s*%s\s*$" % (_RELOCATE_SIDE,
                                                _RELOCATE_SIDE), text)
    if match is None:
        return text
    return "%s: %s" % tuple(_TargetShorthand(side)
                            for side in match.groups())


def _ParseRelocateText(text):
    source = '#usda 1.0\n(\n    relocates = {\n        %s\n    }\n)\n' % text
    layer = _ScratchLayer()
    try:
        if not layer.ImportFromString(source):
            raise ValueParseError("not a relocate")
    except ValueParseError:
        raise
    except Exception as error:
        raise ValueParseError("not a relocate", str(error))
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
                 item=None, count=0, summarized=False, anchor=None):
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
        # The value column holds a summary, not the value: a heavy array
        # or a very long text. Such a row is not edited inline -- the
        # editor would open on the summary -- see SummarizedValueText.
        self.summarized = summarized
        self.children = []
        # Set only on a layer's own rows, whose spec path is the
        # pseudo-root: see `anchor`.
        self._anchor = Sdf.Path(anchor) if anchor else None

    @property
    def primPath(self):
        return self.specPath.GetPrimPath()

    @property
    def anchor(self):
        """
        The prim a relative path typed into this row resolves against:
        the prim the opinion is on (TargetAnchor), or for a layer's own
        relocates -- which are on no prim -- the prim the panel shows.
        """
        if self._anchor is not None:
            return self._anchor
        return TargetAnchor(self.specPath)

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
        # How the layer spells its paths (pathSpelling.Spellings), read
        # once per rebuild -- the file half is a stat and a dict lookup
        # once it has been read.
        self.spellings = {}

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
            group.spellings = pathSpelling.Spellings(layer)
            byLayer[layer] = group
            groups.append(group)
        group.rows.extend(_RowsForSpec(spec, group, winners))
    for group in groups:
        if group.local:
            group.rows[:0] = _LayerArcRows(group, prim.GetPath())
    return groups


def _RowsForSpec(spec, group, winners):
    rows = []
    for key in sorted(spec.ListInfoKeys()):
        value = spec.GetInfo(key)
        listOp = IsListOp(value)
        text, summarized = (("", False) if listOp
                            else SummarizedValueText(value))
        row = OpinionRow(
            layer=spec.layer,
            specPath=spec.path,
            kind="info",
            key=key,
            valueText=text,
            editable=(group.editable and IsInfoEditable(key, value)
                      and not summarized),
            winning=_InfoWinning(winners, key, value),
            infoValue=value,
            summarized=summarized)
        if listOp:
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
            text, summarized = (SummarizedValueText(prop.default, typeName)
                                if prop.HasDefaultValue() else ("", False))
            editable = group.editable and not summarized
            if prop.HasInfo("connectionPaths"):
                # Its own row, not folded into the value: a connection
                # and a default are separate opinions, removed and
                # overridden separately.
                connText, connSummarized = _SummarizedTargets(
                    prop.connectionPathList,
                    _Speller(group.spellings, prop.path, "connectionPaths"))
                rows.append(OpinionRow(
                    layer=spec.layer,
                    specPath=prop.path,
                    kind="connection",
                    key="%s.connect" % prop.name,
                    valueText=connText,
                    editable=group.editable and not connSummarized,
                    winning=_ClaimWinner(winners, "connection", prop.name),
                    summarized=connSummarized))
                if not prop.HasDefaultValue():
                    continue
        else:
            kind = "relationship"
            text, summarized = _SummarizedTargets(
                prop.targetPathList,
                _Speller(group.spellings, prop.path, "targetPaths"))
            typeName = None
            editable = group.editable and not summarized
        rows.append(OpinionRow(
            layer=spec.layer,
            specPath=prop.path,
            kind=kind,
            key=prop.name,
            valueText=text,
            editable=editable,
            winning=_ClaimWinner(winners, kind, prop.name),
            typeName=typeName,
            summarized=summarized))
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
        if key in pathSpelling.FIELDS:
            # An inherit or specialize: shown as spelled, like a target.
            text = "<%s>" % pathSpelling.Spell(group.spellings, spec.path,
                                               key, item)
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


def _LayerArcRows(group, primPath):
    """
    The layer's own composition: its sublayers and its relocates.

    Returned as two parent rows with a child each, or nothing at all
    when the layer has neither -- most layers have no relocates and only
    the root layer has sublayers, so the common case adds no rows.

    `primPath` is the prim the panel is showing, which a relative path
    typed into a relocate row is anchored at.
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
                item=(source, target), count=len(relocates),
                anchor=primPath))
        rows.append(parent)
    return rows


def _Speller(spellings, specPath, field):
    """`path` -> its text, as `spellings` spells it in one field."""
    return lambda path: pathSpelling.Spell(spellings, specPath, field, path)


def _FormatTargets(pathList, spell):
    """
    A target or connection list as usda would write it, each path as
    `spell` spells it: `[ <../B>, </Rig/C> ]` -- relative where it was
    typed or saved relative, absolute where it was absolute. The editor
    opens on this text and accepts it back unchanged.
    """
    items = list(pathList.explicitItems) or list(pathList.prependedItems)
    return _SpelledPathList(items, spell)


def _SummarizedTargets(pathList, spell):
    """(text, summarized) for a target or connection list; see
    SummarizedValueText -- a skinned mesh can carry thousands."""
    items = list(pathList.explicitItems) or list(pathList.prependedItems)
    if len(items) <= MAX_INLINE_ITEMS:
        return _Clipped(_SpelledPathList(items, spell))
    head = _SpelledPathList(items[:_PREVIEW_ITEMS], spell)
    return ("%s paths  %s, ... ]" % ("{:,}".format(len(items)),
                                     head.rstrip().rstrip("]").rstrip()),
            True)


def _SpelledPathList(paths, spell):
    return "[ %s ]" % ", ".join("<%s>" % spell(path) for path in paths)


def FormatPathList(paths, owner):
    """`paths` as a usda list, relative to `owner` where they can be."""
    return _SpelledPathList(paths, lambda path: PreferredPath(path, owner))


def PreferredPath(path, owner):
    """
    `path` as the target picker OFFERS it: relative to the prim `owner`
    -- `<../../Material/Cloth>` says what the rig means, where
    `</Asset/Material/Cloth>` says where it happens to sit today. What
    is picked is then kept as picked (pathSpelling); a path already
    authored is shown the way it is spelled, not this way.
    """
    path = Sdf.Path(path)
    owner = TargetAnchor(owner) if owner else None
    if (owner is None or not path.IsAbsolutePath()
            or not owner.IsAbsolutePath() or owner.IsAbsoluteRootPath()):
        return path
    return path.MakeRelativePath(owner)


def _RootPrim(path):
    prefixes = path.GetPrimPath().GetPrefixes()
    return prefixes[0] if prefixes else path


def RelativizePathListText(text, owner):
    """
    `text` rewritten with every path in PreferredPath form -- the "make
    these relative" helper. The text is parsed first, so a list that
    would be refused comes back refused rather than half rewritten.
    """
    return FormatPathList(ParseTargets(text, owner), owner)


def TargetCandidates(stage, owner, properties=False, limit=200,
                     budget=5000):
    """
    Paths a relationship or connection on the stage prim `owner` would
    plausibly target, in PreferredPath form and nearest first.

    Searched in rings outward from the owner: its parent's subtree
    (siblings, children, nephews), then the grandparent's, and so on up
    to the whole stage -- so `../Wrist` comes before `../../Material/
    Cloth`, which comes before a prim in another asset, and a
    `/Material` scope at the root is still reachable. `budget` bounds
    the prims visited, which is what keeps the picker instant on a
    stage holding a whole shot: the far rings are the ones cut.

    With `properties`, the authored ATTRIBUTES of those prims instead --
    a connection targets a property, not a prim.
    """
    owner = Sdf.Path(owner)
    if stage is None or not owner.IsAbsolutePath() \
            or not stage.GetPrimAtPath(owner):
        return []
    from pxr import Usd
    found = []
    visited = 0
    ring = owner.GetParentPath()
    done = owner
    while visited < budget:
        top = stage.GetPrimAtPath(ring)
        if not top:
            break
        walk = iter(Usd.PrimRange(top))
        for prim in walk:
            path = prim.GetPath()
            if done != owner and path == done:
                # The previous ring, already searched: skip its subtree.
                walk.PruneChildren()
                continue
            visited += 1
            if visited > budget:
                break
            if properties:
                found.extend(attr.GetPath()
                             for attr in prim.GetAuthoredAttributes())
            elif path != owner and not path.IsAbsoluteRootPath():
                found.append(path)
        if ring.IsAbsoluteRootPath():
            break
        done = ring
        ring = ring.GetParentPath()

    def _Distance(path):
        relative = str(PreferredPath(path, owner))
        return (relative.count(".."), relative.count("/"), relative)

    ranked = sorted(found, key=_Distance)[:limit]
    return ["<%s>" % PreferredPath(path, owner) for path in ranked]


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


# Snapshots. Each restores exactly one opinion, so an undo entry is as
# narrow as the edit that made it.

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


# Moved to rigExecUndo, where the gizmo and the panels that build rig
# structure can reach it too. Kept under its old name here because every
# operation below reads SpecCopySnapshot.Capture, and the name says what
# it is: the whole spec, copied.
SpecCopySnapshot = rigExecUndo.SpecSnapshot
_EnsureAncestors = rigExecUndo._EnsureAncestors
_SiblingNames = rigExecUndo._SiblingNames
_FollowingSiblingNames = rigExecUndo._FollowingSiblingNames
_MoveToEnd = rigExecUndo._MoveToEnd
_RemoveSpec = rigExecUndo._RemoveSpec


def _Commit(label, layer, specPath, before, after):
    return rigExecUndo.Edit(
        label, [rigExecUndo.EditEntry(layer, specPath, before, after)])


# Operations. Each applies the change and returns the undoable Edit.

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
    if row.kind == "connection":
        return _SetConnections(row, text)
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
    spelled = {}
    item = _Anchored(row.layer, ParseListOpItem(
        row.infoKey, row.keyword, text, row.item, row.anchor, spelled))
    if row.infoKey in pathSpelling.FIELDS:
        # An inherit or specialize keeps its spelling, as a target does.
        pathSpelling.Record(row.layer, row.specPath, row.infoKey,
                            [(item, spelled.get(item, str(item)))])
    before = InfoSnapshot.Capture(row.layer, row.specPath, row.infoKey)
    with Sdf.ChangeBlock():
        SetListOpItem(row.layer.GetPrimAtPath(row.specPath), row.infoKey,
                      row.arm, row.index, item)
    after = InfoSnapshot.Capture(row.layer, row.specPath, row.infoKey)
    return _Commit("Set %s" % row.key, row.layer, row.specPath,
                   before, after)


def _Anchored(layer, item):
    """
    A reference or payload with its asset path made relative to the
    layer it is written into (AnchoredAssetPath); any other item as is.
    """
    if isinstance(item, Sdf.Reference) and item.assetPath:
        return Sdf.Reference(AnchoredAssetPath(layer, item.assetPath),
                             item.primPath, item.layerOffset,
                             item.customData)
    if isinstance(item, Sdf.Payload) and item.assetPath:
        return Sdf.Payload(AnchoredAssetPath(layer, item.assetPath),
                           item.primPath, item.layerOffset)
    return item


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
    path = AnchoredAssetPath(row.layer, path)
    before = SublayerSnapshot.Capture(row.layer)
    with Sdf.ChangeBlock():
        SetSublayer(row.layer, row.index, path, offset)
    after = SublayerSnapshot.Capture(row.layer)
    return _Commit("Set %s" % row.key, row.layer,
                   Sdf.Path.absoluteRootPath, before, after)


def _SetRelocateRow(row, text):
    source, target = ParseRelocate(text, row.anchor)
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
    spelled = ParseSpelledTargets(text, row.anchor)
    # Recorded BEFORE authoring: the notice the edit sends can rebuild
    # the panel the moment the change block closes, and the rebuilt row
    # has to find the spelling already there.
    pathSpelling.Record(row.layer, row.specPath, "targetPaths", spelled)
    before = SpecCopySnapshot.Capture(row.layer, row.specPath)
    with Sdf.ChangeBlock():
        spec = row.layer.GetRelationshipAtPath(row.specPath)
        spec.targetPathList.ClearEdits()
        spec.targetPathList.explicitItems = [path for path, _ in spelled]
    after = SpecCopySnapshot.Capture(row.layer, row.specPath)
    return _Commit("Set %s" % row.key, row.layer, row.specPath, before, after)


def _SetConnections(row, text):
    spelled = ParseSpelledTargets(text, row.anchor)
    pathSpelling.Record(row.layer, row.specPath, "connectionPaths", spelled)
    before = SpecCopySnapshot.Capture(row.layer, row.specPath)
    with Sdf.ChangeBlock():
        spec = row.layer.GetAttributeAtPath(row.specPath)
        spec.connectionPathList.ClearEdits()
        spec.connectionPathList.explicitItems = [path for path, _ in spelled]
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
    if row.kind == "connection":
        # The connection only: the attribute's default is a separate
        # opinion and stays.
        before = SpecCopySnapshot.Capture(row.layer, row.specPath)
        with Sdf.ChangeBlock():
            spec = row.layer.GetAttributeAtPath(row.specPath)
            if spec is not None:
                spec.connectionPathList.ClearEdits()
        after = SpecCopySnapshot.Capture(row.layer, row.specPath)
        return _Commit("Delete %s" % row.key, row.layer, row.specPath,
                       before, after)
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
