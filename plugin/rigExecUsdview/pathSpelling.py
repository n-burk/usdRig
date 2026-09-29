#
# How each path in a layer is SPELLED, and a save that keeps it.
#
# USD keeps no spelling. A relative target -- `<../../Material/Cloth>` --
# is anchored the moment it is authored (Sdf's path proxies) and again on
# every read (the usda parser), so a layer only ever holds the absolute
# path, and saving it writes that. What was typed is gone.
#
# The Layer Opinions panel is WYSIWYG about paths: what is typed is what
# is shown, and what is shown is what a save writes -- relative stays
# relative, absolute stays absolute, and nothing is rewritten either way.
# So the spelling is kept here, beside the layer, from two sources:
#
#   * what the panel authored this session (Record), and
#   * what the layer's own .usda file says, read back out of it
#     (SpellingsInText) -- so a relative path survives a reload,
#
# and SaveLayer writes it back into the file.
#
# Only where the usda reader takes a relative path back to the same
# target: a relationship's targets, an attribute's connections, a prim's
# inherits and specializes -- each anchored at the prim that holds it --
# and never inside a variant, where the reader anchors into the variant's
# namespace and then refuses the result. A reference's prim path must be
# absolute to the reader, and a relocate is anchored at the layer root,
# so those are always absolute, and the panel shows them that way.
#
# Nothing here parses usda by hand. Both directions go through Sdf, with
# PLACEHOLDER paths standing in for the relative ones: a save writes each
# spelled path as a unique absolute placeholder, exports, and swaps the
# placeholders' text for the spelling; a read swaps every relative path
# in the file's text for a placeholder, imports, and sees which field of
# which spec each one landed in. A placeholder that lands anywhere else --
# in a string, a comment, a reference -- is never harvested. And a save
# reads its own text back before writing it (_Verify): if the spelled
# file would not compose to exactly the layer's paths, the layer is saved
# absolute and the report says why.
#
import io
import os
import re
import uuid

from pxr import Sdf, Tf


# The list-op fields a usda file may spell relative, with the spec type
# each belongs to.
FIELDS = {"targetPaths": Sdf.RelationshipSpec,
          "connectionPaths": Sdf.AttributeSpec,
          "inheritPaths": Sdf.PrimSpec,
          "specializes": Sdf.PrimSpec}

_ARMS = ("explicit", "added", "prepended", "appended", "deleted", "ordered")

# The usda literals a `<` can appear in without starting a path -- a
# string (triple-quoted first), an asset path, a comment -- and the
# <path> itself. Scanned as one alternation, left to right, so a `<`
# inside any of the others is never taken for a path.
PATH_LITERALS = re.compile(
    r'"""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\'|@@@.*?@@@|@[^@\n]*@'
    r'|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'|#[^\n]*|<[^<>\n]*>')

# A `<` opening a path that is not absolute. Most files have none, and
# this is how reading one costs a regex scan rather than a parse.
_RELATIVE_HINT = re.compile(r"<(?![/>])")

# layer identifier -> {(specPath, field, path): spelling}, from the panel.
_typed = {}
# layer identifier -> ((realPath, mtime, size), spellings), from the file.
_fromFile = {}


class SpellingError(Exception):
    """The spelled usda would not read back to the layer's own paths."""


class SaveReport(object):
    """What SaveLayer did: how many paths it kept relative, and why not."""

    def __init__(self, layer, relative, note=""):
        self.layer = layer
        self.relative = relative
        self.note = note

    def __str__(self):
        text = "Saved %s" % (self.layer.GetDisplayName()
                             or self.layer.identifier)
        if self.relative:
            text += " (%d relative path%s kept)" % (
                self.relative, "" if self.relative == 1 else "s")
        if self.note:
            text += " -- %s" % self.note
        return text


def _Key(specPath, field, path):
    return (str(specPath), field, str(path))


def IsRelative(spelling):
    return bool(spelling) and not spelling.startswith("/")


def CanSpellRelative(specPath, path, spelling):
    """
    Whether `spelling`, written into a field of the spec at `specPath`,
    reads back as `path`.

    The reader anchors a relative path at the spec's prim, so that is
    the whole test -- plus the two it cannot take at all: a spec inside
    a variant, and `.`, which is a valid path but not valid usda.
    """
    specPath = Sdf.Path(specPath)
    if (specPath.ContainsPrimVariantSelection() or spelling == "."
            or not Sdf.Path.IsValidPathString(spelling)):
        return False
    relative = Sdf.Path(spelling)
    if relative.IsAbsolutePath():
        return False
    return relative.MakeAbsolutePath(specPath.GetPrimPath()) == Sdf.Path(path)


def Record(layer, specPath, field, spelled):
    """
    Remember how the panel was just told to spell `field` on the spec at
    `specPath`. `spelled` is [(path, text)]: each path as authored and the
    text it was typed as.

    An absolute spelling is remembered too, not skipped: retyping a path
    absolute that the file spells relative is a change of spelling, and
    the save has to write it. So is a relative spelling the reader could
    not take back, which is remembered as the absolute path it anchored
    to -- that is what the file will say, so that is what is shown.
    """
    table = _typed.setdefault(layer.identifier, {})
    for path, text in spelled:
        text = text.strip()
        if not (IsRelative(text) and CanSpellRelative(specPath, path, text)):
            text = str(path)
        table[_Key(specPath, field, path)] = text


def Spellings(layer):
    """
    {(specPath, field, path): spelling} for `layer`: what its file says,
    overridden by what the panel authored since.
    """
    merged = dict(_FileSpellings(layer))
    merged.update(_typed.get(layer.identifier, {}))
    return merged


def Spell(spellings, specPath, field, path):
    """`path` as `spellings` spells it -- absolute when it says nothing."""
    return spellings.get(_Key(specPath, field, path), str(path))


# Writing: the layer's usda, spelled.

def SpelledUsdaText(layer, spellings=None):
    """
    (text, used): `layer` as usda, with each path spelled as `spellings`
    says (Spellings(layer) by default), and the relative spellings that
    went into it.

    Raises SpellingError when the text would not read back to exactly
    the layer's paths. Should not happen -- CanSpellRelative is the
    reader's own rule -- but a save is the wrong place to find out.
    """
    if spellings is None:
        spellings = Spellings(layer)
    wanted = {}
    for (specPath, field, path), text in spellings.items():
        if IsRelative(text) and CanSpellRelative(specPath, path, text):
            wanted.setdefault((specPath, field), {})[path] = text
    if not wanted:
        return layer.ExportToString(), {}

    scratch = Sdf.Layer.CreateAnonymous("spelled.usda")
    scratch.TransferContent(layer)
    prefix = "/__rigExecSpelled_%s_" % uuid.uuid4().hex[:8]
    swaps = {}
    used = {}
    touched = []
    for (specPath, field), byPath in wanted.items():
        spec = scratch.GetObjectAtPath(specPath)
        op = _ListOp(spec, field)
        if op is None:
            continue

        def placeholder(path, byPath=byPath, specPath=specPath,
                        field=field):
            text = byPath.get(str(path))
            if text is None:
                return path
            stand = _Placeholder(prefix, len(swaps), text)
            swaps["<%s>" % stand] = "<%s>" % text
            used[(specPath, field, str(path))] = text
            return Sdf.Path(stand)

        if _MapListOp(spec, field, op, placeholder):
            touched.append((specPath, field))
    if not swaps:
        return layer.ExportToString(), {}

    text = scratch.ExportToString()
    for stand, spelled in swaps.items():
        text = text.replace(stand, spelled)
    _Verify(layer, text, touched)
    return text, used


def _Placeholder(prefix, index, spelling):
    """
    A unique absolute path to stand in for `spelling` -- a property path
    when the spelling is one, since a connection names a property.
    """
    return "%s%d%s" % (prefix, index,
                       ".p" if Sdf.Path(spelling).IsPropertyPath() else "")


def _ListOp(spec, field):
    if spec is None or not isinstance(spec, FIELDS[field]) \
            or not spec.HasInfo(field):
        return None
    return spec.GetInfo(field)


def _Arms(op):
    """A list op as a comparable value: which form, and every arm."""
    if op is None:
        return None
    return (op.isExplicit,
            tuple(tuple(getattr(op, arm + "Items")) for arm in _ARMS))


def _MapListOp(spec, field, op, fn):
    """
    Put `fn(path)` in place of every path in `field`, arm by arm, keeping
    each where it sits. Returns whether anything changed.

    Targets and connections are written through their proxies: Sdf will
    not SetInfo them. The proxy canonicalizes, which a placeholder --
    already absolute -- passes through untouched.
    """
    arms = ("explicit",) if op.isExplicit else _ARMS[1:]
    changed = {}
    for arm in arms:
        items = list(getattr(op, arm + "Items"))
        mapped = [fn(path) for path in items]
        if mapped != items:
            changed[arm] = mapped
    if not changed:
        return False
    if field in ("targetPaths", "connectionPaths"):
        proxy = (spec.targetPathList if field == "targetPaths"
                 else spec.connectionPathList)
        for arm, mapped in changed.items():
            setattr(proxy, arm + "Items", mapped)
    else:
        for arm, mapped in changed.items():
            setattr(op, arm + "Items", mapped)
        spec.SetInfo(field, op)
    return True


def _Verify(layer, text, touched):
    """Read `text` back and check every spelled field names what it did."""
    check = Sdf.Layer.CreateAnonymous("spelledCheck.usda")
    try:
        if not check.ImportFromString(text):
            raise SpellingError("the spelled file does not read back")
    except Tf.ErrorException as error:
        raise SpellingError(_FirstLine(str(error)))
    for specPath, field in touched:
        if (_Arms(_ListOp(layer.GetObjectAtPath(specPath), field))
                != _Arms(_ListOp(check.GetObjectAtPath(specPath), field))):
            raise SpellingError("%s %s would read back as other paths"
                                % (specPath, field))


def _FirstLine(text):
    for line in text.splitlines():
        if line.strip():
            return line.strip()
    return text


# Reading: the spellings a .usda file already has.

def _IsUsda(layer):
    """Whether `layer` is text on disk -- the only form a spelling has."""
    try:
        return layer.GetFileFormat().formatId == "usda"
    except Exception:
        return False


def _FileSpellings(layer):
    """
    The spellings in `layer`'s file, cached until the file changes.

    Read from the FILE, not the layer: the layer has already anchored
    every path it read, which is the whole problem.
    """
    if layer.anonymous or not _IsUsda(layer):
        return {}
    path = layer.realPath
    try:
        stat = os.stat(path)
    except (OSError, TypeError, ValueError):
        return {}
    key = (path, stat.st_mtime_ns, stat.st_size)
    cached = _fromFile.get(layer.identifier)
    if cached is not None and cached[0] == key:
        return cached[1]
    try:
        with io.open(path, encoding="utf-8") as stream:
            spellings = SpellingsInText(stream.read())
    except (OSError, UnicodeDecodeError):
        spellings = {}
    _fromFile[layer.identifier] = (key, spellings)
    return spellings


def SpellingsInText(text):
    """
    {(specPath, field, path): spelling} for every relative path the usda
    `text` spells in a field that can hold one -- see the header for how
    this is read without parsing usda here.
    """
    if not _RELATIVE_HINT.search(text):
        return {}
    prefix = "/__rigExecSpelling_%s_" % uuid.uuid4().hex[:8]
    bodies = []

    def swap(match):
        literal = match.group(0)
        if not literal.startswith("<"):
            return literal
        body = literal[1:-1].strip()
        if (not body or body.startswith("/")
                or not Sdf.Path.IsValidPathString(body)):
            return literal
        bodies.append(body)
        return "<%s>" % _Placeholder(prefix, len(bodies) - 1, body)

    check = Sdf.Layer.CreateAnonymous("spellingRead.usda")
    try:
        if not check.ImportFromString(PATH_LITERALS.sub(swap, text)):
            return {}
    except Tf.ErrorException:
        return {}
    spellings = {}
    for spec, field, op in _PathFields(check):
        anchor = spec.path.GetPrimPath()
        for arm in _ARMS:
            for item in getattr(op, arm + "Items"):
                name = str(item)
                if not name.startswith(prefix):
                    continue
                body = bodies[int(name[len(prefix):].split(".")[0])]
                path = Sdf.Path(body).MakeAbsolutePath(anchor)
                if not path.isEmpty:
                    spellings[_Key(spec.path, field, path)] = body
    return spellings


def _PathFields(layer):
    """(spec, field, listOp) for every spellable field `layer` holds."""
    paths = []
    layer.Traverse(Sdf.Path.absoluteRootPath, paths.append)
    for path in paths:
        if path.ContainsPrimVariantSelection():
            continue
        spec = layer.GetObjectAtPath(path)
        for field in FIELDS:
            op = _ListOp(spec, field)
            if op is not None:
                yield spec, field, op


# Saving.

def SaveLayer(layer):
    """
    Save `layer` with every path spelled as it was typed or read.
    Returns a SaveReport.

    The layer is saved first, the ordinary way -- which is what clears
    its dirty state, and is the whole save for a layer with nothing
    spelled relative -- and the file is then rewritten with the spelled
    text, which reads back to the same layer. A binary layer has no
    spelling to keep and is saved as is.
    """
    if layer.anonymous:
        raise ValueError("%s is anonymous; it has no file to save to"
                         % layer.identifier)
    if not _IsUsda(layer):
        _Save(layer)
        return SaveReport(layer, 0)
    # Gathered before anything is written: the file is one of the two
    # places a spelling lives, and the plain save below overwrites it.
    spellings = Spellings(layer)
    note = ""
    try:
        text, used = SpelledUsdaText(layer, spellings)
    except SpellingError as error:
        text, used = None, {}
        note = "paths written absolute: %s" % error
    _Save(layer)
    if used:
        with io.open(layer.realPath, "w", encoding="utf-8",
                     newline="\n") as stream:
            stream.write(text)
        stat = os.stat(layer.realPath)
        _fromFile[layer.identifier] = (
            (layer.realPath, stat.st_mtime_ns, stat.st_size), dict(used))
    return SaveReport(layer, len(used), note)


def _Save(layer):
    if not layer.Save():
        raise RuntimeError("could not save %s" % layer.identifier)
