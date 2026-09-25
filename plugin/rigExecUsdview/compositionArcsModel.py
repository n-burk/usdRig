#
# Composition-arc authoring for the Layer Opinions panel.
#
# The Qt-free half of the composition-arc flows: what each arc kind
# asks for, whether a filled-in request is legal, the usda the panel
# shows you before you commit, and the undoable Edit that authors it.
# Imports no Qt, so all of it is testable without a display
# (tests/python/test_composition_arcs_model.py).
#
# ADD AND EDIT ARE ONE FLOW. Every dialog opens either on an empty form
# or on the arc a row in the panel already holds, and the difference is
# one attribute on the context: ArcContext.editRow. The fields, the
# explanations, the refusals, the warnings and the live preview are the
# same either way, because the question "is this reference legal" does
# not change according to whether the reference is new. What an edit
# does drop is the two fields that are not questions any more -- which
# layer (the arc is in the layer it is in) and the position (the item
# keeps the arm and index it already occupies, so retyping an asset
# path cannot silently re-rank it against the arcs beside it).
#
# The rows the panel offers for editing come from layerOpinionsModel,
# which expands an arc field into one row per item and owns the writes
# that put an item back. This module decides WHICH arc a row is and what
# its fields mean; that one does the Sdf work, so an arc changed inline
# in the tree and one changed through a dialog take the same path.
#
# WHERE ARCS ARE AUTHORED. layerOpinionsModel groups a prim's opinions
# by every layer in its prim stack, and that includes layers reached
# THROUGH a reference -- where OpinionRow.specPath is the REFERENCED
# prim's path, not the prim's stage path. Authoring an arc into one of
# those would edit the referenced asset for every place it is used,
# which is never what "add a reference to this prim" meant. So the
# flows offer only the stage's LOCAL layer stack (session + root +
# sublayers), which is exactly the set of layers where the prim's stage
# path is also its spec path. AuthoringLayers() is that rule, and
# Author() re-checks it rather than trusting the caller's combo box.
#
# WHY THE PREVIEW IS TRUSTWORTHY. Preview() does not describe what
# Author() will do; it runs the SAME _Apply function against a scratch
# layer seeded with the real spec, and exports the result. A preview
# that drifts from the authoring path is worse than no preview, and
# this makes drift impossible rather than merely unlikely.
#
import math
import os

from pxr import Pcp, Sdf, Tf, Usd

import layerOpinionsModel
import rigExecUndo


class ArcError(Exception):
    """A filled-in arc request that cannot legally be authored."""


def _Editing(context):
    """
    Whether `context` is reopening an existing arc.

    None counts as adding, so a caller that only wants to know what an
    arc's blank form looks like need not build a context first.
    """
    return context is not None and context.editRow is not None


# --------------------------------------------------------------------
# Field descriptors. The dialog builds itself from these, so adding a
# field to an arc is a change here and nowhere in the Qt file.
# --------------------------------------------------------------------

TEXT = "text"              # a line edit
ASSET = "asset"            # a line edit with a file browser beside it
PRIM_PATH = "primPath"     # an editable combo seeded with stage paths
CHOICE = "choice"          # radio buttons; `choices` are (value, label)
DOUBLE = "double"          # a number, kept as text so it can be empty
STRING_LIST = "stringList"  # a small multi-line edit, one item per line
LAYER = "layer"            # the "Author into" combo


class Field(object):
    def __init__(self, key, label, kind, default="", help="",
                 choices=(), optional=False, showIf=None):
        self.key = key
        self.label = label
        self.kind = kind
        self.default = default
        self.help = help
        self.choices = tuple(choices)
        self.optional = optional
        # (key, value): the field is only shown when values[key] == value.
        # Used for the external/internal split, where an asset path is
        # meaningless for an internal reference and showing it greyed
        # out reads as "you forgot something".
        self.showIf = showIf


# The two fields nearly every arc shares. Defined once so the "prepend
# is stronger" wording is stated in exactly one place.
_POSITION_FIELD = Field(
    "position", "Position", CHOICE, default="prepend",
    choices=(("prepend", "Prepend  (stronger than existing arcs)"),
             ("append", "Append  (weaker than existing arcs)")),
    help="List-editable arcs compose in order. Prepending puts this arc "
         "ahead of the ones already authored in this layer, so its "
         "opinions win over theirs.")

_LAYER_FIELD = Field(
    "layer", "Author into", LAYER,
    help="Which layer of the stage's local layer stack receives the arc. "
         "Layers a prim reaches through a reference are not offered: "
         "authoring there would change the referenced asset itself.")


def _OffsetFields(what):
    return (
        Field("offset", "Time offset", DOUBLE, default="0", optional=True,
              help="Shifts %s time samples by this many frames." % what),
        Field("scale", "Time scale", DOUBLE, default="1", optional=True,
              help="Scales %s timeline. 2.0 plays it at half speed."
                   % what),
    )


# --------------------------------------------------------------------
# Context and layer choice
# --------------------------------------------------------------------

class ArcContext(object):
    """The stage and prim a flow is authoring against."""

    def __init__(self, stage, prim, editRow=None):
        self.stage = stage
        self.prim = prim
        # The panel row being changed, when this flow is reopening an
        # arc rather than adding one. Every rule that has to excuse the
        # arc from itself consults it -- "this layer already sublayers
        # that file" and "this layer already relocates that source" are
        # refusals when adding and are the status quo when editing.
        self.editRow = editRow

    @property
    def primPath(self):
        return self.prim.GetPath()

    @property
    def editing(self):
        return self.editRow is not None


def AuthoringLayers(stage):
    """
    The layers an arc may be authored into, strongest first.

    The stage's local layer stack, minus any layer that will not accept
    an edit. GetLayerStack() already returns session-first, root, then
    sublayers in strength order, which is the order the panel groups by
    -- so the combo and the tree agree without a second sort.

    A MUTED layer is not in that stack at all, so it is not offered.
    That is the honest answer rather than an omission: while a layer is
    muted it contributes nothing, and an arc authored into it would look
    like it had silently failed.
    """
    return [layer for layer in stage.GetLayerStack()
            if layer.permissionToEdit and not layer.expired]


def DefaultAuthoringLayer(stage, clickedLayer=None):
    """
    The layer a flow opens on.

    The layer whose group was right-clicked, when that layer can take
    the edit -- clicking a layer's row and getting a dialog aimed
    somewhere else is a trap. Otherwise the stage's edit target, which
    is where every other usdview edit goes, and only then the strongest
    layer that will take an edit.

    The edit target is used only when its MAPPING is the identity. A
    variant edit target maps /A/B to /A{v=x}B, and these flows author at
    the composed prim path; taking just its layer would put the arc on
    the OUTER prim, where it applies to every variant instead of the one
    that was being edited.
    """
    candidates = AuthoringLayers(stage)
    if not candidates:
        return None
    if clickedLayer in candidates:
        return clickedLayer
    editTarget = stage.GetEditTarget()
    if (editTarget.GetMapFunction().isIdentity
            and editTarget.GetLayer() in candidates):
        return editTarget.GetLayer()
    return candidates[0]


def PrimPathCandidates(stage, limit=500):
    """
    Stage prim paths, for seeding an editable path combo.

    Bounded: a production stage has more prims than a combo box is
    worth, and the field stays typeable either way.
    """
    paths = []
    for prim in stage.Traverse():
        paths.append(str(prim.GetPath()))
        if len(paths) >= limit:
            break
    return paths


def LayerPrimPaths(layer, limit=500):
    """
    The prim paths a LAYER holds, defaultPrim first.

    This is what "which prim inside the target to compose" means for an
    external reference or payload: the paths live in the asset, and the
    stage being edited knows nothing about them. A combo seeded from the
    stage instead offers the one set of paths that is certainly wrong --
    on a shot file that has not been assembled yet, that is `/World` and
    nothing else.

    Read off the layer's own prim specs rather than by opening it as a
    stage. An asset layer need not compose on its own (one made of
    `over`s composes nothing at all), and opening a second stage per
    keystroke to fill a combo is not a trade worth making.

    nameChildren only, so variants are not descended into: an arc's
    target must be a plain prim path, and `/A{lod=hi}B` is not one.
    """
    paths = []

    def walk(spec):
        if len(paths) >= limit:
            return
        paths.append(str(spec.path))
        for child in spec.nameChildren:
            walk(child)

    for root in layer.rootPrims:
        walk(root)

    default = DefaultPrimPath(layer)
    if default in paths:
        paths.remove(default)
        paths.insert(0, default)
    return paths


def DefaultPrimPath(layer):
    """
    The layer's defaultPrim as a path, or "" when it declares none.

    Spelled as a NAME in a layer's metadata (`defaultPrim = "Hand"`)
    but as a path everywhere it is used, and USD now also accepts a
    nested path there -- so both spellings arrive here.
    """
    default = layer.defaultPrim
    if not default:
        return ""
    text = str(default)
    return text if text.startswith("/") else "/" + text


def ClassPrimPaths(stage):
    """
    Paths of `class` prims, which is what inherits and specializes
    almost always target. Offered ahead of the general list.
    """
    return [str(p.GetPath()) for p in
            stage.TraverseAll()
            if p.GetSpecifier() == Sdf.SpecifierClass]


# --------------------------------------------------------------------
# Snapshots for the things a prim-spec snapshot does not cover.
#
# They live in layerOpinionsModel, beside the operations that edit and
# remove the entries these arcs add, and are named here because that is
# where the arcs that use them are. Both satisfy the same contract
# rigExecUndo.Edit relies on: a Restore() that puts the layer back
# exactly, called inside the Edit's own change block.
# --------------------------------------------------------------------

SublayerSnapshot = layerOpinionsModel.SublayerSnapshot
RelocatesSnapshot = layerOpinionsModel.RelocatesSnapshot


# --------------------------------------------------------------------
# Arc kinds
# --------------------------------------------------------------------

class _Arc(object):
    """
    One arc kind.

    Subclasses declare their fields and implement _Apply, which is the
    only place that authors anything. Everything else -- validation
    hooks aside -- is shared, so preview, authoring and undo cannot
    disagree about what the arc does.
    """

    key = ""
    label = ""       # the context-menu item
    title = ""       # the dialog's title bar
    summary = ""     # the standing explanation under the form
    fields = ()
    # "prim" arcs live on the prim's spec; "layer" arcs are layer
    # metadata and are authored whether or not the prim has a spec.
    scope = "prim"
    # Whether an arc of this kind that already exists can be reopened in
    # this flow and changed in place.
    editable = True

    # -- subclass hooks ----------------------------------------------

    @classmethod
    def Defaults(cls, context):
        """
        Field values to open the dialog with -- the arc `context` is
        editing, or the blank form when it is adding one.
        """
        if not _Editing(context):
            return {f.key: f.default for f in cls.fields}
        values = {f.key: f.default for f in cls.fields}
        values["layer"] = context.editRow.layer
        values.update(cls._ValuesFromRow(context.editRow))
        return values

    @classmethod
    def _ValuesFromRow(cls, row):
        """The field values describing the arc `row` already holds."""
        raise NotImplementedError

    @classmethod
    def _ApplyEdit(cls, layer, primPath, values, row):
        """Write `values` over the arc `row` addresses, in place."""
        raise NotImplementedError

    @classmethod
    def Check(cls, context, values):
        """
        Raise ArcError if `values` cannot be authored; return a list of
        human-readable warnings for things that are legal but usually a
        mistake. Warnings are shown, never blocking.

        Arc-specific only. Callers want Validate(), which adds the
        checks every arc shares.
        """
        return []

    @classmethod
    def _Apply(cls, layer, primPath, values):
        raise NotImplementedError

    @classmethod
    def _Snapshot(cls, layer, primPath):
        """
        Capture whatever this arc is about to change, as an object with
        a Restore() -- the only thing rigExecUndo.Edit asks of it.

        Declared per arc rather than branched on inside Author: a
        layer-scoped arc added later would otherwise silently inherit
        the wrong snapshot and undo the wrong thing.
        """
        return layerOpinionsModel.SpecCopySnapshot.Capture(
            layer, _CreatedRootPath(layer, primPath))

    # -- shared ------------------------------------------------------

    @classmethod
    def Label(cls, values):
        """The undo-stack label for this authoring."""
        return "Add %s" % cls.title.replace("Add ", "").lower()

    @classmethod
    def EditLabel(cls, values):
        """
        The undo-stack label when the flow changed an existing arc.

        Derived from Label rather than declared again, so an arc that
        puts the target in its label ("Add variant set lod") keeps it.
        """
        return cls.Label(values).replace("Add ", "Edit ", 1)

    @classmethod
    def Title(cls, context):
        return (cls.title.replace("Add ", "Edit ", 1) if _Editing(context)
                else cls.title)

    @classmethod
    def FieldsFor(cls, context):
        """
        The fields the dialog asks for.

        An edit drops two of them, because neither is a question any
        more. "Author into" would be a choice of somewhere ELSE to put
        the arc, which copies it rather than changing it; the arc is in
        the layer it is in. And Position is the arm and index the item
        already occupies -- retyping an asset path must not silently
        re-rank the arc against the ones beside it. Moving an arc is its
        own action in the panel, where the order is visible.
        """
        if not _Editing(context):
            return cls.fields
        return tuple(f for f in cls.fields
                     if f.kind != LAYER and f.key != "position")

    @classmethod
    def PathChoices(cls, context, values, field):
        """
        The paths to seed one PRIM_PATH field's combo with.

        This stage's prims, class prims first: an inherit, a specialize
        and a relocate all name a prim on the stage being edited, and a
        class is what the first two nearly always want.

        Asked per field and per current value rather than computed once
        for the arc, because the answer moves: the target of an EXTERNAL
        reference lives in the asset, and which asset that is changes as
        the path is typed.
        """
        paths = ClassPrimPaths(context.stage)
        for path in PrimPathCandidates(context.stage):
            if path not in paths:
                paths.append(path)
        return paths

    @classmethod
    def _ApplyFor(cls, layer, primPath, values, context):
        """Author, or re-author in place, according to the context."""
        if _Editing(context):
            cls._ApplyEdit(layer, primPath, values, context.editRow)
        else:
            cls._Apply(layer, primPath, values)

    @classmethod
    def _Seed(cls, context, values):
        """
        A scratch layer holding what the chosen layer already has for
        this prim (or, for a layer-scoped arc, its layer metadata).

        Seeded rather than empty so an arc PREPENDED ahead of existing
        ones shows in its real position, and so an existing spec's
        specifier and type appear instead of a misleading bare `over`.
        """
        layer = values.get("layer")
        scratch = Sdf.Layer.CreateAnonymous("arcPreview.usda")
        primPath = context.primPath
        if layer is None:
            return scratch
        if cls.scope == "layer":
            scratch.subLayerPaths = list(layer.subLayerPaths)
            for i, offset in enumerate(layer.subLayerOffsets):
                scratch.subLayerOffsets[i] = offset
            if layer.relocates:
                scratch.relocates = list(layer.relocates)
        else:
            # Ancestors are seeded from the real layer too, not just
            # created as bare overs. Without this, a layer holding
            # `def "A"` but no /A/B previews as `over A { over B ... }`
            # while authoring actually leaves `def A` alone -- so the
            # preview misstates the file AND the diff highlights the
            # untouched ancestors as newly added.
            _SeedAncestors(layer, scratch, primPath)
            if layer.GetPrimAtPath(primPath) is not None:
                Sdf.CopySpec(layer, primPath, scratch, primPath)
        return scratch

    @classmethod
    def PreviewBase(cls, context, values):
        """
        The seed alone, with nothing authored.

        The dialog diffs this against Preview() to highlight the lines
        the arc adds. Without it the one line that matters is lost in
        however many opinions the prim already carries -- which on a rig
        prim is dozens.
        """
        return cls._Seed(context, values).ExportToString()

    @classmethod
    def Preview(cls, context, values):
        """The usda that authoring would produce."""
        scratch = cls._Seed(context, values)
        cls._ApplyFor(scratch, context.primPath, values, context)
        return scratch.ExportToString()

    @classmethod
    def Validate(cls, context, values):
        """
        The arc's own checks plus the ones every arc shares. Raises
        ArcError; returns the warnings.

        One entry point so the dialog's live validation and Author()
        cannot disagree about what is legal -- a dialog that enables
        Author on a request Author then refuses is the worst of both.
        """
        layer = values.get("layer")
        if layer is None:
            raise ArcError("no layer chosen")
        # Re-checked here rather than trusted from the dialog: the combo
        # is a convenience, the rule about which layers may be authored
        # into is a correctness property.
        if layer not in AuthoringLayers(context.stage):
            raise ArcError(
                "%s is not a layer of this stage's local layer stack, or "
                "cannot be edited"
                % (layer.GetDisplayName() or layer.identifier))
        return list(cls.Check(context, values))

    @classmethod
    def Author(cls, context, values):
        """
        Author the arc, or re-author the one being edited in place.
        Returns (rigExecUndo.Edit, warnings).
        """
        warnings = cls.Validate(context, values)
        layer = values.get("layer")
        primPath = context.primPath
        specPath = (Sdf.Path.absoluteRootPath if cls.scope == "layer"
                    else primPath)

        before = cls._Snapshot(layer, primPath)
        with Sdf.ChangeBlock():
            cls._ApplyFor(layer, primPath, values, context)
        after = cls._Snapshot(layer, primPath)

        edit = rigExecUndo.Edit(
            cls.EditLabel(values) if _Editing(context)
            else cls.Label(values),
            [rigExecUndo.EditEntry(layer, specPath, before, after)])
        return edit, warnings


def _SeedAncestors(layer, scratch, primPath):
    """
    Recreate primPath's ancestor chain in `scratch` as the real layer
    has it -- same specifier and type -- without copying their subtrees.

    Sdf.CopySpec on an ancestor would drag in every sibling of the prim
    being previewed, which is both slow and noise.
    """
    ancestors = []
    path = Sdf.Path(primPath).GetParentPath()
    while not path.IsAbsoluteRootPath():
        ancestors.append(path)
        path = path.GetParentPath()
    for path in reversed(ancestors):
        spec = Sdf.CreatePrimInLayer(scratch, path)
        source = layer.GetPrimAtPath(path)
        if source is not None:
            spec.specifier = source.specifier
            if source.typeName:
                spec.typeName = source.typeName


def _CreatedRootPath(layer, primPath):
    """
    The highest path _PrimSpecFor is about to create, or primPath when
    the spec already exists.

    Authoring into a layer that holds nothing for the prim creates the
    whole ancestor chain as overs. Snapshotting only the leaf would undo
    the arc and leave `over "A" { over "B" {} }` behind forever -- an
    empty group the panel then lists as a layer with no opinions.
    """
    path = Sdf.Path(primPath)
    created = None
    while (not path.IsAbsoluteRootPath()
           and layer.GetPrimAtPath(path) is None):
        created = path
        path = path.GetParentPath()
    return created if created is not None else Sdf.Path(primPath)


def _EnsureAncestorSpecs(layer, primPath):
    """
    Create primPath's parent chain, so Sdf.CopySpec has somewhere to
    land. Same reason as layerOpinionsModel._EnsureAncestors: CopySpec
    will not invent ancestors and fails inside SdfData without them.
    """
    parent = primPath.GetParentPath()
    if parent and not parent.IsAbsoluteRootPath():
        Sdf.CreatePrimInLayer(layer, parent)


def _PrimSpecFor(layer, primPath):
    """
    The prim's spec in `layer`, created as an over if it has none.

    Sdf.CreatePrimInLayer makes overs, which is right: adding a
    reference to a prim defined in a weaker layer must not promote the
    stronger layer's spec to a def and start redefining the prim.
    """
    return Sdf.CreatePrimInLayer(layer, primPath)


def _ParseNumber(text, label, default):
    text = (text or "").strip()
    if not text:
        return default
    try:
        value = float(text)
    except ValueError:
        raise ArcError("%s must be a number, not %r" % (label, text))
    # nan/inf parse happily and reach Sdf.LayerOffset, which then makes
    # the arc an invalid-offset composition error at stage open. "1e309"
    # overflows to inf the same way, so the check is on the result.
    if not math.isfinite(value):
        raise ArcError("%s must be a finite number, not %r" % (label, text))
    return value


def _NumberText(value):
    """
    A number as the dialog's own field would have been typed.

    %g rather than str(): the fields are text, and an offset that came
    back as "5.0" would be re-parsed to the same 5.0 but read as though
    the arc carried a precision it does not.
    """
    return "%g" % value


def _LayerOffsetFrom(values):
    offset = _ParseNumber(values.get("offset"), "Time offset", 0.0)
    scale = _ParseNumber(values.get("scale"), "Time scale", 1.0)
    if scale == 0.0:
        raise ArcError("Time scale must not be zero")
    return Sdf.LayerOffset(offset, scale)


def _ParsePrimPath(text, label, allowEmpty=False):
    text = (text or "").strip()
    if not text:
        if allowEmpty:
            return Sdf.Path.emptyPath
        raise ArcError("%s is required" % label)
    path = Sdf.Path(text)
    if not path.IsAbsolutePath() or not path.IsPrimPath():
        raise ArcError("%s must be an absolute prim path, like /Asset/Rig, "
                       "not %r" % (label, text))
    return path


def _AddToListOp(listOp, position, item):
    """
    Add `item` to a list-op through its Prepend/Append operations.

    NOT by appending to listOp.prependedItems. Writing an individual arm
    of a list op is destructive and mis-ordered, both:

      * an EXPLICIT opinion (`references = [@a@, @b@]`) is thrown away
        the moment `prependedItems` is written -- the result is
        `prepend references = @new@` and the two existing arcs are gone;
      * appending to `prependedItems` puts the new arc AFTER the
        prepends already there, which is the opposite of what the
        dialog's "Prepend (stronger than existing arcs)" promises.

    Prepend()/Append() keep an explicit list explicit, insert at the
    right end of it, and fall back to the prepend/append arms only when
    the opinion is already in that form.
    """
    if position == "prepend":
        listOp.Prepend(item)
    else:
        listOp.Append(item)


class _TargetedArc(_Arc):
    """
    Shared machinery for references and payloads: an external file or an
    internal prim, an optional target prim, and a layer offset.
    """

    fields = (
        _LAYER_FIELD,
        Field("source", "Source", CHOICE, default="external",
              choices=(("external", "External file"),
                       ("internal", "Internal  (a prim in this layer stack)")),
              help="An internal arc targets a prim already on this stage "
                   "and needs no file -- the usual way to reuse a class "
                   "prim without a second asset."),
        Field("assetPath", "Asset", ASSET, showIf=("source", "external"),
              help="The layer to compose in. A relative path resolves "
                   "against the layer you are authoring into."),
        Field("primPath", "Target prim", PRIM_PATH, optional=True,
              help="Which prim inside the target to compose. Leave empty "
                   "for an external file to use its defaultPrim."),
    ) + _OffsetFields("the target's") + (_POSITION_FIELD,)

    @classmethod
    def _Item(cls, assetPath, primPath, offset):
        raise NotImplementedError

    @classmethod
    def _ListOp(cls, spec):
        raise NotImplementedError

    @classmethod
    def Check(cls, context, values):
        warnings = []
        internal = values.get("source") == "internal"
        assetPath = (values.get("assetPath") or "").strip()
        primPath = _ParsePrimPath(
            values.get("primPath"), "Target prim",
            allowEmpty=not internal)

        if internal:
            if assetPath:
                raise ArcError("an internal arc takes no asset path")
            # A prim referencing itself or an ancestor is a composition
            # cycle: Pcp reports it as an error and drops the arc, so
            # the flow refuses it rather than authoring a broken stage.
            _CheckNotCyclic(context.primPath, primPath)
            if not context.stage.GetPrimAtPath(primPath):
                warnings.append(
                    "%s does not exist on this stage yet; the arc will "
                    "compose nothing until it does." % primPath)
        else:
            if not assetPath:
                raise ArcError("an external arc needs an asset path")
            resolved = _ResolveAgainst(values.get("layer"), assetPath)
            if resolved is None:
                warnings.append(
                    "%s does not resolve to a layer from here; check the "
                    "path is relative to the layer you are authoring into."
                    % assetPath)
            elif not primPath and not resolved.defaultPrim:
                warnings.append(
                    "%s declares no defaultPrim, so an arc with no target "
                    "prim will compose nothing. Name a target prim."
                    % assetPath)
        _LayerOffsetFrom(values)
        return warnings

    @classmethod
    def PathChoices(cls, context, values, field):
        """
        The target prim's candidates come from the ASSET, not the stage.

        "Which prim inside the target to compose" is a question about
        the layer being referenced. The stage being edited does not
        contain those prims -- that is the whole point of the arc -- so
        offering its paths here is offering the one list that cannot
        contain the answer.

        An INTERNAL arc is the case where the stage IS the target, and
        it falls through to the stage's own prims. So does an asset path
        that does not resolve yet, which is legal to author: there is
        nothing to read, and an empty combo would read as "this asset
        has no prims" rather than "no asset yet".
        """
        assetPath = (values.get("assetPath") or "").strip()
        if (field.key != "primPath" or not assetPath
                or values.get("source") == "internal"):
            return super(_TargetedArc, cls).PathChoices(
                context, values, field)
        resolved = _ResolveAgainst(values.get("layer"), assetPath)
        if resolved is None:
            return []
        return LayerPrimPaths(resolved)

    @classmethod
    def _NewItem(cls, values):
        internal = values.get("source") == "internal"
        asset = "" if internal else _AssetPathFrom(values)
        target = _ParsePrimPath(values.get("primPath"), "Target prim",
                                allowEmpty=not internal)
        return cls._Item(asset, target, _LayerOffsetFrom(values))

    @classmethod
    def _Apply(cls, layer, primPath, values):
        spec = _PrimSpecFor(layer, primPath)
        _AddToListOp(cls._ListOp(spec), values.get("position"),
                     cls._NewItem(values))

    @classmethod
    def _ValuesFromRow(cls, row):
        item = row.item
        offset = item.layerOffset
        return {
            # An internal arc is exactly one with no asset path, which
            # is how Sdf stores it -- not a separate flavour of item.
            "source": "external" if item.assetPath else "internal",
            "assetPath": item.assetPath,
            "primPath": str(item.primPath) if item.primPath else "",
            "offset": _NumberText(offset.offset),
            "scale": _NumberText(offset.scale),
        }

    @classmethod
    def _ApplyEdit(cls, layer, primPath, values, row):
        layerOpinionsModel.SetListOpItem(
            _PrimSpecFor(layer, primPath), row.infoKey, row.arm, row.index,
            cls._NewItem(values))


def _CheckNotCyclic(primPath, targetPath):
    """
    Refuse the arc targets that compose into a cycle.

    Three cases, not one. Targeting yourself is the obvious one;
    targeting an ANCESTOR pulls your own subtree in underneath you; and
    targeting a DESCENDANT does the same thing from the other end --
    /A referencing /A/B composes /A/B (and so /A) under /A. All three
    are Pcp.ErrorArcCycle, and Pcp drops the arc, so the flow refuses
    them rather than authoring something inert.
    """
    if targetPath == primPath:
        raise ArcError("a prim cannot target itself")
    if primPath.HasPrefix(targetPath):
        raise ArcError("%s is an ancestor of %s -- that is a composition "
                       "cycle" % (targetPath, primPath))
    if targetPath.HasPrefix(primPath):
        raise ArcError("%s is inside %s -- targeting your own descendant "
                       "is a composition cycle" % (targetPath, primPath))


def _IsLocalOnly(stage, prim):
    """
    True when no composition arc brought any of `prim` in -- so there is
    nothing for a relocate to move.

    Asked of the prim INDEX, not of which layers hold its specs. An
    INTERNAL reference or inherit targets a prim in this same layer
    stack, so every spec is in a local layer while the prim is very much
    arriving through an arc; a layer-based test calls that local-only
    and refuses a relocate that would have worked.

    PrimCompositionQuery reports the root arc for every prim, so "local
    only" means the root arc is the only one.
    """
    query = Usd.PrimCompositionQuery(prim)
    return all(arc.GetArcType() == Pcp.ArcTypeRoot
               for arc in query.GetCompositionArcs())


def _SublayersTransitively(layer, target, seen=None):
    """
    Whether `layer` reaches `target` through its own sublayers.

    Bounded by `seen`: the stage being edited may already contain a
    cycle, and a plain recursion would not come back.
    """
    if layer is None or target is None:
        return False
    if seen is None:
        seen = set()
    if layer.identifier in seen:
        return False
    seen.add(layer.identifier)
    for path in layer.subLayerPaths:
        child = _ResolveAgainst(layer, path)
        if child is None:
            continue
        if child == target or _SublayersTransitively(child, target, seen):
            return True
    return False


def AnchoredAssetPath(layer, assetPath):
    """
    `assetPath` as the flows author it: relative to `layer`'s own file
    (`./rig.usda`, `../shared/hand.usda`) whenever it can be.

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


def _AssetPathFrom(values):
    """The asset field anchored to the layer being authored into."""
    return AnchoredAssetPath(values.get("layer"), values.get("assetPath"))


def _ResolveAgainst(layer, assetPath):
    """
    `assetPath` opened as a layer, relative to `layer`, or None.

    Only used to warn: an unresolvable path is legal to author (the
    asset may not exist yet) and the flow must not refuse it.
    """
    if layer is None:
        return None
    try:
        return Sdf.Layer.FindOrOpenRelativeToLayer(layer, assetPath)
    except Exception:
        return None


class ReferenceArc(_TargetedArc):
    key = "reference"
    label = "Reference..."
    title = "Add Reference"
    summary = (
        "A reference composes another prim's contents underneath this one. "
        "Opinions authored here override the referenced ones, and the "
        "referenced layer's own stack is brought along. This is the arc "
        "for assembling a shot out of assets.")

    @classmethod
    def _Item(cls, assetPath, primPath, offset):
        return Sdf.Reference(assetPath, primPath, offset)

    @classmethod
    def _ListOp(cls, spec):
        return spec.referenceList

    @classmethod
    def Label(cls, values):
        return "Add reference"


class PayloadArc(_TargetedArc):
    key = "payload"
    label = "Payload..."
    title = "Add Payload"
    summary = (
        "A payload is a reference the stage may leave unloaded. It composes "
        "exactly like one, but a stage opened without payloads -- or with "
        "this prim unloaded -- skips it entirely. This is the arc for heavy "
        "geometry you do not want to pay for while animating.")

    @classmethod
    def _Item(cls, assetPath, primPath, offset):
        return Sdf.Payload(assetPath, primPath, offset)

    @classmethod
    def _ListOp(cls, spec):
        return spec.payloadList

    @classmethod
    def Label(cls, values):
        return "Add payload"


class _ClassArc(_Arc):
    """Shared machinery for inherits and specializes."""

    fields = (
        _LAYER_FIELD,
        Field("primPath", "Target prim", PRIM_PATH,
              help="The prim to compose from -- conventionally a `class` "
                   "prim, which composes nowhere on its own."),
        _POSITION_FIELD,
    )

    @classmethod
    def _ListOp(cls, spec):
        raise NotImplementedError

    @classmethod
    def Check(cls, context, values):
        warnings = []
        path = _ParsePrimPath(values.get("primPath"), "Target prim")
        _CheckNotCyclic(context.primPath, path)
        target = context.stage.GetPrimAtPath(path)
        if not target:
            warnings.append(
                "%s does not exist on this stage. The arc is legal and "
                "will compose if that prim appears later." % path)
        elif target.GetSpecifier() != Sdf.SpecifierClass:
            warnings.append(
                "%s is not a `class` prim, so it also composes in its own "
                "right. That is legal, but a class is the usual target."
                % path)
        return warnings

    @classmethod
    def _Apply(cls, layer, primPath, values):
        path = _ParsePrimPath(values.get("primPath"), "Target prim")
        spec = _PrimSpecFor(layer, primPath)
        _AddToListOp(cls._ListOp(spec), values.get("position"), path)

    @classmethod
    def _ValuesFromRow(cls, row):
        return {"primPath": str(row.item)}

    @classmethod
    def _ApplyEdit(cls, layer, primPath, values, row):
        layerOpinionsModel.SetListOpItem(
            _PrimSpecFor(layer, primPath), row.infoKey, row.arm, row.index,
            _ParsePrimPath(values.get("primPath"), "Target prim"))


class InheritArc(_ClassArc):
    key = "inherit"
    label = "Inherit..."
    title = "Add Inherit"
    summary = (
        "An inherit composes a class prim's contents underneath this one, "
        "and keeps listening: a later edit to the class reaches every prim "
        "that inherits it. Stronger than a reference, weaker than a local "
        "opinion. This is the arc for 'all of these are the same kind of "
        "thing'.")

    @classmethod
    def _ListOp(cls, spec):
        return spec.inheritPathList

    @classmethod
    def Label(cls, values):
        return "Add inherit"


class SpecializeArc(_ClassArc):
    key = "specialize"
    label = "Specialize..."
    title = "Add Specialize"
    summary = (
        "A specialize composes like an inherit but is the WEAKEST arc "
        "there is: anything the referencing context says wins over it, "
        "even across a reference. This is the arc for a base whose values "
        "are meant to be refinable defaults -- a shader preset, not a "
        "class of things.")

    @classmethod
    def _ListOp(cls, spec):
        return spec.specializesList

    @classmethod
    def Label(cls, values):
        return "Add specialize"


class VariantSetArc(_Arc):
    key = "variantSet"
    label = "Variant Set..."
    title = "Add Variant Set"
    # A row of `variantSetNames` is the NAME of a set, not the set. The
    # variants themselves are VariantSetSpecs hanging off the prim spec,
    # and retyping the name here would leave those behind under the old
    # name while declaring one that has none -- so this flow only adds.
    # The panel still removes a name and edits a selection, and the
    # variants are authored by selecting one and editing the prim.
    editable = False
    summary = (
        "A variant set is a named switch on this prim; each variant holds "
        "its own opinions and only the selected one composes. The set and "
        "its variants are created empty -- author into a variant by "
        "setting the selection, then editing the prim as usual.")

    fields = (
        _LAYER_FIELD,
        Field("name", "Set name", TEXT,
              help="The switch's name, as it appears in usdview's "
                   "variant picker. Conventionally lowerCamelCase: "
                   "modelingVariant, lod, damage."),
        Field("variants", "Variants", STRING_LIST, default="",
              help="One variant name per line. Each becomes an empty "
                   "branch you can author into."),
        Field("selection", "Selected", TEXT, optional=True,
              help="Which variant this layer selects. Leave empty to "
                   "author the set without choosing -- a weaker layer's "
                   "selection then still applies."),
        _POSITION_FIELD,
    )

    @classmethod
    def Label(cls, values):
        return "Add variant set %s" % (values.get("name") or "").strip()

    @classmethod
    def Check(cls, context, values):
        warnings = []
        name = (values.get("name") or "").strip()
        if not name:
            raise ArcError("a variant set needs a name")
        if not Sdf.Path.IsValidIdentifier(name):
            raise ArcError("%r is not a valid identifier for a variant set "
                           "name" % name)
        variants = _VariantNames(values)
        # NOT IsValidIdentifier: USD 26.08 accepts variant names an
        # identifier rule would reject -- "8k", "foo-bar", ".abc" are
        # all legal and in use. Only the SET name is an identifier.
        for variant in variants:
            if not _IsValidVariantName(variant):
                raise ArcError("%r is not a valid variant name" % variant)
        if len(set(variants)) != len(variants):
            raise ArcError("the variant names must be distinct")
        selection = (values.get("selection") or "").strip()
        existingVariants = []
        if name in context.prim.GetVariantSets().GetNames():
            existingVariants = context.prim.GetVariantSet(
                name).GetVariantNames()
        # An existing variant is a legal selection: adding one variant to
        # a set while selecting another that is already there is ordinary.
        if selection and selection not in variants + list(existingVariants):
            raise ArcError("the selection %r is not one of this set's "
                           "variants" % selection)
        if not variants:
            warnings.append(
                "The set is being created with no variants. It will show "
                "in the picker with nothing to pick.")
        elif not selection:
            warnings.append(
                "No selection is being authored, so any selection already "
                "in this layer stands, and otherwise the choice is left "
                "to a weaker layer.")
        if existingVariants or name in context.prim.GetVariantSets(
                ).GetNames():
            warnings.append(
                "%s already has a variant set named %r; the new variants "
                "are added to it." % (context.primPath.name, name))
        return warnings

    @classmethod
    def _Apply(cls, layer, primPath, values):
        name = (values.get("name") or "").strip()
        spec = _PrimSpecFor(layer, primPath)
        # An existing set is reused rather than replaced: a second flow
        # adding one variant to a set must not drop the others.
        vsetSpec = spec.variantSets.get(name)
        if vsetSpec is None:
            vsetSpec = Sdf.VariantSetSpec(spec, name)
        for variant in _VariantNames(values):
            if variant not in vsetSpec.variants:
                Sdf.VariantSpec(vsetSpec, variant)
        # Creating the VariantSetSpec does NOT touch the variantSetNames
        # list op -- that is a separate field, and without it the set
        # composes but does not appear in the prim's declared sets.
        nameList = spec.variantSetNameList
        if name not in (list(nameList.prependedItems)
                        + list(nameList.appendedItems)
                        + list(nameList.explicitItems)):
            _AddToListOp(nameList, values.get("position"), name)
        selection = (values.get("selection") or "").strip()
        if selection:
            spec.variantSelections[name] = selection


def _IsValidVariantName(name):
    """
    Whether Sdf will accept `name` as a variant.

    Asked of Sdf by trying it on a scratch layer, not reimplemented:
    there is no IsValidVariantSelection in the Python API, and the rule
    is not the identifier rule -- "8k", "foo-bar" and ".abc" are all
    legal variant names, while an empty name or one containing a space,
    a slash or a brace is not. Same reason ParseValue round-trips
    through usda rather than parsing values itself.
    """
    if not name:
        return False
    layer = Sdf.Layer.CreateAnonymous("variantNameCheck.usda")
    prim = Sdf.CreatePrimInLayer(layer, "/P")
    prim.specifier = Sdf.SpecifierDef
    vset = Sdf.VariantSetSpec(prim, "s")
    try:
        Sdf.VariantSpec(vset, name)
    except Tf.ErrorException:
        return False
    return True


def _VariantNames(values):
    return [line.strip() for line in
            (values.get("variants") or "").splitlines() if line.strip()]


class SublayerArc(_Arc):
    key = "sublayer"
    label = "Sublayer..."
    title = "Add Sublayer"
    scope = "layer"
    summary = (
        "A sublayer composes a whole layer into this one, for every prim "
        "at once -- it is a property of the LAYER, not of the selected "
        "prim. A sublayer is weaker than the opinions of the layer that "
        "pulls it in, and stronger than anything arriving through a "
        "reference.")

    fields = (
        Field("layer", "Add to layer", LAYER,
              help="The layer that will carry the sublayer. This arc does "
                   "not involve the selected prim at all."),
        Field("assetPath", "Sublayer", ASSET,
              help="A relative path resolves against the layer you are "
                   "adding it to."),
        Field("position", "Position", CHOICE, default="prepend",
              choices=(("prepend", "First  (strongest of the sublayers)"),
                       ("append", "Last  (weakest of the sublayers)")),
              help="A layer's sublayers compose strongest-FIRST, which is "
                   "the opposite of how a list of files usually reads. "
                   "This orders it against the other sublayers only; the "
                   "host layer's own opinions still win over all of them."),
    ) + _OffsetFields("the sublayer's")

    @classmethod
    def _Snapshot(cls, layer, primPath):
        return SublayerSnapshot.Capture(layer)

    @classmethod
    def Label(cls, values):
        return "Add sublayer"

    @classmethod
    def Check(cls, context, values):
        warnings = []
        assetPath = (values.get("assetPath") or "").strip()
        if not assetPath:
            raise ArcError("a sublayer needs a path")
        layer = values.get("layer")
        # The entry being edited is not a duplicate of itself. Without
        # this, reopening a sublayer to change only its offset is
        # refused for already being there.
        others = list(layer.subLayerPaths) if layer is not None else []
        if _Editing(context):
            del others[context.editRow.index]
        # Compared as it would be authored, so picking a file that is
        # already listed as ./rig.usda is caught as the duplicate it is.
        if _AssetPathFrom(values) in others:
            raise ArcError("%s already sublayers %s"
                           % (layer.GetDisplayName(), assetPath))
        resolved = _ResolveAgainst(layer, assetPath)
        if resolved is None:
            warnings.append(
                "%s does not resolve to a layer from here. It will be "
                "authored anyway and compose nothing until it exists."
                % assetPath)
        elif layer is not None and resolved == layer:
            raise ArcError("a layer cannot sublayer itself")
        elif layer is not None and _SublayersTransitively(resolved, layer):
            # Not only the direct case: if Root already sublayers Sub,
            # adding Root under Sub closes the loop just as surely, and
            # Pcp reports ErrorSublayerCycle rather than composing.
            raise ArcError(
                "%s already pulls in %s, so adding it here would make a "
                "sublayer cycle"
                % (resolved.GetDisplayName() or assetPath,
                   layer.GetDisplayName() or layer.identifier))
        _LayerOffsetFrom(values)
        return warnings

    @classmethod
    def _Apply(cls, layer, primPath, values):
        # Anchored to the chosen layer, not to `layer`: the preview runs
        # this against an anonymous scratch layer with no file to be
        # relative to.
        assetPath = _AssetPathFrom(values)
        index = 0 if values.get("position") == "prepend" else len(
            layer.subLayerPaths)
        layer.subLayerPaths.insert(index, assetPath)
        offset = _LayerOffsetFrom(values)
        if offset != Sdf.LayerOffset():
            layer.subLayerOffsets[index] = offset

    @classmethod
    def _ValuesFromRow(cls, row):
        path, offset = row.item
        return {"assetPath": path,
                "offset": _NumberText(offset.offset),
                "scale": _NumberText(offset.scale)}

    @classmethod
    def _ApplyEdit(cls, layer, primPath, values, row):
        layerOpinionsModel.SetSublayer(
            layer, row.index, _AssetPathFrom(values),
            _LayerOffsetFrom(values))


class RelocateArc(_Arc):
    key = "relocate"
    label = "Relocate..."
    title = "Add Relocate"
    scope = "layer"
    summary = (
        "A relocate renames a prim that arrived through a reference or "
        "payload, so this layer can author on it under a name of its own. "
        "It is the only way to move a prim you do not own. Relocates are "
        "LAYER metadata in USD 26.08 -- the older per-prim `relocates` "
        "field is parsed but no longer composes.")

    fields = (
        Field("layer", "Author into", LAYER,
              help="Relocates are metadata on the layer, so this is the "
                   "layer whose relocates map gains an entry."),
        Field("source", "Move from", PRIM_PATH,
              help="The prim as it currently composes -- the path that "
                   "arrived through the arc."),
        Field("target", "Move to", PRIM_PATH,
              help="The path it should appear at instead. Its parent must "
                   "already exist. Moving it under a DIFFERENT parent is "
                   "allowed, not only renaming it in place."),
    )

    @classmethod
    def _Snapshot(cls, layer, primPath):
        return RelocatesSnapshot.Capture(layer)

    @classmethod
    def Label(cls, values):
        return "Add relocate"

    @classmethod
    def Check(cls, context, values):
        warnings = []
        source = _ParsePrimPath(values.get("source"), "Move from")
        target = _ParsePrimPath(values.get("target"), "Move to")
        stage = context.stage
        # The relocate as it stands, when one is being edited. The stage
        # already REFLECTS it, which turns two of the checks below on
        # their head: the prim is not at its source path any more (this
        # relocate moved it), and something does exist at its target
        # (this relocate put it there). Asked of the entry being edited
        # they would refuse every edit, including one that changes
        # nothing.
        current = context.editRow.item if _Editing(context) else None

        if source == target:
            raise ArcError("the source and target are the same path")
        if target.HasPrefix(source):
            raise ArcError("%s is inside %s -- a prim cannot be relocated "
                           "into itself" % (target, source))
        # Pcp's own rule, reported as "Root prims cannot be the source of
        # a relocate" and then ignored -- so it has to be refused here or
        # the arc is authored and silently does nothing.
        if source.GetParentPath().IsAbsoluteRootPath():
            raise ArcError("%s is a root prim, and a root prim cannot be "
                           "relocated" % source)
        if stage.GetPrimAtPath(target) and not (current
                                                and target == current[1]):
            raise ArcError("%s already exists on this stage" % target)

        sourcePrim = stage.GetPrimAtPath(source)
        if current and source == current[0]:
            # Unchanged source: the prim composes at current[1], not
            # here, and both of the checks below would misread that as
            # "there is nothing to relocate".
            pass
        elif not sourcePrim:
            warnings.append("%s does not compose on this stage, so the "
                            "relocate will do nothing." % source)
        elif _IsLocalOnly(stage, sourcePrim):
            # Not a warning. A relocate only moves what a composition arc
            # brought in; applied to a prim whose every opinion is local,
            # it removes the prim from its old path and composes nothing
            # at the new one -- the prim just disappears, with no error.
            raise ArcError(
                "%s is defined only in this stage's own layers, so "
                "relocating it would delete it rather than move it. A "
                "relocate is for prims arriving through a reference or "
                "payload; to rename a local prim, rename its spec."
                % source)

        # A relocates map has to stay internally consistent. USD does not
        # follow a chain: authoring /D/One -> /D/X alongside an existing
        # /D/X -> /D/Y is a conflict, and Pcp then ignores BOTH entries,
        # so the working relocate that was already there stops working.
        layer = values.get("layer")
        existing = list(layer.relocates) if layer is not None else []
        if _Editing(context):
            del existing[context.editRow.index]
        for oldSource, oldTarget in existing:
            if oldSource == source:
                raise ArcError(
                    "%s already relocates %s (to %s). Edit that entry "
                    "rather than adding a second one."
                    % (layer.GetDisplayName() or layer.identifier,
                       source, oldTarget))
            if oldTarget == source or oldSource == target:
                raise ArcError(
                    "this would chain with the existing relocate %s -> "
                    "%s. USD does not follow a chain; it reports a "
                    "conflict and ignores both."
                    % (oldSource, oldTarget))
            if oldTarget == target:
                raise ArcError("%s is already the target of the relocate "
                               "%s -> %s" % (target, oldSource, oldTarget))

        # The target's parent has to exist, or the prim composes nowhere
        # and simply vanishes from both paths.
        targetParent = target.GetParentPath()
        if (not targetParent.IsAbsoluteRootPath()
                and not stage.GetPrimAtPath(targetParent)):
            raise ArcError("%s does not exist, so there is nothing for %s "
                           "to be relocated under"
                           % (targetParent, target.name))

        # Reparenting IS legal -- a relocate may move a prim under a
        # different parent, not only rename it in place -- so only the
        # unusual case is worth remarking on.
        if source.GetParentPath() != target.GetParentPath():
            warnings.append(
                "This moves %s under %s rather than renaming it in place. "
                "That is legal, but check it is what you meant."
                % (source.name, target.GetParentPath()))
        return warnings

    @classmethod
    def _Apply(cls, layer, primPath, values):
        source = _ParsePrimPath(values.get("source"), "Move from")
        target = _ParsePrimPath(values.get("target"), "Move to")
        layer.relocates = list(layer.relocates) + [(source, target)]

    @classmethod
    def _ValuesFromRow(cls, row):
        source, target = row.item
        return {"source": str(source), "target": str(target)}

    @classmethod
    def _ApplyEdit(cls, layer, primPath, values, row):
        layerOpinionsModel.SetRelocate(
            layer, row.index,
            _ParsePrimPath(values.get("source"), "Move from"),
            _ParsePrimPath(values.get("target"), "Move to"))


# Menu order: the arcs an artist reaches for most, first; the two
# layer-scoped ones last, because they are not about the selected prim.
ARC_KINDS = (ReferenceArc, PayloadArc, InheritArc, SpecializeArc,
             VariantSetArc, SublayerArc, RelocateArc)


def ArcByKey(key):
    for arc in ARC_KINDS:
        if arc.key == key:
            return arc
    raise KeyError("no arc kind %r" % key)


# Which arc a panel row is an item of. Keyed on the info field rather
# than on the row's kind, because the field is what says WHICH arc an
# item is -- a list-op row is a reference or an inherit only by virtue
# of where it came from.
_ARC_BY_FIELD = {
    "references": ReferenceArc,
    "payload": PayloadArc,
    "inheritPaths": InheritArc,
    "specializes": SpecializeArc,
    "variantSetNames": VariantSetArc,
    "subLayerPaths": SublayerArc,
    "relocates": RelocateArc,
}

# The row kinds that address ONE arc. The parent rows above them
# ("references", "subLayers") name the field, which is a list, and a
# flow that opened on one of those would not know which arc it meant.
_ARC_ROW_KINDS = ("arcItem", "sublayer", "relocate")


def ArcForRow(row):
    """
    The guided flow that can reopen `row`, or None when there is none --
    the row is not an arc, or is an arc this flow only adds.
    """
    if row is None or row.kind not in _ARC_ROW_KINDS:
        return None
    arc = _ARC_BY_FIELD.get(row.infoKey)
    return arc if arc is not None and arc.editable else None


def CanEditRow(row):
    """Whether the panel should offer to reopen `row` in its flow."""
    return bool(row is not None and row.editable and ArcForRow(row))


def VisibleFields(arc, values, context=None):
    """
    The fields to show for the current `values`, honouring showIf.

    `context` decides whether this is the add form or the edit form; it
    is optional only so a caller with no flow in hand can still ask what
    an arc's fields are.
    """
    fields = arc.FieldsFor(context)
    out = []
    for field in fields:
        if field.showIf is not None:
            key, expected = field.showIf
            if values.get(key) != expected:
                continue
        out.append(field)
    return out
