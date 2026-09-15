#
# RigExec usdview plugin: the Avar Editor's model.
#
# Everything the Avar Editor decides that does not need a widget: which
# attributes on a prim are avar channels, what kind each one is (so a
# rotation gets degrees and a 0..1 float gets a slider), what "reset"
# means for it, and how an edit lands on the stage -- as a key at the
# current frame or as the default -- inside one undo entry.
#
# Deliberately Qt-free, the same rule layerOpinionsModel and gizmoMath
# follow, so tests/python/test_avar_editor_model.py can exercise every
# rule against an in-memory stage. avarEditorUI.py is the Qt shell.
#
# Channels are discovered by NAME PREFIX, not from a list: the schema
# gives every RigExecXformable `avars:tx/ty/tz`, `avars:rx/ry/rz`,
# `avars:rspin`, `avars:sx/sy/sz`, `avars:rotationOrder` and
# `avars:unitScaleFactor`, but a rig is free to author its own -- the
# biped's `avars:ikfk` is a custom float on each limb root that the
# limb's RigExecBlendPointFrames reads as its weight -- and those have to
# show up without anyone editing this file.
#
import math
import os
import sys

from pxr import Sdf, Tf, Usd, UsdGeom

try:
    import gizmoMath
    import rigExecUndo
except ImportError:                    # loader that did not add our dir
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoMath
    import rigExecUndo


AVAR_PREFIX = "avars:"

# Other namespaces a rig parks animator dials in. `foot:` is the reverse
# foot's roll/bank/toePlantAngle on `bank_?` -- the conventional footRoll and
# footRock. They are float customs rather than avars because a
# RigExecFloatMathMover target must be `float` while every avar is
# `double`, so they could never have BEEN avars; that is a typing detail,
# not a reason to hide the three dials that roll the foot from the panel
# an animator uses to roll the foot.
RIG_PREFIXES = (AVAR_PREFIX, "foot:")


def RigPrefixOf(name):
    """The rig namespace this attribute belongs to, or None."""
    for prefix in RIG_PREFIXES:
        if name.startswith(prefix):
            return prefix
    return None


# Where an edit lands. The tokens are gizmoMath's so the Avar Editor and
# the viewport gizmo mean the same thing by "Animation" and "Default".
WRITE_ANIMATION = gizmoMath.WRITE_ANIMATION
WRITE_DEFAULT = gizmoMath.WRITE_DEFAULT

# Channel kinds, in the order the panel lists them.
KIND_TRANSLATE = "translate"
KIND_ROTATE = "rotate"
KIND_SCALE = "scale"
KIND_CUSTOM = "custom"
KIND_OTHER = "other"
KIND_ORDER = (KIND_TRANSLATE, KIND_ROTATE, KIND_SCALE, KIND_CUSTOM,
              KIND_OTHER)
KIND_LABELS = {
    KIND_TRANSLATE: "Translate",
    KIND_ROTATE: "Rotate",
    KIND_SCALE: "Scale",
    KIND_CUSTOM: "Custom",
    KIND_OTHER: "Other",
}

# Value families the editor knows how to build a widget for.
VALUE_FLOAT = "float"
VALUE_INT = "int"
VALUE_BOOL = "bool"
VALUE_TOKEN = "token"
VALUE_STRING = "string"
VALUE_OTHER = "other"

_SCHEMA_KINDS = {
    "avars:tx": KIND_TRANSLATE, "avars:ty": KIND_TRANSLATE,
    "avars:tz": KIND_TRANSLATE,
    "avars:rx": KIND_ROTATE, "avars:ry": KIND_ROTATE,
    "avars:rz": KIND_ROTATE, "avars:rspin": KIND_ROTATE,
    "avars:sx": KIND_SCALE, "avars:sy": KIND_SCALE, "avars:sz": KIND_SCALE,
    "avars:rotationOrder": KIND_OTHER,
    "avars:unitScaleFactor": KIND_OTHER,
}
# Within a kind, the schema channels come in this order and custom ones
# follow alphabetically.
_SCHEMA_RANK = {name: i for i, name in enumerate((
    "avars:tx", "avars:ty", "avars:tz",
    "avars:rx", "avars:ry", "avars:rz", "avars:rspin",
    "avars:sx", "avars:sy", "avars:sz",
    "avars:rotationOrder", "avars:unitScaleFactor"))}

# Channels that carry no scalar to edit. avars:defaultSpace is a matrix
# the evaluator falls back through (schema.usda: "Zero pose supplied to
# avar evaluation") -- authoring it from a slider would be wrong, and it
# is not a channel an animator poses. Listed in the footnote instead.
_HIDDEN = ("avars:defaultSpace",)

_FLOAT_TYPES = ("double", "float", "half")
_INT_TYPES = ("int", "uint", "int64", "uint64", "uchar")

# Slider extents. The spin box is unbounded; these only decide what a
# drag of the full slider covers, and SliderRange widens them to keep
# the current value inside.
ROTATE_SLIDER_DEGREES = 180.0
SCALE_SLIDER_MAX = 2.0
# Translate: one metre either way, expressed in the stage's own unit.
TRANSLATE_SLIDER_METERS = 1.0

_UNIT_LABELS = (
    (1.0, "m"),
    (0.01, "cm"),
    (0.001, "mm"),
    (0.0254, "in"),
    (0.3048, "ft"),
)


def UnitLabel(stage):
    """
    The stage's linear unit as a short label: 'cm' for metersPerUnit =
    0.01, 'm' for 1, and so on; 'units' when it is none of the common
    ones. Translation avars are in stage units when
    avars:unitScaleFactor is 1 (schema.usda, unitScaleFactor doc).
    """
    if not stage:
        return "units"
    try:
        metersPerUnit = float(UsdGeom.GetStageMetersPerUnit(stage))
    except Exception:
        return "units"
    for value, label in _UNIT_LABELS:
        if math.isclose(metersPerUnit, value, rel_tol=1e-6):
            return label
    return "units"


def UnitsPerMeter(stage):
    """How many stage units make one metre (100 for a centimetre stage)."""
    if not stage:
        return 1.0
    try:
        metersPerUnit = float(UsdGeom.GetStageMetersPerUnit(stage))
    except Exception:
        return 1.0
    if metersPerUnit <= 0.0:
        return 1.0
    return 1.0 / metersPerUnit


def ValueFamily(attr):
    """Which widget family an attribute's value type belongs to."""
    typeName = attr.GetTypeName()
    if typeName.isArray:
        return VALUE_OTHER
    scalar = typeName.scalarType
    text = str(scalar) if scalar else str(typeName)
    if text in _FLOAT_TYPES:
        return VALUE_FLOAT
    if text in _INT_TYPES:
        return VALUE_INT
    if text == "bool":
        return VALUE_BOOL
    if text == "token":
        return VALUE_TOKEN
    if text == "string":
        return VALUE_STRING
    return VALUE_OTHER


def IsAnimated(attr):
    """True when a key outranks the attribute's default anywhere."""
    if not attr:
        return False
    if attr.GetNumTimeSamples() > 0:
        return True
    try:
        return bool(attr.HasSpline())
    except AttributeError:
        return False


def KindOf(attr):
    """
    The channel kind for an avar attribute: the schema's own channels are
    fixed, and anything else is custom.
    """
    return _SCHEMA_KINDS.get(attr.GetName(), KIND_CUSTOM)


def FallbackValue(prim, attr):
    """
    The value "reset" returns a channel to: the schema fallback (the rig's
    rest pose), or for a custom avar the composed default opinion, or
    the natural zero for its kind when there is neither.
    """
    definition = prim.GetPrimDefinition()
    if definition is not None:
        try:
            fallback = definition.GetAttributeFallbackValue(attr.GetName())
        except Exception:
            fallback = None
        if fallback is not None:
            return fallback
    authored = attr.Get(Usd.TimeCode.Default())
    if authored is not None:
        return authored
    family = ValueFamily(attr)
    if family == VALUE_FLOAT:
        return 1.0 if KindOf(attr) == KIND_SCALE else 0.0
    if family == VALUE_INT:
        return 0
    if family == VALUE_BOOL:
        return False
    if family in (VALUE_TOKEN, VALUE_STRING):
        return ""
    return None


class Channel(object):
    """One editable avar on one prim, with everything a widget needs."""

    def __init__(self, prim, attr, stage):
        self.prim = prim
        self.attr = attr
        self.name = attr.GetName()
        # "tx" for "avars:tx"; the panel shows the short name and keeps the
        # full one in the tooltip.
        prefix = RigPrefixOf(self.name) or AVAR_PREFIX
        self.shortName = self.name[len(prefix):]
        self.kind = KindOf(attr)
        self.family = ValueFamily(attr)
        self.custom = bool(attr.IsCustom())
        self.fallback = FallbackValue(prim, attr)
        self.allowedTokens = []
        if self.family == VALUE_TOKEN:
            tokens = attr.GetMetadata("allowedTokens")
            if tokens:
                self.allowedTokens = [str(t) for t in tokens]
        self.unit = self._Unit(stage)
        self.editable = self.family != VALUE_OTHER

    def _Unit(self, stage):
        if self.kind == KIND_TRANSLATE:
            label = UnitLabel(stage)
            scale = self.prim.GetAttribute(AVAR_PREFIX + "unitScaleFactor")
            factor = scale.Get() if scale else None
            if factor is not None and not math.isclose(float(factor), 1.0):
                # The avar is multiplied by unitScaleFactor before it
                # becomes a distance, so the label says so rather than
                # claiming a unit the value is not in.
                return "x%g %s" % (float(factor), label)
            return label
        if self.kind == KIND_ROTATE:
            return "deg"
        return ""

    def Value(self, time):
        return self.attr.Get(time)

    def Animated(self):
        return IsAnimated(self.attr)

    def SliderRange(self, stage, value=None):
        """
        (low, high) for a slider over this channel, or None when a slider
        makes no sense (tokens, strings, non-scalars). Widened to hold
        `value` so a typed number never lands outside the slider.
        """
        if self.family == VALUE_FLOAT:
            if self.kind == KIND_TRANSLATE:
                span = TRANSLATE_SLIDER_METERS * UnitsPerMeter(stage)
                low, high = -span, span
            elif self.kind == KIND_ROTATE:
                low, high = -ROTATE_SLIDER_DEGREES, ROTATE_SLIDER_DEGREES
            elif self.kind == KIND_SCALE:
                low, high = 0.0, SCALE_SLIDER_MAX
            elif self.name == AVAR_PREFIX + "unitScaleFactor":
                low, high = 0.0, SCALE_SLIDER_MAX
            else:
                # A custom float: the biped's ikfk dial is 0 = FK, 1 = IK,
                # and a blend weight is the ordinary meaning of a bare
                # float avar. Anything outside widens the range below.
                low, high = 0.0, 1.0
        elif self.family == VALUE_INT:
            low, high = 0.0, 10.0
        else:
            return None
        if value is not None:
            try:
                v = float(value)
            except (TypeError, ValueError):
                v = None
            if v is not None and math.isfinite(v):
                if v < low:
                    low = v
                if v > high:
                    high = v
        return (low, high)

    def SpinStep(self):
        if self.kind == KIND_ROTATE:
            return 1.0
        if self.kind in (KIND_SCALE, KIND_CUSTOM):
            return 0.01 if self.family == VALUE_FLOAT else 1
        if self.family == VALUE_INT:
            return 1
        return 0.1

    def Decimals(self):
        return 3 if self.family == VALUE_FLOAT else 0

    def Coerce(self, value):
        """
        `value` as something Set() accepts for this attribute: a float
        for a float channel, an int for an int channel, and so on.
        Raises ValueError for a value that cannot be one.
        """
        if self.family == VALUE_FLOAT:
            v = float(value)
            if not math.isfinite(v):
                raise ValueError("%s: %r is not finite" % (self.name, value))
            if self.kind == KIND_SCALE:
                # The evaluator floors tiny scales (gizmoMath's
                # NormalizeAvarScale, mirroring avarScale.h); write what
                # will be shown rather than a value the rig will not use.
                v = gizmoMath.NormalizeAvarScale(v)
            return v
        if self.family == VALUE_INT:
            return int(value)
        if self.family == VALUE_BOOL:
            if isinstance(value, str):
                return value.strip().lower() in ("1", "true", "yes", "on")
            return bool(value)
        if self.family in (VALUE_TOKEN, VALUE_STRING):
            return str(value)
        raise ValueError("%s: not an editable channel" % self.name)

    def __repr__(self):
        return "<Channel %s %s %s>" % (self.name, self.kind, self.family)


def HasAvars(prim):
    """True when the prim carries at least one `avars:*` attribute."""
    if not prim or not prim.IsValid() or prim.IsPseudoRoot():
        return False
    return any(RigPrefixOf(a.GetName()) is not None
               for a in prim.GetAttributes())


def DiscoverChannels(prim, stage=None):
    """
    Every avar channel on `prim`, ordered for display: translate, rotate,
    scale, custom, other; the schema's channels in their canonical order
    and custom ones alphabetically after them.

    Returns (channels, hidden) where `hidden` names the avars that are
    present but not editable here (avars:defaultSpace and any array or
    matrix typed avar), so the panel can say they exist.
    """
    if not prim or not prim.IsValid():
        return [], []
    stage = stage or prim.GetStage()
    channels = []
    hidden = []
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if RigPrefixOf(name) is None:
            continue
        if name in _HIDDEN or ValueFamily(attr) == VALUE_OTHER:
            hidden.append(name)
            continue
        channels.append(Channel(prim, attr, stage))

    def key(channel):
        return (KIND_ORDER.index(channel.kind),
                _SCHEMA_RANK.get(channel.name, len(_SCHEMA_RANK)),
                channel.name)

    channels.sort(key=key)
    return channels, sorted(hidden)


def FocusPrim(usdviewApi):
    """
    The prim the editor edits, and how many OTHER prims are selected.

    usdview's focus prim, with the gizmo's fallback (gizmoUI._FocusPrim):
    the selection model keeps the pseudo-root focused when a prim is
    added to an already-cleared selection, which is what every script
    driving usdview does. With several prims selected the editor edits
    the focus prim only and reports the count so the panel can say so.
    """
    api = usdviewApi
    selected = [p for p in (api.selectedPrims or [])
                if p and p.IsValid() and not p.IsPseudoRoot()]
    prim = api.prim
    if not (prim and prim.IsValid() and not prim.IsPseudoRoot()):
        prim = selected[-1] if selected else None
    if prim is None:
        return None, 0
    others = len([p for p in selected if p.GetPath() != prim.GetPath()])
    return prim, others


# ---------------------------------------------------------------------------
# Writing
# ---------------------------------------------------------------------------

def _EditTargetSpec(attr):
    stage = attr.GetStage()
    target = stage.GetEditTarget()
    layer = target.GetLayer()
    if layer is None:
        return None, None
    return layer, layer.GetAttributeAtPath(target.MapToSpecPath(attr.GetPath()))


def PromoteAnimationToEditTarget(attr):
    """
    Copy an animated channel's keys into the edit target before a key
    is authored there, when the keys live in a weaker layer.

    USD does not merge time samples or splines across layers: the
    strongest layer with any wins outright. usdview edits the session
    layer, so keying frame 36 of a channel whose keys are in the file
    would leave the session layer holding ONE sample -- and every other
    frame of the shot would read that one value. That is the "silently
    stomp an animated channel" failure. Copying the resolved keys first
    turns the edit into what the artist meant: this frame changes, the
    rest of the curve stays.

    Returns True when something was copied.
    """
    layer, spec = _EditTargetSpec(attr)
    if layer is None:
        return False
    if spec is not None:
        if spec.HasSpline() or layer.ListTimeSamplesForPath(spec.path):
            return False           # the edit target already owns the keys
    copied = False
    try:
        hasSpline = attr.HasSpline()
    except AttributeError:
        hasSpline = False
    if hasSpline:
        from pxr import Ts
        attr.SetSpline(Ts.Spline(attr.GetSpline()))
        copied = True
    elif attr.GetNumTimeSamples() > 0:
        # Read EVERY sample before writing any: the first Set() gives the
        # edit target time samples of its own, and from then on the
        # composed read resolves from that layer alone -- so reading and
        # writing in one pass copies sample 1 into every later frame.
        samples = [(t, attr.Get(t)) for t in attr.GetTimeSamples()]
        with Sdf.ChangeBlock():
            for t, value in samples:
                if value is not None:
                    attr.Set(value, Usd.TimeCode(t))
        copied = True
    return copied


_KEYED_SOURCES = None


def _KeyedSources():
    """The resolve-info sources that mean "a key won", by this USD."""
    global _KEYED_SOURCES
    if _KEYED_SOURCES is None:
        names = ("ResolveInfoSourceTimeSamples", "ResolveInfoSourceSpline",
                 "ResolveInfoSourceValueClips")
        _KEYED_SOURCES = tuple(getattr(Usd, n) for n in names
                               if hasattr(Usd, n))
    return _KEYED_SOURCES


def _DefaultWarning(attr, time, hadKeys):
    """
    After a Default-mode write: what the viewport will actually show.

    USD resolves the STRONGEST layer with any opinion, whatever kind it
    is. So a default written where the keys already are is outranked by
    them (the value is invisible), while a default written in a stronger
    layer -- usdview's session layer over keys in the file -- HIDES the
    keys, and every frame of the shot now reads it. Either is worth a
    sentence; they are different sentences. `hadKeys` is whether the
    channel read as animated before the write, since afterwards the
    hidden keys no longer show through GetNumTimeSamples.
    """
    if not hadKeys:
        return None
    try:
        source = attr.GetResolveInfo(time).GetSource()
    except Exception:
        source = None
    if source in _KeyedSources():
        return ("%s: the default is outranked by the channel's keys; the "
                "viewport shows the key. Use Animation to key this frame."
                % attr.GetPath())
    return ("%s: this default now HIDES the channel's keys in a weaker "
            "layer, so every frame reads it. Undo, or use Animation to "
            "key this frame instead." % attr.GetPath())


def WriteValue(channel, value, time, mode):
    """
    Author `value` on the channel. Returns a warning string or None.

    WRITE_ANIMATION keys the current frame through gizmoMath.SetAnimated
    (a spline knot, or a time sample on a channel that already has
    them), after promoting a weaker layer's keys into the edit target
    (see PromoteAnimationToEditTarget). WRITE_DEFAULT authors the
    default, and says so when a key outranks it -- the value is then
    invisible in the viewport, which is worth a sentence rather than a
    drag that looks broken.

    Non-numeric channels (rotation order, a bool, a string) are always
    written as the default: keying a rotation order per frame is not
    something a rig means, and Ts cannot spline a token anyway.
    """
    attr = channel.attr
    value = channel.Coerce(value)
    if channel.family not in (VALUE_FLOAT, VALUE_INT) or \
            mode == WRITE_DEFAULT:
        hadKeys = IsAnimated(attr) if mode == WRITE_DEFAULT else False
        attr.Set(value)
        return _DefaultWarning(attr, time, hadKeys)
    if not time.IsDefault():
        PromoteAnimationToEditTarget(attr)
    gizmoMath.SetAnimated(attr, value, time)
    return None


def ResetValue(channel, time, mode):
    """
    Put the channel back to the rig's rest: what attr.Clear() leaves.

    A channel with keys, in Animation mode, gets its fallback KEYED at
    the current frame instead -- Clear() would only remove the default
    opinion, which the keys outrank, and the artist would see nothing
    happen. Everything else is attr.Clear() at the edit target, so a
    session override of a file value goes away and the file value
    shows through, which is the rest pose the rig was built with.
    """
    attr = channel.attr
    if channel.Animated() and mode == WRITE_ANIMATION and \
            channel.family in (VALUE_FLOAT, VALUE_INT) and \
            not time.IsDefault():
        PromoteAnimationToEditTarget(attr)
        gizmoMath.SetAnimated(attr, channel.Coerce(channel.fallback), time)
        return None
    hadKeys = channel.Animated()
    attr.Clear()
    if hadKeys and mode == WRITE_DEFAULT:
        # Clearing a default under keys changes nothing the artist can
        # see; say why, and where the reset that would work lives.
        return ("%s: the channel's keys outrank the cleared default; use "
                "Animation to key the rest value at this frame."
                % attr.GetPath())
    return None


class EditScope(object):
    """
    One undoable edit over a set of channels: snapshots them on entry,
    and on exit pushes the difference onto the shared undo stack (or
    nothing, when nothing changed). A slider drag opens one of these on
    press and closes it on release, so Ctrl+Z undoes the drag, not the
    last mouse sample.
    """

    def __init__(self, stage, channels, undoStack, label):
        self._recorder = rigExecUndo.EditRecorder(
            stage, [c.attr.GetPath() for c in channels])
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
        self.Commit()
        return False

    def Commit(self):
        edit = self._recorder.Commit(self._label)
        self.edit = edit
        if edit is not None and self._undo is not None:
            self._undo.Push(edit)
        return edit

    def Abort(self):
        self._recorder.Abort()
