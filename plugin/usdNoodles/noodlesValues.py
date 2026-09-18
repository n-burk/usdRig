#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#

"""Pure value model for the node graph's inline attribute value cells.

No Qt, no GL, no GraphView: everything here is arithmetic over a
``Usd.Prim`` plus a property name, so the whole thing is unit-testable
headless (``testenv/testUsdNoodlesValueModel.py``).

Scope is deliberately narrow.  Only *simple* attribute types get a value
cell -- bools, ints, floats/doubles, strings, tokens and small vectors.
Matrices, arrays, asset paths, quaternions, timecodes and everything else
keep the row they have today: label + pin, no value, no editor.  The
whitelist below IS that rule; nothing else in the feature re-decides it.
"""

import math
from collections import namedtuple

from pxr import Gf, Sdf, Usd

from ._schema_pin_names import get_attribute_type_name


# ---------------------------------------------------------------------------
# The type whitelist
#
# Membership is tested against ``str(attr.GetTypeName())``, which is what
# ``_schema_pin_names.get_attribute_type_name`` returns.  Array types
# stringify with a ``[]`` suffix ("float3[]"), so they fall out of every set
# here automatically and need no separate rejection.
# ---------------------------------------------------------------------------

SCALAR_BOOL = frozenset({"bool"})
SCALAR_INT = frozenset({"int", "int64", "uint", "uint64"})
SCALAR_FLOAT = frozenset({"half", "float", "double"})
SCALAR_TEXT = frozenset({"string", "token"})

VEC_INT = {"int2": 2, "int3": 3, "int4": 4}

VEC_FLOAT = {
    "half2": 2,
    "half3": 3,
    "half4": 4,
    "float2": 2,
    "float3": 3,
    "float4": 4,
    "double2": 2,
    "double3": 3,
    "double4": 4,
    # role-typed 3-vectors
    "point3f": 3,
    "point3d": 3,
    "point3h": 3,
    "normal3f": 3,
    "normal3d": 3,
    "normal3h": 3,
    "vector3f": 3,
    "vector3d": 3,
    "vector3h": 3,
    "color3f": 3,
    "color3d": 3,
    "color3h": 3,
    "color4f": 4,
    "color4d": 4,
    "color4h": 4,
    "texCoord2f": 2,
    "texCoord2d": 2,
    "texCoord2h": 2,
    "texCoord3f": 3,
    "texCoord3d": 3,
    "texCoord3h": 3,
}

VALUE_TYPE_WHITELIST = (
    SCALAR_BOOL
    | SCALAR_INT
    | SCALAR_FLOAT
    | SCALAR_TEXT
    | frozenset(VEC_INT)
    | frozenset(VEC_FLOAT)
)

COLOR_TYPES = frozenset(
    {"color3f", "color3d", "color3h", "color4f", "color4d", "color4h"}
)

# Cell kinds.  One per interaction model, not one per USD type.
KIND_BOOL = "bool"
KIND_INT = "int"
KIND_FLOAT = "float"
KIND_TEXT = "text"
KIND_TOKEN_ENUM = "token_enum"

# Value states, used for styling (see noodlesSettings value*Color).
STATE_AUTHORED = "authored"
STATE_FALLBACK = "fallback"
STATE_ANIMATED = "animated"
STATE_CONNECTED = "connected"
STATE_READONLY = "readonly"

# Vector component constructors, by type name.
_VEC_CLASS = {
    "int2": Gf.Vec2i,
    "int3": Gf.Vec3i,
    "int4": Gf.Vec4i,
    "half2": Gf.Vec2h,
    "half3": Gf.Vec3h,
    "half4": Gf.Vec4h,
    "float2": Gf.Vec2f,
    "float3": Gf.Vec3f,
    "float4": Gf.Vec4f,
    "double2": Gf.Vec2d,
    "double3": Gf.Vec3d,
    "double4": Gf.Vec4d,
    "point3f": Gf.Vec3f,
    "point3d": Gf.Vec3d,
    "point3h": Gf.Vec3h,
    "normal3f": Gf.Vec3f,
    "normal3d": Gf.Vec3d,
    "normal3h": Gf.Vec3h,
    "vector3f": Gf.Vec3f,
    "vector3d": Gf.Vec3d,
    "vector3h": Gf.Vec3h,
    "color3f": Gf.Vec3f,
    "color3d": Gf.Vec3d,
    "color3h": Gf.Vec3h,
    "color4f": Gf.Vec4f,
    "color4d": Gf.Vec4d,
    "color4h": Gf.Vec4h,
    "texCoord2f": Gf.Vec2f,
    "texCoord2d": Gf.Vec2d,
    "texCoord2h": Gf.Vec2h,
    "texCoord3f": Gf.Vec3f,
    "texCoord3d": Gf.Vec3d,
    "texCoord3h": Gf.Vec3h,
}


def is_value_type(type_name):
    """Whether *type_name* gets a value cell at all."""
    return str(type_name) in VALUE_TYPE_WHITELIST


def component_count(type_name):
    """Number of editable sub-cells for *type_name* (0 when not supported)."""
    name = str(type_name)
    if name in VEC_INT:
        return VEC_INT[name]
    if name in VEC_FLOAT:
        return VEC_FLOAT[name]
    if name in VALUE_TYPE_WHITELIST:
        return 1
    return 0


def is_color_type(type_name):
    return str(type_name) in COLOR_TYPES


def cell_kind(type_name, has_allowed_tokens=False):
    """The interaction model a row uses, or "" when it gets no cell."""
    name = str(type_name)
    if name in SCALAR_BOOL:
        return KIND_BOOL
    if name in SCALAR_INT or name in VEC_INT:
        return KIND_INT
    if name in SCALAR_FLOAT or name in VEC_FLOAT:
        return KIND_FLOAT
    if name in SCALAR_TEXT:
        return KIND_TOKEN_ENUM if has_allowed_tokens else KIND_TEXT
    return ""


def is_mungable(kind):
    """Whether press-drag changes the value (floats, doubles, ints)."""
    return kind in (KIND_INT, KIND_FLOAT)


# ---------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------


def read_components(prim, property_name, time_code=None):
    """Resolved value of *property_name* as a tuple of components.

    A connected or schema-fallback attribute still resolves: ``attr.Get``
    composes the answer USD would give a consumer, which is exactly the
    number a reader wants to see on the node.
    """
    attr = prim.GetAttribute(property_name) if prim else None
    if not attr or not attr.IsValid():
        return ()
    type_name = str(attr.GetTypeName())
    count = component_count(type_name)
    if count == 0:
        return ()

    if time_code is None:
        time_code = Usd.TimeCode.Default()
    try:
        value = attr.Get(time_code)
    except Exception:
        value = None
    if value is None:
        value = _zero_value(type_name, count)

    if count == 1:
        return (_coerce_scalar(value, type_name),)
    try:
        return tuple(_coerce_scalar(value[i], type_name) for i in range(count))
    except (TypeError, IndexError):
        return tuple(_zero_component(type_name) for _ in range(count))


def _zero_component(type_name):
    if type_name in SCALAR_BOOL:
        return False
    if type_name in SCALAR_TEXT:
        return ""
    if type_name in SCALAR_INT or type_name in VEC_INT:
        return 0
    return 0.0


def _zero_value(type_name, count):
    if count == 1:
        return _zero_component(type_name)
    return [_zero_component(type_name)] * count


def _coerce_scalar(value, type_name):
    if type_name in SCALAR_BOOL:
        return bool(value)
    if type_name in SCALAR_TEXT:
        return str(value)
    if type_name in SCALAR_INT or type_name in VEC_INT:
        try:
            return int(value)
        except (TypeError, ValueError):
            return 0
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def allowed_tokens(prim, property_name):
    """The ``allowedTokens`` list for *property_name*, or ``[]``.

    Falls back to the composed prim definition so an applied-API-schema
    attribute with no authored opinion (and therefore possibly no runtime
    attribute) still offers its enumeration.
    """
    if not prim:
        return []
    attr = prim.GetAttribute(property_name)
    if attr and attr.IsValid():
        try:
            toks = attr.GetMetadata("allowedTokens")
        except Exception:
            toks = None
        if toks:
            return [str(t) for t in toks]

    prim_def = prim.GetPrimDefinition()
    if prim_def is not None:
        getter = getattr(prim_def, "GetPropertyMetadata", None)
        if getter is not None:
            try:
                toks = getter(property_name, "allowedTokens")
            except Exception:
                toks = None
            if toks:
                return [str(t) for t in toks]
        # Older USD builds without GetPropertyMetadata: read the spec.
        try:
            spec = prim_def.GetSchemaAttributeSpec(property_name)
        except Exception:
            spec = None
        if spec is not None:
            try:
                toks = spec.GetInfo("allowedTokens")
            except Exception:
                toks = None
            if toks:
                return [str(t) for t in toks]
    return []


def value_state(prim, attr, editable=True):
    """Styling bucket for a row: authored / fallback / animated / ..."""
    if not attr or not attr.IsValid():
        return STATE_READONLY
    try:
        if attr.HasAuthoredConnections():
            return STATE_CONNECTED
    except Exception:
        pass
    if not editable:
        return STATE_READONLY
    try:
        if attr.GetNumTimeSamples() > 0:
            return STATE_ANIMATED
    except Exception:
        pass
    try:
        if attr.HasAuthoredValue():
            return STATE_AUTHORED
    except Exception:
        pass
    return STATE_FALLBACK


def value_is_editable(stage, prim, attr):
    """(ok, reason) for authoring *attr*.

    A schema fallback is editable -- "no authored opinion" is not
    read-only, it is the state the first edit leaves behind.  A connected
    attribute is not: its value comes from somewhere else, and the honest
    answer is to send the user to that source.
    """
    if not prim or not prim.IsValid():
        return False, "the prim is gone"
    if prim.IsInstanceProxy():
        return False, "inside an instance"
    if not attr or not attr.IsValid():
        return False, "no such attribute"
    if str(attr.GetTypeName()) not in VALUE_TYPE_WHITELIST:
        return False, "unsupported type"
    try:
        if attr.HasAuthoredConnections():
            return False, "driven by a connection"
    except Exception:
        pass
    layer = stage.GetEditTarget().GetLayer() if stage else None
    if layer is None or layer.expired or not layer.permissionToEdit:
        return False, "edit target is locked"
    return True, ""


# ---------------------------------------------------------------------------
# Formatting
# ---------------------------------------------------------------------------


def format_component(value, type_name, decimals=4):
    """One component as display text."""
    name = str(type_name)
    if name in SCALAR_BOOL:
        return "true" if value else "false"
    if name in SCALAR_TEXT:
        return str(value)
    if name in SCALAR_INT or name in VEC_INT:
        try:
            return str(int(value))
        except (TypeError, ValueError):
            return "0"
    try:
        f = float(value)
    except (TypeError, ValueError):
        return "0"
    if decimals is None:
        return "%.3g" % f
    if not math.isfinite(f):
        return "%.3g" % f
    return "%.*f" % (max(0, int(decimals)), f)


def middle_elide(text, max_width, measure, head=3, tail=2):
    """Middle-elide *text* with an ellipsis until it fits *max_width*.

    Keeps at least *head* leading and *tail* trailing characters, so the
    two ends a reader identifies a token by both survive.
    """
    if measure(text) <= max_width:
        return text
    n = len(text)
    if n <= head + tail + 1:
        return _hard_clip(text, max_width, measure)
    # Eat the middle outwards, one character at a time, until it fits.
    for keep in range(n - 1, head + tail - 1, -1):
        keep_head = max(head, (keep + 1) // 2)
        keep_tail = keep - keep_head
        if keep_tail < tail:
            keep_tail = tail
            keep_head = keep - tail
        if keep_head < head:
            break
        candidate = text[:keep_head] + "…" + text[n - keep_tail:]
        if measure(candidate) <= max_width:
            return candidate
    candidate = text[:head] + "…" + text[n - tail:]
    if measure(candidate) <= max_width:
        return candidate
    return _hard_clip(text, max_width, measure)


def _hard_clip(text, max_width, measure):
    for n in range(len(text), 0, -1):
        candidate = text[:n] + "…"
        if measure(candidate) <= max_width:
            return candidate
    return "…"


def fit_component(value, type_name, max_width, measure):
    """Display text for one component, narrowed until it fits *max_width*.

    Floats drop decimals before they elide (4 -> 3 -> 2 -> 1 -> ``%.3g``),
    because a shorter number is still a number while an elided one is not.
    Strings and tokens middle-elide.
    """
    name = str(type_name)
    if name in SCALAR_TEXT:
        return middle_elide(format_component(value, name), max_width, measure)

    if name in SCALAR_BOOL:
        return format_component(value, name)

    if name in SCALAR_INT or name in VEC_INT:
        text = format_component(value, name)
        if measure(text) <= max_width:
            return text
        compact = "%.3g" % float(value)
        if measure(compact) <= max_width:
            return compact
        return _hard_clip(text, max_width, measure)

    for decimals in (4, 3, 2, 1):
        text = format_component(value, name, decimals)
        if measure(text) <= max_width:
            return text
    text = format_component(value, name, None)
    if measure(text) <= max_width:
        return text
    return _hard_clip(text, max_width, measure)


def fit_components(components, type_name, max_width, measure):
    """Display text for every component of one value, SHARING a format.

    A vector whose components each chose their own decimal count -- because
    "1.0000" happens to measure narrower than "0.0000" -- reads as three
    unrelated numbers. One format for the whole value is what makes it read
    as one value.
    """
    name = str(type_name)
    if name in SCALAR_FLOAT or name in VEC_FLOAT:
        for decimals in (4, 3, 2, 1, None):
            texts = tuple(
                format_component(c, name, decimals) for c in components
            )
            if all(measure(t) <= max_width for t in texts):
                return texts
        return tuple(_hard_clip(t, max_width, measure) for t in texts)
    return tuple(fit_component(c, name, max_width, measure) for c in components)


def parse_component(text, type_name):
    """(ok, value) for typed text in a cell of *type_name*."""
    name = str(type_name)
    raw = text.strip()
    if name in SCALAR_BOOL:
        lowered = raw.lower()
        if lowered in ("1", "true", "yes", "on"):
            return True, True
        if lowered in ("0", "false", "no", "off"):
            return True, False
        return False, None
    if name in SCALAR_TEXT:
        return True, raw
    if name in SCALAR_INT or name in VEC_INT:
        try:
            return True, int(round(float(raw)))
        except (TypeError, ValueError):
            return False, None
    try:
        return True, float(raw)
    except (TypeError, ValueError):
        return False, None


# ---------------------------------------------------------------------------
# The mung ladder
# ---------------------------------------------------------------------------


def ladder_step(base, type_name):
    """World units of value per pixel of horizontal drag.

    Houdini-flavoured: the step tracks the magnitude of the value being
    dragged, so a value near 1 moves by 0.01/px (a 100 px drag adds 1.0)
    and a value near 100 moves by 1.0/px.  Integers always step exactly 1.
    """
    name = str(type_name)
    if name in SCALAR_INT or name in VEC_INT:
        return 1.0
    try:
        a = abs(float(base))
    except (TypeError, ValueError):
        a = 0.0
    if a < 1e-6:
        return 0.01
    decade = math.floor(math.log10(a))
    return min(max(10.0 ** (decade - 2), 1e-4), 1e3)


def mung_multiplier(shift=False, ctrl=False):
    """Modifier multiplier on the ladder step: Shift x10, Ctrl x0.1."""
    mult = 1.0
    if shift:
        mult *= 10.0
    if ctrl:
        mult *= 0.1
    return mult


def apply_delta(original, comp_index, value, type_name):
    """*original* with component *comp_index* replaced by *value*.

    Every other component is carried through byte-for-byte.  That matters
    for ``half``: reformatting an untouched component at 4 decimals and
    re-parsing it would quietly rewrite a value the user never touched.
    """
    name = str(type_name)
    components = list(original)
    if not (0 <= comp_index < len(components)):
        return tuple(components)
    if name in SCALAR_INT or name in VEC_INT:
        components[comp_index] = int(round(float(value)))
    elif name in SCALAR_BOOL:
        components[comp_index] = bool(value)
    elif name in SCALAR_TEXT:
        components[comp_index] = str(value)
    else:
        components[comp_index] = float(value)
    return tuple(components)


# ---------------------------------------------------------------------------
# Authoring
# ---------------------------------------------------------------------------


def pack_components(components, type_name):
    """Components back into the value USD wants for *type_name*."""
    name = str(type_name)
    count = component_count(name)
    if count == 0:
        raise ValueError("unsupported value type: %s" % name)
    if count == 1:
        return _coerce_scalar(components[0], name)
    cls = _VEC_CLASS.get(name)
    if cls is None:
        raise ValueError("unsupported vector type: %s" % name)
    return cls(*[_coerce_scalar(c, name) for c in components[:count]])


def write_value(
    stage, prim, property_name, components, time_code=None, mode="auto", warn=True
):
    """Author *components* onto *property_name*.  (ok, reason).

    Where: the stage's current edit target, always -- no ``SetEditTarget``,
    no ``EditContext``, matching every other authoring site in this plugin
    and in the avar editor.

    When: Default, unless the attribute already carries time samples, in
    which case a sample at *time_code*.  ``uniform`` always authors at
    Default, because a uniform attribute cannot vary over time at all.

    How: ``attr.Set`` and nothing else.  There is no ``CreateAttribute``
    anywhere in this feature -- every row that can be edited came from the
    composed prim definition, so the attribute exists, and ``Set`` on a
    schema attribute with no authored opinion creates the spec itself.
    """
    attr = prim.GetAttribute(property_name) if prim else None
    ok, reason = value_is_editable(stage, prim, attr)
    if not ok:
        return False, reason

    type_name = str(attr.GetTypeName())
    try:
        value = pack_components(components, type_name)
    except (ValueError, IndexError, TypeError) as e:
        return False, str(e)

    if warn:
        _warn_edit_target(stage)

    if time_code is None:
        time_code = Usd.TimeCode.Default()

    try:
        with Sdf.ChangeBlock():
            if attr.GetVariability() == Sdf.VariabilityUniform:
                attr.Set(value)
            elif mode == "default":
                attr.Set(value)
            elif mode == "animation" or attr.GetNumTimeSamples() > 0:
                attr.Set(value, time_code)
            else:
                attr.Set(value)
    except Exception as e:  # a locked layer USD refuses late, a bad value
        return False, str(e)
    return True, ""


def _warn_edit_target(stage):
    """Sanity-warn once per committed edit, never per mung frame."""
    try:
        from .nodeGraph import warn_if_non_persistent_edit_target

        warn_if_non_persistent_edit_target(stage)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# One row's worth of value state
# ---------------------------------------------------------------------------

ValueRow = namedtuple(
    "ValueRow",
    "pin_name property_name type_name kind count components texts tokens "
    "state editable reason has_swatch mungable",
)


def build_value_row(stage, prim, pin_name, property_name, time_code=None):
    """A ``ValueRow`` for *property_name*, or None when it gets no cell.

    This is the single decision point for "does this row show a value".
    Type suppression lives here; the row-slot / fold / collapse
    suppressions are geometry and live with the geometry.
    """
    if not prim or not prim.IsValid():
        return None
    type_name = get_attribute_type_name(prim, property_name)
    if type_name not in VALUE_TYPE_WHITELIST:
        return None
    attr = prim.GetAttribute(property_name)
    if not attr or not attr.IsValid():
        return None

    tokens = allowed_tokens(prim, property_name) if type_name == "token" else []
    kind = cell_kind(type_name, bool(tokens))
    if not kind:
        return None
    count = component_count(type_name)
    components = read_components(prim, property_name, time_code)
    if len(components) != count:
        return None

    editable, reason = value_is_editable(stage, prim, attr)
    state = value_state(prim, attr, editable)
    texts = tuple(format_component(c, type_name) for c in components)
    return ValueRow(
        pin_name=pin_name,
        property_name=property_name,
        type_name=type_name,
        kind=kind,
        count=count,
        components=components,
        texts=texts,
        tokens=tuple(tokens),
        state=state,
        editable=editable,
        reason=reason,
        has_swatch=is_color_type(type_name),
        mungable=editable and is_mungable(kind),
    )
