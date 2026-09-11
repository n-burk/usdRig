#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#

"""
Applied API Schema Catalog.

Discovers every applied API schema registered in the current USD build and
applies them to prims by name. The node creation hotbox lists these alongside
the concrete prim types from the node libraries: choosing a prim type creates
a prim, choosing an API schema applies it to the selected prims.

Enumeration goes through ``Plug.Registry`` rather than
``Tf.Type.GetAllDerivedTypes``. The latter only reports schemas whose shared
libraries happen to be loaded already (5 of 51 in a freshly imported process),
while ``Plug.Registry.GetAllDerivedTypes`` forces plugin discovery without
dlopen'ing the libraries. That is the same path UsdSchemaRegistry itself takes
internally, so it also covers codeless schemas, which have no Python class.
"""

from pxr import Plug, Tf, Usd

# Marker stored in hotbox item data to tell API schema entries apart from
# prim-type entries. See GraphView._createNodeFromHotbox.
API_SCHEMA_KIND = "apiSchema"

# Outcomes of applying a schema to one prim.
#
# APPLIED and ALREADY_PRESENT both leave the prim carrying the schema, but only
# APPLIED authored anything, and only APPLIED may be undone - see apply_named.
APPLIED = "applied"
ALREADY_PRESENT = "alreadyPresent"
SKIPPED = "skipped"

# Instance names to try per multiple-apply schema before giving up. Only a
# pathological prim (dozens of instances, or a schema whose plugInfo restricts
# the allowed names) ever gets past the first few.
_MAX_INSTANCE_NAME_TRIES = 64

_catalog = None


def _terse(error):
    """Condense a Tf.ErrorException into something that fits in a popup.

    Tf formats exceptions across several lines with a file, line number and
    the internal function that raised. Only the trailing message is useful to
    a user, so keep that and drop the diagnostic preamble.
    """
    text = str(error).strip()
    lastLine = text.splitlines()[-1].strip() if text else ""
    _, separator, message = lastLine.rpartition(" : ")
    if separator:
        lastLine = message.strip()
    return lastLine.strip("'") or "could not be applied"


class ApiSchemaInfo:
    """A single applied API schema available for authoring.

    Attributes:
        identifier: USD schema identifier, e.g. "PhysicsRigidBodyAPI". This is
            what ApplyAPI takes - never the C++ class name ("UsdPhysics...").
        isMultipleApply: True if applying it requires an instance name.
        plugin: Name of the plugin defining it, used to group the hotbox list.
        canOnlyApplyTo: Prim type names the schema restricts itself to, empty
            if it applies to anything.
    """

    def __init__(self, identifier, isMultipleApply, plugin, canOnlyApplyTo):
        self.identifier = identifier
        self.isMultipleApply = isMultipleApply
        self.plugin = plugin
        self.canOnlyApplyTo = canOnlyApplyTo

    def __repr__(self):
        kind = "multiple" if self.isMultipleApply else "single"
        return f"<ApiSchemaInfo {self.identifier} ({kind}-apply, {self.plugin})>"


def get_api_schemas(refresh=False):
    """Return every applied API schema in this build, sorted for display.

    Sorted by plugin then identifier so that hotbox entries sharing a group
    stay adjacent - the list widget emits a group header whenever the group
    changes, so a non-contiguous group would repeat its header.

    Args:
        refresh: Rebuild the cache instead of reusing it. Only useful if
            plugins were registered after the first call.

    Returns:
        List[ApiSchemaInfo]: Possibly empty if schema discovery failed.
    """
    global _catalog
    if _catalog is not None and not refresh:
        return _catalog

    registry = Usd.SchemaRegistry
    schemas = []
    try:
        plugRegistry = Plug.Registry()
        # Static method, and it must be based off UsdSchemaBase rather than
        # UsdAPISchemaBase to match USD's own enumeration.
        derivedTypes = Plug.Registry.GetAllDerivedTypes(Tf.Type.Find(Usd.SchemaBase))
    except Exception as e:
        Tf.Warn(f"Failed to enumerate API schemas: {e}")
        _catalog = []
        return _catalog

    for schemaType in derivedTypes:
        try:
            kind = registry.GetSchemaKind(schemaType)
            if kind not in (
                Usd.SchemaKind.SingleApplyAPI,
                Usd.SchemaKind.MultipleApplyAPI,
            ):
                continue

            info = registry.FindSchemaInfo(schemaType)
            if info is None:
                continue

            plugin = plugRegistry.GetPluginForType(schemaType)
            schemas.append(
                ApiSchemaInfo(
                    identifier=info.identifier,
                    isMultipleApply=(kind == Usd.SchemaKind.MultipleApplyAPI),
                    plugin=plugin.name if plugin else "unknown",
                    canOnlyApplyTo=list(
                        registry.GetAPISchemaCanOnlyApplyToTypeNames(info.identifier)
                    ),
                )
            )
        except Exception as e:
            Tf.Warn(f"Skipping API schema {schemaType}: {e}")

    schemas.sort(key=lambda schema: (schema.plugin, schema.identifier))
    _catalog = schemas
    return _catalog


def is_multiple_apply(identifier):
    """True if *identifier* names a multiple-apply API schema."""
    try:
        return bool(Usd.SchemaRegistry.IsMultipleApplyAPISchema(identifier))
    except Exception:
        return False


def make_instance_name(prim, identifier):
    """Pick an unused, legal instance name for a multiple-apply schema.

    Derived from the schema identifier ("CollectionAPI" -> "collection"), with
    a numeric suffix when that name is already applied to the prim. Names the
    schema disallows - those colliding with its own property names, or outside
    an ``apiSchemaAllowedInstanceNames`` list - are skipped.

    Args:
        prim: The prim the schema will be applied to.
        identifier: Multiple-apply schema identifier.

    Returns:
        str: The instance name, or "" if no legal name could be found.
    """
    registry = Usd.SchemaRegistry
    base = identifier[: -len("API")] if identifier.endswith("API") else identifier
    if not base:
        base = "instance"
    base = base[0].lower() + base[1:]

    applied = set()
    if prim and prim.IsValid():
        applied = {str(schema) for schema in prim.GetAppliedSchemas()}

    for attempt in range(_MAX_INSTANCE_NAME_TRIES):
        candidate = base if attempt == 0 else f"{base}{attempt}"
        if f"{identifier}:{candidate}" in applied:
            continue
        try:
            if not registry.IsAllowedAPISchemaInstanceName(identifier, candidate):
                continue
        except Exception:
            continue
        return candidate

    return ""


def has_applied_schema(prim, identifier, instanceName=""):
    """True if *identifier* is already applied to *prim*.

    ApplyAPI is idempotent: applying a schema a prim already carries is a no-op
    that still returns True. This is what tells that no-op apart from a real
    authoring edit, so that undo only removes what was actually added. Undoing
    a no-op would otherwise strip pre-existing state, and where the schema came
    from a reference it would author a new "delete apiSchemas" override
    suppressing it.
    """
    if not prim or not prim.IsValid():
        return False

    token = f"{identifier}:{instanceName}" if instanceName else identifier
    return token in {str(schema) for schema in prim.GetAppliedSchemas()}


def apply_named(prim, identifier, instanceName=""):
    """Apply *identifier* to *prim* using an already chosen instance name.

    Used for redo, where reusing the original instance name matters. Callers
    authoring a schema for the first time should use apply_to_prim instead.

    Args:
        prim: Target prim.
        identifier: Schema identifier.
        instanceName: Instance name for multiple-apply schemas, "" otherwise.

    Returns:
        (str, str): One of APPLIED / ALREADY_PRESENT / SKIPPED, and the reason
        when the prim was skipped.
    """
    if not prim or not prim.IsValid():
        return SKIPPED, "invalid prim"

    if has_applied_schema(prim, identifier, instanceName):
        return ALREADY_PRESENT, ""

    try:
        # CanApplyAPI has to be consulted explicitly: ApplyAPI itself does not
        # enforce canOnlyApplyTo and will happily author a schema onto a prim
        # type the schema forbids.
        if instanceName:
            allowed = prim.CanApplyAPI(identifier, instanceName)
        else:
            allowed = prim.CanApplyAPI(identifier)
        if not allowed:
            return SKIPPED, allowed.whyNot or "not applicable to this prim"

        if instanceName:
            applied = prim.ApplyAPI(identifier, instanceName)
        else:
            applied = prim.ApplyAPI(identifier)
    except Tf.ErrorException as e:
        return SKIPPED, _terse(e)

    if not applied:
        return SKIPPED, "ApplyAPI failed"
    return APPLIED, ""


def apply_to_prim(prim, identifier):
    """Apply *identifier* to *prim*, choosing an instance name if one is needed.

    Args:
        prim: Target prim.
        identifier: Schema identifier.

    Returns:
        (str, str, str): One of APPLIED / ALREADY_PRESENT / SKIPPED, the
        instance name used ("" for single-apply schemas), and the reason when
        the prim was skipped.
    """
    if not prim or not prim.IsValid():
        return SKIPPED, "", "invalid prim"

    instanceName = ""
    if is_multiple_apply(identifier):
        instanceName = make_instance_name(prim, identifier)
        if not instanceName:
            return SKIPPED, "", "no unused instance name available"

    status, reason = apply_named(prim, identifier, instanceName)
    return status, instanceName, reason


def remove_from_prim(prim, identifier, instanceName=""):
    """Remove a previously applied API schema. Used to undo apply_to_prim."""
    if not prim or not prim.IsValid():
        return False

    try:
        if instanceName:
            return bool(prim.RemoveAPI(identifier, instanceName))
        return bool(prim.RemoveAPI(identifier))
    except Tf.ErrorException as e:
        Tf.Warn(f"Failed to remove {identifier} from {prim.GetPath()}: {e}")
        return False
