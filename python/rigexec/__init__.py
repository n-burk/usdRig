"""RigExec Python API -- programmatic rig authoring and evaluation.

This package wraps the native ``_rigexec`` module (built from
libs/rigExecRigging and libs/rigExec) so a Python tool can create and wire an
entire RigExec rig on a ``pxr.Usd.Stage`` and evaluate it in-process, the way
other OpenUSD python bindings are used:

    import pxr
    from pxr import Usd, UsdGeom
    import rigexec

    stage = Usd.Stage.CreateInMemory()

    dots = UsdGeom.Points.Define(stage, "/Model/Geom/Dots")
    dots.CreatePointsAttr().Set([(0, 0, 0), (2, 0, 0), (4, 0, 0)])

    builder = rigexec.Builder.create(stage, "/Rig")
    ctrl = builder.add_control("Ctrl")
    chain = builder.new_mover_chain("chain")
    chain.add_matrix_mover(
        "move", ctrl.path, target="/Model/Geom/Dots.points")

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()
    pose = rig.evaluate(100)
    print(pose.moved_property("/Model/Geom/Dots.points"))

Conventions:
  * Matrices are lists of 16 numbers in row-major order (GfMatrix4d layout);
    ``rigexec.identity()`` is the identity.
  * Vectors are tuples/lists of 3 numbers.
  * Control and joint handles expose ``set_avar_scale(sx, sy, sz)`` for the
    schema-declared local scale channels. Values must be finite; magnitudes
    below ``1e-4`` are raised to that floor with sign preserved, and identity
    scale is ``(1, 1, 1)``.
  * Times are frame numbers; a negative time evaluates at the stage default.
  * Mover chains apply operations in REVERSE ADD ORDER: the last operation
    added runs first, each earlier one wraps its result (spec section 4.2).
    Add outermost passes first.

Runtime note (Windows): Python 3.8+ does not search PATH when loading an
extension module's dependencies, so this package re-enables directory
searching with ``os.add_dll_directory`` before importing the native module.
The directories come from ``RIGEXEC_DLL_PATH`` (a PATH-style list) or, when
that is unset, from ``PATH`` -- e.g. the build directory plus
<usd-install>/bin and <usd-install>/lib.
"""

import os
import math
import platform
import sys


def _dll_search_dirs():
    """Directories to add to the process DLL search path (Windows 3.8+).

    Mirrors pxr's WindowsImportWrapper: entries are added in reversed order
    because, in practice, most-recently-added directories take precedence.
    """
    if not (sys.version_info >= (3, 8) and platform.system() == "Windows"):
        return []
    import_paths = os.getenv("RIGEXEC_DLL_PATH") or os.getenv("PATH", "")
    dirs = []
    for path in reversed(import_paths.split(os.pathsep)):
        if not path or path == ".":
            continue
        abs_path = os.path.abspath(path)
        if os.path.isdir(abs_path):
            dirs.append(abs_path)
    return dirs


def _add_dll_directories():
    """Add the search directories; returns handles kept alive for the process."""
    handles = []
    for d in _dll_search_dirs():
        try:
            handles.append(os.add_dll_directory(d))
        except OSError:
            pass
    return handles


# Keep these open for the lifetime of the process: once _rigexec is loaded,
# its dependent DLLs stay resident, and later lazy imports (e.g. pxr modules)
# benefit from the same search path.
_dll_dir_handles = _add_dll_directories()

try:
    import _rigexec as _native
except ImportError as _exc:  # pragma: no cover - environment diagnostic
    raise ImportError(
        "could not import the native '_rigexec' module (%s). Build it with "
        "-DRIGEXEC_BUILD_PYTHON=ON and make sure its directory (plus the "
        "rigExec/USD library directories on Windows) is importable/findable."
        % _exc
    ) from _exc

__version__ = _native.__version__

from .bake import export_baked
from .inverse import InverseResult, solve_parameters
from .curvenet import create_curvenet_weight

# ---------------------------------------------------------------------------
# Re-exported native surface.
# ---------------------------------------------------------------------------

_NativeBuilder = _native.Builder
SchemaPrim = _native.SchemaPrim
Rig = _native.Rig
Pose = _native.Pose
PointFrame = _native.PointFrame

Handle = _native.Handle
Mover = _native.Mover
Control = _native.Control
Joint = _native.Joint
Solver = _native.Solver
FkChain = _native.FkChain
TwoBoneIk = _native.TwoBoneIk
BlendPointFrames = _native.BlendPointFrames
TwistDistribution = _native.TwistDistribution
Ribbon = _native.Ribbon
SplineIk = _native.SplineIk

Constraint = _native.Constraint
SourceConstraint = _native.SourceConstraint
AimConstraint = _native.AimConstraint
PositionConstraint = _native.PositionConstraint
RotationConstraint = _native.RotationConstraint
ScaleConstraint = _native.ScaleConstraint
ParentConstraint = _native.ParentConstraint
SingleChainIkConstraint = _native.SingleChainIkConstraint

Weight = _native.Weight
StaticWeight = _native.StaticWeight
DynamicWeight = _native.DynamicWeight
VolumeWeight = _native.VolumeWeight
SphereWeight = _native.SphereWeight
PlaneWeight = _native.PlaneWeight
CurveWeight = _native.CurveWeight
CombineWeight = _native.CombineWeight

BlendInput = _native.BlendInput
BlendSample = _native.BlendSample
Curvenet = _native.Curvenet

MatrixMover = _native.MatrixMover
SkinMover = _native.SkinMover
LatticeMover = _native.LatticeMover
BlendShapeMover = _native.BlendShapeMover
CurveMover = _native.CurveMover
SurfaceMover = _native.SurfaceMover
SmoothMover = _native.SmoothMover
VolumeCorrectMover = _native.VolumeCorrectMover
CurvenetMover = _native.CurvenetMover
CurvenetAdjustment = _native.CurvenetAdjustment
CurvenetAdjusterMover = _native.CurvenetAdjusterMover
FloatMathMover = _native.FloatMathMover
Vec3fMathMover = _native.Vec3fMathMover
MatrixMathMover = _native.MatrixMathMover

MoverChain = _native.MoverChain

__all__ = [
    "Builder", "SchemaPrim", "schema", "ControlAPI", "MoverAPI",
    "Rig", "Pose", "PointFrame",
    "Handle", "Mover", "Control", "Joint", "Solver", "FkChain", "TwoBoneIk",
    "BlendPointFrames", "TwistDistribution", "Ribbon", "SplineIk",
    "Constraint", "SourceConstraint", "AimConstraint", "PositionConstraint",
    "RotationConstraint", "ScaleConstraint", "ParentConstraint",
    "SingleChainIkConstraint",
    "Weight", "StaticWeight", "DynamicWeight", "VolumeWeight", "SphereWeight",
    "PlaneWeight", "CurveWeight", "CombineWeight",
    "BlendInput", "BlendSample", "Curvenet", "CurvenetAdjustment", "CurvenetAdjusterMover",
    "MatrixMover", "SkinMover", "LatticeMover", "BlendShapeMover", "CurveMover",
    "SurfaceMover", "SmoothMover", "VolumeCorrectMover", "CurvenetMover",
    "FloatMathMover", "Vec3fMathMover", "MatrixMathMover",
    "MoverChain",
    "load_schema_plugin", "identity",
    "export_baked", "InverseResult", "solve_parameters",
    "create_curvenet_weight",
]

# ---------------------------------------------------------------------------
# Schema plugin registration.
# ---------------------------------------------------------------------------

_registered_dirs = []


def _candidate_dirs():
    """Plausible rigExecSchema resources directories, best first."""
    here = os.path.dirname(os.path.abspath(__file__))
    # Build tree: <build>/python[/Config]/rigexec -> <build>/usd/rigExecSchema/
    # resources. Walk up from the package directory so both single-config and
    # multi-config (Visual Studio) layouts resolve; non-existent candidates
    # are skipped by the caller's plugInfo.json check. This is the GENERATED
    # copy (library-type plugInfo with a LibraryPath), so it is preferred: it
    # can also register compute-extent on demand.
    d = here
    for _ in range(5):
        parent = os.path.dirname(d)
        if parent == d:
            break
        yield os.path.normpath(
            os.path.join(parent, "usd", "rigExecSchema", "resources"))
        d = parent
    # Source tree: usdRig/python/rigexec -> usdRig/plugin/rigExecSchema/resources
    # (data-only fallback).
    yield os.path.normpath(
        os.path.join(here, "..", "..", "plugin", "rigExecSchema", "resources"))


def load_schema_plugin(plugin_dir=None):
    """Register the codeless RigExec schema plugin with USD's Plug registry.

    ``Builder.create`` and ``schema.<Type>.define`` call this automatically.
    Call it directly before using raw ``SchemaPrim`` methods or opening a
    hand-authored stage whose RigExec type names must already resolve.
    Idempotent: re-registering an already-registered directory is a no-op.

    Args:
        plugin_dir: explicit path to a rigExecSchema resources directory (the
            one containing plugInfo.json). When omitted, candidates are probed
            in order: the ``RIGEXEC_SCHEMA_RESOURCE_DIR`` environment variable,
            then the build-tree generated copy, then the source-tree copy.

    Returns:
        The directory that was registered, or the one this function
        previously registered (idempotent no-op). When the schema types are
        already resolvable through some other provider (e.g. an embedded
        copy in rigExecImaging.dll), nothing is registered and ``None`` is
        returned.

    Raises:
        FileNotFoundError: no candidate contains a plugInfo.json.
    """
    from pxr import Plug, Tf

    # Already done by us (possibly through a different path copy of the
    # same plugin): report that directory and do nothing else. Registering
    # a second physical copy would make USD reject the duplicate type
    # declarations.
    if _registered_dirs:
        return _registered_dirs[0]

    # Someone else already provided the schema (e.g. an application loaded
    # rigExecImaging.dll, which embeds the same plugin). Nothing to do.
    if not Tf.Type.FindByName("RigExecControl").isUnknown:
        return None

    if plugin_dir is not None:
        candidates = [plugin_dir]
    else:
        env = os.environ.get("RIGEXEC_SCHEMA_RESOURCE_DIR")
        candidates = ([env] if env else []) + list(_candidate_dirs())

    for d in candidates:
        if not os.path.isfile(os.path.join(d, "plugInfo.json")):
            continue
        Plug.Registry().RegisterPlugins([d])
        _registered_dirs.append(d)
        return d

    raise FileNotFoundError(
        "no rigExecSchema resources directory (with plugInfo.json) found; "
        "pass plugin_dir explicitly or set RIGEXEC_SCHEMA_RESOURCE_DIR. "
        "Tried: %s" % ", ".join(candidates))


def identity():
    """The 16-number row-major identity matrix4d."""
    return [1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1]


# ---------------------------------------------------------------------------
# OpenUSD-style, low-level schema facade.
# ---------------------------------------------------------------------------

_CONCRETE_SCHEMA_NAMES = (
    "AimConstraint",
    "BlendInput",
    "BlendPointFrames",
    "BlendSample",
    "BlendShapeMover",
    "CombineWeight",
    "Control",
    "CurveMover",
    "CurveWeight",
    "Curvenet",
    "CurvenetMover",
    "CurvenetAdjustment",
    "CurvenetAdjusterMover",
    "CurvenetWeight",
    "DynamicWeight",
    "FkChain",
    "FloatMathMover",
    "Joint",
    "LatticeMover",
    "MatrixMathMover",
    "MatrixMover",
    "SkinMover",
    "ParentConstraint",
    "PlaneWeight",
    "PositionConstraint",
    "Ribbon",
    "Root",
    "RotationConstraint",
    "ScaleConstraint",
    "SingleChainIkConstraint",
    "SmoothMover",
    "SphereWeight",
    "SplineIk",
    "StaticWeight",
    "SurfaceMover",
    "TwistDistribution",
    "TwoBoneIk",
    "Vec3fMathMover",
    "VolumeCorrectMover",
)

_MOVER_SCHEMA_NAMES = frozenset(
    name for name in _CONCRETE_SCHEMA_NAMES
    if name.endswith("Mover") or name.endswith("Constraint"))


class _ConcreteSchema:
    """Base for the classes exposed below as ``rigexec.schema.<Type>``."""

    short_name = ""
    schema_type = ""
    applied_schemas = ()

    @classmethod
    def define(cls, stage, path):
        """Define this type and apply its standard authoring APIs."""
        load_schema_plugin()
        from pxr import Usd

        registry = Usd.SchemaRegistry()
        for api_schema in cls.applied_schemas:
            if not registry.FindAppliedAPIPrimDefinition(api_schema):
                raise RuntimeError(
                    "no registered applied API definition for %s" %
                    api_schema)

        path = str(path)
        existed = bool(stage.GetPrimAtPath(path))
        try:
            result = SchemaPrim.define(stage, path, cls.schema_type)
            for api_schema in cls.applied_schemas:
                result.apply_api(api_schema)
            return result
        except Exception:
            # Define is a single authoring operation at this layer. Remove
            # only the prim this call created; never touch a pre-existing prim.
            if not existed:
                stage.RemovePrim(path)
            raise

    @classmethod
    def get(cls, stage, path):
        """Get an existing prim, failing if it is not exactly this type."""
        load_schema_plugin()
        return SchemaPrim.get(stage, str(path), cls.schema_type)


class _SchemaNamespace:
    """Registered RigExec concrete schema classes.

    This mirrors generated OpenUSD wrappers while keeping the project's
    codeless schemas. For example::

        control = rigexec.schema.Control.define(stage, "/Rig/Controls/Main")
        control.set_attribute("rigExec:channelRole", "pose")

    ``define`` applies the appropriate API schemas in the same call. Every
    property operation is checked against OpenUSD's composed prim definition.
    """

    @staticmethod
    def names():
        return tuple("RigExec" + name for name in _CONCRETE_SCHEMA_NAMES)

    @staticmethod
    def define(stage, path, schema_type):
        """Strict generic definition when the concrete type is data-driven."""
        load_schema_plugin()
        short_name = str(schema_type)
        if short_name.startswith("RigExec"):
            short_name = short_name[len("RigExec"):]
        schema_class = getattr(schema, short_name, None)
        if schema_class is None:
            raise ValueError("unknown concrete RigExec schema: %s" % schema_type)
        return schema_class.define(stage, path)


schema = _SchemaNamespace()

for _short_name in _CONCRETE_SCHEMA_NAMES:
    _apis = []
    if _short_name == "Control":
        _apis.append("RigExecControlAPI")
    if _short_name in _MOVER_SCHEMA_NAMES:
        _apis.append("RigExecMoverAPI")
    if _short_name != "Root":
        _apis.append("NodeGraphNodeAPI")
    _schema_class = type(
        _short_name,
        (_ConcreteSchema,),
        {
            "short_name": _short_name,
            "schema_type": "RigExec" + _short_name,
            "applied_schemas": tuple(_apis),
            "__module__": __name__,
        },
    )
    setattr(schema, _short_name, _schema_class)
    setattr(schema, "RigExec" + _short_name, _schema_class)

del _short_name, _apis, _schema_class


def _schema_prim_from_usd_prim(prim):
    if isinstance(prim, SchemaPrim):
        return prim
    if not hasattr(prim, "GetStage") or not hasattr(prim, "GetPath"):
        raise TypeError("expected a SchemaPrim or pxr.Usd.Prim")
    type_name = str(prim.GetTypeName())
    if not type_name:
        raise ValueError("cannot apply a typed API to an untyped prim")
    load_schema_plugin()
    return SchemaPrim.get(prim.GetStage(), str(prim.GetPath()), type_name)


def _validated_relationship_targets(values):
    """Materialize and validate path-like targets before mutating a prim."""
    from pxr import Sdf

    is_single_path = (
        isinstance(values, str) or hasattr(values, "pathString") or
        hasattr(values, "path") or hasattr(values, "GetPath"))
    targets = [values] if is_single_path else list(values)
    for value in targets:
        if isinstance(value, str):
            text = value
        elif hasattr(value, "pathString"):
            text = str(value.pathString)
        elif hasattr(value, "path"):
            text = str(value.path)
        elif hasattr(value, "GetPath"):
            text = str(value.GetPath())
        else:
            raise TypeError(
                "relationship targets must be paths, RigExec handles, "
                "or Usd.Prim objects")
        valid_path = Sdf.Path.IsValidPathString(text)
        if not valid_path:
            raise ValueError("invalid relationship target path: %s" % text)
    return targets


class ControlAPI:
    """Single-call application of ``RigExecControlAPI``."""

    schema_identifier = "RigExecControlAPI"

    @classmethod
    def apply(cls, prim, channel_role=None):
        result = _schema_prim_from_usd_prim(prim)
        if result.schema_type != "RigExecControl":
            raise TypeError("RigExecControlAPI requires a RigExecControl prim")
        if channel_role is not None and not isinstance(channel_role, str):
            raise TypeError("channel_role must be a string")
        result.apply_api(cls.schema_identifier)
        if channel_role is not None:
            result.set_attribute("rigExec:channelRole", channel_role)
        return result


class MoverAPI:
    """Single-call application and wiring of ``RigExecMoverAPI``."""

    schema_identifier = "RigExecMoverAPI"

    @classmethod
    def apply(
            cls, prim, moves=None, enabled=None, default_weight=None,
            weight_object=None):
        result = _schema_prim_from_usd_prim(prim)
        short_name = result.schema_type
        if short_name.startswith("RigExec"):
            short_name = short_name[len("RigExec"):]
        if short_name not in _MOVER_SCHEMA_NAMES:
            raise TypeError(
                "RigExecMoverAPI is not a standard API for %s" %
                result.schema_type)
        if enabled is not None and not isinstance(enabled, bool):
            raise TypeError("enabled must be a bool")
        if default_weight is not None:
            if (isinstance(default_weight, bool) or
                    not isinstance(default_weight, (int, float))):
                raise TypeError("default_weight must be a number")
            default_weight = float(default_weight)
            if not math.isfinite(default_weight) or not 0 <= default_weight <= 1:
                raise ValueError("default_weight must be finite and in [0, 1]")
        validated_moves = (
            _validated_relationship_targets(moves)
            if moves is not None else None)
        if validated_moves is not None:
            validated_moves = result._validate_relationship_targets(
                validated_moves)
        validated_weight_object = None
        if weight_object is not None:
            validated_weight_object = _validated_relationship_targets(
                weight_object)
            if len(validated_weight_object) != 1:
                raise ValueError("weight_object must name exactly one target")
            validated_weight_object = result._validate_relationship_targets(
                validated_weight_object)
        result.apply_api(cls.schema_identifier)
        if validated_moves is not None:
            result.set_relationship("rigExec:moves", validated_moves)
        if enabled is not None:
            result.set_attribute("inputs:enabled", enabled)
        if default_weight is not None:
            result.set_attribute("inputs:defaultWeight", default_weight)
        if validated_weight_object is not None:
            result.set_relationship(
                "rigExec:weightObject", validated_weight_object)
        return result


class Builder:
    """High-level rig construction.

    ``create`` registers the RigExec schemas before constructing the native
    builder, so callers need only one authoring call. The facade forwards its
    authoring methods to that native implementation while remaining a real
    ``rigexec.Builder`` instance.
    """

    __slots__ = ("_builder",)

    def __init__(self, native_builder):
        self._builder = native_builder

    @classmethod
    def create(cls, stage, rig_root="/Rig", partition=""):
        load_schema_plugin()
        return cls(
            _NativeBuilder.create(stage, str(rig_root), str(partition)))

    @property
    def root_path(self):
        return self._builder.root_path

    def __getattr__(self, name):
        return getattr(self._builder, name)

    def __repr__(self):
        return "<rigexec.Builder %r>" % self.root_path
