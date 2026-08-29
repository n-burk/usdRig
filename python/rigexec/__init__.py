"""RigExec Python API -- programmatic rig authoring and evaluation.

This package wraps the native ``_rigexec`` module (built from
libs/rigExecRigging and libs/rigExec) so a Python tool can create and wire an
entire RigExec rig on a ``pxr.Usd.Stage`` and evaluate it in-process, the way
other OpenUSD python bindings are used:

    import pxr
    from pxr import Usd, UsdGeom
    import rigexec

    stage = Usd.Stage.CreateInMemory()
    rigexec.load_schema_plugin()          # register the codeless schema

    dots = UsdGeom.Points.Define(stage, "/Model/Geom/Dots")
    dots.CreatePointsAttr().Set([(0, 0, 0), (2, 0, 0), (4, 0, 0)])

    builder = rigexec.Builder.create(stage, "/Rig")
    ctrl = builder.add_control("Ctrl")
    weight = builder.add_static_weight(
        "W", "/Model/Geom/Dots.points", [1.0, 0.5, 0.0])
    chain = builder.new_mover_chain("chain")
    chain.add_matrix_mover(
        "move", ctrl.path, weight.path, "/Model/Geom/Dots.points")

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()
    pose = rig.evaluate(100)
    print(pose.moved_property("/Model/Geom/Dots.points"))

Conventions:
  * Matrices are lists of 16 numbers in row-major order (GfMatrix4d layout);
    ``rigexec.identity()`` is the identity.
  * Vectors are tuples/lists of 3 numbers.
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

# ---------------------------------------------------------------------------
# Re-exported native surface.
# ---------------------------------------------------------------------------

Builder = _native.Builder
Rig = _native.Rig
Pose = _native.Pose
PointFrame = _native.PointFrame

Handle = _native.Handle
Control = _native.Control
Joint = _native.Joint
Solver = _native.Solver
FkChain = _native.FkChain
TwoBoneIk = _native.TwoBoneIk
BlendPointFrames = _native.BlendPointFrames
TwistDistribution = _native.TwistDistribution
Ribbon = _native.Ribbon

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
LatticeMover = _native.LatticeMover
BlendShapeMover = _native.BlendShapeMover
CurveMover = _native.CurveMover
SurfaceMover = _native.SurfaceMover
SmoothMover = _native.SmoothMover
VolumeCorrectMover = _native.VolumeCorrectMover
CurvenetMover = _native.CurvenetMover
FloatMathMover = _native.FloatMathMover
Vec3fMathMover = _native.Vec3fMathMover
MatrixMathMover = _native.MatrixMathMover

MoverChain = _native.MoverChain

__all__ = [
    "Builder", "Rig", "Pose", "PointFrame",
    "Handle", "Control", "Joint", "Solver", "FkChain", "TwoBoneIk",
    "BlendPointFrames", "TwistDistribution", "Ribbon",
    "Constraint", "SourceConstraint", "AimConstraint", "PositionConstraint",
    "RotationConstraint", "ScaleConstraint", "ParentConstraint",
    "SingleChainIkConstraint",
    "Weight", "StaticWeight", "DynamicWeight", "VolumeWeight", "SphereWeight",
    "PlaneWeight", "CurveWeight", "CombineWeight",
    "BlendInput", "BlendSample", "Curvenet",
    "MatrixMover", "LatticeMover", "BlendShapeMover", "CurveMover",
    "SurfaceMover", "SmoothMover", "VolumeCorrectMover", "CurvenetMover",
    "FloatMathMover", "Vec3fMathMover", "MatrixMathMover",
    "MoverChain",
    "load_schema_plugin", "identity",
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

    Required before creating mover chains (they apply RigExecMoverAPI) and
    recommended whenever a stage will carry RigExec prims, so type names such
    as ``RigExecControl`` resolve through the schema instead of being bare
    strings. Idempotent: re-registering an already-registered directory is a
    no-op.

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
