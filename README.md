# RigExec

RigExec is an experimental live-rig evaluator for OpenUSD 26.08. It evaluates
controls, solvers, constraints, transform stacks, and geometry deformers in
memory, then publishes ordinary transforms and geometry to Hydra. A rig can
therefore animate in stock `usdview` without RigExec editing its source stage.

> **Status:** v0.1-alpha. The core evaluation and viewport path work end to
> end, but APIs and authoring contracts may still change.

## What RigExec provides

- Live FK, two-bone IK, IK/FK blending, twist distribution, ribbons, and
  FBX-style constraints.
- Ordered operations called **movers** that can revise transforms, native
  `UsdGeom` geometry, or scalar/vector/matrix properties.
- Matrix, blend-shape, smooth, lattice, surface, curve, curvenet, and
  volume-correction deformation, with derived normals and bounds.
- Painted, dynamic, and volumetric weight fields.
- Live Hydra 2.0 publication for stock `usdview`, including rig guides and a
  weight-visualization overlay.
- Non-destructive evaluation: compilation and runtime state stay in memory;
  generated prims, properties, and layers are never authored onto the stage.

RigExec uses an unchanged OpenUSD build. It does not fork or patch OpenExec,
VDF, Hydra, or USD.

### Interactive updates

Geometry mover graphs, schedules, and intermediate results persist across
evaluations. Editing a value or a rest pose refreshes the inputs and dirties
the affected downstream revisions. Unchanged branches retain their results;
derived normals and bounds update from the final points. A point-count change
replaces that target's graph when its VDF element masks must change.

Two-bone IK measures both bone lengths from live joint rest frames. There is
no absolute length attribute: the kernel measures root-to-mid and mid-to-end
from the joints named in `rigExec:joints`, plus
`rigExec:upperLengthOffset`/`rigExec:lowerLengthOffset`, and re-measures on
every evaluation, so a rest edit re-proportions the limb with no recompile.
`rigExec:joints` carries two meanings: the chain a solver POSES, and the
chain it MEASURES. A solver whose aggregate another solver reads does not
pose -- its consumer does -- so an IK feeding an IK/FK blend names the same
joints the blend claims, as a rest reference. Rest orientation edits also
reach the IK pole fallback.

Controls, constraints, and solvers run in one compiled dependency order, so a
constraint can drive an IK goal and a later constraint can consume the solved
joints in the same evaluation. Each solver caches its result until its time,
USD dependencies, or supplied inputs change. Solver guide frames use the same
resolved outputs as the posed joints. Tap batches share one OpenExec system
per stage.

Constraints targeting native `points` lower to matrix revisions in the same
point chain as deformers. Their deltas compose in mover order and participate
in dirty suffix evaluation, envelope handling, normals, and bounds maintenance.

Structural edits splice the persistent point chains: existing mover nodes
survive insertions, removals, rewiring, and reordering. Cardinality changes
replace only the affected target graph. Prim deletion safely retires the
stage's OpenExec requests before the stock adapter processes a missing prim;
the point graphs and their unaffected cached results remain available.

Default-space channels and translation units are evaluated and editable; see
[space composition](docs/xformable-default-spaces.md). Twist distribution
supports animated fractional `inputs:twistTurns`. Blend samples support base,
preceding, final, and named checkpoints, including independent target fan-out.
Use `rigExec:deltaSpace = "surfaceFrame"` to transport sculpt detail with the
preceding mesh deformation.

[Curvenet authoring](docs/curvenet.md) includes adjustments, posed guides, and
cached surface weight parametrization. [Bake and inverse APIs](docs/python-bake-inverse.md)
provide standard USD export and a bounded numeric inverse solver.

`RigExecRigPose` exposes per-evaluation graph creation, execution, and schedule
counts (`mover_graph_revisions_created`, `mover_graph_revisions_executed`, and
`mover_graph_schedules_built` in Python). Enable `cpuParityMode` in C++, or
`rig.cpu_parity_mode` in Python, to run the independent scalar reference checks.
They are disabled by default for interactive evaluation.
`solver_evaluations` counts refreshed solver aggregates; it is zero when all
solvers reuse their cached snapshots.

## Quick start

You need:

- OpenUSD **26.08**, built with OpenExec and `usdview`
- CMake 3.26 or newer, Ninja, and a C++17 compiler
- a Python environment compatible with the OpenUSD build

Every helper in `bin/` comes as a pair: a `.sh` for macOS and Linux (and for
Windows shells that run bash, such as Git Bash and WSL) and a `.bat` for
`cmd.exe`. The two are kept deliberately parallel -- same names, same
arguments, same environment variables -- so a command in this README differs
only in its extension and its slashes. The interactive launcher is the one
place the names differ (`bin/usdview.sh` and `bin\launch_usdview.bat`), and
`bin/launch.sh`, which adds a Muse provider-readiness banner, is the single
POSIX-only helper.

Nothing is pinned to one machine. The helpers discover the interpreter, the
USD python modules, and the Visual Studio toolchain, and every discovery can
be overridden:

| Variable | Means | Default |
|---|---|---|
| `USD` | the OpenUSD install | `../usd-install`, beside this checkout |
| `VENV` | a virtualenv holding a USD-compatible python | a venv beside the checkout, if one is there |
| `PY` | the interpreter itself, when it is not in a venv | the venv's python, else `python3`/`python` on `PATH` |
| `RIG` | this checkout | the parent of `bin/` |
| `RIGEXEC_VCVARS` | Windows: which `vcvars64.bat` initializes the toolchain | whatever `vswhere` reports, else the well-known VS locations |

The interpreter has to be the one OpenUSD was built against, and the helpers
check that it can actually load `pxr` -- not merely import it -- before
anything else runs, so a version mismatch is reported here rather than as a
DLL error deep inside a test.

### macOS and Linux

```sh
export USD=/absolute/path/to/usd-install
export VENV=/absolute/path/to/python-venv   # or PY=/path/to/python

bin/build_rigexec.sh
build/rigExecPose examples/ArmShotAnim.usda \
  --frames 1001,1024,1048 --joints --targets
bin/usdview.sh examples/ArmShotAnim.usda
```

`build_rigexec.sh` configures the project, builds it, and runs every enabled
CTest suite. In `usdview`, scrub frames 1001-1048 to see the arm deform.
RigExec guide geometry is enabled automatically.

To verify the viewport path without opening the interactive viewer:

```sh
bin/run_testusdview.sh
```

### Windows

OpenUSD is looked for at the sibling path `..\usd-install`; point `USD`
elsewhere if it lives somewhere else:

```bat
set USD=C:\absolute\path\to\usd-install

bin\build_rigexec.bat
build\rigExecPose.exe examples\ArmShotAnim.usda --frames 1001,1024,1048 --joints --targets
bin\launch_usdview.bat examples\ArmShotAnim.usda
bin\run_testusdview.bat
```

The Windows helpers initialize the x64 toolchain themselves, from whichever
Visual Studio `vswhere` reports -- any edition, any year, any drive. A
Developer Command Prompt is already initialized and is left alone. If the
compiler is somewhere neither finds, point `RIGEXEC_VCVARS` at its
`vcvars64.bat`, or configure manually as below.

## Start with an example

The examples are self-contained animated stages. These are the best entry
points:

| Stage | What it demonstrates |
|---|---|
| [`rigexec_flat.usda`](examples/rigexec_flat.usda) | The smallest complete rig: one aim constraint driving a plain `UsdGeomXform` |
| [`ArmShotAnim.usda`](examples/ArmShotAnim.usda) | The full reference character with FK, IK, twist, ribbon, skinning, blend shape, and volume correction |
| [`08_AimEyes.usda`](examples/08_AimEyes.usda) | Aim constraints followed by geometry movers that consume the final joint poses |
| [`11_VolumeWeights.usda`](examples/11_VolumeWeights.usda) | Sphere, plane, and curve weight fields plus the live influence overlay |
| [`12_CurvenetProfile.usda`](examples/12_CurvenetProfile.usda) | Curvenet-driven profile deformation |
| [`13_ReadPhases.usda`](examples/13_ReadPhases.usda) | Reading authored, intermediate, and final values from mover chains |

See the [complete example catalog](examples/README.md) for all 13 focused
demos and their authoring notes.

## How it works

```text
composed OpenUSD stage
        |
        v
in-memory RigExec compilation
  |-- OpenExec computations: controls, joints, solvers, transforms
  |-- VDF mover graph: geometry revision chains
  `-- evaluator-side ordered scalar, vector and matrix property chains
        |
        v
immutable evaluation snapshot
        |
        v
standard Hydra xforms, points, normals, bounds, and primvars
```

The composed USD stage remains the source of truth. `Compile()` validates the
rig and creates execution state in memory. Evaluation produces immutable
snapshots; the imaging layer overlays their standard values onto Hydra without
changing the stage or exposing RigExec-specific data to the renderer.

### Movers and execution order

A **mover** is an ordered operation with a `rigExec:moves` relationship naming
the transform, geometry, or property it changes. Mover order comes from the
final composed hierarchy under `<RigExecRoot>/Movers`.

RigExec walks that hierarchy **children before parents, with lower sibling rows
before upper sibling rows**. The technical name is reverse-sibling post-order.
For example:

```text
Movers                         Logical execution
├─ FinishMover                 4. FinishMover
├─ GroupMover                  3. GroupMover
│  └─ ChildMover               2. ChildMover
└─ StartMover                  1. StartMover
```

This matches the order shown in `usdview`: the bottom sibling fires first and
the top sibling fires last. If a mover contains another mover, the child fires
before its mover parent. The sequence describes logical value revisions;
thread scheduling and cache hits do not change it.

### Reading a mover chain

A mover input can select which revision of another value it needs with
`rigExecReadPhase` metadata:

| Phase | Value read |
|---|---|
| `base` | The composed, authored value before any mover writes it |
| `preceding` | The value immediately before the consuming mover in its chain |
| `final` | The value after every writer, when that dependency is acyclic |
| absolute mover or scope path | The value after that named point in the hierarchy has completed |

Invalid forward reads and cycles fail compilation instead of producing a
partially ordered result. See
[`13_ReadPhases.usda`](examples/13_ReadPhases.usda) for a compact example.

## Authoring a rig

The easiest way to author a first rig is to copy
[`rigexec_flat.usda`](examples/rigexec_flat.usda) and expand it.

1. Add a `RigExecRoot` under the character or asset root.
2. Put aggregate solvers under `<rig>/Solvers` and ordered movers under
   `<rig>/Movers`. Controls and joints are discovered beneath the rig root.
3. Give every active mover a `rigExec:moves` target. A mover with no target is
   inert and reports a diagnostic.
4. Arrange mover prims in the hierarchy in the order described above.
5. Author default-time rest values for bind inputs such as cages, driver
   curves, and blend targets, even when they also have time samples.
6. Run `rigExecPose` while editing; it exits non-zero on compile or evaluation
   failure and can print joints, moved targets, and diagnostics.
7. To find where evaluation time goes, pass `--profile <file.trace>`; it
   records per-phase timings (compile, property chains, pose seed, each
   solver batch and constraint, the exec snapshot, each geometry chain)
   as Chrome Trace JSON for Perfetto or `chrome://tracing`, and prints a
   per-phase summary.

A rig may publish control guides, joints, placed volume guides, driven
transforms, revised properties, or any combination of them. Control-only and
placed-volume-only rigs are valid. Disconnected movers are accepted while
being authored; compilation rejects a rig with none of these objects.

### Authoring from Python

The `rigexec` package lives in [`python/rigexec`](python/rigexec); its native
`_rigexec` extension is built from [`python/_rigexec.cpp`](python/_rigexec.cpp).
CMake copies both into `build/python`, and installation puts them together in
the configured Python library directory.

Most tools should start with `Builder`. It registers the RigExec schema plugin,
creates the standard rig namespaces, returns typed handles, and wires
dependencies in the same calls:

```python
from pxr import Usd, UsdGeom
import rigexec

stage = Usd.Stage.CreateInMemory()
points = UsdGeom.Points.Define(stage, "/Character/Points")
points.CreatePointsAttr([(0, 0, 0), (1, 0, 0)])

builder = rigexec.Builder.create(stage, "/Character/Rig")
control = builder.add_control("Main")

chain = builder.new_mover_chain(
    "Deform", points.GetPath().AppendProperty("points"))
move = chain.add_matrix_mover("MainMove", control)
move.set_default_weight(1.0)
```

Dependencies may be RigExec handles, `Usd.Prim` objects, `Sdf.Path` objects, or
absolute path strings. Handles expose focused methods such as `set_controls`,
`set_joints`, `set_sources`, `set_moves`, and `set_blend_inputs`. Every mover
also exposes the common `set_default_weight` fallback and optional
`set_weight_object`; a bound object supersedes the scalar fallback. Use
`chain.under(parent_mover)` to author a child mover that executes before its
mover parent.

For schema-level tools, `rigexec.schema` is the lower-level API:

```python
control = rigexec.schema.Control.define(stage, "/Character/RigControl")
control.set_attribute("rigExec:channelRole", "tweak")

mover = rigexec.schema.MatrixMover.define(stage, "/Character/DirectMover")
mover.set_relationship("rigExec:moves", ["/Character/Points.points"])
mover.set_read_phase("rigExec:transform", "final")
```

`schema.<Type>.define()` defines the concrete prim and applies its standard
OpenUSD API schemas in one call. Attribute types and relationship kinds come
from the composed schema definition; misspelled or undeclared properties raise
an exception and are never created as custom USD properties.

The full codeless schema is in
[`libs/rigExecSchema/schema.usda`](libs/rigExecSchema/schema.usda). It currently
contains 44 classes, including 37 concrete types. Built-in constraints are Aim, Position, Rotation, Scale,
Parent, and SingleChainIK; there is intentionally no `FbxCharacter` schema.

## Building, testing, and installing

The helper scripts are the shortest supported source-tree workflow --
`bin/build_rigexec.sh` on macOS and Linux, `bin\build_rigexec.bat` on Windows.
To configure manually instead:

```sh
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSD_INSTALL_DIR=/absolute/path/to/usd-install \
  -DCMAKE_PREFIX_PATH=/absolute/path/to/usd-install

cmake --build build
ctest --test-dir build --output-on-failure
```

The same three cache variables on Windows, from a shell where the x64
toolchain is initialized (`bin\_vcvars.bat` does it, as does a Developer
Command Prompt):

```bat
cmake -S . -B build -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DUSD_INSTALL_DIR=C:/absolute/path/to/usd-install ^
  -DCMAKE_PREFIX_PATH=C:/absolute/path/to/usd-install

cmake --build build
ctest --test-dir build --output-on-failure
```

`CMAKE_PREFIX_PATH` is required so OpenUSD's transitive dependencies, including
OpenSubdiv, resolve from the same installation. `USD_INSTALL_DIR` defaults to
`../usd-install` and must be passed whenever OpenUSD is anywhere else.

The enabled CTest suites cover math and solvers, constraints, curvenets, weight
fields, the reference arm, mover graphs, strict schema authoring, the Python
facade, Hydra publication, native bounds, and the no-authoring contract.

The same build produces the Noodles node-graph editor,
[`plugin/usdNoodles`](plugin/usdNoodles), localized from OpenUSD PR #4156,
whenever the USD install was built with Python and imaging
(`-DRIGEXEC_BUILD_USDNOODLES=OFF` skips it). It links the noodles core library:
a copy under `NOODLES_ROOT` (default: the USD install, where
`build_usd.py --build-noodles` puts it) is used as is, and otherwise the pinned
commit is fetched and built at configure time. Its unittest suite runs as the
`testUsdNoodles` CTest entry. The package imports as the top-level `UsdNoodles`,
not `pxr.UsdNoodles`, because nothing outside the USD install can add to `pxr`.
`bin/launch.sh` and `bin/usdview.sh` register it, except against a USD install
that ships its own `pxr.UsdNoodles`: the two register the same usdview
commands, and usdview then loads no plugins at all.

The Qt-free plugin tests also run on their own, with no build and no display,
which is the quick loop while editing a panel:

```sh
bin/run_python_tests.sh            # bin\run_python_tests.bat on Windows
```

Separate `testusdview` tests exercise the real application integration. Useful
checks include:

```sh
bin/run_testusdview.sh             # bin\run_testusdview.bat
bin/run_testusdview_overlay.sh     # bin\run_testusdview_overlay.bat
```

If you change `libs/rigExecSchema/schema.usda`, regenerate and review the
checked-in schema resources before rebuilding:

```sh
bin/gen_schema.sh                  # bin\gen_schema.bat
```

Install the libraries, headers, plugins, Python modules, and CMake package with:

```sh
cmake --install build --prefix /absolute/path/to/rigexec-install
```

The usdview plugins install as whole directories — every panel module and the
toolbar's artwork, not a hand-kept list — so an installed tree opens the same
panels the source tree does. Point usdview at one with:

```sh
export PXR_PLUGINPATH_NAME=<prefix>/lib/usd/rigExecSchema/resources:<prefix>/lib/usd/rigExecImaging/resources:<prefix>/lib/python/rigExecUsdview
export PYTHONPATH=<prefix>/lib/python:<prefix>/lib/python/rigExecUsdview:$PYTHONPATH
```

Add `<prefix>/lib/python/UsdNoodles` to `PXR_PLUGINPATH_NAME` for the Noodles
editor as well; the `PYTHONPATH` above already covers its package.

## Integrating RigExec

Choose the smallest layer your application needs:

- **Evaluation only:** use
  [`RigExecRigEvaluator`](libs/rigExec/rigEvaluator.h) to compile and evaluate a
  rig from C++.
- **Hydra application:** use
  [`RigExecImagingRegistry`](libs/rigExecImaging/registry.h) to publish one or
  every `RigExecRoot` on a stage through the filtering scene indices.
- **Stock usdview:** load the codeless schema, imaging, and Python plugin paths;
  the source-tree launchers configure these automatically.

An installed CMake consumer can use:

```cmake
find_package(rigExec CONFIG REQUIRED)
target_link_libraries(myTarget PRIVATE rigExec::rigExec)
```

The package also exports `rigExec::rigExecMath`, `rigExec::rigExecStandalone`,
and `rigExec::rigExecImaging`, plus `rigExec_PLUGINPATHS`, `rigExec_PYTHON_DIR`, and
`rigExec_LIBRARY_DIR` for host setup.

## Platform status

| Platform | Status |
|---|---|
| macOS arm64 | Verified with AppleClang and Python 3.11: 17/17 enabled CTest suites and the live `usdview` path pass |
| Windows x64 | Release build with Visual Studio 2022 and Ninja; 35/35 CTest suites pass for the September 2026 changes |
| Linux | Supported by the build and by every helper -- the python layout, the loader variable, and the interpreter are all discovered rather than assumed -- but no full CTest run has been recorded here yet |
| iOS/iPadOS | Core/static integration is design work only; no device build has been verified |

## Current limitations

- Structural edits still validate the binding plan before splicing affected
  point chains. Prim deletion lazily rebuilds shared OpenExec requests to work
  around an invalid-prim callback in the pinned dependency. A host that owns
  other, separate ExecUsd systems must manage those systems' deletion lifecycle.
- Geometry checkpoints trade memory for fast edits: storage grows with point
  count times the number of revisions. Parameter assembly and property/pose
  constraint walks still inspect their operations on each evaluation.
- Dirty propagation currently operates on source and revision outputs. Sparse
  per-point dirty masks and a configurable checkpoint eviction policy remain
  future performance work.
- The numeric inverse API is a dense solver for small selected parameter sets;
  scene-edit transactions and an inverse manipulation UI are separate work.
- Curvenet adjustment frames are available to viewport gizmos. Using these
  point-graph outputs as earlier pose or MatrixMover inputs is rejected during
  compilation; authored scalar adjustment channels remain readable.
- The experimental [standalone backend and rigpack](docs/standalone-pack.md)
  execute supported providers through Esf without a USD stage. Whole-rig mover
  lowering and imaging profiles remain outside that backend's current scope.
- PRMan-class production render delegates have not been qualified.
- This project is pinned to OpenUSD 26.08; newer OpenUSD releases require a
  separate compatibility pass.

See the [September 2026 code review](docs/code-review-2026-09-05.md) for
verified fixes, regression coverage, and remaining scope limits.

## Repository map

| Path | Purpose |
|---|---|
| [`libs/rigExecMath`](libs/rigExecMath) | Point-frame math, solvers, deformation kernels, and weight fields |
| [`libs/rigExecSchema`](libs/rigExecSchema) | Codeless RigExec USD schema definitions |
| [`libs/rigExec`](libs/rigExec) | OpenExec registrations, mover graph, taps, and evaluator |
| [`libs/rigExecStandalone`](libs/rigExecStandalone) | Independent Esf provider runtime and typed rigpack export/load |
| [`libs/rigExecImaging`](libs/rigExecImaging) | Hydra scene-index publication and activation API |
| [`plugin/rigExecUsdview`](plugin/rigExecUsdview) | usdview activation and authoring panels |
| [`plugin/museAssistant`](plugin/museAssistant) | Optional live usdview assistant; see its [setup guide](plugin/museAssistant/README.md) |
| [`plugin/usdNoodles`](plugin/usdNoodles) | Noodles node-graph editor for usdview, localized from OpenUSD PR #4156 |
| [`examples`](examples) | Reference character and focused animated stages |
| [`tests`](tests) | C++ conformance tests and Python usdview integration tests |
| [`tools`](tools) | The `rigExecPose` command-line diagnostic tool |

## Further reading

- [The biped rig](docs/biped-rig.md) — building, opening and animating the
  ported Maya character, including how its side layers compose. Start here
  if you want a real rig on screen rather than an example.
- [Architecture and implementation specification](docs/spec.md)
- [OpenUSD 26.08 capability validation](docs/spec-validation-2026-07-24.md)
- [Hydra integration notes](docs/hydra-integration-notes.md)
- [Control and solver guides](docs/control-guides.md)
- [Curvenet design and authoring](docs/curvenet.md)
- [Volumetric weights](docs/volume-weights.md)
- [Viewport gizmos in usdview](docs/viewport-gizmos.md)
- [Graph editor in usdview](docs/graph-editor.md)
- [ViewCube in usdview](docs/view-cube.md)
- [Guided composition arcs in usdview](docs/composition-arcs.md)
- [OpenExec API notes](docs/exec-api-notes.md) and
  [ExecUsd API notes](docs/execusd-api-notes.md)
