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

## Quick start

You need:

- OpenUSD **26.08**, built with OpenExec and `usdview`
- CMake 3.26 or newer, Ninja, and a C++17 compiler
- a Python environment compatible with the OpenUSD build

The checked-in POSIX helpers currently expect OpenUSD's Python 3.11 install
layout. They look for `usd-install` and `usd-pr4156-venv` beside this checkout
unless `USD` and `VENV` point somewhere else.

### macOS

```sh
export USD=/absolute/path/to/usd-install
export VENV=/absolute/path/to/python-venv

bin/build_rigexec.sh
build/rigExecPose examples/ArmShotAnim.usda \
  --frames 1001,1024,1048 --joints --targets
bin/usdview.sh examples/ArmShotAnim.usda
```

`build_rigexec.sh` configures the project, builds it, and runs all 12 CTest
suites. In `usdview`, scrub frames 1001–1048 to see the arm deform. RigExec
guide geometry is enabled automatically.

To verify the viewport path without opening the interactive viewer:

```sh
bin/run_testusdview.sh
```

### Windows

With OpenUSD installed at the default sibling path `..\usd-install`:

```bat
bin\build_rigexec.bat
build\rigExecPose.exe examples\ArmShotAnim.usda --frames 1001,1024,1048 --joints --targets
bin\launch_usdview.bat examples\ArmShotAnim.usda
```

The Windows helper initializes the Visual Studio 2022 x64 toolchain. Use the
manual CMake setup below if your compiler or OpenUSD layout differs.

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
  `-- VDF mover graph: geometry and property revision chains
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

A rig may publish joints, driven transforms, revised properties, or any
combination of them. Jointless rigs are valid, but a rig with neither a joint
nor a mover has no output and is rejected.

The full codeless schema is in
[`libs/rigExecSchema/schema.usda`](libs/rigExecSchema/schema.usda). It currently
contains 42 classes. Built-in constraints are Aim, Position, Rotation, Scale,
Parent, and SingleChainIK. `RigExecCustomConstraint` is an extension metadata
carrier rather than a generic evaluator, and there is intentionally no
`FbxCharacter` schema.

## Building, testing, and installing

The helper scripts are the shortest supported source-tree workflow. To
configure manually on a POSIX shell:

```sh
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSD_INSTALL_DIR=/absolute/path/to/usd-install \
  -DCMAKE_PREFIX_PATH=/absolute/path/to/usd-install

cmake --build build
ctest --test-dir build --output-on-failure
```

`CMAKE_PREFIX_PATH` is required so OpenUSD's transitive dependencies, including
OpenSubdiv, resolve from the same installation.

The 12 CTest suites cover math and solvers, constraints, curvenets, weight
fields, the reference arm, mover graphs, Hydra publication, native bounds, and
the no-authoring contract. Separate Python/`testusdview` tests exercise the
real application integration. Useful checks include:

```sh
bin/run_testusdview.sh
bin/run_testusdview_overlay.sh
```

If you change `libs/rigExecSchema/schema.usda`, regenerate and review the
checked-in schema resources before rebuilding:

```sh
bin/gen_schema.sh
```

Install the libraries, headers, plugins, Python modules, and CMake package with:

```sh
cmake --install build --prefix /absolute/path/to/rigexec-install
```

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

The package also exports `rigExec::rigExecMath` and
`rigExec::rigExecImaging`, plus `rigExec_PLUGINPATHS`, `rigExec_PYTHON_DIR`, and
`rigExec_LIBRARY_DIR` for host setup.

## Platform status

| Platform | Status |
|---|---|
| macOS arm64 | Verified with AppleClang and Python 3.11: 12/12 CTest suites and the live `usdview` path pass |
| Windows x64 | Built and tested during project development with Visual Studio 2022 and Ninja |
| Linux | Intended, but not yet verified; the current POSIX environment helper is macOS-oriented |
| iOS/iPadOS | Core/static integration is design work only; no device build has been verified |

## Current limitations

- Structural edits currently trigger a full in-memory recompile rather than an
  incremental graph update.
- Multi-target blend-mover fan-out is rejected; it is not silently evaluated
  with shared target parameters.
- The standalone Esf backend and `.rigpack` format are not implemented.
- Inverse solves and baked export are not implemented.
- PRMan-class production render delegates have not been qualified.
- This project is pinned to OpenUSD 26.08; newer OpenUSD releases require a
  separate compatibility pass.

## Repository map

| Path | Purpose |
|---|---|
| [`libs/rigExecMath`](libs/rigExecMath) | Point-frame math, solvers, deformation kernels, and weight fields |
| [`libs/rigExecSchema`](libs/rigExecSchema) | Codeless RigExec USD schema definitions |
| [`libs/rigExec`](libs/rigExec) | OpenExec registrations, mover graph, taps, and evaluator |
| [`libs/rigExecImaging`](libs/rigExecImaging) | Hydra scene-index publication and activation API |
| [`plugin/rigExecUsdview`](plugin/rigExecUsdview) | usdview activation and authoring panels |
| [`plugin/museAssistant`](plugin/museAssistant) | Optional live usdview assistant; see its [setup guide](plugin/museAssistant/README.md) |
| [`examples`](examples) | Reference character and focused animated stages |
| [`tests`](tests) | C++ conformance tests and Python usdview integration tests |
| [`tools`](tools) | The `rigExecPose` command-line diagnostic tool |

## Further reading

- [Architecture and implementation specification](docs/spec.md)
- [OpenUSD 26.08 capability validation](docs/spec-validation-2026-07-24.md)
- [Hydra integration notes](docs/hydra-integration-notes.md)
- [Control and solver guides](docs/control-guides.md)
- [Curvenet design and authoring](docs/curvenet.md)
- [Volumetric weights](docs/volume-weights.md)
- [Viewport gizmos in usdview](docs/viewport-gizmos.md)
- [Graph editor in usdview](docs/graph-editor.md)
- [OpenExec API notes](docs/exec-api-notes.md) and
  [ExecUsd API notes](docs/execusd-api-notes.md)
