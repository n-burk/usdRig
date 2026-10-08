# RigExec architecture

RigExec separates scene authoring, evaluation, and display. It targets an
unchanged OpenUSD 26.08 installation and uses one shared operation graph for typed dependency
evaluation. See the [public references](../references.md) for upstream design
documentation and the published numerical methods used by the project.

## Scene and authoring

`RigExecRoot` defines a rig namespace. Controls expose animation parameters;
joints and native transforms receive solved frames. Solvers and constraints
read declared inputs, and movers apply ordered revisions to their targets.
The schema source is `libs/rigExecSchema/schema.usda`. The C++ rig builder and
Python package create and connect these prims.

Authoring tools may edit layers. Evaluation itself reads scene inputs and
keeps compiled state and results in memory. It does not author generated
prims, layers, or evaluated values back into the source stage.

## Evaluation

`RigExecRigEvaluator` compiles dependencies and produces a `RigExecRigPose`.
Transform providers, constraints, and solvers execute in dependency order.
Geometry mover graphs retain intermediate results so changed inputs can
invalidate affected downstream work while unchanged branches reuse results.
Topology and binding changes require structural validation and may rebuild
affected graphs. Normals and bounds follow the final evaluated geometry.

The compiled program stores typed operation declarations and dense values.
Native, frozen, and binary runtime execution use the shared graph compiler,
readiness rules, and kernels. Independent source references and exact golden
checks are optional judges. Background warming evaluates detached snapshots;
edits invalidate incompatible cached results before they reach the viewport.
See [evaluation and checks](../concepts/baked-vs-dynamic.md) and
[frame warming](../concepts/frame-cache-warming.md).

## Display

`libs/rigExecImaging` publishes results through Hydra filtering scene indices.
The viewer plugins expose controls, guides, picking, and editing. Display
state is separate from authoring state; a published pose must match the
requested stage, time, and generation.

## Export and runtime

`libs/rigExecBake` captures supported rig data into the `.rigexec` format.
`libs/rigExecBinary` defines its records, and `libs/rigExecRuntime` evaluates
the exported program without a USD stage. This is distinct from the
experimental Esf adapter in `libs/rigExecStandalone`, which has its own
[rigpack contract](standalone-pack.md).

Schema, binary, and runtime changes must be verified together. Unsupported
operations should fail explicitly instead of silently changing a rig's result.

## Numerical contracts

USD transforms use row vectors. Some numerical kernels use column frames;
conversions belong at explicit boundaries. Preserve quaternion conventions,
units, array cardinalities, and solver tolerances when changing code.
Regression suites cover math, invalidation, parity, authoring, and export.
Current behavior is defined by source and tests rather than archived plans.
