# Standalone provider runtime

`rigExecStandalone` evaluates registered provider computations using a generic
`ExecSystem` and custom Esf stage, prim, property, relationship and attribute-query
adapters. Its owned `RigExecSceneDb` contains resolved values and scene descriptors;
it retains no `UsdStage`, `EsfUsd` object or `ExecUsdSystem`. The implementation
reuses the existing schema computation registrations and native value types.
It links the shared OpenUSD libraries and the RigExec computation library; this
is stage independence, not a promise of a USD-free binary dependency tree.

The supported boundary is explicit provider computations, including control
frames, FK/IK/blended/twist frame arrays, self-contained weight packets and blend
sample/channel descriptors. Native USD attributes are source providers. The
runtime and [pack exporter/loader](standalone-pack.md) use the same capability
validation. Whole-rig mover revisions, solver-bound joint publication, ribbon and
volume adapters, curvenet adjustment placement, and non-base input read phases
require evaluator lowering and are rejected. An unknown requested computation
fails instead of producing a successful empty aggregate.

```cpp
#include "rigExecStandalone/pack.h"
#include "rigExecStandalone/system.h"
#include <stdexcept>

std::string error;
auto database = rigExec::RigExecLoadRigPack("character.rigpack", &error);
if (!database) throw std::runtime_error(error);
rigExec::RigExecStandaloneSystem runtime(*database);
int frames = runtime.AddTap(rigExec::RigExecValueAddress::Prim(
    pxr::SdfPath("/Rig/FK"), pxr::TfToken("computePointFrameArray")));
if (!runtime.Prepare(&error)) throw std::runtime_error(error);
auto snapshot = runtime.Evaluate(pxr::UsdTimeCode::Default());
if (snapshot.valid) {
    auto value = snapshot.Get<rigExec::RigExecPointFrameArray>(frames);
    // Inspect the typed frame validity/degeneracy flags as required by the host.
}
```

`RigExecStandaloneResult::valid` means that every requested value was extracted
without an execution error. It has the completeness meaning of the existing
`RigExecSnapshot`; it does not override a typed packet's semantic failure flags.
An empty tap list produces a valid, empty generation. Results own their copied
`VtValue` payloads and survive later evaluation, dirty edits and runtime teardown.

Direct taps on raw attributes, including RigExec avars and custom attributes,
report an explicit no-value state before execution. The check follows a valid
single source connection; an inactive connection target uses the destination's
raw fallback, matching stock `computeValue`. Explicit `computeResolvedValue`
taps also report missing raw states. Only the five registered matrix-space
expressions on Control and Joint bypass this source check: `default:space`,
`avars:defaultSpace`, `posed:defaultSpace`, `parent:space` and
`parent:defaultSpace`. Their callbacks retain stock missing-input semantics.

The qualified data-only applied APIs are CollectionAPI, GeomModelAPI, MotionAPI,
VisibilityAPI, MaterialBindingAPI and SkelBindingAPI. Unknown applied APIs are
rejected because they may register additional expressions. This is an explicit
capability list: the installed package does not ship the private computation
definition registry headers needed for arbitrary expression introspection.

Only explicitly exported Default, exact numeric and PreTime identities are
accepted. There is no fallback to a nearby row and no custom interpolation or
spline sampler. `EvaluateResolved` can temporarily supply a complete resolved
value/block set for every retained attribute at another identity. It validates
all slots before changing state and restores the prior rows after evaluation.

The host serializes all calls on a runtime. `SetValue` dirties the affected
attribute through the generic Exec change processor; unchanged values do
nothing. `SetConnections` and `SetTargets` resync the changed property without
replacing the Exec system. Connection updates require exact native Sdf types;
RigExec scalar attributes permit at most one source. `SetPrimActive` retains
records so provider queries can be invalidated and revived safely. Durable edits,
new objects and new schema descriptors belong in the USD authoring scene and a
new export. The public database is copied at construction; callers do not mutate
the runtime's backing maps while parallel Exec queries are running.
Schema configuration identities are interned once for the process under a mutex,
because Exec's definition registry retains keys beyond individual database
lifetimes. Compiler queries only look up already interned identities.

`testRigExecStandalone` compares the independent adapter with `RigExecTapSet` at
Default, numeric and PreTime identities, including native geometry arrays and
per-element `GfVec3f` inputs to blend sample computations. It exercises repeated
rest edits, unrelated-source dirty isolation, relationship and connection
rewiring, type/cardinality validation, activity removal/revival, complete transient
states, unsupported requests and retained snapshots after source-stage destruction.
`testRigExecPack` covers the independent producer/loader boundary. The preliminary
[Esf compatibility probe](esf-compatibility-probe.md) remains a separately buildable
check of the internal exported request hooks used by this version.
