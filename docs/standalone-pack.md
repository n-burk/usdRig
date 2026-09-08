# Experimental provider execution and rigpack

This implementation is a provider-level OpenExec/Esf path over a resolved
`RigExecSceneDb`. It does not replace `RigExecRigEvaluator` and does not claim
whole-rig, mover-chain, imaging or production-profile parity. The current
compatibility boundary targets the installed stock OpenUSD 26.08 interfaces.

`RigExecExportRigPack(stage, times, packPath, sourceAssetId, error)` creates a
ZIP archive with two generic payloads:

- `source.usdc` is the ordinary flattened authored USD scene, including native
  geometry. It remains an auditable source payload; the runtime loader does not
  open it as a stage.
- `manifest.usda` records native prim/attribute/relationship descriptors,
  applied schemas, metadata and connections, timing units/policy, the logical
  source asset ID, and generic typed resolved attribute states.

Every discovered attribute is resolved through stock `UsdAttribute::Get` at
every requested Default, numeric or PreTime identity. Numeric keys preserve
the exact IEEE-754 payload; signed zero canonicalizes to zero. A blocked or
otherwise missing value has an explicit no-value row. Native arrays and roles
retain their `SdfValueTypeName`; the manifest has no geometry-specific schema
or custom animation sampler. Asset values preserve their authored, evaluated
and resolved path fields. Referenced textures and other external resources
are **not embedded** by this exporter.

`RigExecLoadRigPack(packPath, error)` reads the ZIP and parses its manifest
through Sdf, copies the data into `RigExecSceneDb`, then releases all archive
and layer objects. It constructs no `UsdStage`. Unexported numeric or PreTime
requests fail instead of borrowing another row or interpolating. The runtime
can accept explicit complete already-resolved state sets and ephemeral edits;
durable authoring remains USD.

```cpp
auto database = rigExec::RigExecLoadRigPack("controls.rigpack", &error);
if (!database) { /* report error */ }
else {
    rigExec::RigExecStandaloneSystem runtime(*database);
    const int control = runtime.AddTap(rigExec::RigExecValueAddress::Prim(
        pxr::SdfPath("/Character/Rig/Control"), pxr::TfToken("computePointFrame")));
    auto result = runtime.Evaluate(pxr::UsdTimeCode(1001));
    if (result.valid) {
        const auto frame = result.Get<rigExec::RigExecPointFrame>(control);
        // Inspect frame.IsValid()/IsDegenerate() before consuming it.
    }
}
```

Include `rigExecStandalone/pack.h`, `rigExecStandalone/system.h`, and
`rigExec/types.h`; link `rigExec::rigExecStandalone`. Calls on one runtime are
serialized by its host. `SetValue`, `SetConnections`, `SetTargets`, and
`SetPrimActive` deliver ephemeral changes to the retained compiler. The result
owns copied values; no cache view crosses the API boundary. `EvaluateResolved`
accepts a complete exact-typed state set for a transient identity and restores
the prior database afterward.

The pack exporter rejects mover applications, reverse solver-to-joint output
bindings, native instances and provider types that require evaluator-only
packet adapters. In particular, Ribbon's driver packet, volume falloff LUTs
and curvenet adjustment placement are not silently replaced by their empty
callback defaults. The supported RigExec provider classes are Root, Control,
Joint without solver-output bindings, FkChain, TwoBoneIk, BlendPointFrames,
TwistDistribution, StaticWeight, DynamicWeight, CombineWeight and
CurvenetWeight. BlendInput and BlendSample also support their registered
descriptor computations; Curvenet may carry ordinary source data. Standard
USD prims and attributes can be retained, subject to the
computations actually registered by the installed OpenExec library. Aggregate
solver requests use explicit dependencies. `rigExec:joints` is permitted and
supplies solver REST inputs -- a two-bone IK measures its bone lengths through
it -- but a packed rig is never posed through it, so lowering must still
resolve posed outputs explicitly. Attribute read-phase metadata and BlendSample
read phases must select authored base values: preceding/final mover revisions
are not lowered by this provider runtime. Export, load and runtime preparation
share the same capability validation.

Applied APIs are limited to the qualified data-only CollectionAPI, GeomModelAPI,
MotionAPI, VisibilityAPI, MaterialBindingAPI and SkelBindingAPI. Unknown applied
APIs are rejected consistently by export, load and runtime preparation; they may
introduce computation expressions beyond this provider scope.

The version-1 manifest identifies this limited `providers` scope. It is not
the complete multi-profile deployment format in specification §11.3: profile
composition, normalized sparse-weight semantic hashes, source-closure resource
packaging, complete evaluation-identity contexts, ABI/plugin hashes, whole-rig
lowering and production qualification remain outside this slice. Unchanged
input produces stable archive bytes; payload timestamps are normalized before
ZIP creation. Export failures leave an existing archive intact and the
exporter refuses to overwrite a source layer.

`testRigExecPack` covers typed rows against direct USD resolution, Default and
paired exact/PreTime values, blocks, native geometry and assets, stage-free
database use, deterministic output, malformed identities/schema values and
explicit unsupported-scope rejection. `testRigExecStandalone` exercises the
separate provider runtime and ephemeral edit path. Final integration results
are recorded in the [review report](code-review-2026-09-05.md).
