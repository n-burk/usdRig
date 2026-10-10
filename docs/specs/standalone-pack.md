# Experimental scene execution and rigpack

A rigpack carries a resolved `RigExecSceneDb` that the
[standalone scene runtime](standalone-runtime.md) evaluates without a
`UsdStage`. The runtime lowers the captured scene into the shared operation
graph and does not run OpenExec; the Esf adapter in `libs/rigExecStandalone`
has no callers. The runtime does not replace `RigExecRigEvaluator` and does
not claim imaging or production-profile parity.

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

Export, load and runtime preparation share one capability validation: each
prepares the standalone scene program (`RigExecSceneDb::ValidateCapabilities`,
`libs/rigExecStandalone/sceneDb.cpp:93`). Export rejects native instances and
instance proxies. Preparation rejects an active prim of an unknown RigExec
type and any applied API outside the list in the
[runtime guide](standalone-runtime.md). Movers, solver joint bindings and
read phases lower into the same graph as the rest of the scene.

The version-3 manifest identifies the `scene` scope; the loader rejects any
other version or scope. It is not a complete multi-profile deployment format:
profile composition, normalized sparse-weight semantic hashes, source-closure resource
packaging, complete evaluation-identity contexts, ABI/plugin hashes and
production qualification remain outside this slice. Unchanged
input produces stable archive bytes; payload timestamps are normalized before
ZIP creation. Export failures leave an existing archive intact and the
exporter refuses to overwrite a source layer.

`testRigExecPack` covers typed rows against direct USD resolution, Default and
paired exact/PreTime values, blocks, native geometry and assets, stage-free
database use, deterministic output, malformed identities/schema values and
explicit unsupported-scope rejection. `testRigExecStandalone` exercises the
standalone scene runtime and ephemeral edit path.
