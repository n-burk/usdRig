# How usdExecImaging integrates OpenExec into Hydra (OpenUSD v26.08)

Reference notes for implementing **rigExecImaging**, based on a source study of
OpenUSD v26.08. All source paths below are relative to the OpenUSD root, and
line numbers refer to that release.

---

## 1. Integration / activation: who creates the exec scene index

### 1.1 The factory

`usdExecImaging` exposes exactly one public construction entry point:

```cpp
// pxr/usdImaging/usdExecImaging/stageSceneIndexFactory.h:26
USDEXECIMAGING_API
UsdExecImagingStageSceneIndexInterfaceRefPtr UsdExecImagingCreateStageSceneIndex();
```

Two alternative TUs implement it, selected by CMake:

- `stageSceneIndexFactory_execOn.cpp:14-18` — returns
  `UsdExecImaging_StageSceneIndex::New()` (compiled when `PXR_BUILD_EXEC=ON`).
- `stageSceneIndexFactory_execOff.cpp:12-16` — returns `nullptr`
  (compiled when `PXR_BUILD_EXEC=OFF`).

`pxr/usdImaging/usdExecImaging/CMakeLists.txt:7-33` selects the TU: with
`PXR_BUILD_EXEC` on, it links `execGeom execIr execUsd` and compiles the
private classes (`adapterRegistry debugCodes geomXformablePrimAdapter
irXformablePrimAdapter request requestBuilder stageSceneIndex`); with it off,
only the stub factory is built. **The library itself is always built** when
`PXR_BUILD_USD_IMAGING=ON` (comment at CMakeLists.txt:4-5), so downstream code
can link it unconditionally.

The concrete implementation is deliberately private
(`UsdExecImaging_StageSceneIndex`, underscore-prefixed,
`stageSceneIndex.h:23-27`); clients only see the abstract
`UsdExecImagingStageSceneIndexInterface` (public, `stageSceneIndexInterface.h:37-72`),
which derives `HdSceneIndexBase` and adds three pure virtuals:
`SetStage(UsdStageRefPtr)`, `SetTime(UsdTimeCode)`, `ApplyPendingUpdates()`.

### 1.2 The one and only caller: UsdImagingGLEngine

The **only** consumer in the entire tree is
`pxr/usdImaging/usdImagingGL/engine.cpp` (a repo-wide grep for
`UsdExecImaging` matches nothing else outside usdExecImaging itself — notably,
`pxr/usdImaging/usdImaging` never references it).

Gate: an env setting, off by default:

```cpp
// engine.cpp:90-96
TF_DEFINE_ENV_SETTING(
    USDIMAGINGGL_ENGINE_ENABLE_EXEC_SCENE_INDEX, false,
    "Inserts an initial scene index which provides values computed by exec. ...
     To use this feature, usdImaging must be built with PXR_BUILD_EXEC=ON.");
```

Instantiation happens inside the engine's *overrides scene index callback*:

```cpp
// engine.cpp:1611-1644  UsdImagingGLEngine::_AppendOverridesSceneIndices
if (TfGetEnvSetting(USDIMAGINGGL_ENGINE_ENABLE_EXEC_SCENE_INDEX)) {
    _execStageSceneIndex = UsdExecImagingCreateStageSceneIndex();
    if (TF_VERIFY(_execStageSceneIndex)) {
        const HdMergingSceneIndexRefPtr mergingSceneIndex = HdMergingSceneIndex::New();
        mergingSceneIndex->AddInputScene(_execStageSceneIndex,  // FIRST => strongest
                                         SdfPath::AbsoluteRootPath());
        mergingSceneIndex->AddInputScene(inputScene, SdfPath::AbsoluteRootPath());
        _noticeBatchingStageSceneIndex = HdNoticeBatchingSceneIndex::New(mergingSceneIndex);
        _noticeBatchingStageSceneIndex->SetBatchingEnabled(true);
        sceneIndex = _noticeBatchingStageSceneIndex;
    }
}
```

Key mechanics:

- The exec SI is an **initial (input) scene index** (no upstream inputs). It is
  merged with the UsdImaging input scene via `HdMergingSceneIndex`; being added
  *first*, its data sources **overshadow** the UsdImaging data sources for the
  same prim path (comment at engine.cpp:1620-1623).
- An `HdNoticeBatchingSceneIndex` wraps the merge so that notices from both the
  UsdImaging stage SI and the exec SI are held back until the exec request has
  been refreshed, then flushed together (engine.cpp:1633-1640; flush sites
  below).

### 1.3 Where the callback plugs into the UsdImaging chain

`_AppendOverridesSceneIndices` is bound as the
`UsdImagingSceneIndexAppendCallback` handed to `UsdImagingSceneIndex::New`:

```cpp
// engine.cpp:1677-1701  _CreateUsdImagingSceneIndices
sceneIndex = _usdImagingSceneIndex =
    UsdImagingSceneIndex::New(
        sceneIndexCreateArgs,
        std::bind(&UsdImagingGLEngine::_AppendOverridesSceneIndices,
                  this, std::placeholders::_1));
sceneIndex = _displayStyleSceneIndex = HdsiLegacyDisplayStyleOverrideSceneIndex::New(sceneIndex);
if (!_sceneDelegateId.IsAbsoluteRootPath())
    sceneIndex = HdPrefixingSceneIndex::New(sceneIndex, _sceneDelegateId);
```

`UsdImagingSceneIndexAppendCallback` is
`std::function<HdSceneIndexBaseRefPtr(HdSceneIndexBaseRefPtr const&)>`
(`pxr/usdImaging/usdImaging/sceneIndex.h:21-23`). Internally
`UsdImagingSceneIndex` delegates to `UsdImagingCreateSceneIndices`
(`pxr/usdImaging/usdImaging/sceneIndices.cpp:191-325`), and the callback is
invoked **very early** in the chain:

```
UsdImagingStageSceneIndex                          (sceneIndices.cpp:201-206)
  -> HdsiLocatorCachingSceneIndex (materials)      (:218-220)
  -> overridesSceneIndexCallback(sceneIndex)       (:222-225)   <== exec merge inserted HERE
  -> UsdImagingUnloadedDrawModeSceneIndex (opt)    (:227-230)
  -> UsdImagingExtentResolvingSceneIndex           (:232-234)
  -> UsdImagingPiPrototypePropagatingSceneIndex    (:236-241)
  -> UsdImagingNiPrototypePropagatingSceneIndex    (:243-284)   (draw-mode SI inserted inside; xform flattening happens inside prototype propagation)
  -> HdNoticeBatchingSceneIndex (postInstancing)   (:286-287)
  -> UsdImaging_InstanceProxyPathTranslationSI     (:295-296)
  -> UsdImagingMaterialBindingsResolvingSceneIndex (:298-299)
  -> _AddPluginSceneIndices  (UsdImagingSceneIndexPlugin, see §6.4)  (:301-302)
  -> UsdImagingSelectionSceneIndex                 (:304-305)
  -> UsdImagingRenderSettingsFlatteningSceneIndex  (:307-308)
```

So the exec overlay sits **before instancing propagation and before
flattening**, on raw stage paths — this is why the exec adapters can publish a
world-space matrix with `resetXformStack = true` and have flattening honor it.

### 1.4 How usdview reaches this code

Stock usdview constructs the engine directly:
`pxr/usdImaging/usdviewq/stageView.py:965-968` —
`self._renderer = UsdImagingGL.Engine(params)`. The engine builds the full
chain in `_CreateSceneIndexChainAndRenderer` (engine.cpp:1748-1794). Scene index
mode is on by default (`USDIMAGINGGL_ENGINE_ENABLE_SCENE_INDEX` default `true`,
engine.cpp:78-79). **Net: running stock usdview with
`USDIMAGINGGL_ENGINE_ENABLE_EXEC_SCENE_INDEX=1` (and a `PXR_BUILD_EXEC=ON`
build) activates the exec scene index; nothing else is required.**

usdview does **not** pass any append callback itself and there is **no
registry** of such callbacks — the engine hard-binds its own member function.

---

## 2. Stage access and ExecUsdSystem creation

The engine forwards the stage explicitly, right next to the UsdImaging SI:

```cpp
// engine.cpp:536-543 (during first PrepareBatch/population)
_usdImagingSceneIndex->SetStage(stage);
if (_execStageSceneIndex) {
    _execStageSceneIndex->SetStage(stage);
}
```

`UsdExecImaging_StageSceneIndex::SetStage` (stageSceneIndex.cpp:42-51) simply
creates/destroys the request object:

```cpp
if (stage) { _request = UsdExecImaging_Request::New(std::move(stage)); }
else       { _request.reset(); }
```

`UsdExecImaging_Request` (request.h:55-142) owns everything:

```cpp
// request.cpp:83-94 (ctor)
_system.emplace(_stage);                    // std::optional<ExecUsdSystem>, request.h:131
_objectsChangedListener = _ObjectsChangedListener::New(this);  // AFTER system, request.cpp:91-93
```

So the `ExecUsdSystem` is created **inside the request, per stage**, at
`SetStage` time. `ExecUsdSystem` is constructed from a
`UsdStageConstRefPtr` (`pxr/exec/execUsd/system.h:48-49`) and extends the
stage's lifetime (system.h:40-43). The request also registers its own
`TfNotice` listener for `UsdNotice::ObjectsChanged` scoped to that stage
(request.cpp:33-68, key registration at :49-53) — this is *in addition to* the
listener ExecUsdSystem itself installs.

There is no discovery mechanism: the stage arrives only because
`UsdImagingGLEngine` pushes it via the interface's `SetStage`.

---

## 3. Evaluation & time: when Compute happens, and how values reach Hydra

### 3.1 Driving methods (all called by the engine on the render thread)

The engine drives the exec SI in lock-step with the UsdImaging SI, from
`UsdImagingGLEngine::Render` / `_PreSetTime` — i.e. on whatever thread calls
`Render()` (the main thread in usdview), with the GIL released
(`TF_PY_ALLOW_THREADS_IN_SCOPE`, engine.cpp:481):

```cpp
// engine.cpp:484-498 (Render -> scene time)
_usdImagingSceneIndex->SetTime(params.frame);
if (_execStageSceneIndex) {
    _execStageSceneIndex->SetTime(params.frame);
    _noticeBatchingStageSceneIndex->Flush();     // release batched notices
}

// engine.cpp:2368-2376 (_PreSetTime -> pending edits)
_usdImagingSceneIndex->ApplyPendingUpdates();
if (_execStageSceneIndex) {
    _execStageSceneIndex->ApplyPendingUpdates();
    _noticeBatchingStageSceneIndex->Flush();
}
```

`UsdExecImaging_StageSceneIndex` (stageSceneIndex.cpp):

```cpp
void SetTime(UsdTimeCode time) {            // :53-61
    _request->SetTime(time);                // -> ExecUsdSystem::ChangeTime (request.cpp:121)
    _request->Refresh();                    // rebuild/recompute as needed
    _SendPrimsDirtied(_request->TakeDirtiedPrimEntries());
}
void ApplyPendingUpdates() {                // :63-70
    _request->Refresh();
    _SendPrimsDirtied(_request->TakeDirtiedPrimEntries());
}
```

`UsdExecImaging_Request::Refresh` (request.cpp:96-109):

```cpp
if (_requiresRebuild || !_request || !_request->IsValid()) _Rebuild();
if (_requiresRecompute)                                    _Recompute();
```

- `_Rebuild` (request.cpp:205-277): full `_stage->Traverse()` (:218); for each
  prim with an adapter (§5) it calls `requestBuilder.SetAdaptedPrim(prim,
  *adapter)` then `adapter->BuildRequest(prim, requestBuilder)` (:232-233),
  then conservatively dirties the whole prim
  (`_primToDirtyDataSourcesMap[path] = HdDataSourceLocatorSet::UniversalSet()`,
  :239-240 — flagged as a TODO/optimization). It then builds the exec request:

  ```cpp
  // request.cpp:246-249
  _request = _system->BuildRequest(
      requestBuilder.TakeValueKeys(),
      std::bind(&This::_InvalidateRequestIndices, this, _1),   // valueCallback
      std::bind(&This::_InvalidateRequestIndices, this, _1));  // timeCallback
  ```

  (`ExecUsdSystem::BuildRequest(std::vector<ExecUsdValueKey>&&,
  ExecRequestComputedValueInvalidationCallback&&,
  ExecRequestTimeChangeInvalidationCallback&&)`, execUsd/system.h:105-110.)
  Sets `_requiresRebuild = false; _requiresRecompute = true` (:255-256).
  Debug flag `USDEXECIMAGING_GRAPH_AFTER_REBUILD` dumps the network to a
  `.dot` file via `_system->PrepareRequest` + `ExecSystem::Diagnostics`
  (:259-276).

- `_Recompute` (request.cpp:279-292):
  `_cacheView.emplace(_system->Compute(*_request));` — the *only* place
  `Compute` is called. (`ExecUsdCacheView Compute(const ExecUsdRequest&)`,
  execUsd/system.h:129-130; implicitly compiles/schedules.)

### 3.2 GetPrim never computes

`GetPrim` (stageSceneIndex.cpp:23-34) returns:

```cpp
return { TfToken(),                                     // empty prim type: type comes
                                                        // from UsdImagingStageSceneIndex
                                                        // through the merging SI
         _request ? _request->GetPrimData(primPath) : nullptr };
```

`GetChildPrimPaths` returns `{}` (:36-40) — the exec SI **never contributes
topology and never sends PrimsAdded**; the merging scene index gets topology
from the UsdImaging input.

`UsdExecImaging_Request::GetPrimData` (request.cpp:151-176):
- Looks up `_valueKeyMap.primToAdapterMap` — no adapter, no data source
  (nullptr; not an error) (:161-164).
- Builds a `UsdExecImagingRequestAccessorInterfaceSharedPtr` as an **aliasing
  shared_ptr onto the request itself** (`shared_from_this()`, :168-170) — data
  sources therefore keep the request (and the ExecUsdSystem) alive.
- Delegates to `adapter->GetPrimData(primPath, requestAccessor)` (:175).

Actual value extraction is lazy, inside the returned Hd data sources:

- `UsdExecImagingComputedSampledDataSource` /
  `UsdExecImagingComputedTypedSampledDataSource<T>`
  (computedDataSource.h:61-132) wrap a `UsdExecImaging_ComputedDataSourceImpl`
  whose `GetValue(shutterOffset)` **ignores the shutter offset** and calls
  `_requestAccessor->GetComputedValue(_valueKey)` (computedDataSource.cpp:49-53).
- `GetContributingSampleTimesForInterval` returns `false` — **no motion-blur
  samples** from exec values (computedDataSource.cpp:55-62).
- `UsdExecImaging_Request::GetComputedValue` (request.cpp:124-149) does *not*
  compute either; it `TF_VERIFY`s that
  `!_requiresRebuild && !_requiresRecompute && _request->IsValid()`, then maps
  the value key to a request index via `_valueKeyMap.valueKeyToIndexMap` and
  returns `_cacheView->Get(index)`.

**Gotcha:** if a consumer pulls a computed data source after invalidation but
before the next `SetTime`/`ApplyPendingUpdates` (Refresh), the TF_VERIFY fires
and an empty `VtValue` is returned. The whole design assumes the host app calls
`SetTime`/`ApplyPendingUpdates` before Hydra pulls — the notice batching SI
plus the engine's call order guarantee that in usdImagingGL.

### 3.3 Which locators are overlaid — confirmed xform-only

Only two adapters exist, both publishing exactly `HdXformSchema`:

- `UsdExecImaging_GeomXformablePrimAdapter::GetPrimData`
  (geomXformablePrimAdapter.cpp:44-63): container
  `{ xform: { matrix: <computed GfMatrix4d from
  ExecGeomXformableTokens->computeLocalToWorldTransform>, resetXformStack:
  retained true } }`. `resetXformStack=true` because the computed matrix is
  world-space (comment :24-26).
- `UsdExecImaging_IrXformablePrimAdapter::GetPrimData`
  (irXformablePrimAdapter.cpp:32-51): same shape, value key =
  `{primPath.AppendProperty(ExecIrTokens->posedSpace),
  ExecBuiltinComputations->computeValue}`.

Nothing publishes points/normals/extent/primvars. **Confirmed: v26.08
usdExecImaging is xform-only.**

---

## 4. Invalidation

### 4.1 Exec invalidation -> dirty locators

Both the value-invalidation and time-change callbacks of the exec request bind
to `UsdExecImaging_Request::_InvalidateRequestIndices(const ExecRequestIndexSet&)`
(request.cpp:294-330):

```cpp
for (const int valueKeyIndex : invalidIndices) {
    const ValueKeyInfo &info = _valueKeyMap.indexToValueKeyInfo[valueKeyIndex];
    HdDataSourceLocatorSet &dirty = _primToDirtyDataSourcesMap[info.adaptedPrimPath];
    info.primAdapter->InvalidatePrimData(info.adaptedPrimPath, info.valueKey, &dirty);
}
_requiresRecompute = true;
```

i.e. exec reports invalid *request indices*; the value-key map (built during
`_Rebuild` by `UsdExecImaging_RequestBuilder::_AddValueKey`,
requestBuilder.cpp:62-77) maps index -> `{valueKey, adaptedPrimPath,
primAdapter}` (valueKeyMap.h:32-61). The adapter translates the value key into
Hd locators, e.g. geomXformablePrimAdapter.cpp:65-81 appends
`HdDataSourceLocator(xform, matrix)`.

The accumulated `_primToDirtyDataSourcesMap` is drained by
`TakeDirtiedPrimEntries` (request.cpp:178-203) into
`HdSceneIndexObserver::DirtiedPrimEntries`, which the scene index passes to
`_SendPrimsDirtied` inside `SetTime`/`ApplyPendingUpdates`
(stageSceneIndex.cpp:59, :68).

### 4.2 Threading

- `ExecUsdSystem::ChangeTime` may invoke the time callback **synchronously on
  the calling thread** (comment "This may invoke
  _TimeChangedInvalidationCallback", request.cpp:120-121). Since the engine
  calls `SetTime` from `Render()`, everything runs on the render/main thread.
- The value callback runs during change processing of authored edits — i.e.
  synchronously on whichever thread authored the USD edit (main thread in
  usdview). The dirty set is just accumulated; `_SendPrimsDirtied` only happens
  later inside `SetTime`/`ApplyPendingUpdates`.
- **Hard rule from exec** (execUsd/system.h:99-102): callbacks must NOT call
  back into execution (no Compute, no extraction) — hence the deferred
  `_requiresRecompute` flag + Refresh design.
- Value callbacks are only guaranteed after `Compute()` has been called, and
  interest must be renewed by computing again (system.h:85-90).

### 4.3 Structural (resync) changes

`_ObjectsChangedCallback` (request.cpp:332-393) exists to detect *adapter
membership changes* that exec cannot see: for each resynced path, it walks
`UsdPrimRange(resyncedPrim)` and compares the previously mapped adapter
(`_valueKeyMap.primToAdapterMap`) to `UsdExecImaging_AdapterRegistry::GetPrimAdapter(prim)`:
- resynced adapted prim => `_requiresRecompute = true` (:376-378);
- adapter changed (prim gained/lost an adapter) => `_requiresRebuild = true`
  and early-out (:382-389).
Prims failing `UsdPrimDefaultPredicate` are skipped (exec expires deleted
providers on its own, :347-354).

### 4.4 Time changes dirtying prims

Time flow: engine `SetTime` -> `ChangeTime` (invokes time callback for
*actually* time-varying value keys only, filtered by exec — system.h:92-97) ->
adapters convert to locators -> `Refresh` recomputes -> `_SendPrimsDirtied` ->
`_noticeBatchingStageSceneIndex->Flush()` releases the batch downstream.

---

## 5. Adapters and the adapter registry

Interfaces (all public headers):

```cpp
// primAdapterInterface.h:33-86
class UsdExecImagingPrimAdapterInterface {
    virtual void BuildRequest(const UsdPrim &, UsdExecImagingRequestBuilderInterface &) const = 0;
    virtual HdContainerDataSourceHandle GetPrimData(
        const SdfPath &, const UsdExecImagingRequestAccessorInterfaceSharedPtr &) const = 0;
    virtual void InvalidatePrimData(
        const SdfPath &, const UsdExecImagingValueKey &, HdDataSourceLocatorSet *) const = 0;
};

// requestBuilderInterface.h:25-43
class UsdExecImagingRequestBuilderInterface {
    virtual void AddValueKey(const UsdPrim &providerPrim, const TfToken &computationName) = 0;
    virtual void AddValueKey(const UsdAttribute &providerAttribute) = 0;  // computeValue
};

// valueKey.h:26-51 — {SdfPath providerPath; TfToken computationName;} with TfHash/==
```

### 5.1 The registry is CLOSED in v26.08

`UsdExecImaging_AdapterRegistry` (note the underscore: **private class**,
adapterRegistry.h:25-35) is a static function with hard-coded types:

```cpp
// adapterRegistry.cpp:28-51
UsdExecImagingPrimAdapterInterface *
UsdExecImaging_AdapterRegistry::GetPrimAdapter(const UsdPrim &prim) {
    // TODO: The adapter currently has a few hard-coded adapter types. This
    // will change in the future to generically handle adapters registered in plugins.
    if (prim.IsA<UsdGeomXformable>() && enableUsdGeomXformableAdapter) { ... static GeomXformable adapter ... }
    if (prim.IsA<ExecIrXformable>())                                   { ... static IrXformable adapter ... }
    return nullptr;
}
```

- `USDEXECIMAGING_ENABLE_USDGEOM_XFORMABLE_ADAPTER` (default `true`,
  adapterRegistry.cpp:19-26) can disable the Xformable adapter (it only honors
  `xformOp:transform`, per the setting's docstring).
- There is **no plugin metadata for adapters**. usdExecImaging's
  `plugInfo.json` has an **empty `Info` block** (plugInfo.json:4) — it exists
  only to declare the library to Plug for resource purposes. Compare
  usdImaging's `UsdImagingPrimAdapter` plugInfo entries, which usdExecImaging
  deliberately does not have yet.
- `adapterRegistry`, `requestBuilder`, `request`, `stageSceneIndex`,
  `valueKeyMap` are all `PRIVATE_CLASSES`/`PRIVATE_HEADERS` in CMake
  (CMakeLists.txt:13-23), so external code cannot even include them.

**Conclusion for question 6: you cannot register a rigExec adapter into
UsdExecImagingAdapterRegistry in v26.08. The registry is closed, private, and
xform-only.** The public pieces (`computedDataSource.h`,
`primAdapterInterface.h`, `requestAccessorInterface.h`,
`requestBuilderInterface.h`, `stageSceneIndexInterface.h`,
`stageSceneIndexFactory.h`, `valueKey.h` — CMakeLists.txt:46-56) are the
*shape* of a future plugin API, not a working one.

---

## 6. External hooks available WITHOUT modifying OpenUSD

### 6.1 (a) HdSceneIndexPlugin via HdSceneIndexPluginRegistry

Declaration (docs in `pxr/imaging/hd/sceneIndexPlugin.h:25-70`):

```jsonc
// plugInfo.json — cf. pxr/imaging/hdGp/plugInfo.json:4-23
"Info": { "Types": {
    "RigExecImagingSceneIndexPlugin": {
        "bases": ["HdSceneIndexPlugin"],
        "displayName": "...", "priority": 0,
        "loadWithRenderer": "",              // "" == all renderers (mandatory; this is what triggers library preload)
        "loadWithApps": [],                  // optional filter
        "tags": [...], "ordering": {"after": [...], "before": [...], "position": "firstAfter"}
    } } }
```

C++ side (pattern from `pxr/imaging/hdGp/sceneIndexPlugin.cpp:20-40`):

```cpp
TF_REGISTRY_FUNCTION(TfType) { HdSceneIndexPluginRegistry::Define<RigExecImagingSceneIndexPlugin>(); }
TF_REGISTRY_FUNCTION(HdSceneIndexPlugin) {
    HdSceneIndexPluginRegistry::GetInstance().RegisterSceneIndexForRenderer(
        HdSceneIndexPluginRegistryTokens->allRenderers,   // ""
        TfToken("RigExecImagingSceneIndexPlugin"), nullptr,
        /*insertionPhase=*/0, HdSceneIndexPluginRegistry::InsertionOrderAtStart);
}
// subclass HdSceneIndexPlugin, override
// _AppendSceneIndex(renderInstanceId, inputScene, inputArgs)  (hd/sceneIndexPlugin.h:107-125)
// and optionally _IsEnabled(inputArgs)                        (:127-134)
```

Mechanics (all in `pxr/imaging/hd/sceneIndexPluginRegistry.cpp`):
- Discovery: registry ctor forces Plug discovery (:1154-1164);
  `_CollectAdditionalMetadata` (:1300-1356) reads `loadWithRenderer`,
  `loadWithApps`, `tags`, `ordering` from plugInfo.
- `AppendSceneIndicesForRenderer(rendererDisplayName, inputScene,
  renderInstanceId, appName)` (:1421-1475) first `_LoadPluginsForRenderer`
  (:1358-1402 — loads libraries whose `loadWithRenderer` matches "" or the
  renderer, which runs your `TF_REGISTRY_FUNCTION`s), then orders entries
  (policy `HD_SCENE_INDEX_PLUGIN_ORDERING_POLICY_DEFAULT`, default `Hybrid`,
  :40-48) and chains each plugin's `_AppendSceneIndex` (or registered
  callback) over the input scene (:1448-1467).
- `inputArgs` always contains `__rendererDisplayName` (underlay, :1435-1439).

**Insertion point relative to UsdImaging**: `UsdImagingGLEngine`
`_CreateSceneIndexChainAndRenderer` (engine.cpp:1748-1794) calls
`AppendSceneIndicesForRenderer` on `_mergingSceneIndex` (engine.cpp:1776-1779),
i.e. **after** the entire UsdImaging chain (which was inserted into that
merging SI, engine.cpp:1592-1593 also merges the task-controller SI) and
**before** the optional terminal caching SI and the renderer. So such a plugin
sees fully instanced, draw-mode-resolved, xform-flattened data, plus
task-controller prims.

The engine itself uses the *callback* flavor
(`RegisterSceneIndexForRenderer(rendererDisplayName,
SceneIndexAppendCallback, ...)`, sceneIndexPluginRegistry.h:150-189) for its
`_AppSceneIndices` (sceneGlobals, material pruning, etc.) at phase 0 /
`InsertionOrderAtStart` (engine.cpp:183-202). Application code may call this
callback registration **before engine construction** — that is a supported
public path.

**Can an HdSceneIndexPlugin discover the UsdStage? No.** `_AppendSceneIndex`
receives only `(renderInstanceId, inputScene, inputArgs)`; there is no stage in
`inputArgs`, `UsdImagingStageSceneIndex` has no public `GetStage()`
(stageSceneIndex.h — only `SetStage`, :67), and stock usdview does not put the
stage into `UsdUtils.StageCache` (verified: no `StageCache` use in usdviewq;
appController.py:1268-1273 opens the stage directly). What such a plugin *can*
rely on: the Hydra prim data sources (points, xforms, primvars, etc. as
already resolved by UsdImaging) and its own out-of-band channels.

### 6.2 (b) UsdImagingSceneIndexAppendCallback / CreateSceneIndicesInfo

`UsdImagingCreateSceneIndicesInfo::overridesSceneIndexCallback`
(`pxr/usdImaging/usdImaging/sceneIndices.h:34-52`) / the second argument of
`UsdImagingSceneIndex::New` (sceneIndex.h:48-55). This is the hook
usdExecImaging itself uses (§1.3) and is the *only* pre-instancing hook.

- Who can pass it: **only the code that constructs the UsdImaging scene index
  chain.** In stock usdview that is `UsdImagingGLEngine`, which hard-binds
  `_AppendOverridesSceneIndices` (engine.cpp:1687-1689). There is **no
  registry, env var, or Python API to inject an additional callback**; the
  header explicitly notes callback-through-registration "might be revisited
  later" (sceneIndex.h:42-46). The `UsdImagingCreateSceneIndices` overloads are
  deprecated in favor of `UsdImagingSceneIndex` (sceneIndices.h:64,73).
- Therefore (b) is unusable for stock usdview without patching OpenUSD or
  replacing the engine.

### 6.3 (c) Other public mechanisms

- The callback flavor of `RegisterSceneIndexForRenderer` (see 6.1) — usable
  from C++ application code loaded before engine construction, but it inserts
  at the same post-UsdImaging position.
- `HdGp` generative procedurals — resolved post-scene-assembly; not suitable
  for stage-backed exec evaluation.
- **`UsdImagingSceneIndexPlugin` (the sleeper hit — see 6.4).**

### 6.4 UsdImagingSceneIndexPlugin — plugin hook INSIDE the UsdImaging chain

New in v26.x (`pxr/usdImaging/usdImaging/sceneIndexPlugin.h`, Copyright 2025):
"A base class for scene index plugins that can insert filtering scene indices
into UsdImaging ... intended for UsdSkelImaging" (:27-33). It is TfType/Plug
discovered — **open to third parties**:

```cpp
// sceneIndexPlugin.h:51-128
class UsdImagingSceneIndexPlugin {
    virtual HdSceneIndexBaseRefPtr AppendSceneIndex(HdSceneIndexBaseRefPtr const &inputScene) = 0;
    virtual HdContainerDataSourceHandle FlattenedDataSourceProviders();   // extend flattening
    virtual TfTokenVector InstanceDataSourceNames();                      // extend NI aggregation keys
    virtual TfTokenVector ProxyPathTranslationDataSourceNames();
    template<typename T> static void Define();   // TfType::Define<T, Bases<UsdImagingSceneIndexPlugin>> + factory
};
```

Registration (exactly like UsdSkel,
`pxr/usdImaging/usdSkelImaging/resolvingSceneIndexPlugin.cpp:22-41` and
`usdSkelImaging/plugInfo.json:42-47`):

```cpp
TF_REGISTRY_FUNCTION(UsdImagingSceneIndexPlugin)
{ UsdImagingSceneIndexPlugin::Define<RigExecImagingResolvingSceneIndexPlugin>(); }
```

```jsonc
"RigExecImagingResolvingSceneIndexPlugin": { "bases": ["UsdImagingSceneIndexPlugin"] }
```

Instantiation: `UsdImagingSceneIndexPlugin::GetAllSceneIndexPlugins()`
(sceneIndexPlugin.cpp:41-89) enumerates all Plug-derived types, `Load()`s
their libraries and creates instances; called from `_AddPluginSceneIndices`
at sceneIndices.cpp:66-78, applied at **sceneIndices.cpp:301-302** — i.e.
inside the UsdImaging chain, after instancing/material-binding resolution and
before selection/render-settings flattening. UsdSkel implements full
skinning (points!) at exactly this level
(`UsdSkelImagingPointsResolvingSceneIndex`), which proves points overlay works
here. No env var gates it; presence of the plugInfo entry is enough.

Caveats:
- Runs for every UsdImaging consumer (usdview, usdrecord, tests) — gate
  yourself with your own TfEnvSetting inside `AppendSceneIndex`.
- Still no UsdStage handle and no time callback (see workarounds below).
- Position is post-instancing: prims inside PI/NI prototypes have propagated
  (renamed) paths.

---

## 7. usdExecImaging build/plugInfo declaration (for mirroring)

- `pxr_library(usdExecImaging ... RESOURCE_FILES plugInfo.json)`
  (CMakeLists.txt:35-69). `Type: "library"`, `Info: {}`, standard
  `@PLUG_INFO_*@` substitution tokens (plugInfo.json:1-12).
- Public/private split as listed in §5; the exec-off build compiles only the
  stub factory so the library has no exec dependency
  (CMakeLists.txt:27-33).
- Debug codes: `USDEXECIMAGING_REQUEST`, `USDEXECIMAGING_GRAPH_AFTER_REBUILD`
  (debugCodes.cpp).

For rigExecImaging, mirror: a `pxr_library`/equivalent with a `plugInfo.json`
resource, `Type: "library"`, and (unlike usdExecImaging) a real
`Info.Types` block declaring your plugin class(es) with their `bases`.

---

## 8. Recommendation: how rigExecImaging should integrate for stock usdview (v26.08)

**Do not plan on UsdExecImagingAdapterRegistry — it is closed, private, and
xform-only in v26.08** (adapterRegistry.cpp:31-33 TODO). Also do not plan on
`UsdImagingSceneIndexAppendCallback` — stock usdview cannot inject one.

### Recommended architecture

**Primary hook: a `UsdImagingSceneIndexPlugin`** (§6.4) — the only public,
plugInfo-driven hook that inserts *inside* the UsdImaging chain, proven to
support points-level overlays (UsdSkel). In
`AppendSceneIndex(inputScene)`, replicate the usdExecImaging overlay pattern
from engine.cpp:1617-1643:

```cpp
HdSceneIndexBaseRefPtr RigPlugin::AppendSceneIndex(HdSceneIndexBaseRefPtr const &input) {
    if (!TfGetEnvSetting(RIGEXECIMAGING_ENABLE)) return input;
    _rigSceneIndex = RigExecImagingSceneIndex::New();      // initial SI, no inputs
    auto merge = HdMergingSceneIndex::New();
    merge->AddInputScene(_rigSceneIndex, SdfPath::AbsoluteRootPath()); // first => wins
    merge->AddInputScene(input,          SdfPath::AbsoluteRootPath());
    return merge;   // (optionally wrap in HdNoticeBatchingSceneIndex you Flush after refresh)
}
```

Your `RigExecImagingSceneIndex` should copy `UsdExecImaging_StageSceneIndex` +
`UsdExecImaging_Request` structurally: own `std::optional<ExecUsdSystem>` +
`ExecUsdRequest` + `ExecUsdCacheView`, a value-key map
(index -> {valueKey, primPath, locators}), rebuild/recompute flags, exec
invalidation callbacks that only accumulate `HdDataSourceLocatorSet`s, lazy
sampled data sources that read the cache view, and a `Refresh()` that
rebuilds/`Compute()`s before `_SendPrimsDirtied`. For your payload, publish and
invalidate these locators instead of/in addition to xform:

- points:  `HdPrimvarsSchema` -> `primvars/points/primvarValue` (typed
  `VtVec3fArray` data source under an `HdPrimvarSchema` container with
  interpolation `vertex`);
- normals: `primvars/normals/...` likewise;
- extent:  `HdExtentSchema` (`extent/min`, `extent/max`) — safe because
  `UsdImagingExtentResolvingSceneIndex` runs upstream of the plugin insertion
  point (sceneIndices.cpp:232-234);
- xform:   `xform/matrix` + `xform/resetXformStack=true` — but note that at
  this chain position xforms are already flattened inside NI propagation, so a
  world matrix overlay affects only that prim, not descendants.

**Stage and time feed: a usdview `PluginContainer`.** The scene index plugin
API gives you neither the `UsdStage` nor `SetTime`; stock usdview gives you
both through its own plugin system (`pxr/usdImaging/usdviewq/plugin.py:100-129`;
a Python `PluginContainer` declared via plugInfo, discovered through
`Plug.Registry.GetAllDerivedTypes(PluginContainerTfType)`, plugin.py:296-297).
usdview loads these **before opening the stage** (appController.py:426-432:
"We do this before loading the stage in case a plugin wants to modify global
settings"). Your container:

1. registers with `usdviewApi` and connects to
   `dataModel.signalStageReplaced` / frame-changed signals;
2. pushes the current `UsdStageRefPtr` and `UsdTimeCode` into a small
   process-wide C++ singleton in rigExecImaging (exposed via a python module),
   mirroring `SetStage`/`SetTime` of `UsdExecImagingStageSceneIndexInterface`
   (stageSceneIndexInterface.h:53-71);
3. the singleton forwards to the live `RigExecImagingSceneIndex` instance(s),
   which run `ChangeTime` + `Refresh` + `_SendPrimsDirtied` — all on the main
   thread, matching usdExecImaging's threading model (§3.1, §4.2).

This combination requires **zero OpenUSD modifications**, works in stock
usdview, and is also reusable outside usdview by any host that calls the
singleton's `SetStage`/`SetTime` directly.

### Fallback / alternatives

- If post-UsdImaging insertion is acceptable (non-instanced rigs, per-prim
  primvars only), an `HdSceneIndexPlugin` with `loadWithRenderer: ""` (§6.1)
  works identically (same stage/time feeding via the usdview plugin) and gives
  you plugInfo-based ordering (`tags`/`ordering`), at the cost of seeing
  propagated prototype paths and task-controller prims.
- Longer term, track upstream: the adapter registry TODO
  (adapterRegistry.cpp:31-33) says plugin-registered exec imaging adapters are
  planned; when that lands, rigExecImaging should become a
  `UsdExecImagingPrimAdapterInterface` implementation (the public headers —
  `primAdapterInterface.h`, `requestBuilderInterface.h`, `computedDataSource.h`,
  `valueKey.h` — are already shaped for it) and ride
  `USDIMAGINGGL_ENGINE_ENABLE_EXEC_SCENE_INDEX`.

### Gotchas checklist

- Never call `Compute`/extraction from inside exec invalidation callbacks
  (execUsd/system.h:99-102); only set flags and accumulate locators.
- Re-`Compute` after every invalidation to renew callback interest
  (system.h:85-90).
- Send `PrimsDirtied` only from your `SetTime`/`ApplyPendingUpdates`
  equivalents, not from notice/exec callbacks (usdExecImaging pattern,
  stageSceneIndex.cpp:53-70).
- On request rebuild, dirty adapted prims with
  `HdDataSourceLocatorSet::UniversalSet()` (request.cpp:235-240) unless you can
  be precise.
- Your merging-overlay SI must return an **empty prim type** from `GetPrim`
  and `{}` from `GetChildPrimPaths` (stageSceneIndex.cpp:23-40) so topology and
  types come from UsdImaging.
- Watch `UsdNotice::ObjectsChanged` yourself to detect prims gaining/losing
  rig adapters (request.cpp:332-393); exec handles value/topology expiry for
  existing providers.
- `GetContributingSampleTimesForInterval` -> `false` unless you implement
  motion samples (computedDataSource.cpp:55-62).
- Gate everything behind your own `TF_DEFINE_ENV_SETTING` so the plugin is
  inert for other UsdImaging hosts.
