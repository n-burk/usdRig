# Redesign: remove the UsdStage from rigExecImaging

Goal: `rigExecImaging` must not hold, receive, or reach a `UsdStage`. Everything
it needs about the scene arrives through Hydra 2.0 data sources; everything that
genuinely needs a stage moves to the UsdImaging side of the boundary, where
UsdImaging already owns one.

Reference implementation to mirror: `pxr/usdImaging/usdSkelImaging` — prim
adapters publish custom data sources into `UsdImagingStageSceneIndex`, and the
resolving scene indices consume *only* those data sources. `usdSkelImaging` does
full points-level skinning this way with no stage handle in any scene index.

---

## 0. Decisions taken

Settled; the rest of this document is analysis behind them. Where a section
still argues an alternative, this section wins.

1. **Transport: keep the `UsdImagingSceneIndexPlugin`.** Not spec §10.1's
   app-controlled construction. Works in every UsdImaging host with no
   application code.
2. **Backend: `RigExecSceneDb` implementing `EsfStage`, with
   `RigExecSystem : ExecSystem`** (§8.6). Supersedes §8.4 — no `UsdStage` in
   the imaging path at all, so the §8.4 (a)/(b) question is moot. Sequenced
   before the session work, since building on `ExecUsdSystem` first is
   throwaway.
3. **Points publish as a delta**: `points` + `pointsBase`, consumer applies
   `final − base` over whatever upstream holds — generalizing the existing
   `xform`/`xformBase` mechanism (§9).
4. **Ordering: accept whatever the plugin system gives.** No ordering metadata,
   no negotiation. See the exposure in §9.
5. **No conflict refusal.** A prim driven by both RigExec and UsdSkel, or by
   RigExec and another xform manipulator, composes rather than erroring. This
   is what (3) buys.

**Known exposure from (3)+(4).** Deltas compose only when both parties compose.
`UsdSkelImagingPointsResolvingSceneIndex` *replaces* points rather than
deltaing them, and `GetAllSceneIndexPlugins` iterates a `std::set<TfType>`
with no ordering control (`sceneIndexPlugin.cpp:41-88`). If skel runs after
RigExec on the same mesh, RigExec's delta is overwritten. If RigExec runs
after skel, composition is correct. Accepted, not solved. Cheapest mitigation
if it bites: a test asserting the observed order on a rig+skin mesh, so a
regression is loud rather than silent geometry.

## 1. What actually couples imaging to the stage today

The three filtering scene indices in `sceneIndices.cpp` are **already
stage-free** — they read exclusively through `_GetInputSceneIndex()->GetPrim()`.
The coupling is entirely in the activation/publication path:

| # | Site | Stage use | Replacement |
|---|------|-----------|-------------|
| 1 | `registry.cpp:187-207` `RigExecImaging_Activate` | `UsdUtilsStageCache::Find(id)`, then `stage->Traverse()` to find the first `RigExecRig` | Adapter-driven discovery: `PrimsAdded` of imaging prim type `rigExecRig` |
| 2 | `registry.cpp:51` `Activate` | constructs `RigExecImagingBridge(stage, rigPath, store)` | Session created adapter-side, handed downstream in a data source |
| 3 | `registry.cpp:66-70` | `TfNotice::Register(UsdNotice::ObjectsChanged, stage)` for edit-driven re-eval | `_PrimsDirtied` on the rig's data-source locators |
| 4 | `registry.h:111` `RigExecImaging_SetTime` | app pushes the frame in | Time-varying `rigExec/time` leaf; `UsdImagingStageSceneIndex::SetTime` dirties it |
| 5 | `bridge.h:109` `_stage` + `bridge.cpp:159-199` `_FillGuides` | `_stage->GetPrimAtPath(jointPath)` to read `guide:length`, `guide:radius`, `guide:displayColor`, `guide:displayOpacity` | `RigExecImagingGuideSchema` data source from a joint adapter |
| 6 | `bridge.cpp:109` | `RigExecRigEvaluator(stage, rigPath)` | **Irreducible** — see §2 |
| 7 | `plugin/rigExecUsdview/rigExecUsdview.py` | ctypes + `UsdUtils.StageCache` + frame signal | Deleted entirely |

Note (5): today the *evaluator* reads guide styling, so styling edits force a
full rig re-evaluation. Moving styling to a data source makes a colour tweak a
pure data-source dirty with no evaluation at all.

## 2. The one irreducible constraint

`RigExecRigEvaluator` is built on OpenExec, and `ExecUsdSystem` is constructed
from a `UsdStageConstRefPtr` (`pxr/exec/execUsd/system.h:48-49`). Rig topology —
`rel rigExec:moves` targets, solver→joint bindings, weight objects, the composed
`Movers` namespace post-order — is *not* expressible in the Hydra scene without
reinventing the whole rig object model as Hd schemas, and evaluating it would
mean abandoning OpenExec, which is the point of the project.

So "imaging has no stage" cannot mean "nothing has a stage". It means the stage
lives on the **UsdImaging adapter side**, which is the layer whose job is
already stage→Hydra translation, and the filter chain downstream of it sees only
data sources and the immutable snapshot store.

This is the same split OpenUSD itself uses: `UsdSkelImaging*Adapter` touches the
stage; `UsdSkelImagingPointsResolvingSceneIndex` does not.

## 3. Target architecture

```
UsdImagingStageSceneIndex
  ├─ RigExecImagingRigAdapter        (RigExecRig)      ── owns UsdStage
  │     emits  rigExec/session       opaque session handle
  │            rigExec/rigPath, assetRoot, generatedScope
  │            rigExec/time          time-varying trigger leaf
  ├─ RigExecImagingJointAdapter      (RigExecJoint, RigExecControl)
  │     emits  rigExecGuide/{length,radius,displayColor,displayOpacity}
  └─ RigExecImagingMoverAPIAdapter   (RigExecMoverAPI, optional)
        emits  rigExecMover/moves    for pruning-scope + dependency reporting
        ...
  ▼  (instancing, flattening, material binding — unchanged)
_AddPluginSceneIndices → RigExecUsdImagingSceneIndexPlugin
  ├─ RigExecInternalPrimPruningSceneIndex     no stage
  ├─ RigExecBindingResolvingSceneIndex        no stage
  └─ RigExecResultsSceneIndex                 no stage
```

### 3.1 The session handle replaces the global registry

`RigExecImagingRegistry` (process-global singleton, `TfWeakBase`, mutex, weak
chain list, C API) disappears. Its job — rendezvous between the stage-owning
side and the filter chain — is done by a data source, exactly the way
`usdExecImaging` hands its request into its data sources via an aliasing
`shared_ptr` (`request.cpp:168-170`):

```cpp
// rigExecUsdImaging (adapter side, has the stage)
class RigExecImagingSession {                 // one per (stage, rigPath)
    RigExecRigEvaluator      _evaluator;      // stage-bound, private
    RigExecSnapshotStore     _store;          // the only thing downstream sees
public:
    const std::shared_ptr<RigExecSnapshotStore> &GetStore() const;
    SdfPath GetGeneratedScope() const;
    // Serialized evaluate-then-publish; called from the filter chain's
    // notice path, never from GetPrim().
    RigExecPublishResult EvaluateAndPublish(UsdTimeCode time);
};
using RigExecImagingSessionHandle = std::shared_ptr<RigExecImagingSession>;
using RigExecImagingSessionDataSource =
    HdRetainedTypedSampledDataSource<RigExecImagingSessionHandle>;
```

The filter chain pulls `rigExec/session` off the rig prim's data source and gets
the store. It never learns that a stage exists. The session type lives in a
header that has no `usd/stage` include; the evaluator member is behind a pimpl
so `rigExecImaging` can hold the handle without linking `rigExec`.

Sessions are cached in the adapter layer keyed by `(stage, rigPath)` so two
engines over one stage share one evaluation.

### 3.2 Time and invalidation stop being pushed

`RigExecImaging_SetTime` is deleted. Instead the rig adapter publishes a
`rigExec/time` leaf built from `UsdImagingDataSourceStageGlobals::GetTime()` and
calls `stageGlobals.FlagAsTimeVarying(rigPath, HdDataSourceLocator(rigExec,
time))`. Then:

- `UsdImagingStageSceneIndex::SetTime(frame)` (which `UsdImagingGLEngine::Render`
  already calls) dirties `rigExec/time` on the rig prim;
- an authored edit under the asset dirties whatever rigExec locators the
  adapters' `InvalidateImagingSubprim` maps it to;
- both arrive at `RigExecResultsSceneIndex::_PrimsDirtied` as ordinary upstream
  notices, on the render/main thread, serialized by Hydra's notice contract.

`_PrimsDirtied` is therefore the evaluate-and-publish trigger:

```
_PrimsDirtied(entries):
    for each entry whose path is a known rig root and whose locators
        intersect rigExec/:
            session = <cached handle>
            time    = pull rigExec/time from upstream          # current frame
            result  = session->EvaluateAndPublish(time)        # completes first
            merge result.dirtied into the outgoing entries     # §10.4 expansion
    forward
```

This preserves the spec §8.2 fence exactly — evaluation completes, the snapshot
swaps atomically, *then* dirty notices go out — and it keeps §10.3's rule that
`GetPrim()` never computes. It also matches `usdExecImaging`, which computes in
`SetTime`/`ApplyPendingUpdates` and only then calls `_SendPrimsDirtied`.

Initial population is the same path from `_PrimsAdded`: rig prim appears →
compile → evaluate → publish → emit dirties for the published set.

### 3.3 Guides become adapter subprims

See §7 for the `usdIrImaging` precedent that drove this. `_FillGuides` stops
reading the stage; more than that, the synthesized-guide-child machinery in the
results scene index goes away.

`RigExecImagingJointAdapter::GetImagingSubprims()` returns
`{guideSphere, guideCone}` (n-element solvers: `{guideSphere_i, guideCone_i}`),
each typed `HdPrimTypeTokens->sphere` / `->cone`, with `purpose = guide`,
`HdConeSchema.height` driven by a `UsdImagingDataSourceAttribute<double>` over
`guide:length`, and `displayColor`/`displayOpacity` as constant primvars over
the authored attributes. Placement stays the evaluator's job — the results
scene index overlays each guide subprim's `xform/matrix` from the snapshot's
`guideFrames`, exactly as it already overlays driven provider transforms.

Deleted by this: `_SyncGuideChildren`, `_announcedGuides`, `_ParseGuideName`
and its string-prefix protocol (`sceneIndices.cpp:332-366`), the hand-inherited
`visibility` and `primOrigin` (`:438-455`), and — if §7.4 holds —
`_ResolveAssetRootWorld`.

Consequence: a styling edit dirties the guide subprim's primvars through the
adapter's `InvalidateImagingSubprim` and **no rig evaluation runs**.

Guide schemas are generated the way `usdSkelImaging` generates
`UsdSkelImagingAnimationSchema` — a `hdSchemaDefs.py` next to the sources.

### 3.4 Pruning scope becomes data-driven

`SetOwnedScopes()` is currently pushed by the registry after `Activate`. Instead
the rig adapter publishes `rigExec/generatedScope`; the pruning SI collects
owned scopes from the rig prims it sees in `_PrimsAdded`/`_PrimsRemoved`. This
keeps the spec §10.1 requirement that ownership comes from the compiler's
reserved scope and never from a name convention, while removing the last
push-side API.

### 3.5 Free wins

- **Multi-rig.** The registry holds one `_bridge`; adapter-driven discovery
  gives one session per `RigExecRig` prim with no extra work.
- **Instancing (deferred task #16, `instancedBy`).** Discovery currently happens
  by stage traversal while the filter chain sits *downstream* of
  `UsdImagingNiPrototypePropagatingSceneIndex`, so a rig inside a native
  instance has propagated paths the snapshot's stage paths never match. With
  adapters, the rig data source propagates with the prototype; the results SI
  resolves the snapshot key by reading `HdPrimOriginSchema` off the upstream
  prim. Requires listing the rigExec data-source names in the plugin's
  `InstanceDataSourceNames()` / `ProxyPathTranslationDataSourceNames()`.
- **No application glue.** `rigExecUsdview.py`, the ctypes C API, the
  `UsdUtils.StageCache` handoff, and `RIGEXEC_IMAGING_DLL` all go away. Opening
  a stage with a `RigExecRig` in stock usdview just works — including in
  `usdrecord` and any other UsdImaging host, which the ctypes plugin never
  covered.

## 4. Library boundary and the acceptance test

| Target | Links | Contains |
|---|---|---|
| `rigExecImaging` | `hd`, `sdf`, `gf`, `vt` | `sceneIndices.*`, `snapshotStore.h`, `session.h` (handle + store only), generated Hd schemas |
| `rigExecUsdImaging` **(new)** | `usdImaging`, `rigExec`, `rigExecImaging` | prim/API adapters, `RigExecImagingSession` impl, the `UsdImagingSceneIndexPlugin` |

**The invariant is mechanically checkable:** `rigExecImaging` must not link
`usd`, `usdImaging`, or `rigExec`, and must not include `pxr/usd/usd/stage.h`.
Add that as a build assertion plus a grep test — it is the whole point of the
change and it is the thing that will silently rot otherwise.

## 5. Phases

Each phase leaves the tree building and `testRigExecImaging` + `probeImagingPipeline`
green.

**Sequencing note (per §0).** Two phases land ahead of the ones numbered below:

**Phase −1 — dead surface removal.** *Done.* Deleted `RigExecTapSet::AddResolved`,
`_resolutions` and its branch in `Prepare()`, `GetAddress`, `GetSystem`, and
`RigExecSnapshot::GetTime`; the exec invalidation callbacks are now explicitly
empty with the §10.4 reverse-map seam documented in place, rather than
recording a bool that nothing read. `AddResolved` in particular advertised a
generated-provider indirection the engine no longer has.

**Phase A — `RigExecSceneDb` / Esf backend** (§8.6). 42 pure virtuals across
`stage/prim/property/relationship/attribute/attributeQuery/object`, plus
`RigExecSystem : ExecSystem`. Runs before Phase 2, because building the session
on `ExecUsdSystem` and then moving it is throwaway work.
*Verify:* run `RigExecSceneDb` and `EsfUsdSceneAdapter` side by side over the
same stage and assert **identical journals and identical computed values** —
journal under-capture renders plausible wrong answers, which is the failure
mode this codebase already treats as the dangerous one.

**Phase B — points delta** (§9). Add `pointsBase`, compose in the results scene
index, audit revision-vs-generator movers, hard-stop on count mismatch.
*Verify:* a rig-driven mesh whose upstream points are perturbed by a stub
scene index still lands in the right place; count mismatch publishes nothing
and diagnoses.

**Phase 0 — schemas and adapter skeleton.**
Add `rigExecUsdImaging` with `hdSchemaDefs.py`-generated
`RigExecImagingRigSchema` / `RigExecImagingGuideSchema`, a `RigExecRig` adapter
returning imaging prim type `rigExecRig` and a rig data source, and a
`RigExecJoint`/`RigExecControl` adapter returning an empty subprim type (so the
prim stays typeless downstream) plus a `rigExecGuide` container. Register both
in plugInfo by `primTypeName`. Nothing consumes them yet.
*Verify:* Hydra Scene Browser shows `rigExecRig` and `rigExecGuide` containers
on the right prims at the right values, and existing behaviour is unchanged.

**Phase 1 — guides become adapter subprims** (§3.3, §7.3).
Joint adapter emits `{guideSphere, guideCone}` subprims carrying
`HdSphereSchema`/`HdConeSchema`, `purpose = guide`, and styling primvars over
the authored attributes. Results SI overlays each subprim's `xform/matrix` from
`guideFrames`. Delete `_SyncGuideChildren`, `_announcedGuides`,
`_ParseGuideName`, the manual visibility/primOrigin inheritance, and
`_ReadGuideStyle` + the `guide:*` stage reads in `bridge.cpp`.
*Verify:* guides draw and follow the pose identically; guide colour/opacity/
radius edits redraw; snapshot generation counter does **not** advance for a
pure styling edit (new assertion — this is the behaviour change); confirm
§7.4 by checking guides still track a driven asset root with
`_ResolveAssetRootWorld` removed.

**Phase 2 — session handle replaces the registry.**
Move `RigExecImagingBridge` into `rigExecUsdImaging` as
`RigExecImagingSession`; the rig adapter creates/caches it and publishes
`rigExec/session`. The filter chain resolves the store from the rig prim's data
source instead of `RigExecImagingRegistry::GetInstance().GetStore()`. Delete the
registry, the chain list, and `RegisterChain`.
*Verify:* usdview still draws a posed rig with `RigExecImaging_Activate` still
being called (the C API is kept alive one more phase as a no-op-tolerant
shim so the two changes are bisectable).

**Phase 3 — invalidation and time off the stage.**
Add the time-varying `rigExec/time` leaf; move evaluate-and-publish onto
`_PrimsAdded`/`_PrimsDirtied`. Delete the `UsdNotice::ObjectsChanged` listener,
`SetTime`, `Activate`, `Deactivate`, the C API, and `rigExecUsdview.py`.
*Verify:* scrubbing the timeline and editing an avar both redraw in stock
usdview with **no** rigExec usdview plugin installed; `testUsdviewRigExec.py`
rewritten to assert that.

**Phase 4 — instancing + boundary lock.**
`InstanceDataSourceNames()`, `HdPrimOriginSchema`-based snapshot key resolution,
and the link/include assertion from §4. Closes the `instancedBy` half of open
task #16.

**Phase 5 — precise invalidation (§7.2), optional and separable.**
Prerequisite in `rigExec`, not imaging: `RigExecTapSet` retains *which* taps
each invalidation callback reported instead of collapsing them into one
`std::atomic<bool>`. Then build the spec §10.4 reverse map — tap →
`[(prim path, locator)]` — and derive dirty locators from it, retiring the
whole-generation diff in `RigExecSnapshotStore::Publish`.
*Verify:* per-frame dirty-locator sets are identical to the diff-derived ones
across the existing imaging tests, then the diff path is deleted. Keep both
running side by side first — this is the same parity discipline
`moverGraphParityMismatches` already uses.

## 6. Risks and open questions

1. **Evaluation on the notice path.** A heavy rig evaluates inside
   `_PrimsDirtied`, i.e. inside `UsdImagingStageSceneIndex::SetTime`, i.e.
   inside `Render()`. That is exactly where it happens today (via the usdview
   frame signal) and where `usdExecImaging` puts it, so it is not a regression —
   but it removes the option of ever moving evaluation to a worker without a
   notice-batching stage of our own. If async evaluation is a near-term goal,
   Phase 3 should introduce an `HdNoticeBatchingSceneIndex` wrapper at the same
   time rather than after.
2. **Adapters see pre-instancing stage paths; the filter chain sees propagated
   paths.** Phase 4 handles it via `primOrigin`, but if rigs are never natively
   instanced in practice this can stay deferred — it is deferred today.
3. **Adapters run for every UsdImaging host.** Cheap (they only build data
   sources), but the session must be created lazily on first *pull* of
   `rigExec/session`, not in `GetImagingSubprimData`, so a host that never
   builds a RigExec filter chain never compiles a rig. Gate the plugin itself
   behind a `TF_DEFINE_ENV_SETTING` as the notes already recommend.
4. **Does anything else need the stage?** `EvaluateAndPublishSamples` (motion
   samples, §10.5) needs `UsdGeomPointBased::ComputePointsAtTimes` — stage-side,
   so it belongs in the session, which is correct. Render preflight
   (`motionBlurSupport` from `HdSceneIndexCreateArgsSchema`) is Hydra-side and
   stays in the filter chain, which means preflight has to be *requested*
   through the session handle rather than called on the bridge directly. Worth
   settling the exact signature before Phase 2 since it shapes the handle's API.
5. ~~**Guide subprim count must be derivable from the stage.**~~ **Resolved —
   it is, for every solver kind, and each count is `uniform`.**
   `GetImagingSubprims` returns a static name list, so an evaluation-dependent
   or time-varying count would have forced aggregate-solver guides to stay
   synthesized children. Checked against the schema:

   | Solver | Count source | Declared |
   |---|---|---|
   | `RigExecRibbon` | `rigExec:sampleCount` | `uniform int` (default 5) |
   | `RigExecTwistDistribution` | `rigExec:count` | `uniform int` (default 1) |
   | `RigExecFkChain` | `rigExec:joints` target count | relationship |

   All three are uniform or relational, i.e. structural — they cannot vary
   over time, and a change to one is already a resync. `moverKernels.cpp:750-772`
   confirms `sampleCount` is what actually sizes the ribbon's frame array
   (`result.frames.reserve(sampleCount)`), and that a sample-count mismatch
   bails rather than silently producing a different element count.

   **Consequence: §3.3 applies in full.** Joint *and* aggregate-solver guides
   both become adapter subprims; nothing stays synthesized, so
   `_SyncGuideChildren` and the whole `rigGuideSphere_`/`rigGuideCone_` naming
   protocol delete outright rather than surviving for the solver case.
6. **Spec §10.1 divergence.** The spec's canonical construction has the
   application wrap the completed UsdImaging branch explicitly. Adapters are a
   deeper commitment to the `UsdImagingSceneIndexPlugin` transport that the
   current code already documents as a compatibility choice. If the explicit
   application-side construction is still the intended end state, the adapters
   remain valid (they are stage→Hydra, independent of where the filters are
   installed) but the session-handle rendezvous of §3.1 would need the rig prim
   to be reachable from wherever the filters end up — it is, since they wrap the
   same branch.

---

## 7. Comparison: the waddler rig (`extras/exec/examples/invertibleRigsExample`)

### 7.0 It has no imaging plugin

The example ships `invertibleRigsExampleUsdviewPlugin/plugin.py` — a pure Qt
authoring dialog (avar line-edits, switch-compensation button). It touches the
stage only to author, never to image. Its README's setup is three environment
variables and nothing else: `PXR_PLUGINPATH_NAME`,
`USDIMAGINGGL_ENGINE_ENABLE_EXEC_SCENE_INDEX=1`,
`USDVIEW_ENABLE_OPENEXEC_DEMO_PLUGIN=1`.

All of its imaging comes from two *stock* libraries:

- `usdExecImaging` — the exec-computed transform;
- `usdIrImaging` — the guide geometry.

That is the baseline this redesign should be measured against: the example needs
zero custom imaging code because its outputs are shaped so the stock machinery
can carry them. RigExec cannot reach that today (§7.5), but every step toward it
is a step toward deleting code rather than moving it.

### 7.1 Dataflow in: adapters *declare*, they never read values

`UsdExecImaging_IrXformablePrimAdapter` is 60 lines
(`usdExecImaging/irXformablePrimAdapter.cpp`) and has exactly three moves:

```cpp
// 1. Declare what to pull. One line. No stage, no value.
void BuildRequest(const UsdPrim &prim, UsdExecImagingRequestBuilderInterface &b) {
    b.AddValueKey(prim.GetAttribute(ExecIrTokens->posedSpace));
}

// 2. Return a data source bound to (accessor, valueKey). Lazy; never computes.
HdContainerDataSourceHandle GetPrimData(primPath, requestAccessor) {
    return HdRetainedContainerDataSource::New(
        HdXformSchema::GetSchemaToken(),
        HdXformSchema::Builder()
            .SetResetXformStack(...true...)          // computed matrix is world-space
            .SetMatrix(UsdExecImagingComputedTypedSampledDataSource<GfMatrix4d>::New(
                requestAccessor,
                UsdExecImagingValueKey(primPath.AppendProperty(posedSpace),
                                       ExecBuiltinComputations->computeValue)))
            .Build());
}

// 3. Map an invalid value key back to a locator.
void InvalidatePrimData(primPath, valueKey, HdDataSourceLocatorSet *out);
```

The adapter has a `UsdPrim` but uses it only to *name* a value key. The stage
lives in `UsdExecImaging_Request`, handed over by the engine
(`engine.cpp:536-543`), and nothing else in the imaging chain sees it.

**Versus this plan.** §3.1 has the `RigExecRig` adapter call `prim.GetStage()`
to construct the session. That is a real deviation from the waddler pattern, and
it is forced: usdExecImaging gets its stage from `UsdImagingGLEngine`, which
hard-binds its own append callback with no registry to hook
(`hydra-integration-notes.md` §6.2). Inside a `UsdImagingSceneIndexPlugin`,
`prim.GetStage()` is the only stage source that exists. State it as a deviation
rather than dressing it up as the same thing.

**What to copy verbatim.** The `requestAccessor` lifetime trick — an aliasing
`shared_ptr` onto the request itself (`request.cpp:168-170`) so data sources keep
the exec system alive — is precisely the `rigExec/session` handle of §3.1. Model
the handle on `UsdExecImagingRequestAccessorInterface` and keep the same rule:
*the data source holds an accessor plus a key and never computes.* Then, when
the adapter registry opens (`adapterRegistry.cpp:31-33` TODO), swapping RigExec
onto the real `UsdExecImagingPrimAdapterInterface` is mechanical.

### 7.2 Invalidation: value-key map versus snapshot diffing

This is the sharpest divergence, and it is upstream of imaging entirely.

| | waddler / usdExecImaging | RigExec today |
|---|---|---|
| Source of truth | exec's invalidation callback delivers invalid *request indices* | `RigExecTapSet` collapses every callback into one `std::atomic<bool> _dirty` (`tapSet.h:163,175`) |
| Index → meaning | `_valueKeyMap.indexToValueKeyInfo[i]` → `{valueKey, adaptedPrimPath, primAdapter}` (`valueKeyMap.h:32-61`) | discarded |
| Meaning → locator | `primAdapter->InvalidatePrimData(...)` appends `xform/matrix` | reconstructed by diffing whole published generations (`snapshotStore.h::_Diff`) |
| Cost per frame | proportional to what exec says changed | proportional to everything published, including full `VtVec3fArray` compares |

Spec §10.4 calls for an "authoritative explicit reverse map" from
`(request index/tap)` to `[(prim path, locator, policy)]`. usdExecImaging *is*
that map, implemented. RigExec currently approximates it after the fact by
comparing outputs.

This does not block the stage removal — diffing keeps working — but it is the
missing phase, and its prerequisite is a `rigExec` change, not an imaging one:
`RigExecTapSet` must stop throwing away which taps were invalidated. Added as
Phase 5.

### 7.3 Guides: adapter subprims versus synthesized children

`UsdIrImagingJointScopeAdapter` (`usdIrImaging/jointScopeAdapter.cpp`, ~180
lines) returns two subprims per joint scope:

```cpp
GetImagingSubprims()      -> { baseSphere, zAxisCone }
GetImagingSubprimType()   -> HdPrimTypeTokens->sphere / ->cone
GetImagingSubprimData()   -> HdSphereSchema / HdConeSchema(axis=Z,
                               height = UsdImagingDataSourceAttribute<double>(guide:length)),
                             HdPurposeSchema(purpose = guide),
                             displayColor/displayOpacity as constant primvars
                               over UsdImagingDataSourceAttribute
InvalidateImagingSubprim() -> UniversalSet if any guide:* property changed
```

RigExec reimplements all of this downstream and by hand:
`_SyncGuideChildren` + `_announcedGuides` bookkeeping, a `rigGuideSphere_` /
`rigGuideCone_` string-prefix naming protocol with a parser
(`sceneIndices.cpp:332-366`), manual `PrimsAdded`/`PrimsRemoved` emission,
hand-inherited `visibility` and `primOrigin` (`:438-455`), and
`_ResolveAssetRootWorld` to place asset-space frames.

Every one of those exists because the guides are synthesized *downstream of
flattening*. Emitted as adapter subprims they are ordinary prims, upstream,
and Hydra's own machinery does the inheritance. Hence the rewritten §3.3.

### 7.4 The reason this is a simplification, not a relocation

Objection: if guide subprims are created pre-flatten, flattening bakes the
*authored* joint transform into them, and the pose overlay lands downstream —
so won't the guides be left behind?

No — and this is the part worth checking early, because if it holds it deletes
the most awkward code in the file. RigExec already solves exactly this problem
for geometry parented under a driven joint: `_ComputeDrivenXform` resolves a
prim's world transform against constraint-driven ancestors and `_DirtySubtree`
dirties the descendants. Guide subprims parented under a driven joint are just
more descendants. They stop being a special case and become an instance of the
general one — which also means the asset-space→world composition in
`_ResolveAssetRootWorld` is subsumed by the same delta.

The residue is the aggregate-solver case, where each element needs its own
matrix rather than inheriting one. Those get a per-subprim `xform/matrix`
overlay from `guideFrames[i]` — still ordinary prims, still no synthesis. See
risk §6.5 for the element-count precondition.

### 7.5 Where the waddler pattern does not transfer

- **usdExecImaging is xform-only.** Two hard-coded adapters, both publishing
  only `HdXformSchema`; nothing publishes points, normals, extent, or primvars
  (`hydra-integration-notes.md` §3.3). RigExec publishes all of them, so it
  cannot ride `USDIMAGINGGL_ENGINE_ENABLE_EXEC_SCENE_INDEX`.
- **The adapter registry is closed.** `UsdExecImaging_AdapterRegistry` is a
  private class with hard-coded `IsA<UsdGeomXformable>` / `IsA<ExecIrXformable>`
  branches and no plugin metadata (`adapterRegistry.cpp:28-51`). The waddler
  works because `ExecIrXformable` is one of the two types on that list. RigExec
  cannot join it in v26.08.
- **usdExecImaging computes in a host-driven `Refresh`.** The engine calls
  `SetTime`/`ApplyPendingUpdates` (`engine.cpp:484-498`, `:2368-2376`) and the
  batching SI holds notices until it flushes. A `UsdImagingSceneIndexPlugin` gets
  no such hook, which is why §3.2 triggers on `_PrimsDirtied` instead. Same
  ordering guarantee, different driver — and the reason risk §6.1 (no async
  evaluation without our own batching stage) is real for RigExec and not for
  usdExecImaging.
- **usdIrImaging's guides are pre-flatten.** See §7.4.

### 7.6 Net effect on the plan

- §3.3 rewritten: guides move to adapter subprims, not merely their styling.
- §3.1 annotated: `prim.GetStage()` is a forced deviation; the session handle is
  modelled on `UsdExecImagingRequestAccessorInterface`.
- New Phase 5 (§5): replace snapshot diffing with a value-key → locator map,
  gated on `RigExecTapSet` retaining per-tap invalidation.
- New risk §6.5: guide subprim counts must be derivable from authored scene
  data.

---

## 8. How far adapters can conform — and the wall they hit

### 8.1 The wall: adapters dispatch by schema, RigExec addresses by relationship

usdExecImaging's outputs live **on the prim that owns them**: `posed:space` is an
attribute of the `ExecIrXformable` prim, and adapter dispatch is by schema, so
"prim has the schema → adapter publishes its computed output" is a closed loop.

RigExec's outputs live on **remote targets named by relationship**:
`rel rigExec:moves` points from a mover to `/Asset/Geom/Mesh.points`. The target
carries nothing, and spec §10.1 makes that normative — *"Output ownership comes
only from the compiler's canonicalized exact `rigExec:moves` property targets
and derived native-property dependencies; geometry prims carry no ownership
API."* Confirmed against the schema: `rigExecSchema/schema.usda` defines exactly
two applied API schemas, `RigExecControlAPI` and `RigExecMoverAPI`, neither on
driven geometry.

`UsdImaging_AdapterManager::_ComputeAdapters` (`adapterManager.cpp:150-186`)
dispatches on prim type and applied API schemas only, in strength order:

```
keyless API adapters   (strongest)
prim-type adapter
applied API adapters   (weakest)
```

A driven mesh has neither a RigExec type nor a RigExec API, so it gets no
RigExec adapter. And note the ordering: even if an ownership API *were*
authored, an applied-API adapter is **weaker than the prim-type adapter**, so
its `primvars/points` would lose to `UsdImagingMeshAdapter`'s authored points.
Only a keyless adapter outranks the prim adapter — and a keyless adapter runs on
every prim of every stage in every UsdImaging host.

**Conclusion: points, normals, and extent on driven geometry can never be
published by a UsdImaging prim adapter without violating spec §10.1.** The
filtering scene index is not a workaround for missing conformance; it is the
only mechanism that can overlay an arbitrary path. This is the structural reason
RigExec needs one and usdExecImaging does not.

### 8.1a But the *network* still resolves without a stage — the usdSkel pattern

An earlier draft of this section concluded that because driven geometry gets no
adapter, rig-network discovery had to keep coming from `stage->Traverse()`. That
is wrong. `usdSkelImaging` resolves a relationship-linked network of remote
prims with **zero stage access** — verified: a grep for `UsdStage|GetStage`
across the whole library matches only `_stageGlobals`, which is the time /
time-varying context, not a stage.

The mechanism is two halves:

**Adapters publish edges as paths, never resolved data.**
`dataSourceBindingAPI.cpp:37-57` turns a relationship into a retained path leaf:

```cpp
HdDataSourceBaseHandle _PathFromRelationshipDataSourceFactory(const UsdRelationship &rel, ...) {
    if (!rel.HasAuthoredTargets()) return nullptr;
    SdfPathVector result;
    rel.GetForwardedTargets(&result);              // composition resolved here, once
    return HdRetainedTypedSampledDataSource<SdfPath>::New(std::move(result[0]));
}
```

wired up through `UsdImagingDataSourceMapped::RelationshipMapping` — `skel:skeleton`
→ `skelBinding/skeleton`, `skel:animationSource` → `skelBinding/animationSource`.

**The scene index walks those edges through the Hydra scene and keeps a reverse
map.** `UsdSkelImagingDataSourceResolvedPointsBasedPrim::New(_GetInputSceneIndex(),
path, prim.dataSource)` (`pointsResolvingSceneIndex.cpp:621-623`) hands the
resolved prim the **input scene index itself**, so it can follow the published
paths to the skeleton and blend-shape prims with `GetPrim()`. Dependency
tracking is explicit and hand-maintained:

```cpp
std::map<SdfPath, SdfPathSet> _skelPathToPrimPaths;        // pointsResolvingSceneIndex.h:127
std::map<SdfPath, SdfPathSet> _blendShapePathToPrimPaths;  // :129
std::map<SdfPath, SdfPathSet> _instancerPathToPrimPaths;   // :132

_AddDependenciesForResolvedPrim(primPath, resolvedPrim);   // :636-650
_PopulateFromDependencies(...)                             // :505-520, on dirty
```

A dirty notice on a skeleton is looked up in `_skelPathToPrimPaths` and fanned
out to every dependent mesh.

**This maps onto RigExec directly, and inverted from skel.** UsdSkel authors the
binding on the *consumer* (the mesh carries `SkelBindingAPI`); RigExec authors it
on the *producer* (the mover carries `rel rigExec:moves`). Producers are typed —
`RigExecMatrixMover`, `RigExecRibbon`, `RigExecAimConstraint` — so they get
adapters, and a mover adapter can publish `rigExecMover/moves` as retained path
leaves exactly like the factory above. The results scene index then builds the
reverse map `targetPath → moverPaths` and overlays each target by path. The
driven mesh still carries no API schema, so §10.1 is honored — in fact this is
the design that honors it, rather than working around it.

Net: **discovery, binding, hierarchy, and dependency fan-out all come off data
sources.** `stage->Traverse()` at `registry.cpp:201` goes away, and so does the
`UsdNotice::ObjectsChanged` listener, replaced by the reverse map. That is a
larger reduction than §3 claimed.

### 8.2 What therefore splits which way

| Output | Lives on | Adapter-publishable |
|---|---|---|
| guide sphere/cone geometry + styling | the `RigExecJoint` prim itself | **yes** — the usdIrImaging pattern exactly |
| rig identity, generated scope, session handle, time | the `RigExecRig` prim itself | **yes** |
| driven xform on a Joint/Control | `RigExecXformable`-derived, so typed | **yes** |
| driven xform on a plain `UsdGeomXformable` provider | remote, named by a constraint | no |
| points / normals / extent | remote gprim, named by `rigExec:moves` | **no** |

The design is a hybrid, and that is the correct shape rather than a compromise:
adapters publish everything RigExec owns **by schema**; the filtering scene index
publishes everything RigExec owns **by relationship**.

Partial payoff worth having: outputs that move to adapters are published
*pre-flattening*, where `resetXformStack = true` works the way
`irXformablePrimAdapter.cpp:40-42` uses it. For those the `xformBase` world-space
delta is unnecessary. It does **not** disappear — relationship-addressed remote
xforms are still published post-flatten by the filter — so the mechanism shrinks
rather than retires.

### 8.3 Conformance rules for the adapters we do write

Each is mechanically checkable in review:

1. **No members.** All three reference adapters (`irXformablePrimAdapter`,
   `jointScopeAdapter`, `animationAdapter`) are stateless.
2. **Never call `prim.GetStage()`.** Where `rigExec` needs a stage, pass the
   `UsdPrim` and let it call `GetStage()` internally — change
   `RigExecRigEvaluator(stage, rigPath)` to `RigExecRigEvaluator(UsdPrim)`. The
   adapter's surface then matches `BuildRequest(const UsdPrim &)`: prims and
   attributes, never a stage.
3. **Never eagerly `.Get()` a value.** Wrap in
   `UsdImagingDataSourceAttribute<T>::New(attr, stageGlobals)` so it resolves at
   pull time at the stage scene index's current time. Note that
   `jointScopeAdapter.cpp:145-155` *breaks* this for the cone offset matrix and
   compensates with `UniversalSet` invalidation — do not copy that.
4. **Never traverse.** The stage scene index owns traversal; adapters see one
   prim per callback. This is what retires `stage->Traverse()` from
   `registry.cpp:201`.
5. **`InvalidateImagingSubprim` is a pure property-name → locator map**, with no
   scene access at all.

### 8.4 The one remaining handoff, and the two honest ways to make it

After §8.1a the residue is much smaller than "imaging needs a stage." Discovery,
binding, hierarchy, and invalidation fan-out all come off data sources. What is
left is exactly one thing: **constructing `ExecUsdSystem`, which takes a
`UsdStageConstRefPtr`.** Nothing else in the imaging path needs a stage.

No data source carries a `UsdStage` — that is Hydra's whole point — so the stage
cannot arrive through the scene. Two paths, and the codebase already frames them:

- **(a) Canonical, fully conformant.** Spec §10.1's construction sequence: the
  application creates the chain and calls `SetStage` on both
  `UsdImagingSceneIndex` and a `RigExecImagingSceneIndex`, exactly as
  `engine.cpp:536-543` does for the exec SI. Requires a host we control, since
  `UsdImagingGLEngine` hard-binds its own append callback with no registry
  (`hydra-integration-notes.md` §6.2).
- **(b) Compatibility transport.** One `prim.GetStage()` in the `RigExecRig`
  adapter, feeding the same request object. Works in stock usdview with no
  application code at all.

Recommendation: build (a) as the primary integration and keep (b) as the
usdview transport — the same framing `sceneIndexPlugin.cpp:9-13` already uses
for the plugin insertion point ("a compatibility transport, not a second RigExec
integration"). Then the deviation is a named, bounded, single-line one rather
than the architecture.

### 8.5 The fork this exposes: does the imaging path use OpenExec at all?

§8.1a leaves exec construction as the *only* stage dependency. That invites the
question usdSkel already answered for itself: **UsdSkel does not evaluate through
exec at all.** `UsdSkelImagingPointsResolvingSceneIndex` computes skinning in
Hydra, from data sources, with no stage and no execution system — which is why
its stage count is zero rather than one.

RigExec could follow. The pieces are closer than they look: the point chains
already evaluate through the in-memory `RigExecMoverGraph` rather than through
exec (`rigEvaluator.h:216-273`), so the exec-dependent residue is joint frames,
solvers, and blend channels. If adapters published rest frames, avars, solver
parameters, and weights as data sources, a resolving scene index could run the
same kernels with no stage anywhere in the imaging path.

The cost is real and should not be waved through: **the imaging path would stop
being an OpenExec integration**, which is the project's premise. Exec would
remain the authoritative path for standalone evaluation, `.rigpack`, and
conformance, with the Hydra path as a second implementation of the same kernels
— and the two would need parity testing, exactly the discipline
`moverGraphParityMismatches` already applies to the graph-vs-lowered split.

This is a decision, not a next step. Recorded here because §8.1a is what makes
it reachable: once discovery and binding come off data sources, exec is the only
thing left holding the stage, and it becomes fair to ask whether it should.

### 8.6 Esf is the right seam — but for §11, not for imaging

Decision taken: adopt the Esf seam despite `esf/README.md`'s "not meant for
public use". The design intent supports it — `EsfStage`'s holder comment
(`esf/stage.h:104-106`) states the buffer size is a literal *"to prevent
introducing Usd as a dependency"*, i.e. non-USD scene backends are what the
abstraction is for. `usd_esf.lib` and `usd_esfUsd.lib` are both installed.

The fit test splits cleanly, and not where §8.5 assumed.

**Where it does not fit: a Hydra-backed EsfStage.** The blocker is
`EsfAttributeQueryInterface` (`attributeQuery.h:105-111`):

```cpp
virtual bool _Get(VtValue *value, UsdTimeCode time) const = 0;   // arbitrary absolute time
virtual std::optional<TsSpline> _GetSpline() const = 0;          // the authored spline
virtual bool _ValueMightBeTimeVarying() const = 0;
virtual bool _IsTimeVarying(UsdTimeCode from, UsdTimeCode to) const = 0;  // arbitrary interval
```

Exec's compiler needs random access in time: value at any absolute `UsdTimeCode`,
time-variability over an arbitrary interval, and the authored spline. Hydra
publishes **one time at a time** — `HdSampledDataSource::GetValue(Time)` takes a
*shutter offset relative to the scene's current time*, and there is no absolute
time coordinate anywhere in the scene. `ExecUsdSystem::ChangeTime`'s contract
("determines which of these inputs are *actually* changing between the old and
new time") cannot be served from a single-time snapshot.

That is a category mismatch, not a difficulty. An EsfHd is not on the table.

**Where it fits exactly: `RigExecSceneDb` (spec §11).** Compare the required
interface against the snapshot the spec already specifies:

| Esf requirement | `RigExecSceneSnapshot` field |
|---|---|
| `EsfPrim::_GetType`, `_GetAppliedSchemas` | `SchemaTable concreteAndAppliedSchemas` |
| `EsfPrim::_GetAttribute`, `_GetRelationship` | `PropertyTable attributesAndRelationships` |
| `EsfAttribute::_GetConnections` | `ConnectionTable incomingAndOutgoingConnections` |
| `EsfAttributeQuery::_Get(value, time)`, `_IsTimeVarying(from, to)` | `TypedValueTable staticDefaults` + `ResolvedStateTable exportedResolvedStates` |
| `EsfStage::_Get*AtPath` | `InternedPathTable paths` |
| `_GetTypeNameAndInstance`, `_GetAPITypeFromSchemaTypeName` | `UsdSchemaRegistry` directly — schema *definitions*, no stage needed |

That is a field-for-field match. Spec §11 was written against this interface.

**The payoff, and why this supersedes §8.4.** A `RigExecSceneDb` implementing
`EsfStage`, plus `RigExecSystem : public ExecSystem` over it, removes the stage
from `rigExec` for real rather than laundering it through data sources. USD
→ SceneDb extraction happens once, at load/export — which is what §11 already
says ("Composition happens in the USD build/export process"). Then the imaging
question dissolves: the session owns a `RigExecSystem`, no `UsdStage` exists
anywhere in the chain, and §8.4's `prim.GetStage()` becomes "hand the adapter's
`UsdPrim` to the extractor," run once, outside the imaging path.

**Sizing.** 42 pure virtuals across `stage/prim/property/relationship/attribute/
attributeQuery/object` (`object.h` alone accounts for 16). Small enough to be
real work rather than a research project.

**The actual risk is journal correctness, not surface area.** Every accessor
takes an `EsfJournal *` that "captures the conditions for recompilation"
(`journal.h:22-36`); the journal produced while compiling a node is exactly the
set of scene changes that will uncompile it. Under-journal and the network goes
stale — which renders *plausible wrong answers*, the failure mode this codebase
already treats as the dangerous one (cf. `moverGraphParityMismatches`). Any
`RigExecSceneDb` needs the same machine-checkable parity discipline: run it
against `EsfUsdSceneAdapter` over the same stage and assert identical journals
and identical computed values.

**Re-sequencing.** `PLAN.md` has the standalone Esf backend at Phase 0/4 with a
"stop-gate if Esf unusable". This assessment closes that gate open: the
interface is small, well-shaped, installed, and explicitly designed for non-USD
backends. It does mean §11 moves ahead of the imaging phases in §5, because it
changes what the session in §3.1 is built on.

---

## 9. Composing with upstream: publish deltas, never absolutes

Decision §0.3/§0.5. RigExec sits downstream of an open-ended chain — skel
skinning, draw modes, instancing, third-party plugin scene indices — and does
not control its position in it (§0.4). Publishing an absolute value asserts
"this prim's points ARE this", which silently discards whatever upstream
computed. Publishing a delta asserts only "RigExec moved these points by this
much", which is the true statement and composes with anything that ran before.

The mechanism already exists for transforms and is already justified in the
code: `RigExecPublishedPrim::xform` + `xformBase` (`snapshotStore.h:30-39`),
because "the imaging chain's own notion of a prim's world transform need not
equal the stage's." That reasoning was never transform-specific; it was just
only ever noticed for transforms.

### 9.1 Shape

```cpp
struct RigExecPublishedPrim {
    bool hasPoints = false;
    VtVec3fArray points;       // final, as evaluated
    VtVec3fArray pointsBase;   // the base those points were computed from
    ...
};
```

`RigExecResultsSceneIndex::GetPrim` overlays
`upstreamPoints[i] + (points[i] − pointsBase[i])` rather than `points[i]`.
Normals follow the same rule. Extent is recomputed from the composed result,
not published from the snapshot, since a delta invalidates a precomputed bound.

### 9.2 Rules

- **Count mismatch is a hard stop.** If `upstreamPoints.size() != pointsBase.size()`
  the composition is undefined — upstream changed topology or substituted
  geometry (draw mode cards, a proxy). Publish nothing for that prim and
  diagnose; do not fall back to absolute, which would look like it worked.
- **Replacing movers cannot delta.** A mover that computes points from scratch
  rather than revising them has no meaningful base. Those must publish
  absolute, and must be marked as such in the snapshot so the consumer does not
  compose a garbage delta. Audit which mover kinds are revisions
  (`RigExecRevisionOp`) versus generators before implementing.
- **Zero delta still publishes.** Ownership, not value comparison, drives the
  velocities/accelerations blocks (§10.5). An all-zero delta is still RigExec
  owning the leaf.
- **Cost**: one extra `VtVec3fArray` per published prim per generation. Real,
  and the reason `pointsBase` should be stored only when the chain actually
  had a base.

### 9.3 What this does not fix

Composition requires the other party to compose too. Skel replaces; see the
exposure note in §0. A delta also cannot recover from upstream having removed
the prim or changed its type — that stays a structural resync.
