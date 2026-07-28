//
// RigExec imaging registry and C activation surface.
//
#include "registry.h"

#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usdUtils/stageCache.h"

#include <algorithm>
#include <cstdio>

namespace rigExec {

RigExecImagingRegistry &
RigExecImagingRegistry::GetInstance()
{
    static RigExecImagingRegistry instance;
    return instance;
}

RigExecImagingRegistry::RigExecImagingRegistry()
    : _store(std::make_shared<RigExecSnapshotStore>())
{
}

void
RigExecImagingRegistry::RegisterChain(
    const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
    const RigExecBindingResolvingSceneIndexRefPtr &binding,
    const RigExecResultsSceneIndexRefPtr &results)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _chains.push_back(
        {TfWeakPtr<RigExecInternalPrimPruningSceneIndex>(
             get_pointer(pruning)),
         TfWeakPtr<RigExecBindingResolvingSceneIndex>(get_pointer(binding)),
         TfWeakPtr<RigExecResultsSceneIndex>(get_pointer(results))});
    // A chain constructed after activation adopts the current epoch scope.
    if (!_generatedScope.IsEmpty()) {
        pruning->SetOwnedScopes({_generatedScope});
    }
}

bool
RigExecImagingRegistry::Activate(
    const UsdStageRefPtr &stage, const SdfPath &rigPath,
    UsdTimeCode initialTime, std::vector<std::string> *errors)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _bridge = std::make_unique<RigExecImagingBridge>(stage, rigPath, _store);
    if (!_bridge->Compile(errors)) {
        _bridge.reset();
        return false;
    }
    _generatedScope = _bridge->GetGeneratedScope();
    for (Chain &chain : _chains) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes({_generatedScope});
        }
    }
    // Edit-driven re-evaluation: listen on the source stage so property
    // edits republish at the current time (the rig's inputs all live
    // beneath the asset root; cross-asset writes are rejected).
    _assetRoot = rigPath.GetParentPath();
    _lastTime = initialTime;
    TfNotice::Revoke(_changeKey);
    _changeKey = TfNotice::Register(
        TfCreateWeakPtr(this), &RigExecImagingRegistry::_OnObjectsChanged,
        stage);
    _Broadcast(_bridge->EvaluateAndPublishResult(initialTime));
    return true;
}

bool
RigExecImagingRegistry::SetTime(UsdTimeCode time)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_bridge) {
        return false;
    }
    _lastTime = time;
    const RigExecImagingBridge::PublishResult result =
        _bridge->EvaluateAndPublishResult(time);
    _Broadcast(result);
    return result.ok;
}

void
RigExecImagingRegistry::_OnObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &)
{
    SdfPath assetRoot;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_bridge || _assetRoot.IsEmpty()) {
            return;
        }
        assetRoot = _assetRoot;
    }
    // Any edit touching the asset can factor into the final frame
    // (solvers, joints, movers, controls, weights, driver geometry,
    // guide styling): re-evaluate at the current time. The evaluator's
    // epoch digest turns structural edits into recompiles; value edits
    // flow through exec invalidation on the shared layers.
    auto touchesAsset = [&assetRoot](const SdfPath &path) {
        const SdfPath primPath = path.GetPrimPath();
        return primPath.HasPrefix(assetRoot) ||
               assetRoot.HasPrefix(primPath);
    };
    bool relevant = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (touchesAsset(path)) {
            relevant = true;
            break;
        }
    }
    if (!relevant) {
        for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
            if (touchesAsset(path)) {
                relevant = true;
                break;
            }
        }
    }
    if (relevant) {
        UsdTimeCode time;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            time = _lastTime;
        }
        SetTime(time);
    }
}

void
RigExecImagingRegistry::Deactivate()
{
    std::lock_guard<std::mutex> lock(_mutex);
    TfNotice::Revoke(_changeKey);
    _changeKey = TfNotice::Key();
    _assetRoot = SdfPath();
    _bridge.reset();
    _generatedScope = SdfPath();
    RigExecImagingBridge::PublishResult cleared;
    cleared.ok = true;
    cleared.dirtied = _store->Publish(nullptr);
    _Broadcast(cleared);
}

void
RigExecImagingRegistry::_Broadcast(
    const RigExecImagingBridge::PublishResult &result)
{
    if (!result.ok) {
        return;
    }
    // Prune chains whose scene index graphs were destroyed.
    _chains.erase(
        std::remove_if(_chains.begin(), _chains.end(),
                       [](const Chain &c) { return !c.results; }),
        _chains.end());
    for (Chain &chain : _chains) {
        if (result.epoch && chain.binding) {
            chain.binding->SetBindingEpoch(result.epoch);
        }
        if (chain.results) {
            chain.results->NotifyGenerationPublished(result.dirtied);
        }
    }
}

}  // namespace rigExec

// ---------------------------------------------------------------------------
// C activation surface.
// ---------------------------------------------------------------------------

using rigExec::RigExecImagingRegistry;

extern "C" {

int
RigExecImaging_Activate(
    long long stageCacheId, const char *rigPath, double initialFrame)
{
    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdUtilsStageCache::Get().Find(
        PXR_NS::UsdStageCache::Id::FromLongInt(
            static_cast<long int>(stageCacheId)));
    if (!stage) {
        std::printf("rigExecImaging: no stage for cache id %lld\n",
                    stageCacheId);
        return 1;
    }

    PXR_NS::SdfPath path;
    if (rigPath && rigPath[0]) {
        path = PXR_NS::SdfPath(rigPath);
    } else {
        // Discover the first RigExecRig prim on the stage.
        for (const PXR_NS::UsdPrim &prim : stage->Traverse()) {
            if (prim.GetTypeName() == "RigExecRig") {
                path = prim.GetPath();
                break;
            }
        }
    }
    if (path.IsEmpty()) {
        std::printf("rigExecImaging: no RigExecRig prim found\n");
        return 2;
    }

    std::vector<std::string> errors;
    if (!RigExecImagingRegistry::GetInstance().Activate(
            stage, path, PXR_NS::UsdTimeCode(initialFrame), &errors)) {
        for (const std::string &e : errors) {
            std::printf("rigExecImaging: %s\n", e.c_str());
        }
        return 3;
    }
    std::printf("rigExecImaging: activated %s\n", path.GetText());
    return 0;
}

int
RigExecImaging_SetTime(double frame)
{
    return RigExecImagingRegistry::GetInstance().SetTime(
               PXR_NS::UsdTimeCode(frame))
        ? 0 : 1;
}

void
RigExecImaging_Deactivate()
{
    RigExecImagingRegistry::GetInstance().Deactivate();
}

long long
RigExecImaging_GetGeneration()
{
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    return snapshot ? static_cast<long long>(snapshot->generation) : 0;
}

}  // extern "C"
