//
// RigExec imaging registry: process-global rendezvous between the
// UsdImaging scene-index plugin (which builds filter chains when engines
// construct) and the application activation (which owns the stage,
// evaluator, and time source). Order-independent: chains read the shared
// snapshot store atomically, so activation may happen before or after
// chain construction.
//
#ifndef RIGEXEC_IMAGING_REGISTRY_H
#define RIGEXEC_IMAGING_REGISTRY_H

#include "bridge.h"
#include "sceneIndices.h"
#include "snapshotStore.h"

#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/weakBase.h"
#include "pxr/usd/usd/notice.h"

#include <memory>
#include <mutex>
#include <vector>

namespace rigExec {

class RigExecImagingRegistry : public TfWeakBase {
public:
    static RigExecImagingRegistry &GetInstance();

    /// The shared store every results scene index reads from.
    const std::shared_ptr<RigExecSnapshotStore> &GetStore() const {
        return _store;
    }

    /// Called by the scene-index plugin for each constructed chain.
    void RegisterChain(
        const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
        const RigExecBindingResolvingSceneIndexRefPtr &binding,
        const RigExecResultsSceneIndexRefPtr &results);

    /// Activates evaluation for one rig on a stage; compiles and
    /// publishes the initial generation at the given frame.
    bool Activate(
        const UsdStageRefPtr &stage, const SdfPath &rigPath,
        UsdTimeCode initialTime, std::vector<std::string> *errors);

    /// Serialized evaluate-then-publish, broadcast to every chain.
    bool SetTime(UsdTimeCode time);

    /// Drops the bridge; chains remain and read the (cleared) store.
    void Deactivate();

    bool IsActive() const { return static_cast<bool>(_bridge); }

private:
    RigExecImagingRegistry();

    // Weak references: chains are owned by their scene index graphs and
    // die with their engines; the registry prunes expired entries.
    struct Chain {
        TfWeakPtr<RigExecInternalPrimPruningSceneIndex> pruning;
        TfWeakPtr<RigExecBindingResolvingSceneIndex> binding;
        TfWeakPtr<RigExecResultsSceneIndex> results;
    };

    void _Broadcast(const RigExecImagingBridge::PublishResult &result);

    /// Edit-driven re-evaluation: any authored change touching the rig's
    /// asset (every input that factors into the final frame lives
    /// beneath it) re-evaluates at the last-set time and republishes, so
    /// property edits redraw exactly like timeline changes.
    void _OnObjectsChanged(
        const UsdNotice::ObjectsChanged &notice,
        const UsdStageWeakPtr &sender);

    std::mutex _mutex;
    std::shared_ptr<RigExecSnapshotStore> _store;
    std::vector<Chain> _chains;
    std::unique_ptr<RigExecImagingBridge> _bridge;
    SdfPath _generatedScope;
    SdfPath _assetRoot;
    UsdTimeCode _lastTime = UsdTimeCode::Default();
    TfNotice::Key _changeKey;
};

}  // namespace rigExec

// C surface for language-neutral activation (e.g. the usdview Python
// plugin via ctypes + UsdUtilsStageCache ids). Returns 0 on success.
extern "C" {

__declspec(dllexport) int RigExecImaging_Activate(
    long long stageCacheId, const char *rigPath, double initialFrame);
__declspec(dllexport) int RigExecImaging_SetTime(double frame);
__declspec(dllexport) void RigExecImaging_Deactivate();
/// Current published snapshot generation (0 before first publication).
__declspec(dllexport) long long RigExecImaging_GetGeneration();

}

#endif  // RIGEXEC_IMAGING_REGISTRY_H
