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

// Visibility for the C surface below. Windows needs dllexport while building
// rigExecImaging and dllimport when a consumer includes this header -- the
// build defines RIGEXEC_IMAGING_EXPORTS to tell the two apart. Elsewhere the
// ELF/Mach-O equivalent is default visibility, which also keeps the symbols
// alive if the tree is ever built with -fvisibility=hidden.
#if defined(_WIN32)
#  if defined(RIGEXEC_IMAGING_EXPORTS)
#    define RIGEXEC_IMAGING_C_API __declspec(dllexport)
#  else
#    define RIGEXEC_IMAGING_C_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define RIGEXEC_IMAGING_C_API __attribute__((visibility("default")))
#else
#  define RIGEXEC_IMAGING_C_API
#endif

// C surface for language-neutral activation (e.g. the usdview Python
// plugin via ctypes + UsdUtilsStageCache ids). Returns 0 on success.
extern "C" {

RIGEXEC_IMAGING_C_API int RigExecImaging_Activate(
    long long stageCacheId, const char *rigPath, double initialFrame);
RIGEXEC_IMAGING_C_API int RigExecImaging_SetTime(double frame);
RIGEXEC_IMAGING_C_API void RigExecImaging_Deactivate();
/// Current published snapshot generation (0 before first publication).
RIGEXEC_IMAGING_C_API long long RigExecImaging_GetGeneration();

/// ASSET-SPACE axis-aligned bounds of everything the current generation
/// draws for \p primPath, as min xyz then max xyz in \p outMinMax.
/// Returns 1 when the prim draws something, 0 otherwise (\p outMinMax is
/// then untouched).
///
/// This exists because a rig draws nothing a bounding box can be computed
/// from the ordinary way. Guides are synthesized inside the imaging chain
/// and never authored, and the RigExec prim types are not UsdGeomImageable,
/// so UsdGeomBBoxCache -- which is what usdview's frame-selection goes
/// through -- correctly reports an empty box for every joint, control, and
/// solver on the stage. Framing a control therefore moved the camera
/// nowhere. The snapshot is the only place the drawn extent exists, so the
/// answer has to come from here.
///
/// Asset space, not world: guide frames carry no stage placement (see
/// RigExecImagingSnapshot::assetRoot), so the caller composes the asset
/// root's own world transform. No Hydra dependency -- snapshot data only.
RIGEXEC_IMAGING_C_API int RigExecImaging_GetGuideBoundsAssetSpace(
    const char *primPath, double outMinMax[6]);

/// The union of RigExecImaging_GetGuideBoundsAssetSpace over every prim in
/// the current generation, so framing the rig frames its whole guide set.
/// Returns 1 when anything at all draws, 0 otherwise.
RIGEXEC_IMAGING_C_API int RigExecImaging_GetAllGuideBoundsAssetSpace(
    double outMinMax[6]);

}

#endif  // RIGEXEC_IMAGING_REGISTRY_H
