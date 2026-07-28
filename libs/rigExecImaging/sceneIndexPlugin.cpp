//
// RigExec UsdImaging scene-index plugin: inserts the three RigExec
// filters inside the UsdImaging chain (the same mechanism UsdSkelImaging
// uses for its points-resolving skinning scene indices). Discovered
// through Plug metadata; instantiated by UsdImagingCreateSceneIndices for
// every constructed UsdImaging graph.
//
// Note on placement (spec §10.1): the spec's canonical construction wraps
// the completed UsdImaging branch explicitly. Stock usdview offers no
// application hook, so this transport inserts inside the UsdImaging chain
// via the sanctioned plugin point instead — a compatibility transport,
// not a second RigExec integration.
//
#include "registry.h"
#include "sceneIndices.h"

#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/type.h"
#include "pxr/usdImaging/usdImaging/sceneIndexPlugin.h"

PXR_NAMESPACE_OPEN_SCOPE

class RigExecUsdImagingSceneIndexPlugin final
    : public UsdImagingSceneIndexPlugin {
public:
    HdSceneIndexBaseRefPtr AppendSceneIndex(
        HdSceneIndexBaseRefPtr const &inputScene) override
    {
        rigExec::RigExecImagingRegistry &registry =
            rigExec::RigExecImagingRegistry::GetInstance();

        auto pruning =
            rigExec::RigExecInternalPrimPruningSceneIndex::New(inputScene);
        auto binding =
            rigExec::RigExecBindingResolvingSceneIndex::New(pruning);
        auto results = rigExec::RigExecResultsSceneIndex::New(
            binding, registry.GetStore());
        registry.RegisterChain(pruning, binding, results);
        return results;
    }
};

TF_REGISTRY_FUNCTION(UsdImagingSceneIndexPlugin)
{
    UsdImagingSceneIndexPlugin::Define<RigExecUsdImagingSceneIndexPlugin>();
}

PXR_NAMESPACE_CLOSE_SCOPE
