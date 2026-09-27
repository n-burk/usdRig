//
// Keyless UsdImaging API schema adapter: the rig's time trigger.
//
// WHY THIS FILE EXISTS. The results scene index overlays evaluated data, but
// nothing in a stock UsdImaging host says WHEN to evaluate: usdview drives
// the registry through the ctypes plugin's frame signal, and usdrecord has no
// such hook -- its FrameRecorder only calls UsdImagingStageSceneIndex::SetTime
// per frame. So a rig recorded with usdrecord renders its rest pose on every
// frame: evaluation never runs.
//
// This adapter closes that gap with no application code (docs/specs/
// imaging-datasource-redesign.md §3.2). It contributes one time-varying leaf,
// rigExec/time, to every RigExecRoot prim's data; it notes each root at
// subprim-enumeration time so the results index's _PrimsAdded can force the
// data (and the activation) into existence during populate; and it activates
// the rig the first time the data is built, as the backup for chains that
// populated unobserved. From then on SetTime dirties the leaf,
// RigExecResultsSceneIndex::_PrimsDirtied pulls the frame out of it and
// evaluates, and the recorded frames are the evaluated ones.
//
// Keyless (apiSchemaName is the empty string in plugInfo -- omitting the key
// is a discovery error) rather than a prim adapter on purpose: a prim adapter
// would REPLACE the fallback data source a RigExecRoot prim gets today, while
// an API schema adapter's contribution is OVERLAID onto it
// (apiSchemaAdapter.h:71-74). The rig prim keeps the exact representation it
// has always had; this file only adds one container to it.
//
// THE STAGE KEY (docs/multistage-imaging.md). Several stages may be imaged in
// one process, each with its own imaging context, and a scene-index chain is
// built with no stage at all. So the same container also carries
// rigExec/stageKey -- the key of ForStage(prim.GetStage()) -- on TOP-LEVEL
// prims (the pseudo-root's children), merged with the trigger when a
// top-level prim is also a rig root. The results index pulls it on
// population to bind its chain to that context (registry.h, "Chain ->
// context binding"). Every other prim is untouched: a prim that is neither a
// RigExecRoot nor top-level pays one type-name and one path-length
// comparison and gets no subprim data, exactly as before.
//

#include "registry.h"
#include "sceneIndices.h"

#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/type.h"
#include "pxr/imaging/hd/dataSource.h"
#include "pxr/imaging/hd/dataSourceLocator.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usdImaging/usdImaging/apiSchemaAdapter.h"
#include "pxr/usdImaging/usdImaging/dataSourceStageGlobals.h"

#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

namespace {

const TfToken &
_RigExecRootTypeName()
{
    static const TfToken root("RigExecRoot");
    return root;
}

/// The rigExec/time value: the stage scene index's current frame, read live
/// off its globals. A plain HdSampledDataSource rather than a typed one: the
/// value IS a UsdTimeCode, so Default-ness survives the pull -- a double leaf
/// would silently turn the default frame into frame 0.
class _RigExecTimeDataSource final : public HdSampledDataSource {
public:
    HD_DECLARE_DATASOURCE(_RigExecTimeDataSource);

    VtValue GetValue(Time shutterOffset) override {
        return VtValue(_stageGlobals.GetTime());
    }

    bool GetContributingSampleTimesForInterval(
        Time, Time, std::vector<Time> *) override {
        // A trigger, not render data: it names the frame Hydra is pulling,
        // and per-frame invalidation arrives through FlagAsTimeVarying, not
        // through samples.
        return false;
    }

private:
    explicit _RigExecTimeDataSource(
        const UsdImagingDataSourceStageGlobals &stageGlobals)
        : _stageGlobals(stageGlobals) {}

    // The same reference every UsdImagingDataSourceAttribute holds: the
    // globals outlive the data sources built from them.
    const UsdImagingDataSourceStageGlobals &_stageGlobals;
};

/// Whether \p prim is a child of the pseudo-root. The path's element count
/// answers without touching prim data: a top-level prim's path has exactly
/// one element, and so does nothing else UsdImaging populates.
bool
_IsTopLevel(UsdPrim const& prim)
{
    return prim.GetPath().GetPathElementCount() == 1;
}

}  // namespace

/// Contributes the rigExec/time trigger leaf to RigExecRoot prims and the
/// rigExec/stageKey leaf to top-level prims, and activates the rig on first
/// sight so hosts without explicit activation (usdrecord) evaluate.
class RigExecImagingRigAdapter final : public UsdImagingAPISchemaAdapter {
public:
    TfTokenVector GetImagingSubprims(
        UsdPrim const& prim, TfToken const& appliedInstanceName) override {
        if (!appliedInstanceName.IsEmpty()) {
            return {};
        }
        const bool rigRoot = prim.GetTypeName() == _RigExecRootTypeName();
        if (!rigRoot && !_IsTopLevel(prim)) {
            return {};
        }
        if (rigRoot) {
            // Noted for eager activation: this runs during populate, before
            // any data pull, so the results index's _PrimsAdded can activate
            // the rig on the populate thread (see NoteRigRoot). Cheap and
            // idempotent -- ignored once active -- because this also runs
            // on every traversal. Noted on the prim's OWN stage's context:
            // a chain only matches notes of the context it is bound to.
            if (const rigExec::RigExecImagingRegistry::Ptr context =
                    rigExec::RigExecImagingRegistry::ForStage(
                        prim.GetStage())) {
                context->NoteRigRoot(prim.GetPath());
            }
        }
        // The primary subprim only: this contributes DATA to the prim,
        // never new prims.
        return {TfToken()};
    }

    TfToken GetImagingSubprimType(
        UsdPrim const&, TfToken const&, TfToken const&) override {
        // No opinion: the prim adapter (or the fallback, for a rig prim
        // nothing else adapted) owns the prim's Hydra type.
        return TfToken();
    }

    HdContainerDataSourceHandle GetImagingSubprimData(
        UsdPrim const& prim, TfToken const& subprim,
        TfToken const& appliedInstanceName,
        const UsdImagingDataSourceStageGlobals &stageGlobals) override {
        if (!subprim.IsEmpty() || !appliedInstanceName.IsEmpty()) {
            return nullptr;
        }
        const bool rigRoot = prim.GetTypeName() == _RigExecRootTypeName();
        const bool topLevel = _IsTopLevel(prim);
        if (!rigRoot && !topLevel) {
            return nullptr;
        }
        const rigExec::RigExecImagingRegistry::Ptr context =
            rigExec::RigExecImagingRegistry::ForStage(prim.GetStage());
        if (!context) {
            return nullptr;
        }
        TfToken names[2];
        HdDataSourceBaseHandle values[2];
        size_t count = 0;
        if (rigRoot) {
            // First sight of a rig in a host with no explicit activation:
            // compile it and publish the first generation now, so the chain
            // below already serves evaluated data. Idempotent -- a host that
            // activates explicitly (the usdview plugin) is already active
            // and this is a no-op -- and serialized inside the context, so
            // two rigs populating at once cost a compile, not correctness.
            // Each stage activates its own context; none refuses another.
            context->EnsureActivated(prim.GetStage(), stageGlobals.GetTime());
            // The trigger itself: from now on every SetTime dirties
            // rigExec/time on this prim, and the results index evaluates
            // through it.
            stageGlobals.FlagAsTimeVarying(
                prim.GetPath(),
                HdDataSourceLocator(rigExec::RigExecTriggerContainerToken(),
                                    rigExec::RigExecTriggerLeafToken()));
            names[count] = rigExec::RigExecTriggerLeafToken();
            values[count] = _RigExecTimeDataSource::New(stageGlobals);
            ++count;
        }
        if (topLevel) {
            // The chain -> context binding transport: a constant, so it is
            // never flagged time-varying and never invalidated -- a stage
            // replaced under the engine re-adds its prims with new data.
            names[count] = rigExec::RigExecStageKeyLeafToken();
            values[count] =
                HdRetainedTypedSampledDataSource<uint64_t>::New(
                    context->GetKey());
            ++count;
        }
        return HdRetainedContainerDataSource::New(
            rigExec::RigExecTriggerContainerToken(),
            HdRetainedContainerDataSource::New(count, names, values));
    }

    HdDataSourceLocatorSet InvalidateImagingSubprim(
        UsdPrim const&, TfToken const&, TfToken const&,
        TfTokenVector const&,
        UsdImagingPropertyInvalidationType) override {
        // Authored edits reach the rig through its context's own stage
        // notice (_OnObjectsChanged), which re-evaluates and republishes; no
        // USD property maps onto the trigger or the stage key.
        return HdDataSourceLocatorSet();
    }
};

TF_REGISTRY_FUNCTION(TfType)
{
    using Adapter = RigExecImagingRigAdapter;
    TfType type =
        TfType::Define<Adapter, TfType::Bases<UsdImagingAPISchemaAdapter>>();
    type.SetFactory<UsdImagingAPISchemaAdapterFactory<Adapter>>();
}

PXR_NAMESPACE_CLOSE_SCOPE
