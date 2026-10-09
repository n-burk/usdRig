// UsdImaging prim adapter for RigExecTouchOverlay.
// The live TouchPose highlight is not this prim. It is a Storm shader
// tint on the body mesh (touchPoseHighlight.h): Hydra primvars carry the
// per-face region id and the colour table, and nothing is authored on the
// stage or in the session layer.
// RigExecTouchOverlay remains an empty, invisible Mesh shipped beside a
// RigExecTouchRegions scope. UsdImaging binds an adapter by concrete type
// name, so a type with no adapter is not an rprim at all. This file only
// registers the type as the stock mesh adapter. It does not build points,
// write a session layer, or light a region.
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/type.h"
#include "pxr/usdImaging/usdImaging/meshAdapter.h"
#include "pxr/usdImaging/usdImaging/primAdapter.h"

PXR_NAMESPACE_OPEN_SCOPE

/// Draws a RigExecTouchOverlay as the mesh it is, if anything makes it visible.
class RigExecTouchOverlayAdapter final : public UsdImagingMeshAdapter {
public:
    using BaseAdapter = UsdImagingMeshAdapter;

    RigExecTouchOverlayAdapter() : UsdImagingMeshAdapter() {}
    ~RigExecTouchOverlayAdapter() override = default;
};

TF_REGISTRY_FUNCTION(TfType)
{
    using Adapter = RigExecTouchOverlayAdapter;
    TfType type = TfType::Define<Adapter, TfType::Bases<Adapter::BaseAdapter>>();
    type.SetFactory<UsdImagingPrimAdapterFactory<Adapter>>();
}

PXR_NAMESPACE_CLOSE_SCOPE
