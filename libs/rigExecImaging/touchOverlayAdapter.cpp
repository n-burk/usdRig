//
// A UsdImaging prim adapter for RigExecTouchOverlay.
//
// WHY THIS FILE EXISTS. `RigExecTouchOverlay` inherits Mesh in the
// schema, and that is not enough to draw. UsdImaging binds an adapter to
// a prim by its CONCRETE type name, and a concrete type nobody has
// registered an adapter for produces a prim with no imaging
// representation at all: it composes perfectly, reports the right
// points, counts, extent, visibility and purpose, and changes zero
// pixels. Measured on the biped, the identical overlay carrying 17,466
// points at opacity 0.85 moved 0 of 80,730 sampled pixels as a
// RigExecTouchOverlay and lit its region immediately as a plain Mesh.
// Nothing in USD reports this; the prim is simply not an rprim.
//
// So the type is declared to Hydra here, and the declaration is the
// whole adapter: a RigExecTouchOverlay IS a mesh, drawn by exactly the
// stock mesh adapter, with no behaviour of its own. Subclassing rather
// than aliasing is the sanctioned route -- it is what UsdGeom's own
// derived gprim types do -- and it means the overlay gains every
// improvement the stock adapter ever gets for free.
//
// The point of paying for this at all is that the overlay then lives in
// the asset, under the RigExecTouchRegions scope that owns it, instead
// of at the stage root. A shot with three characters gets three
// overlays inside three assets, each shipped with the regions it lights,
// and no global prim for them to fight over.
//
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/type.h"
#include "pxr/usdImaging/usdImaging/meshAdapter.h"
#include "pxr/usdImaging/usdImaging/primAdapter.h"

PXR_NAMESPACE_OPEN_SCOPE

/// Draws a RigExecTouchOverlay as the mesh it is.
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
