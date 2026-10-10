#ifndef RIGEXEC_PROJECTOR_CAPTURE_NOTICE_H
#define RIGEXEC_PROJECTOR_CAPTURE_NOTICE_H
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/sdf/path.h"
namespace rigExec {
// Only the raw static mesh transform captured by a projector is structural.
// Ordinary provider transform values continue through their sampled graph leaves.
inline bool RigExecProjectorMeshCaptureAffected(
    const PXR_NS::UsdNotice::ObjectsChanged &notice,const PXR_NS::SdfPath &points)
{
    const auto mesh=points.GetPrimPath();
    if(mesh.IsEmpty())return false;
    const auto xform=[&](const PXR_NS::SdfPath &path) {
        if(!path.IsPropertyPath() || !mesh.HasPrefix(path.GetPrimPath()))return false;
        const auto &name=path.GetName();
        return name=="xformOpOrder" || name.compare(0,8,"xformOp:")==0;
    };
    for(const auto &path:notice.GetChangedInfoOnlyPaths())if(xform(path))return true;
    for(const auto &path:notice.GetResyncedPaths())
        if(xform(path) || (path.IsPrimPath() && mesh.HasPrefix(path)))return true;
    for(const auto &path:notice.GetResolvedAssetPathsResyncedPaths())
        if(xform(path) || (path.IsPrimPath() && mesh.HasPrefix(path)))return true;
    return false;
}
}
#endif
