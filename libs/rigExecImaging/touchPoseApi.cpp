//
// TouchPose C surface -- see touchPose.h.
//
#include "touchPose.h"

#include "registry.h"
#include "touchPoseHighlight.h"
#include "touchPoseMesh.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/stageCache.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usdImaging/usdImaging/tokens.h"
#include "pxr/usd/usdUtils/stageCache.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

struct _Context {
    UsdStageWeakPtr stage;
    SdfPath meshPath;
    RigExecTouchPoseMesh geometry;

    // What the geometry was last synced from, so a hover that finds nothing
    // moved costs a pointer compare.
    VtVec3fArray syncedPoints;      // keeps the storage alive for the compare
    uint64_t syncedGeneration = ~uint64_t(0);
    bool syncedFromSnapshot = false;
    bool haveSyncedTime = false;
    bool syncedDefault = true;
    double syncedFrame = 0.0;

    // regions
    int regionCount = 0;
    std::vector<int32_t> regionOf;
    std::vector<GfVec3f> hoverColors;
    std::vector<GfVec3f> editColors;

    // highlight
    bool highlightOn = false;
    VtVec4fArray table;
};

std::mutex &
_HandlesMutex()
{
    static std::mutex mutex;
    return mutex;
}

std::map<long long, std::shared_ptr<_Context>> &
_Handles()
{
    static std::map<long long, std::shared_ptr<_Context>> handles;
    return handles;
}

long long
_Register(std::shared_ptr<_Context> context)
{
    static long long next = 0;
    std::lock_guard<std::mutex> lock(_HandlesMutex());
    const long long handle = ++next;
    _Handles()[handle] = std::move(context);
    return handle;
}

std::shared_ptr<_Context>
_Get(long long handle)
{
    std::lock_guard<std::mutex> lock(_HandlesMutex());
    auto it = _Handles().find(handle);
    return it == _Handles().end() ? nullptr : it->second;
}

GfMatrix4d
_Matrix(const double *m)
{
    return GfMatrix4d(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
                      m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
}

size_t
_TableRows(const _Context &context)
{
    // A constant array of ONE element is declared by Storm as a scalar, and
    // the shader indexes it; two rows is the floor that keeps it an array.
    return std::max<size_t>(size_t(context.regionCount) + 1, 2);
}

VtFloatArray
_FaceSlots(const _Context &context)
{
    VtFloatArray slots(context.geometry.GetFaceCount(), 0.0f);
    const size_t n = std::min(slots.size(), context.regionOf.size());
    float *out = slots.data();
    for (size_t f = 0; f < n; ++f) {
        const int32_t r = context.regionOf[f];
        out[f] = (r >= 0 && r < context.regionCount) ? float(r + 1) : 0.0f;
    }
    return slots;
}

void
_Publish(_Context &context, bool slotsChanged)
{
    if (!context.highlightOn || context.meshPath.IsEmpty()) {
        return;
    }
    RigExecTouchPoseHighlights &highlights =
        RigExecTouchPoseHighlights::GetInstance();
    if (context.table.size() != _TableRows(context)) {
        context.table = VtVec4fArray(_TableRows(context), GfVec4f(0.0f));
    }
    const RigExecTouchPoseHighlightMeshConstPtr current =
        highlights.Find(context.meshPath);
    if (!current || slotsChanged ||
        current->table.size() != context.table.size()) {
        highlights.SetMesh(context.meshPath, _FaceSlots(context), context.table);
    } else {
        highlights.SetTable(context.meshPath, context.table);
    }
}

}  // namespace
}  // namespace rigExec

using namespace rigExec;

extern "C" {

long long
RigExecTouchPose_Open(long long stageCacheId, const char *meshPath)
{
    if (!meshPath || !meshPath[0]) {
        return 0;
    }
    const UsdStageRefPtr stage = UsdUtilsStageCache::Get().Find(
        UsdStageCache::Id::FromLongInt(static_cast<long int>(stageCacheId)));
    if (!stage) {
        return 0;
    }
    const SdfPath path(meshPath);
    const UsdGeomMesh mesh(stage->GetPrimAtPath(path));
    if (!mesh) {
        return 0;
    }
    VtIntArray counts, indices;
    VtVec3fArray points;
    mesh.GetFaceVertexCountsAttr().Get(&counts);
    mesh.GetFaceVertexIndicesAttr().Get(&indices);
    mesh.GetPointsAttr().Get(&points);
    auto context = std::make_shared<_Context>();
    context->stage = stage;
    context->meshPath = path;
    if (!context->geometry.SetTopology(counts, indices, points.size())) {
        return 0;
    }
    context->regionOf.assign(counts.size(), -1);
    context->geometry.SetPoints(points);
    long long handle = _Register(context);
    RigExecTouchPose_SyncPose(handle, 0.0, 1, 1);
    return handle;
}

long long
RigExecTouchPose_OpenTopology(
    const int32_t *counts, int faceCount, const int32_t *indices,
    int indexCount, int pointCount, const char *meshPath)
{
    if (!counts || !indices || faceCount < 0 || indexCount < 0 ||
        pointCount < 0) {
        return 0;
    }
    auto context = std::make_shared<_Context>();
    if (meshPath && meshPath[0]) {
        context->meshPath = SdfPath(meshPath);
    }
    VtIntArray c(counts, counts + faceCount);
    VtIntArray i(indices, indices + indexCount);
    if (!context->geometry.SetTopology(c, i, size_t(pointCount))) {
        return 0;
    }
    context->regionOf.assign(size_t(faceCount), -1);
    return _Register(context);
}

void
RigExecTouchPose_Close(long long handle)
{
    std::shared_ptr<_Context> context;
    {
        std::lock_guard<std::mutex> lock(_HandlesMutex());
        auto it = _Handles().find(handle);
        if (it == _Handles().end()) {
            return;
        }
        context = it->second;
        _Handles().erase(it);
    }
    if (context->highlightOn && !context->meshPath.IsEmpty()) {
        RigExecTouchPoseHighlights::GetInstance().RemoveMesh(context->meshPath);
    }
}

int
RigExecTouchPose_GetFaceCount(long long handle)
{
    const auto context = _Get(handle);
    return context ? int(context->geometry.GetFaceCount()) : -1;
}

int
RigExecTouchPose_GetPointCount(long long handle)
{
    const auto context = _Get(handle);
    return context ? int(context->geometry.GetPointCount()) : -1;
}

int
RigExecTouchPose_SyncPose(long long handle, double frame, int isDefault,
                          int force)
{
    const auto context = _Get(handle);
    if (!context) {
        return -1;
    }
    const UsdStageRefPtr stage = context->stage;
    if (!stage) {
        return 0;       // a topology-only handle: its points are explicit
    }
    const UsdTimeCode time = isDefault ? UsdTimeCode::Default()
                                       : UsdTimeCode(frame);
    const bool timeMoved = !context->haveSyncedTime ||
                           context->syncedDefault != bool(isDefault) ||
                           (!isDefault && context->syncedFrame != frame);
    bool moved = false;

    // The points the viewport is drawing: the rig's, when this generation
    // publishes them for this mesh on this stage.
    const RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    const RigExecPublishedPrim *published = nullptr;
    if (snapshot && get_pointer(snapshot->stage) == get_pointer(stage)) {
        auto it = snapshot->prims.find(context->meshPath);
        if (it != snapshot->prims.end() && it->second.hasPoints &&
            it->second.points.size() == context->geometry.GetPointCount()) {
            published = &it->second;
        }
    }
    const uint64_t generation = snapshot ? snapshot->generation : 0;
    if (published) {
        if (force || !context->syncedFromSnapshot ||
            published->points.cdata() != context->syncedPoints.cdata()) {
            context->geometry.SetPoints(published->points);
            context->syncedPoints = published->points;
            context->syncedFromSnapshot = true;
            moved = true;
        }
    } else if (force || context->syncedFromSnapshot || timeMoved) {
        VtVec3fArray rest;
        const UsdGeomMesh mesh(stage->GetPrimAtPath(context->meshPath));
        if (mesh && mesh.GetPointsAttr().Get(&rest, time) &&
            rest.size() == context->geometry.GetPointCount() &&
            (force || context->syncedFromSnapshot ||
             rest.cdata() != context->syncedPoints.cdata())) {
            context->geometry.SetPoints(rest);
            context->syncedPoints = rest;
            moved = true;
        }
        context->syncedFromSnapshot = false;
    }

    // The transform: re-read when the time or the generation moved. A rig
    // that animates the mesh's own ancestors is not reflected here -- the
    // mesh's world transform is the stage's.
    if (force || timeMoved || generation != context->syncedGeneration) {
        const UsdPrim prim = stage->GetPrimAtPath(context->meshPath);
        if (prim) {
            UsdGeomXformCache cache(time);
            const GfMatrix4d xform = cache.GetLocalToWorldTransform(prim);
            if (xform != context->geometry.GetTransform()) {
                context->geometry.SetTransform(xform);
                moved = true;
            }
        }
    }
    context->syncedGeneration = generation;
    context->haveSyncedTime = true;
    context->syncedDefault = bool(isDefault);
    context->syncedFrame = frame;
    return moved ? 1 : 0;
}

int
RigExecTouchPose_SetPoints(long long handle, const float *xyz, int pointCount)
{
    const auto context = _Get(handle);
    if (!context || !xyz || pointCount < 0 ||
        size_t(pointCount) != context->geometry.GetPointCount()) {
        return -1;
    }
    VtVec3fArray points(static_cast<size_t>(pointCount));
    std::memcpy(points.data()->data(), xyz, sizeof(float) * 3 * pointCount);
    context->geometry.SetPoints(points);
    context->syncedPoints = points;
    context->syncedFromSnapshot = false;
    return 0;
}

int
RigExecTouchPose_SetTransform(long long handle, const double *m)
{
    const auto context = _Get(handle);
    if (!context || !m) {
        return -1;
    }
    context->geometry.SetTransform(_Matrix(m));
    return 0;
}

int
RigExecTouchPose_GetPoints(long long handle, float *out, int capacity)
{
    const auto context = _Get(handle);
    if (!context) {
        return -1;
    }
    const VtVec3fArray &points = context->geometry.GetPoints();
    const size_t n = std::min(points.size(), size_t(std::max(capacity, 0)));
    if (out && n) {
        std::memcpy(out, points.cdata()->data(), sizeof(float) * 3 * n);
    }
    return int(points.size());
}

int
RigExecTouchPose_GetBounds(long long handle, double *out)
{
    const auto context = _Get(handle);
    GfVec3d lo, hi;
    if (!context || !out || !context->geometry.GetWorldBounds(&lo, &hi)) {
        return 0;
    }
    for (int k = 0; k < 3; ++k) {
        out[k] = lo[k];
        out[3 + k] = hi[k];
    }
    return 1;
}

int
RigExecTouchPose_SetFaceRegions(long long handle, const int32_t *regionOf,
                                int faceCount, int regionCount)
{
    const auto context = _Get(handle);
    if (!context || faceCount < 0 || regionCount < 0) {
        return -1;
    }
    const size_t faces = context->geometry.GetFaceCount();
    std::vector<int32_t> next(faces, -1);
    if (regionOf) {
        std::copy(regionOf, regionOf + std::min(size_t(faceCount), faces),
                  next.begin());
    }
    for (int32_t &r : next) {
        if (r >= regionCount) {
            r = -1;
        }
    }
    const bool resized = regionCount != context->regionCount;
    const bool changed = resized || next != context->regionOf;
    context->regionOf.swap(next);
    context->regionCount = regionCount;
    context->geometry.SetFaceRegions(context->regionOf.data(),
                                     context->regionOf.size());
    if (resized) {
        context->hoverColors.resize(size_t(regionCount), GfVec3f(1.0f));
        context->editColors.resize(size_t(regionCount), GfVec3f(1.0f));
        context->table = VtVec4fArray(_TableRows(*context), GfVec4f(0.0f));
    }
    if (changed) {
        _Publish(*context, true);
    }
    return 0;
}

int
RigExecTouchPose_Cast(long long handle, const double *o, const double *d,
                      double *tOut)
{
    const auto context = _Get(handle);
    if (!context || !o || !d) {
        return -1;
    }
    return context->geometry.Cast(GfVec3d(o[0], o[1], o[2]),
                                  GfVec3d(d[0], d[1], d[2]), tOut);
}

int
RigExecTouchPose_CastBruteForce(long long handle, const double *o,
                                const double *d, double *tOut)
{
    const auto context = _Get(handle);
    if (!context || !o || !d) {
        return -1;
    }
    return context->geometry.CastBruteForce(GfVec3d(o[0], o[1], o[2]),
                                            GfVec3d(d[0], d[1], d[2]), tOut);
}

int
RigExecTouchPose_RegionsInRect(
    long long handle, const double *viewProjection, double width,
    double height, const double *eye, double x0, double y0, double x1,
    double y1, int32_t *out, int capacity)
{
    const auto context = _Get(handle);
    if (!context || !viewProjection || !eye) {
        return -1;
    }
    const std::vector<int> regions = context->geometry.RegionsInRect(
        _Matrix(viewProjection), width, height,
        GfVec3d(eye[0], eye[1], eye[2]), x0, y0, x1, y1);
    const size_t n = std::min(regions.size(), size_t(std::max(capacity, 0)));
    for (size_t i = 0; out && i < n; ++i) {
        out[i] = regions[i];
    }
    return int(regions.size());
}

int
RigExecTouchPose_Brush(long long handle, const double *center,
                       const double *direction, double radius, int32_t *out,
                       int capacity)
{
    const auto context = _Get(handle);
    if (!context || !center) {
        return -1;
    }
    const GfVec3d dir = direction
        ? GfVec3d(direction[0], direction[1], direction[2]) : GfVec3d(0.0);
    const std::vector<int> faces = context->geometry.Brush(
        GfVec3d(center[0], center[1], center[2]), dir, radius);
    const size_t n = std::min(faces.size(), size_t(std::max(capacity, 0)));
    for (size_t i = 0; out && i < n; ++i) {
        out[i] = faces[i];
    }
    return int(faces.size());
}

int
RigExecTouchPose_FaceCentroid(long long handle, int face, double *out)
{
    const auto context = _Get(handle);
    if (!context || !out || face < 0 ||
        size_t(face) >= context->geometry.GetFaceCount() ||
        !context->geometry.HasPoints()) {
        return 0;
    }
    const GfVec3d c = context->geometry.FaceCentroid(face);
    out[0] = c[0];
    out[1] = c[1];
    out[2] = c[2];
    return 1;
}

int
RigExecTouchPose_SetRegionColors(long long handle, const float *hoverRgb,
                                 const float *editRgb, int regionCount)
{
    const auto context = _Get(handle);
    if (!context || regionCount < 0) {
        return -1;
    }
    context->hoverColors.assign(size_t(context->regionCount), GfVec3f(1.0f));
    context->editColors.assign(size_t(context->regionCount), GfVec3f(1.0f));
    const int n = std::min(regionCount, context->regionCount);
    for (int r = 0; r < n; ++r) {
        if (hoverRgb) {
            context->hoverColors[r] = GfVec3f(
                hoverRgb[3 * r], hoverRgb[3 * r + 1], hoverRgb[3 * r + 2]);
        }
        if (editRgb) {
            context->editColors[r] = GfVec3f(
                editRgb[3 * r], editRgb[3 * r + 1], editRgb[3 * r + 2]);
        }
    }
    return 0;
}

int
RigExecTouchPose_SetHighlightEnabled(long long handle, int enabled)
{
    const auto context = _Get(handle);
    if (!context || context->meshPath.IsEmpty()) {
        return -1;
    }
    const bool on = enabled != 0;
    if (on == context->highlightOn) {
        return 0;
    }
    context->highlightOn = on;
    if (on) {
        context->table = VtVec4fArray(_TableRows(*context), GfVec4f(0.0f));
        _Publish(*context, true);
    } else {
        RigExecTouchPoseHighlights::GetInstance().RemoveMesh(context->meshPath);
    }
    return 0;
}

int
RigExecTouchPose_SetHighlightState(
    long long handle, int hover, int lead, const int32_t *selected,
    int selectedCount, int editing, float opacity, const float *leadRgb,
    const float *selectedRgb)
{
    const auto context = _Get(handle);
    if (!context) {
        return -1;
    }
    const int regions = context->regionCount;
    const float alpha = std::min(std::max(opacity, 0.0f), 1.0f);
    VtVec4fArray table(_TableRows(*context), GfVec4f(0.0f));
    GfVec4f *rows = table.data();
    auto light = [&](int region, const GfVec3f &rgb) {
        if (region >= 0 && region < regions) {
            rows[region + 1] = GfVec4f(rgb[0], rgb[1], rgb[2], alpha);
        }
    };
    if (editing) {
        for (int r = 0; r < regions; ++r) {
            light(r, context->editColors[r]);
        }
    }
    if (selected && selectedRgb) {
        const GfVec3f rgb(selectedRgb[0], selectedRgb[1], selectedRgb[2]);
        for (int i = 0; i < selectedCount; ++i) {
            light(selected[i], rgb);
        }
    }
    if (leadRgb) {
        light(lead, GfVec3f(leadRgb[0], leadRgb[1], leadRgb[2]));
    }
    if (hover >= 0 && hover < regions) {
        light(hover, editing ? context->editColors[hover]
                             : context->hoverColors[hover]);
    }
    if (table == context->table) {
        return 0;
    }
    context->table = table;
    _Publish(*context, false);
    return 1;
}

int
RigExecTouchPose_GetHighlightTable(long long handle, float *out,
                                   int capacityRows)
{
    const auto context = _Get(handle);
    if (!context) {
        return -1;
    }
    const VtVec4fArray &table = context->table;
    const size_t n = std::min(table.size(), size_t(std::max(capacityRows, 0)));
    if (out && n) {
        std::memcpy(out, table.cdata()->data(), sizeof(float) * 4 * n);
    }
    return int(table.size());
}

int
RigExecTouchPose_GetSceneIndexCount()
{
    return int(RigExecTouchPoseHighlights::GetInstance().GetSceneIndexCount());
}

long long
RigExecTouchPose_GetTableUpdateCount()
{
    return static_cast<long long>(
        RigExecTouchPoseHighlights::GetInstance().GetTableUpdateCount());
}

int
RigExecTouchPose_GetShaderSource(char *out, int capacity)
{
    const std::string source =
        RigExecTouchPoseGetWrappedSource(TfToken("UsdPreviewSurface"));
    if (out && capacity > 0) {
        const size_t n = std::min(source.size(), size_t(capacity - 1));
        std::memcpy(out, source.data(), n);
        out[n] = '\0';
    }
    return int(source.size());
}

}  // extern "C"
