//
// TouchPose highlight: a Storm fragment-shader tint, driven by two primvars
// that never exist on the USD stage.
//
// THE MECHANISM, end to end:
//
//   rigExecTouchRegion   uniform float, one per face of the touched mesh:
//                        the face's region index + 1, 0 for unpainted skin.
//                        Published ONCE when TouchPose attaches, and again
//                        only when a paint stroke moves faces between
//                        regions.
//
//   rigExecTouchTable    constant vec4[N]: the colour and strength each
//                        region is lit with right now (hover, lead,
//                        selected, the paint-mode edit colours, or alpha 0
//                        for unlit). THIS is what a hover changes -- a
//                        few hundred floats, one constant-primvar upload.
//
//   the material         every material bound to the mesh (or to one of its
//                        GeomSubsets) has its surface terminal swapped for a
//                        generated glslfx node that runs the ORIGINAL
//                        terminal's shader unchanged and then mixes
//                        table[region] into the lit colour. Guarded by
//                        HD_HAS_rigExecTouchRegion, so any other mesh sharing
//                        the material compiles to exactly its old shader.
//
// Both primvars and the material edit are added by
// RigExecTouchPoseSceneIndex, a UsdImaging scene-index filter. Nothing is
// authored, no prim is created, deleted or toggled, and the rig never sees
// a change notice: a hover is a Hydra dirty on one constant primvar of one
// rprim.
//
// WHY A SHADER AND NOT A SECOND MESH. The overlay mesh it replaces had to be
// authored into the stage (a Hydra resync of a 17k-point rprim per region
// crossing, measured at 71-91 ms to the next drawn frame), lifted off the
// skin to avoid z-fighting, re-pointed on every pose change, and could be
// picked by usdview instead of the skin under it. A tint in the body's own
// shader has none of those properties: it is exactly on the surface, it
// deforms with the surface because it IS the surface, and it is invisible
// to picking.
//
#ifndef RIGEXEC_IMAGING_TOUCH_POSE_HIGHLIGHT_H
#define RIGEXEC_IMAGING_TOUCH_POSE_HIGHLIGHT_H

#include "pxr/pxr.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/imaging/hd/filteringSceneIndex.h"
#include "pxr/usd/sdf/path.h"

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The primvar and shader names, in one place.
struct RigExecTouchPoseTokens {
    static const TfToken &RegionPrimvar();   // rigExecTouchRegion
    static const TfToken &TablePrimvar();    // rigExecTouchTable
};

/// What one attached mesh is lit with. Immutable once published; a change
/// publishes a new one.
struct RigExecTouchPoseHighlightMesh {
    SdfPath path;
    VtFloatArray faceSlots;     ///< per face: region + 1, 0 for none
    VtVec4fArray table;         ///< [slot] = rgb + strength; [0] unlit
};
using RigExecTouchPoseHighlightMeshConstPtr =
    std::shared_ptr<const RigExecTouchPoseHighlightMesh>;

class RigExecTouchPoseSceneIndex;
using RigExecTouchPoseSceneIndexRefPtr = TfRefPtr<RigExecTouchPoseSceneIndex>;

/// Process-global rendezvous between the TouchPose C surface (which the
/// usdview plugin drives) and every TouchPose scene index (one per imaging
/// chain). Mirrors RigExecImagingRegistry's role for the rig itself.
class RigExecTouchPoseHighlights {
public:
    static RigExecTouchPoseHighlights &GetInstance();

    enum class Change { Attached, Detached, Slots, Table };

    /// Attaches \p path, or replaces its slots and table. A table whose SIZE
    /// changes is re-announced as an attach, because a constant array's
    /// length is part of the shader's layout.
    void SetMesh(const SdfPath &path, const VtFloatArray &faceSlots,
                 const VtVec4fArray &table);

    /// Replaces only the colours. Same size required; returns false (and
    /// changes nothing) otherwise. Identical values notify nobody.
    bool SetTable(const SdfPath &path, const VtVec4fArray &table);

    /// Replaces only the per-face slots (a paint stroke). Same size required.
    bool SetFaceSlots(const SdfPath &path, const VtFloatArray &faceSlots);

    void RemoveMesh(const SdfPath &path);

    RigExecTouchPoseHighlightMeshConstPtr Find(const SdfPath &path) const;

    /// Cheap test the scene index makes on every GetPrim.
    bool HasMeshes() const { return _count.load() != 0; }

    std::vector<SdfPath> GetMeshPaths() const;

    /// Called by each scene index at construction; held weakly.
    void RegisterSceneIndex(const RigExecTouchPoseSceneIndexRefPtr &index);

    /// Number of live scene indices -- a test can tell "no chain" from "no
    /// highlight".
    size_t GetSceneIndexCount();

    /// Counters for tests and the benchmark.
    uint64_t GetTableUpdateCount() const { return _tableUpdates.load(); }

private:
    RigExecTouchPoseHighlights() = default;

    void _Notify(const SdfPath &path, Change change);

    mutable std::mutex _mutex;
    std::map<SdfPath, RigExecTouchPoseHighlightMeshConstPtr> _meshes;
    std::atomic<size_t> _count{0};
    std::atomic<uint64_t> _tableUpdates{0};
    std::vector<TfWeakPtr<RigExecTouchPoseSceneIndex>> _indices;
};

/// The generated glslfx that wraps a base terminal shader. Empty identifier
/// when the base cannot be wrapped (no glslfx implementation on disk).
///
/// Built on first request and cached per base; call from the main thread
/// first (RigExecTouchPoseHighlights does, on attach) so Sdr parsing never
/// lands on a render thread.
TfToken RigExecTouchPoseGetWrappedTerminal(const TfToken &baseIdentifier);

/// The generated glslfx source for a base terminal, for tests and for
/// anyone debugging what Storm compiled. Empty when it cannot be wrapped.
std::string RigExecTouchPoseGetWrappedSource(const TfToken &baseIdentifier);

/// Adds the region/table primvars to attached meshes and wraps the surface
/// terminal of every material bound to them. See the file comment.
class RigExecTouchPoseSceneIndex final
    : public HdSingleInputFilteringSceneIndexBase {
public:
    static RigExecTouchPoseSceneIndexRefPtr New(
        const HdSceneIndexBaseRefPtr &inputSceneIndex);

    HdSceneIndexPrim GetPrim(const SdfPath &primPath) const override;
    SdfPathVector GetChildPrimPaths(const SdfPath &primPath) const override;

    /// From RigExecTouchPoseHighlights, on the thread that changed it.
    void HighlightChanged(const SdfPath &meshPath,
                          RigExecTouchPoseHighlights::Change change);

    /// The materials currently wrapped because of \p meshPath (tests).
    std::set<SdfPath> GetWrappedMaterials(const SdfPath &meshPath) const;

protected:
    void _PrimsAdded(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::AddedPrimEntries &entries) override;
    void _PrimsRemoved(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::RemovedPrimEntries &entries) override;
    void _PrimsDirtied(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::DirtiedPrimEntries &entries) override;

private:
    explicit RigExecTouchPoseSceneIndex(
        const HdSceneIndexBaseRefPtr &inputSceneIndex);

    /// The materials bound to \p meshPath and its direct GeomSubset
    /// children, read from the input scene.
    std::set<SdfPath> _ComputeBoundMaterials(const SdfPath &meshPath) const;

    /// Recomputes the wrapped set for one mesh (empty set = detach) and
    /// dirties every material that entered or left the union.
    void _SetMeshMaterials(const SdfPath &meshPath,
                           const std::set<SdfPath> &materials,
                           HdSceneIndexObserver::DirtiedPrimEntries *dirty);

    bool _IsWrapped(const SdfPath &materialPath) const;

    mutable std::mutex _mutex;
    std::map<SdfPath, std::set<SdfPath>> _materialsByMesh;
    std::map<SdfPath, int> _wrapCount;
};

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_TOUCH_POSE_HIGHLIGHT_H
