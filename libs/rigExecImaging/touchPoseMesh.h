//
// TouchPose geometry: the posed mesh a touch lands on, and the questions a
// pick asks of it.
//
// WHY THIS IS C++. The Python version cast every ray against all 52,548
// triangles of the biped's body with numpy -- 1.6 ms a ray measured in
// usdview, paid on every hover sample -- and rebuilt its derived arrays
// whenever the pose moved. Here the triangles live in a bounding volume
// hierarchy built ONCE per topology and REFIT per pose, so a pose change is
// one parallel pass over the triangle bounds plus a bottom-up sweep of the
// nodes, and a ray is a few dozen box tests and a handful of triangle tests.
//
// WHICH POINTS. The mesh is posed by RigExec, and the posed points never
// reach the stage: `points` on the stage is the rest mesh. The caller hands
// this class whatever the viewport is drawing (see touchPoseApi.cpp, which
// reads the RigExec snapshot the results scene index publishes from), plus
// the mesh's local-to-world transform, and every query below is answered
// against exactly that.
//
// Everything here is plain data with no Hydra and no USD stage in it, so a
// unit test can build a cube, pose it, and cast at it.
//
#ifndef RIGEXEC_IMAGING_TOUCH_POSE_MESH_H
#define RIGEXEC_IMAGING_TOUCH_POSE_MESH_H

#include "pxr/pxr.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"

#include <cstdint>
#include <mutex>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecTouchPoseMesh {
public:
    RigExecTouchPoseMesh() = default;

    /// Fan-triangulates the faces and remembers which face each triangle
    /// belongs to. Returns false (and holds no topology) when the arrays do
    /// not describe a valid polygon mesh over \p pointCount points.
    bool SetTopology(const VtIntArray &faceVertexCounts,
                     const VtIntArray &faceVertexIndices,
                     size_t pointCount);

    /// The points the viewport draws, in the mesh's LOCAL space. Shares the
    /// array's storage; nothing is copied. Returns false on a size mismatch.
    bool SetPoints(const VtVec3fArray &points);

    /// Local-to-world. Rays and every world-space answer go through it.
    void SetTransform(const GfMatrix4d &localToWorld);

    /// face -> region index, -1 for none. Shorter arrays leave the rest -1.
    void SetFaceRegions(const int32_t *regionOf, size_t count);

    size_t GetFaceCount() const { return _faceStarts.size(); }
    size_t GetTriangleCount() const { return _triFace.size(); }
    size_t GetPointCount() const { return _pointCount; }
    bool HasPoints() const { return _points.size() == _pointCount && _pointCount; }
    const VtVec3fArray &GetPoints() const { return _points; }
    const GfMatrix4d &GetTransform() const { return _xform; }
    int GetFaceRegion(int face) const {
        return (face >= 0 && size_t(face) < _faceRegion.size())
            ? _faceRegion[face] : -1;
    }

    /// Nearest hit along a WORLD ray, either side of the surface. Returns the
    /// face, or -1, and the hit distance along the normalized direction in
    /// \p t. Const and thread-safe once the acceleration is current.
    int Cast(const GfVec3d &origin, const GfVec3d &direction,
             double *t = nullptr) const;

    /// The same answer by testing every triangle. Kept for the unit test,
    /// which checks the hierarchy against it.
    int CastBruteForce(const GfVec3d &origin, const GfVec3d &direction,
                       double *t = nullptr) const;

    /// Regions with at least one FRONT-FACING face whose centroid projects
    /// inside the pixel rectangle. \p viewProjection is USD's row-vector
    /// world-to-clip matrix; pixels are top-left origin, y down, the same
    /// convention a Qt event uses. Sorted, unique.
    std::vector<int> RegionsInRect(const GfMatrix4d &viewProjection,
                                   double width, double height,
                                   const GfVec3d &eye,
                                   double x0, double y0,
                                   double x1, double y1) const;

    /// Faces whose world centroid lies within \p radius of \p center, and,
    /// when \p direction is non-zero, whose normal faces against it (so a
    /// stroke on the front of a limb does not paint its back). Sorted.
    std::vector<int> Brush(const GfVec3d &center, const GfVec3d &direction,
                           double radius) const;

    /// One face's centroid in world space.
    GfVec3d FaceCentroid(int face) const;

    /// The bounds of the posed mesh in world space. False when empty.
    bool GetWorldBounds(GfVec3d *lo, GfVec3d *hi) const;

    /// Makes the acceleration current for the last SetPoints. Called lazily
    /// by the queries; public so a caller can pay for it off the hover path.
    void Prepare() const;

    /// Diagnostics for tests and the benchmark: how often the hierarchy was
    /// built from scratch and how often it was only refit.
    size_t GetBuildCount() const { return _buildCount; }
    size_t GetRefitCount() const { return _refitCount; }

private:
    struct _Node {
        float lo[3];
        float hi[3];
        // Leaf: first index into _order, and count > 0.
        // Interior: index of the left child, count == 0; the right child is
        // always first + 1, and both are stored after their parent, which is
        // what lets a refit sweep the array backwards.
        uint32_t first;
        uint32_t count;
    };

    void _Build() const;
    void _Refit() const;
    void _ComputeTriangleBounds(std::vector<float> *bounds) const;
    void _ComputeCentroids() const;
    bool _HitTriangle(size_t tri, const GfVec3d &o, const GfVec3d &d,
                      double tMax, double *t) const;

    // topology
    std::vector<int> _faceStarts;
    std::vector<int> _faceCounts;
    VtIntArray _indices;
    std::vector<uint32_t> _tris;     // 3 point indices per triangle
    std::vector<int> _triFace;       // triangle -> face
    std::vector<int> _faceRegion;
    size_t _pointCount = 0;

    // pose
    VtVec3fArray _points;
    GfMatrix4d _xform{1.0};
    GfMatrix4d _inverse{1.0};
    bool _identity = true;
    uint64_t _pointsVersion = 0;

    // acceleration, rebuilt/refit lazily under _mutex
    mutable std::mutex _mutex;
    mutable std::vector<_Node> _nodes;
    mutable std::vector<uint32_t> _order;
    mutable uint64_t _bvhVersion = ~uint64_t(0);
    mutable bool _bvhBuilt = false;
    mutable double _builtCost = 0.0;
    mutable size_t _buildCount = 0;
    mutable size_t _refitCount = 0;

    // world centroids and face normals, per pose, lazily
    mutable std::vector<GfVec3f> _centroids;
    mutable std::vector<GfVec3f> _normals;
    mutable uint64_t _centroidVersion = ~uint64_t(0);
    mutable GfMatrix4d _centroidXform{1.0};
};

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_TOUCH_POSE_MESH_H
