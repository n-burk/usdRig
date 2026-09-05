//
// Cut-mesh construction and the cut-aware polygonal Laplacian.
// See cutMesh.h for the two documented departures from the paper's text.
//
#include "cutMesh.h"

#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/vec2d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>

namespace rigExec {

namespace {

constexpr double kEps = 1e-12;

GfVec3d _ToD(const GfVec3f &v) { return GfVec3d(v[0], v[1], v[2]); }

double _SafeNormalize(GfVec3d *v)
{
    const double n = v->GetLength();
    if (n <= kEps) {
        return 0.0;
    }
    *v /= n;
    return n;
}

/// Closest point to \p p on the segment [a, b], with its parameter.
GfVec3d _ClosestOnSegment(const GfVec3d &p, const GfVec3d &a, const GfVec3d &b,
                          double *parameter)
{
    const GfVec3d d = b - a;
    const double dd = GfDot(d, d);
    double t = (dd <= kEps) ? 0.0 : GfDot(p - a, d) / dd;
    t = GfClamp(t, 0.0, 1.0);
    *parameter = t;
    return a + d * t;
}

/// Closest point to \p p on triangle (a, b, c), with barycentric weights.
GfVec3d _ClosestOnTriangle(const GfVec3d &p, const GfVec3d &a, const GfVec3d &b,
                           const GfVec3d &c, double bary[3])
{
    const GfVec3d ab = b - a, ac = c - a, ap = p - a;
    const double d1 = GfDot(ab, ap), d2 = GfDot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        bary[0] = 1.0; bary[1] = 0.0; bary[2] = 0.0;
        return a;
    }
    const GfVec3d bp = p - b;
    const double d3 = GfDot(ab, bp), d4 = GfDot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        bary[0] = 0.0; bary[1] = 1.0; bary[2] = 0.0;
        return b;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = (d1 - d3 != 0.0) ? d1 / (d1 - d3) : 0.0;
        bary[0] = 1.0 - v; bary[1] = v; bary[2] = 0.0;
        return a + ab * v;
    }
    const GfVec3d cp = p - c;
    const double d5 = GfDot(ab, cp), d6 = GfDot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        bary[0] = 0.0; bary[1] = 0.0; bary[2] = 1.0;
        return c;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = (d2 - d6 != 0.0) ? d2 / (d2 - d6) : 0.0;
        bary[0] = 1.0 - w; bary[1] = 0.0; bary[2] = w;
        return a + ac * w;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double denom = (d4 - d3) + (d5 - d6);
        const double w = (denom != 0.0) ? (d4 - d3) / denom : 0.0;
        bary[0] = 0.0; bary[1] = 1.0 - w; bary[2] = w;
        return b + (c - b) * w;
    }
    const double denom = va + vb + vc;
    const double v = (denom != 0.0) ? vb / denom : 0.0;
    const double w = (denom != 0.0) ? vc / denom : 0.0;
    bary[0] = 1.0 - v - w; bary[1] = v; bary[2] = w;
    return a + ab * v + ac * w;
}

/// Triangulated view of a polygonal mesh, used only for closest-point
/// queries; results are always reported against the ORIGINAL face.
struct _MeshView {
    std::vector<GfVec3d> points;
    std::vector<int> faceBegin;      ///< faces+1, into faceCorner
    std::vector<int> faceCorner;     ///< vertex index per corner
    std::vector<GfVec3d> faceNormal;
    std::vector<GfVec3d> faceCentroid;
    // triangles (fan) for queries
    std::vector<int> triangleFace;
    std::vector<int> triangleVertex;  ///< 3 per triangle
    // edge adjacency: sorted vertex pair -> up to two (face, cornerInFace)
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> edgeFaces;
    double bboxDiagonal = 0.0;
    double meanEdge = 0.0;

    size_t GetFaceCount() const { return faceBegin.size() - 1; }
    int GetFaceSize(int f) const { return faceBegin[f + 1] - faceBegin[f]; }
    int GetFaceVertex(int f, int corner) const {
        return faceCorner[faceBegin[f] + corner];
    }
};

_MeshView _BuildMeshView(const std::vector<GfVec3f> &meshPoints,
                         const std::vector<int> &counts,
                         const std::vector<int> &indices)
{
    _MeshView view;
    view.points.reserve(meshPoints.size());
    for (const GfVec3f &p : meshPoints) {
        view.points.push_back(_ToD(p));
    }
    view.faceBegin.push_back(0);
    int cursor = 0;
    double edgeSum = 0.0;
    size_t edgeCount = 0;
    for (size_t f = 0; f < counts.size(); ++f) {
        const int n = counts[f];
        for (int i = 0; i < n; ++i) {
            view.faceCorner.push_back(indices[cursor + i]);
        }
        view.faceBegin.push_back(int(view.faceCorner.size()));

        // Newell normal and centroid.
        GfVec3d normal(0.0), centroid(0.0);
        for (int i = 0; i < n; ++i) {
            const GfVec3d &a = view.points[indices[cursor + i]];
            const GfVec3d &b = view.points[indices[cursor + (i + 1) % n]];
            normal[0] += (a[1] - b[1]) * (a[2] + b[2]);
            normal[1] += (a[2] - b[2]) * (a[0] + b[0]);
            normal[2] += (a[0] - b[0]) * (a[1] + b[1]);
            centroid += a;
            edgeSum += (b - a).GetLength();
            ++edgeCount;
            const int v0 = indices[cursor + i];
            const int v1 = indices[cursor + (i + 1) % n];
            view.edgeFaces[{std::min(v0, v1), std::max(v0, v1)}].push_back(
                {int(f), i});
        }
        _SafeNormalize(&normal);
        view.faceNormal.push_back(normal);
        view.faceCentroid.push_back(n > 0 ? centroid / double(n) : centroid);

        for (int i = 1; i + 1 < n; ++i) {
            view.triangleFace.push_back(int(f));
            view.triangleVertex.push_back(indices[cursor]);
            view.triangleVertex.push_back(indices[cursor + i]);
            view.triangleVertex.push_back(indices[cursor + i + 1]);
        }
        cursor += n;
    }
    view.meanEdge = (edgeCount > 0) ? edgeSum / double(edgeCount) : 0.0;

    if (!view.points.empty()) {
        GfVec3d lo = view.points[0], hi = view.points[0];
        for (const GfVec3d &p : view.points) {
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
        }
        view.bboxDiagonal = (hi - lo).GetLength();
    }
    return view;
}

/// Uniform grid over triangle bounds. Brute force is O(faces) per sample and
/// the paper's own examples run 34k samples against 22k faces, so a grid is
/// not an optimization here so much as the difference between binding in a
/// second and binding in ten minutes.
struct _TriangleGrid {
    GfVec3d origin{0, 0, 0};
    double cell = 1.0;
    int dim[3] = {1, 1, 1};
    std::vector<int> bucketStart;
    std::vector<int> bucketItem;

    int Index(int x, int y, int z) const {
        return (z * dim[1] + y) * dim[0] + x;
    }
    void Clamp(int c[3]) const {
        for (int k = 0; k < 3; ++k) {
            c[k] = std::max(0, std::min(dim[k] - 1, c[k]));
        }
    }
    void Locate(const GfVec3d &p, int c[3]) const {
        for (int k = 0; k < 3; ++k) {
            c[k] = int(std::floor((p[k] - origin[k]) / cell));
        }
        Clamp(c);
    }
};

_TriangleGrid _BuildTriangleGrid(const _MeshView &view)
{
    _TriangleGrid grid;
    const size_t triangles = view.triangleFace.size();
    if (triangles == 0 || view.points.empty()) {
        grid.bucketStart.assign(2, 0);
        return grid;
    }
    GfVec3d lo = view.points[0], hi = view.points[0];
    for (const GfVec3d &p : view.points) {
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], p[k]);
            hi[k] = std::max(hi[k], p[k]);
        }
    }
    const GfVec3d span = hi - lo;
    const double volume = std::max(span[0], kEps) * std::max(span[1], kEps) *
                          std::max(span[2], kEps);
    // Aim at roughly four triangles per cell.
    grid.cell = std::max(std::cbrt(volume * 4.0 / double(triangles)),
                         view.meanEdge * 0.5);
    if (!(grid.cell > 0.0)) {
        grid.cell = 1.0;
    }
    grid.origin = lo - GfVec3d(grid.cell, grid.cell, grid.cell);
    for (int k = 0; k < 3; ++k) {
        grid.dim[k] = std::max(
            1, std::min(256, int(std::ceil(span[k] / grid.cell)) + 3));
    }

    const int cells = grid.dim[0] * grid.dim[1] * grid.dim[2];
    std::vector<int> counts(cells + 1, 0);
    auto forEachCell = [&](size_t t, const std::function<void(int)> &fn) {
        GfVec3d tlo = view.points[view.triangleVertex[3 * t]];
        GfVec3d thi = tlo;
        for (int i = 1; i < 3; ++i) {
            const GfVec3d &p = view.points[view.triangleVertex[3 * t + i]];
            for (int k = 0; k < 3; ++k) {
                tlo[k] = std::min(tlo[k], p[k]);
                thi[k] = std::max(thi[k], p[k]);
            }
        }
        int a[3], b[3];
        grid.Locate(tlo, a);
        grid.Locate(thi, b);
        for (int z = a[2]; z <= b[2]; ++z) {
            for (int y = a[1]; y <= b[1]; ++y) {
                for (int x = a[0]; x <= b[0]; ++x) {
                    fn(grid.Index(x, y, z));
                }
            }
        }
    };
    for (size_t t = 0; t < triangles; ++t) {
        forEachCell(t, [&](int c) { ++counts[c + 1]; });
    }
    for (int i = 0; i < cells; ++i) {
        counts[i + 1] += counts[i];
    }
    grid.bucketStart = counts;
    grid.bucketItem.assign(counts[cells], 0);
    std::vector<int> cursor(counts.begin(), counts.end() - 1);
    for (size_t t = 0; t < triangles; ++t) {
        forEachCell(t, [&](int c) { grid.bucketItem[cursor[c]++] = int(t); });
    }
    return grid;
}

struct _Projection {
    int face = -1;
    int triangle = -1;
    double weights[3] = {0,0,0};
    GfVec3d position{0, 0, 0};
    double distance = 0.0;
};

_Projection _ClosestPoint(const _MeshView &view, const _TriangleGrid &grid,
                          const GfVec3d &query)
{
    _Projection best;
    best.distance = std::numeric_limits<double>::max();
    auto consider = [&](int t) {
        double bary[3];
        const GfVec3d p = _ClosestOnTriangle(
            query, view.points[view.triangleVertex[3 * t]],
            view.points[view.triangleVertex[3 * t + 1]],
            view.points[view.triangleVertex[3 * t + 2]], bary);
        const double d = (p - query).GetLength();
        if (d < best.distance) {
            best.distance = d;
            best.position = p;
            best.face = view.triangleFace[t];
            best.triangle = t;
            for (int i = 0; i < 3; ++i) best.weights[i] = bary[i];
        }
    };

    if (grid.bucketItem.empty()) {
        for (size_t t = 0; t < view.triangleFace.size(); ++t) {
            consider(int(t));
        }
        return best;
    }
    int centre[3];
    grid.Locate(query, centre);
    // Grow the ring until a hit is found and the ring's inner radius exceeds
    // the best distance, which is what makes the grid search exact.
    const int maxRing =
        std::max({grid.dim[0], grid.dim[1], grid.dim[2]});
    for (int ring = 0; ring <= maxRing; ++ring) {
        for (int z = centre[2] - ring; z <= centre[2] + ring; ++z) {
            if (z < 0 || z >= grid.dim[2]) continue;
            for (int y = centre[1] - ring; y <= centre[1] + ring; ++y) {
                if (y < 0 || y >= grid.dim[1]) continue;
                for (int x = centre[0] - ring; x <= centre[0] + ring; ++x) {
                    if (x < 0 || x >= grid.dim[0]) continue;
                    const bool onShell =
                        (std::abs(x - centre[0]) == ring ||
                         std::abs(y - centre[1]) == ring ||
                         std::abs(z - centre[2]) == ring);
                    if (ring > 0 && !onShell) continue;
                    const int c = grid.Index(x, y, z);
                    for (int p = grid.bucketStart[c];
                         p < grid.bucketStart[c + 1]; ++p) {
                        consider(grid.bucketItem[p]);
                    }
                }
            }
        }
        if (best.face >= 0 && best.distance <= double(ring) * grid.cell) {
            break;
        }
    }
    if (best.face < 0) {
        for (size_t t = 0; t < view.triangleFace.size(); ++t) {
            consider(int(t));
        }
    }
    return best;
}

/// A local 2D frame for one input face, used for angular sorting and for
/// segment/edge intersection. The paper computes tangent spaces per
/// cut-vertex for the same purpose; per face is equivalent for a per-face
/// arrangement and cannot disagree between the two faces of an edge, because
/// the shared points are computed in 3D and only sorted locally.
struct _FaceFrame {
    GfVec3d origin{0, 0, 0};
    GfVec3d axisX{1, 0, 0};
    GfVec3d axisY{0, 1, 0};
    GfVec3d normal{0, 0, 1};

    GfVec2d To2D(const GfVec3d &p) const {
        const GfVec3d d = p - origin;
        return GfVec2d(GfDot(d, axisX), GfDot(d, axisY));
    }
};

_FaceFrame _MakeFaceFrame(const _MeshView &view, int face)
{
    _FaceFrame frame;
    frame.origin = view.faceCentroid[face];
    frame.normal = view.faceNormal[face];
    if (frame.normal.GetLength() <= kEps) {
        frame.normal = GfVec3d(0, 0, 1);
    }
    GfVec3d x = view.points[view.GetFaceVertex(face, 0)] - frame.origin;
    x -= frame.normal * GfDot(x, frame.normal);
    if (_SafeNormalize(&x) == 0.0) {
        x = (std::abs(frame.normal[0]) < 0.9) ? GfVec3d(1, 0, 0)
                                              : GfVec3d(0, 1, 0);
        x -= frame.normal * GfDot(x, frame.normal);
        _SafeNormalize(&x);
    }
    frame.axisX = x;
    frame.axisY = GfCross(frame.normal, x);
    return frame;
}

/// Generalized coordinates of a point over one face's corners.
///
/// Fan-triangulate, find the triangle whose plane the point is closest to,
/// and scatter that triangle's barycentrics onto the face's corners. Exact on
/// each fan triangle, a partition of unity everywhere, and -- the property
/// §5's warping needs -- identical for a point on a shared edge whichever
/// face evaluates it, because an edge point lands on the two edge vertices
/// only.
std::vector<double> _FaceWeights(const _MeshView &view, int face,
                                 const GfVec3d &point)
{
    const int n = view.GetFaceSize(face);
    std::vector<double> weights(n, 0.0);
    if (n <= 0) {
        return weights;
    }
    if (n == 1) {
        weights[0] = 1.0;
        return weights;
    }
    double bestDistance = std::numeric_limits<double>::max();
    int bestCorner = 1;
    double bestBary[3] = {1.0, 0.0, 0.0};
    for (int i = 1; i + 1 < n; ++i) {
        double bary[3];
        const GfVec3d p = _ClosestOnTriangle(
            point, view.points[view.GetFaceVertex(face, 0)],
            view.points[view.GetFaceVertex(face, i)],
            view.points[view.GetFaceVertex(face, i + 1)], bary);
        const double d = (p - point).GetLength();
        if (d < bestDistance) {
            bestDistance = d;
            bestCorner = i;
            bestBary[0] = bary[0];
            bestBary[1] = bary[1];
            bestBary[2] = bary[2];
        }
    }
    weights[0] += bestBary[0];
    weights[bestCorner] += bestBary[1];
    weights[bestCorner + 1] += bestBary[2];
    return weights;
}

/// Where a projected sample landed on its face.
enum class _SampleKind { Vertex, Edge, Face };

struct _NodeDraft {
    GfVec3d position{0, 0, 0};
    int face = -1;
    std::vector<double> weights;
    int meshVertex = -1;
    int sample = -1;
};

/// One piece of a curvenet segment lying inside a single input face.
struct _SubEdge {
    int face = -1;
    int node0 = -1;
    int node1 = -1;
    int sampleA = -1;
    int sampleB = -1;
    double t0 = 0.0;
    double t1 = 1.0;
};

/// Union-find over vertex unknowns, used to find components no curve reaches.
struct _DisjointSet {
    std::vector<int> parent;
    explicit _DisjointSet(int n) : parent(n) {
        for (int i = 0; i < n; ++i) {
            parent[i] = i;
        }
    }
    int Find(int a) {
        while (parent[a] != a) {
            parent[a] = parent[parent[a]];
            a = parent[a];
        }
        return a;
    }
    void Union(int a, int b) {
        a = Find(a);
        b = Find(b);
        if (a != b) {
            parent[b] = a;
        }
    }
};

}  // namespace

struct RigExecMeshSurfaceQuery::_Impl {
    _MeshView view;
    _TriangleGrid grid;
};

RigExecMeshSurfaceQuery::RigExecMeshSurfaceQuery(
    const std::vector<GfVec3f> &meshPoints,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices)
    : _impl(new _Impl())
{
    _impl->view =
        _BuildMeshView(meshPoints, faceVertexCounts, faceVertexIndices);
    _impl->grid = _BuildTriangleGrid(_impl->view);
}

RigExecMeshSurfaceQuery::~RigExecMeshSurfaceQuery() = default;

bool RigExecMeshSurfaceQuery::Project(const GfVec3d &point,
    std::vector<std::pair<int,double>> *weights) const
{
    if (!weights) return false;
    weights->clear();
    const _Projection hit = _ClosestPoint(_impl->view, _impl->grid, point);
    if (hit.triangle < 0) return false;
    for (int i = 0; i < 3; ++i)
        if (hit.weights[i] != 0.0)
            weights->emplace_back(_impl->view.triangleVertex[3*hit.triangle+i], hit.weights[i]);
    return true;
}

GfVec3d RigExecMeshSurfaceQuery::Normal(const GfVec3d &point) const
{
    const _Projection hit = _ClosestPoint(_impl->view, _impl->grid, point);
    if (hit.face < 0) {
        return GfVec3d(0.0);
    }
    return _impl->view.faceNormal[hit.face];
}

double RigExecMeshMeanEdgeLength(const std::vector<GfVec3f> &meshPoints,
                                 const std::vector<int> &faceVertexCounts,
                                 const std::vector<int> &faceVertexIndices)
{
    double sum = 0.0;
    size_t count = 0;
    int cursor = 0;
    for (int n : faceVertexCounts) {
        for (int i = 0; i < n; ++i) {
            const int a = faceVertexIndices[cursor + i];
            const int b = faceVertexIndices[cursor + (i + 1) % n];
            if (a < 0 || b < 0 || size_t(a) >= meshPoints.size() ||
                size_t(b) >= meshPoints.size()) {
                continue;
            }
            sum += (_ToD(meshPoints[b]) - _ToD(meshPoints[a])).GetLength();
            ++count;
        }
        cursor += n;
    }
    return (count > 0) ? sum / double(count) : 0.0;
}

GfVec3d RigExecCutMesh::EvaluateBinding(
    const RigExecMeshPoint &point, const std::vector<GfVec3f> &meshPoints,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices) const
{
    (void)faceVertexCounts;
    GfVec3d out(0.0);
    if (point.face < 0 || point.cornerCount <= 0) {
        return out;
    }
    for (int i = 0; i < point.cornerCount; ++i) {
        const int vertex = faceVertexIndices[point.indexOffset + i];
        const double w = bindingWeights[point.cornerBegin + i];
        out += _ToD(meshPoints[vertex]) * w;
    }
    return out;
}

// ---------------------------------------------------------------------------
// §4.1: mesh cutting
// ---------------------------------------------------------------------------

bool RigExecBuildCutMesh(const RigExecCurvenetTopology &topology,
                         const RigExecCurvenetSampling &sampling,
                         const std::vector<GfVec3f> &meshPoints,
                         const std::vector<int> &faceVertexCounts,
                         const std::vector<int> &faceVertexIndices,
                         RigExecCutMesh *cutMesh,
                         RigExecCutMeshReport *report, std::string *error)
{
    auto fail = [&](const std::string &message) {
        if (error) {
            *error = message;
        }
        return false;
    };
    if (meshPoints.empty() || faceVertexCounts.empty()) {
        return fail("the curvenet target has no geometry to cut");
    }
    {
        long total = 0;
        for (int n : faceVertexCounts) {
            if (n < 3) {
                return fail("the curvenet target has a face with " +
                            std::to_string(n) +
                            " vertices; the cut needs polygons");
            }
            total += n;
        }
        if (total != long(faceVertexIndices.size())) {
            return fail("faceVertexCounts sums to " + std::to_string(total) +
                        " but faceVertexIndices holds " +
                        std::to_string(faceVertexIndices.size()));
        }
        for (int v : faceVertexIndices) {
            if (v < 0 || size_t(v) >= meshPoints.size()) {
                return fail("faceVertexIndices names vertex " +
                            std::to_string(v) + " outside the point array");
            }
        }
    }

    *cutMesh = RigExecCutMesh();
    *report = RigExecCutMeshReport();

    const _MeshView view =
        _BuildMeshView(meshPoints, faceVertexCounts, faceVertexIndices);
    const _TriangleGrid grid = _BuildTriangleGrid(view);
    report->meanEdgeLength = view.meanEdge;
    report->sampleCount = int(sampling.GetSampleCount());
    // §4.1: "a numerical tolerance of 0.001% of the diagonal length of the
    // surface bounding box".
    const double tolerance = std::max(view.bboxDiagonal * 1e-5, 1e-9);

    // ---- node table ----------------------------------------------------
    std::vector<_NodeDraft> nodes;
    std::vector<int> nodeOfVertex(meshPoints.size(), -1);
    // Nodes on a mesh edge, keyed by the edge and a quantized parameter.
    std::map<std::pair<int, int>, std::vector<std::pair<double, int>>> edgeNodes;

    // Every mesh vertex is a cut-vertex.
    {
        std::vector<int> anyFace(meshPoints.size(), -1);
        std::vector<int> anyCorner(meshPoints.size(), -1);
        for (size_t f = 0; f < view.GetFaceCount(); ++f) {
            const int n = view.GetFaceSize(int(f));
            for (int i = 0; i < n; ++i) {
                const int v = view.GetFaceVertex(int(f), i);
                if (anyFace[v] < 0) {
                    anyFace[v] = int(f);
                    anyCorner[v] = i;
                }
            }
        }
        for (size_t v = 0; v < meshPoints.size(); ++v) {
            if (anyFace[v] < 0) {
                continue;  // unreferenced point
            }
            _NodeDraft draft;
            draft.position = view.points[v];
            draft.face = anyFace[v];
            draft.weights.assign(view.GetFaceSize(anyFace[v]), 0.0);
            draft.weights[anyCorner[v]] = 1.0;
            draft.meshVertex = int(v);
            nodeOfVertex[v] = int(nodes.size());
            nodes.push_back(std::move(draft));
        }
    }

    auto edgeKey = [](int a, int b) {
        return std::make_pair(std::min(a, b), std::max(a, b));
    };
    // Creates (or reuses) a node on mesh edge (v0, v1) at parameter u
    // measured from v0. Shared between the edge's two faces by construction.
    auto edgeNode = [&](int v0, int v1, double u, const GfVec3d &position,
                        int sample) {
        const auto key = edgeKey(v0, v1);
        const double canonical = (v0 == key.first) ? u : 1.0 - u;
        auto &bucket = edgeNodes[key];
        const double snap =
            (view.points[key.second] - view.points[key.first]).GetLength();
        const double parameterTolerance =
            (snap > kEps) ? tolerance / snap : 1e-9;

        // A crossing that lands on an endpoint IS that mesh vertex.
        //
        // Creating a separate node there instead leaves two coincident
        // cut-vertices, and the arrangement's angular sort around them is
        // then meaningless -- which shows up as sliver faces, cracks that
        // should not exist, and input faces that produce no cut-face at all.
        // A curve traced along an edge loop crosses exactly at vertices
        // constantly, so this is the common path, not an edge case.
        if (canonical <= parameterTolerance && nodeOfVertex[key.first] >= 0) {
            const int node = nodeOfVertex[key.first];
            if (sample >= 0 && nodes[node].sample < 0) {
                nodes[node].sample = sample;
            }
            return node;
        }
        if (canonical >= 1.0 - parameterTolerance &&
            nodeOfVertex[key.second] >= 0) {
            const int node = nodeOfVertex[key.second];
            if (sample >= 0 && nodes[node].sample < 0) {
                nodes[node].sample = sample;
            }
            return node;
        }

        for (auto &entry : bucket) {
            if (std::abs(entry.first - canonical) <= parameterTolerance) {
                if (sample >= 0 && nodes[entry.second].sample < 0) {
                    nodes[entry.second].sample = sample;
                }
                return entry.second;
            }
        }
        _NodeDraft draft;
        draft.position = position;
        // Bind against either adjacent face; the weights land on the two edge
        // vertices only, so both faces agree.
        const auto it = view.edgeFaces.find(key);
        const int face = (it != view.edgeFaces.end() && !it->second.empty())
                             ? it->second.front().first
                             : 0;
        draft.face = face;
        draft.weights.assign(view.GetFaceSize(face), 0.0);
        for (int i = 0; i < view.GetFaceSize(face); ++i) {
            const int v = view.GetFaceVertex(face, i);
            if (v == key.first) {
                draft.weights[i] += 1.0 - canonical;
            } else if (v == key.second) {
                draft.weights[i] += canonical;
            }
        }
        draft.sample = sample;
        const int index = int(nodes.size());
        nodes.push_back(std::move(draft));
        bucket.push_back({canonical, index});
        return index;
    };

    // Faces incident to each vertex, so a vertex-sample can be recognised as
    // sharing a face (or an edge) with its neighbour.
    std::vector<std::vector<int>> facesOfVertex(meshPoints.size());
    for (size_t f = 0; f < view.GetFaceCount(); ++f) {
        const int n = view.GetFaceSize(int(f));
        for (int i = 0; i < n; ++i) {
            facesOfVertex[view.GetFaceVertex(int(f), i)].push_back(int(f));
        }
    }

    // ---- project samples ----------------------------------------------
    const size_t sampleCount = sampling.GetSampleCount();
    std::vector<int> nodeOfSample(sampleCount, -1);
    std::vector<int> faceOfSample(sampleCount, -1);
    // Where each sample sits, which decides how its segments are cut in.
    struct _Site {
        _SampleKind kind = _SampleKind::Face;
        int vertex = -1;
        int edgeA = -1, edgeB = -1;
        double edgeU = 0.0;
        std::vector<int> faces;
    };
    std::vector<_Site> sites(sampleCount);
    std::vector<GfVec3d> projectedSample(sampleCount, GfVec3d(0.0));
    cutMesh->sampleBinding.assign(sampleCount, RigExecMeshPoint());
    cutMesh->sampleResidual.assign(sampleCount, GfVec3d(0.0));
    std::vector<std::vector<double>> sampleWeights(sampleCount);

    for (size_t s = 0; s < sampleCount; ++s) {
        const GfVec3d query = sampling.positions[s];
        const _Projection hit = _ClosestPoint(view, grid, query);
        if (hit.face < 0) {
            return fail("a curvenet sample could not be projected onto the "
                        "target surface");
        }
        faceOfSample[s] = hit.face;
        projectedSample[s] = hit.position;
        cutMesh->sampleResidual[s] = query - hit.position;
        report->maxResidual =
            std::max(report->maxResidual, hit.distance);
        sampleWeights[s] = _FaceWeights(view, hit.face, hit.position);

        // Classify against the ORIGINAL face, not the fan triangle whose
        // interior diagonals are not mesh edges.
        const int n = view.GetFaceSize(hit.face);
        _SampleKind kind = _SampleKind::Face;
        int vertexHit = -1;
        int edgeCorner = -1;
        double edgeParameter = 0.0;
        for (int i = 0; i < n; ++i) {
            const int v = view.GetFaceVertex(hit.face, i);
            if ((view.points[v] - hit.position).GetLength() <= tolerance) {
                kind = _SampleKind::Vertex;
                vertexHit = v;
                break;
            }
        }
        if (kind == _SampleKind::Face) {
            for (int i = 0; i < n; ++i) {
                const int a = view.GetFaceVertex(hit.face, i);
                const int b = view.GetFaceVertex(hit.face, (i + 1) % n);
                double u = 0.0;
                const GfVec3d p = _ClosestOnSegment(
                    hit.position, view.points[a], view.points[b], &u);
                if ((p - hit.position).GetLength() <= tolerance) {
                    kind = _SampleKind::Edge;
                    edgeCorner = i;
                    edgeParameter = u;
                    break;
                }
            }
        }

        sites[s].kind = kind;
        if (kind == _SampleKind::Vertex) {
            nodeOfSample[s] = nodeOfVertex[vertexHit];
            if (nodeOfSample[s] >= 0) {
                nodes[nodeOfSample[s]].sample = int(s);
            }
            sites[s].vertex = vertexHit;
            sites[s].faces = facesOfVertex[vertexHit];
        } else if (kind == _SampleKind::Edge) {
            const int a = view.GetFaceVertex(hit.face, edgeCorner);
            const int b = view.GetFaceVertex(hit.face, (edgeCorner + 1) % n);
            nodeOfSample[s] =
                edgeNode(a, b, edgeParameter, hit.position, int(s));
            sites[s].edgeA = a;
            sites[s].edgeB = b;
            sites[s].edgeU = edgeParameter;
            const auto it = view.edgeFaces.find(edgeKey(a, b));
            if (it != view.edgeFaces.end()) {
                for (const auto &entry : it->second) {
                    sites[s].faces.push_back(entry.first);
                }
            }
        } else {
            sites[s].faces.push_back(hit.face);
            _NodeDraft draft;
            draft.position = hit.position;
            draft.face = hit.face;
            draft.weights = sampleWeights[s];
            draft.sample = int(s);
            nodeOfSample[s] = int(nodes.size());
            nodes.push_back(std::move(draft));
        }
    }

    // ---- trace segments -------------------------------------------------
    std::vector<_SubEdge> subEdges;
    // One hop of a segment that runs along the mesh's own edges. Kept as
    // explicit parameters rather than derived from the two samples' sites,
    // because a segment can span more than one mesh edge and each hop then
    // covers only part of it.
    struct _AlongEdge {
        int sampleA;
        int sampleB;
        std::pair<int, int> edge;
        double uFrom = 0.0;   ///< canonical edge parameter at the hop's start
        double uTo = 1.0;
        double tFrom = 0.0;   ///< parameter along the SEGMENT
        double tTo = 1.0;
    };
    std::vector<_AlongEdge> alongEdge;

    std::vector<std::vector<int>> edgesOfVertexA(meshPoints.size());
    for (const auto &entry : view.edgeFaces) {
        edgesOfVertexA[entry.first.first].push_back(entry.first.second);
        edgesOfVertexA[entry.first.second].push_back(entry.first.first);
    }

    /// Canonical parameter of a site along mesh edge \p key, or -1.
    auto siteParameter = [&](const _Site &site,
                             const std::pair<int, int> &key) {
        if (site.kind == _SampleKind::Vertex) {
            if (site.vertex == key.first) return 0.0;
            if (site.vertex == key.second) return 1.0;
            return -1.0;
        }
        if (site.kind == _SampleKind::Edge &&
            edgeKey(site.edgeA, site.edgeB) == key) {
            return (site.edgeA == key.first) ? site.edgeU : 1.0 - site.edgeU;
        }
        return -1.0;
    };

    /// Mesh edges a site lies on.
    auto siteEdges = [&](const _Site &site) {
        std::vector<std::pair<int, int>> out;
        if (site.kind == _SampleKind::Edge) {
            out.push_back(edgeKey(site.edgeA, site.edgeB));
        } else if (site.kind == _SampleKind::Vertex) {
            for (int other : edgesOfVertexA[site.vertex]) {
                out.push_back(edgeKey(site.vertex, other));
            }
        }
        return out;
    };

    /// The mesh edge both sites lie on, or (-1, -1).
    auto sharedMeshEdge = [&](const _Site &a,
                              const _Site &b) -> std::pair<int, int> {
        const auto none = std::make_pair(-1, -1);
        auto isEdge = [&](int v0, int v1) {
            return view.edgeFaces.count(edgeKey(v0, v1)) > 0;
        };
        if (a.kind == _SampleKind::Edge && b.kind == _SampleKind::Edge) {
            if (edgeKey(a.edgeA, a.edgeB) == edgeKey(b.edgeA, b.edgeB)) {
                return edgeKey(a.edgeA, a.edgeB);
            }
            return none;
        }
        if (a.kind == _SampleKind::Vertex && b.kind == _SampleKind::Edge) {
            if (a.vertex == b.edgeA || a.vertex == b.edgeB) {
                return edgeKey(b.edgeA, b.edgeB);
            }
            return none;
        }
        if (a.kind == _SampleKind::Edge && b.kind == _SampleKind::Vertex) {
            if (b.vertex == a.edgeA || b.vertex == a.edgeB) {
                return edgeKey(a.edgeA, a.edgeB);
            }
            return none;
        }
        if (a.kind == _SampleKind::Vertex && b.kind == _SampleKind::Vertex) {
            if (a.vertex != b.vertex && isEdge(a.vertex, b.vertex)) {
                return edgeKey(a.vertex, b.vertex);
            }
            return none;
        }
        return none;
    };

    auto adjacentFace = [&](int v0, int v1, int from) {
        const auto it = view.edgeFaces.find(edgeKey(v0, v1));
        if (it == view.edgeFaces.end()) {
            return -1;
        }
        for (const auto &entry : it->second) {
            if (entry.first != from) {
                return entry.first;
            }
        }
        return -1;
    };

    for (size_t c = 0; c < topology.curves.size(); ++c) {
        const int begin = sampling.curveBegin[c];
        const int count = sampling.GetCurveSampleCount(c);
        if (count <= 1) {
            continue;
        }
        const bool closed = topology.curves[c].closed;
        const int segments = closed ? count : count - 1;
        for (int i = 0; i < segments; ++i) {
            const int sa = begin + i;
            const int sb = begin + ((i + 1) % count);
            const int fa = faceOfSample[sa];
            const int fb = faceOfSample[sb];
            if (fa < 0 || fb < 0) {
                continue;
            }
            // §4.1's first case: the segment runs along the mesh's own
            // edges. Both adjacent faces must carry it, because the edge IS
            // the two sides of the curve there -- this is what an artist
            // gets whenever they trace an edge loop, so it is the common
            // case, not a corner case. Routing it as an interior chord
            // instead would lay an edge on top of the face boundary, which
            // makes the angular sort at the shared vertex ambiguous and
            // loses the face.
            const std::pair<int, int> shared =
                sharedMeshEdge(sites[sa], sites[sb]);
            if (shared.first >= 0) {
                _AlongEdge hop;
                hop.sampleA = sa;
                hop.sampleB = sb;
                hop.edge = shared;
                hop.uFrom = siteParameter(sites[sa], shared);
                hop.uTo = siteParameter(sites[sb], shared);
                alongEdge.push_back(hop);
                continue;
            }

            // The same thing spanning TWO collinear edges: the segment passes
            // through a mesh vertex. Split it there and both halves are the
            // case above.
            {
                const GfVec3d from = projectedSample[sa];
                const GfVec3d to = projectedSample[sb];
                GfVec3d direction = to - from;
                const double total = _SafeNormalize(&direction);
                bool routed = false;
                if (total > kEps) {
                    const auto edgesA = siteEdges(sites[sa]);
                    const auto edgesB = siteEdges(sites[sb]);
                    for (const auto &ea : edgesA) {
                        if (routed) break;
                        for (const auto &eb : edgesB) {
                            if (ea == eb) continue;
                            // The vertex the two edges share.
                            int via = -1;
                            for (int candidate : {ea.first, ea.second}) {
                                if (candidate == eb.first ||
                                    candidate == eb.second) {
                                    via = candidate;
                                    break;
                                }
                            }
                            if (via < 0) continue;
                            const GfVec3d pivot = view.points[via];
                            const double along = GfDot(pivot - from, direction);
                            if (along < -tolerance ||
                                along > total + tolerance) {
                                continue;
                            }
                            if ((pivot - (from + direction * along))
                                    .GetLength() > tolerance) {
                                continue;
                            }
                            const double t = GfClamp(along / total, 0.0, 1.0);
                            _AlongEdge first;
                            first.sampleA = sa;
                            first.sampleB = sb;
                            first.edge = ea;
                            first.uFrom = siteParameter(sites[sa], ea);
                            first.uTo = (via == ea.first) ? 0.0 : 1.0;
                            first.tFrom = 0.0;
                            first.tTo = t;
                            _AlongEdge second;
                            second.sampleA = sa;
                            second.sampleB = sb;
                            second.edge = eb;
                            second.uFrom = (via == eb.first) ? 0.0 : 1.0;
                            second.uTo = siteParameter(sites[sb], eb);
                            second.tFrom = t;
                            second.tTo = 1.0;
                            if (first.uFrom >= 0.0 && second.uTo >= 0.0) {
                                alongEdge.push_back(first);
                                alongEdge.push_back(second);
                                routed = true;
                                break;
                            }
                        }
                    }
                }
                if (routed) {
                    continue;
                }
            }

            // Otherwise, a face both samples touch takes the whole segment.
            int commonFace = -1;
            for (int candidate : sites[sa].faces) {
                if (std::find(sites[sb].faces.begin(), sites[sb].faces.end(),
                              candidate) != sites[sb].faces.end()) {
                    commonFace = candidate;
                    break;
                }
            }
            if (commonFace >= 0) {
                _SubEdge sub;
                sub.face = commonFace;
                sub.node0 = nodeOfSample[sa];
                sub.node1 = nodeOfSample[sb];
                sub.sampleA = sa;
                sub.sampleB = sb;
                sub.t0 = 0.0;
                sub.t1 = 1.0;
                if (sub.node0 != sub.node1) {
                    subEdges.push_back(sub);
                }
                continue;
            }

            ++report->tracedSegments;
            const GfVec3d start = projectedSample[sa];
            const GfVec3d target = projectedSample[sb];
            const double total = (target - start).GetLength();
            std::vector<_SubEdge> pending;
            int face = fa;
            int node = nodeOfSample[sa];
            GfVec3d position = start;
            double parameter = 0.0;
            bool arrived = false;
            for (int step = 0; step < 64; ++step) {
                if (face == fb) {
                    _SubEdge sub;
                    sub.face = face;
                    sub.node0 = node;
                    sub.node1 = nodeOfSample[sb];
                    sub.sampleA = sa;
                    sub.sampleB = sb;
                    sub.t0 = parameter;
                    sub.t1 = 1.0;
                    if (sub.node0 != sub.node1) {
                        pending.push_back(sub);
                    }
                    arrived = true;
                    break;
                }
                const _FaceFrame frame = _MakeFaceFrame(view, face);
                const GfVec2d from = frame.To2D(position);
                GfVec2d to = frame.To2D(target);
                GfVec2d direction = to - from;
                const double directionLength = direction.GetLength();
                if (directionLength <= kEps) {
                    break;
                }
                direction /= directionLength;

                const int n = view.GetFaceSize(face);
                double bestT = std::numeric_limits<double>::max();
                int bestCorner = -1;
                double bestU = 0.0;
                for (int e = 0; e < n; ++e) {
                    const int va = view.GetFaceVertex(face, e);
                    const int vb = view.GetFaceVertex(face, (e + 1) % n);
                    const GfVec2d a = frame.To2D(view.points[va]);
                    const GfVec2d b = frame.To2D(view.points[vb]);
                    const GfVec2d edge = b - a;
                    const double denominator =
                        direction[0] * edge[1] - direction[1] * edge[0];
                    if (std::abs(denominator) <= 1e-14) {
                        continue;
                    }
                    const GfVec2d delta = a - from;
                    const double t =
                        (delta[0] * edge[1] - delta[1] * edge[0]) / denominator;
                    const double u = (delta[0] * direction[1] -
                                      delta[1] * direction[0]) /
                                     denominator;
                    if (t > 1e-9 && u >= -1e-9 && u <= 1.0 + 1e-9 &&
                        t < bestT) {
                        bestT = t;
                        bestCorner = e;
                        bestU = GfClamp(u, 0.0, 1.0);
                    }
                }
                if (bestCorner < 0) {
                    break;
                }
                const int va = view.GetFaceVertex(face, bestCorner);
                const int vb = view.GetFaceVertex(face, (bestCorner + 1) % n);
                const GfVec3d crossing =
                    view.points[va] * (1.0 - bestU) + view.points[vb] * bestU;
                const int next = adjacentFace(va, vb, face);
                if (next < 0) {
                    break;  // ran off an open boundary
                }
                const int crossingNode =
                    edgeNode(va, vb, bestU, crossing, -1);
                const double nextParameter =
                    (total > kEps) ? GfClamp((crossing - start).GetLength() /
                                                 total,
                                             parameter, 1.0)
                                   : parameter;
                _SubEdge sub;
                sub.face = face;
                sub.node0 = node;
                sub.node1 = crossingNode;
                sub.sampleA = sa;
                sub.sampleB = sb;
                sub.t0 = parameter;
                sub.t1 = nextParameter;
                if (sub.node0 != sub.node1) {
                    pending.push_back(sub);
                }
                face = next;
                node = crossingNode;
                position = crossing;
                parameter = nextParameter;
            }
            if (arrived) {
                subEdges.insert(subEdges.end(), pending.begin(), pending.end());
            } else {
                // The walk could not route this segment. Dropping the cut is
                // the graceful failure: the samples still constrain their own
                // nodes, so the surface follows the curve there, it just does
                // not gain a discontinuity along this stretch.
                ++report->failedTraces;
            }
        }
    }
    // Segments running along a mesh edge, resolved once every node that could
    // land on that edge exists. Each is split at the nodes it passes and
    // handed to BOTH adjacent faces, so the edge's two halfedges become the
    // curve's two sides.
    for (const _AlongEdge &entry : alongEdge) {
        const std::pair<int, int> key = entry.edge;
        const double uA = entry.uFrom;
        const double uB = entry.uTo;
        if (uA < 0.0 || uB < 0.0 || std::abs(uB - uA) <= 1e-15) {
            continue;
        }
        const double lo = std::min(uA, uB);
        const double hi = std::max(uA, uB);

        std::vector<std::pair<double, int>> chain;
        if (nodeOfVertex[key.first] >= 0) {
            chain.push_back({0.0, nodeOfVertex[key.first]});
        }
        if (nodeOfVertex[key.second] >= 0) {
            chain.push_back({1.0, nodeOfVertex[key.second]});
        }
        const auto found = edgeNodes.find(key);
        if (found != edgeNodes.end()) {
            for (const auto &node : found->second) {
                chain.push_back(node);
            }
        }
        std::vector<std::pair<double, int>> inside;
        for (const auto &node : chain) {
            if (node.first >= lo - 1e-12 && node.first <= hi + 1e-12) {
                inside.push_back(node);
            }
        }
        std::sort(inside.begin(), inside.end(),
                  [](const std::pair<double, int> &a,
                     const std::pair<double, int> &b) {
                      return a.first < b.first;
                  });
        inside.erase(std::unique(inside.begin(), inside.end(),
                                 [](const std::pair<double, int> &a,
                                    const std::pair<double, int> &b) {
                                     return a.second == b.second;
                                 }),
                     inside.end());
        if (uB < uA) {
            std::reverse(inside.begin(), inside.end());
        }

        const auto faces = view.edgeFaces.find(key);
        if (faces == view.edgeFaces.end()) {
            continue;
        }
        const double tSpan = entry.tTo - entry.tFrom;
        for (size_t i = 0; i + 1 < inside.size(); ++i) {
            const double t0 =
                entry.tFrom + tSpan * (inside[i].first - uA) / (uB - uA);
            const double t1 =
                entry.tFrom + tSpan * (inside[i + 1].first - uA) / (uB - uA);
            for (const auto &incident : faces->second) {
                _SubEdge sub;
                sub.face = incident.first;
                sub.node0 = inside[i].second;
                sub.node1 = inside[i + 1].second;
                sub.sampleA = entry.sampleA;
                sub.sampleB = entry.sampleB;
                sub.t0 = GfClamp(t0, 0.0, 1.0);
                sub.t1 = GfClamp(t1, 0.0, 1.0);
                if (sub.node0 != sub.node1) {
                    subEdges.push_back(sub);
                }
            }
        }
    }

    if (report->failedTraces > 0) {
        char buffer[224];
        std::snprintf(buffer, sizeof(buffer),
                      "%d curvenet segment(s) could not be traced across the "
                      "surface and were not cut; the curve may leave the mesh "
                      "or cross an open boundary",
                      report->failedTraces);
        report->warnings.push_back(buffer);
    }

    // ---- per-face arrangement -------------------------------------------
    std::vector<std::vector<int>> subEdgesOfFace(view.GetFaceCount());
    for (size_t i = 0; i < subEdges.size(); ++i) {
        subEdgesOfFace[subEdges[i].face].push_back(int(i));
    }
    // Nodes lying on each mesh edge, for building boundary chains.
    // (already accumulated in edgeNodes)

    cutMesh->faceBegin.push_back(0);
    int crackCount = 0;

    for (size_t f = 0; f < view.GetFaceCount(); ++f) {
        const int n = view.GetFaceSize(int(f));
        const _FaceFrame frame = _MakeFaceFrame(view, int(f));
        const size_t facesBefore = cutMesh->faceSource.size();

        std::unordered_map<int, int> localOf;
        std::vector<int> globalOf;
        auto local = [&](int global) {
            auto it = localOf.find(global);
            if (it != localOf.end()) {
                return it->second;
            }
            const int index = int(globalOf.size());
            localOf[global] = index;
            globalOf.push_back(global);
            return index;
        };

        struct _LocalEdge {
            int a = -1;
            int b = -1;
            bool hasCurve = false;
            RigExecCutCurveRef curve;
        };
        std::vector<_LocalEdge> edges;
        std::map<std::pair<int, int>, int> edgeOf;
        auto addEdge = [&](int a, int b, const RigExecCutCurveRef *curve) {
            if (a == b) {
                return;
            }
            const auto key = std::make_pair(std::min(a, b), std::max(a, b));
            auto it = edgeOf.find(key);
            if (it != edgeOf.end()) {
                if (curve && !edges[it->second].hasCurve) {
                    // A curvenet segment running exactly along a mesh edge
                    // (§4.1's first case): annotate the existing edge rather
                    // than laying a duplicate on top of it.
                    _LocalEdge &existing = edges[it->second];
                    existing.hasCurve = true;
                    existing.curve = *curve;
                    if (existing.a != a) {
                        std::swap(existing.curve.tA, existing.curve.tB);
                    }
                }
                return;
            }
            _LocalEdge edge;
            edge.a = a;
            edge.b = b;
            if (curve) {
                edge.hasCurve = true;
                edge.curve = *curve;
            }
            edgeOf[key] = int(edges.size());
            edges.push_back(edge);
        };

        // Boundary chains: each mesh edge of this face, subdivided by the
        // nodes sitting on it.
        for (int i = 0; i < n; ++i) {
            const int va = view.GetFaceVertex(int(f), i);
            const int vb = view.GetFaceVertex(int(f), (i + 1) % n);
            const auto key = edgeKey(va, vb);
            std::vector<std::pair<double, int>> chain;
            chain.push_back({0.0, nodeOfVertex[va]});
            chain.push_back({1.0, nodeOfVertex[vb]});
            const auto it = edgeNodes.find(key);
            if (it != edgeNodes.end()) {
                for (const auto &entry : it->second) {
                    // entry.first is measured from key.first.
                    const double u =
                        (va == key.first) ? entry.first : 1.0 - entry.first;
                    chain.push_back({u, entry.second});
                }
            }
            std::sort(chain.begin(), chain.end(),
                      [](const std::pair<double, int> &a,
                         const std::pair<double, int> &b) {
                          return a.first < b.first;
                      });
            for (size_t k = 0; k + 1 < chain.size(); ++k) {
                if (chain[k].second < 0 || chain[k + 1].second < 0) {
                    continue;
                }
                addEdge(local(chain[k].second), local(chain[k + 1].second),
                        nullptr);
            }
        }

        // Interior cuts.
        for (int index : subEdgesOfFace[f]) {
            const _SubEdge &sub = subEdges[index];
            RigExecCutCurveRef ref;
            ref.sampleA = sub.sampleA;
            ref.sampleB = sub.sampleB;
            ref.tA = sub.t0;
            ref.tB = sub.t1;
            addEdge(local(sub.node0), local(sub.node1), &ref);
        }

        if (edges.empty()) {
            ++report->lostFaces;
            continue;
        }

        // Drop curvenet islands: components not reaching this face's boundary
        // (§4.1's last paragraph -- finer than the surface can represent).
        {
            const int localCount = int(globalOf.size());
            _DisjointSet sets(localCount);
            for (const _LocalEdge &edge : edges) {
                sets.Union(edge.a, edge.b);
            }
            std::set<int> boundaryRoots;
            for (int i = 0; i < n; ++i) {
                const int node = nodeOfVertex[view.GetFaceVertex(int(f), i)];
                if (node >= 0 && localOf.count(node)) {
                    boundaryRoots.insert(sets.Find(localOf[node]));
                }
            }
            std::vector<_LocalEdge> kept;
            kept.reserve(edges.size());
            for (const _LocalEdge &edge : edges) {
                if (boundaryRoots.count(sets.Find(edge.a))) {
                    kept.push_back(edge);
                }
            }
            if (kept.size() != edges.size()) {
                edges.swap(kept);
                edgeOf.clear();
                for (size_t i = 0; i < edges.size(); ++i) {
                    edgeOf[{std::min(edges[i].a, edges[i].b),
                            std::max(edges[i].a, edges[i].b)}] = int(i);
                }
            }
        }

        // Halfedges, sorted counter-clockwise around each node.
        const int halfCount = int(edges.size()) * 2;
        std::vector<int> origin(halfCount), destination(halfCount);
        for (size_t e = 0; e < edges.size(); ++e) {
            origin[2 * e] = edges[e].a;
            destination[2 * e] = edges[e].b;
            origin[2 * e + 1] = edges[e].b;
            destination[2 * e + 1] = edges[e].a;
        }
        std::vector<GfVec2d> planar(globalOf.size());
        for (size_t i = 0; i < globalOf.size(); ++i) {
            planar[i] = frame.To2D(nodes[globalOf[i]].position);
        }
        std::vector<std::vector<int>> outgoing(globalOf.size());
        for (int h = 0; h < halfCount; ++h) {
            outgoing[origin[h]].push_back(h);
        }
        std::vector<int> slotOf(halfCount, 0);
        for (size_t v = 0; v < outgoing.size(); ++v) {
            std::vector<int> &fan = outgoing[v];
            std::sort(fan.begin(), fan.end(), [&](int a, int b) {
                const GfVec2d da = planar[destination[a]] - planar[v];
                const GfVec2d db = planar[destination[b]] - planar[v];
                return std::atan2(da[1], da[0]) < std::atan2(db[1], db[0]);
            });
            for (size_t i = 0; i < fan.size(); ++i) {
                slotOf[fan[i]] = int(i);
            }
        }

        // next(h): the halfedge after twin(h) going CLOCKWISE around its
        // origin. That is the standard left-face traversal, and it walks a
        // dangling edge out and back inside one face -- which is exactly the
        // crack representation Appendix A assumes.
        auto next = [&](int h) {
            const int twin = h ^ 1;
            const int v = origin[twin];
            const std::vector<int> &fan = outgoing[v];
            const int slot = slotOf[twin];
            return fan[(slot + fan.size() - 1) % fan.size()];
        };

        std::vector<char> visited(halfCount, 0);
        for (int start = 0; start < halfCount; ++start) {
            if (visited[start]) {
                continue;
            }
            std::vector<int> loop;
            int h = start;
            bool overrun = false;
            for (int guard = 0; guard <= halfCount + 1; ++guard) {
                visited[h] = 1;
                loop.push_back(h);
                h = next(h);
                if (h == start) {
                    break;
                }
                if (guard == halfCount) {
                    overrun = true;
                }
            }
            if (overrun || loop.size() < 3) {
                continue;
            }
            // Signed area in the face plane: the outer face of the
            // arrangement is the negative one.
            double area = 0.0;
            for (size_t i = 0; i < loop.size(); ++i) {
                const GfVec2d &a = planar[origin[loop[i]]];
                const GfVec2d &b = planar[destination[loop[i]]];
                area += a[0] * b[1] - b[0] * a[1];
            }
            if (area <= 0.0) {
                continue;
            }

            for (size_t i = 0; i < loop.size(); ++i) {
                const int corner = loop[i];
                const int previous = loop[(i + loop.size() - 1) % loop.size()];
                const int node = globalOf[origin[corner]];
                cutMesh->cornerNode.push_back(node);

                // The corner reads its ORIGIN cut-vertex's value. Which side
                // of the curve, when that origin is on the curvenet, is what
                // the annotated halfedge tells us: this cut-face lies to the
                // LEFT of every halfedge in its loop.
                const _LocalEdge &ownEdge = edges[corner / 2];
                const _LocalEdge &previousEdge = edges[previous / 2];
                const bool ownForward = (corner % 2) == 0;
                const bool previousForward = (previous % 2) == 0;

                int constraintBegin = int(cutMesh->cornerConstraintIndex.size());
                cutMesh->cornerConstraintBegin.push_back(constraintBegin);

                auto emit = [&](const RigExecCutCurveRef &ref, double t,
                                bool leftSide, double scale) {
                    const int sideOffset = leftSide ? 0 : 1;
                    if (ref.sampleA >= 0) {
                        cutMesh->cornerConstraintIndex.push_back(
                            2 * ref.sampleA + sideOffset);
                        cutMesh->cornerConstraintWeight.push_back(
                            (1.0 - t) * scale);
                    }
                    if (ref.sampleB >= 0) {
                        cutMesh->cornerConstraintIndex.push_back(
                            2 * ref.sampleB + sideOffset);
                        cutMesh->cornerConstraintWeight.push_back(t * scale);
                    }
                };

                if (ownEdge.hasCurve) {
                    const double t = ownForward ? ownEdge.curve.tA
                                                : ownEdge.curve.tB;
                    emit(ownEdge.curve, t, ownForward, 1.0);
                } else if (previousEdge.hasCurve) {
                    const double t = previousForward ? previousEdge.curve.tB
                                                     : previousEdge.curve.tA;
                    emit(previousEdge.curve, t, previousForward, 1.0);
                } else if (nodes[node].sample >= 0) {
                    // A curvenet cut-vertex this cut-face touches without
                    // sharing an annotated halfedge -- the face lies beyond a
                    // curve's end. Neither side is the right answer, so take
                    // both equally, which keeps C a partition of unity.
                    RigExecCutCurveRef ref;
                    ref.sampleA = nodes[node].sample;
                    ref.sampleB = -1;
                    emit(ref, 0.0, true, 0.5);
                    emit(ref, 0.0, false, 0.5);
                }
            }
            cutMesh->faceBegin.push_back(int(cutMesh->cornerNode.size()));
            cutMesh->faceSource.push_back(int(f));
            for (int h : loop) {
                if (std::find(loop.begin(), loop.end(), h ^ 1) != loop.end()) {
                    ++crackCount;
                }
            }
        }
        if (cutMesh->faceSource.size() == facesBefore) {
            ++report->lostFaces;
        }
    }
    if (report->lostFaces > 0) {
        char buffer[224];
        std::snprintf(buffer, sizeof(buffer),
                      "%d input face(s) produced no cut-face; the surface "
                      "loses its Laplacian support there",
                      report->lostFaces);
        report->warnings.push_back(buffer);
    }
    // A curve is ALLOWED to float off the surface -- §3 draws them near it,
    // not on it, and §4.3 transports the offset. But a curve floating many
    // times the surface's own resolution away is almost always a misplaced
    // net rather than an intentional one, and the failure is silent: the
    // samples all project to nearly the same place, the cut degenerates, and
    // the solve returns a bad deformation instead of an error. Say so.
    if (view.meanEdge > 0.0 && report->maxResidual > 5.0 * view.meanEdge) {
        char buffer[256];
        std::snprintf(
            buffer, sizeof(buffer),
            "the curvenet floats up to %.4g from the surface, %.1fx the mean "
            "edge length %.4g -- profiles are meant to lie near the surface, "
            "and a net this far off cuts degenerately",
            report->maxResidual, report->maxResidual / view.meanEdge,
            view.meanEdge);
        report->warnings.push_back(buffer);
    }
    cutMesh->cornerConstraintBegin.push_back(
        int(cutMesh->cornerConstraintIndex.size()));
    report->crackCount = crackCount / 2;
    report->cutFaceCount = int(cutMesh->GetCutFaceCount());
    if (cutMesh->GetCutFaceCount() == 0) {
        return fail("cutting the target by this curvenet produced no faces");
    }

    // ---- nodes into the cut-mesh ----------------------------------------
    cutMesh->nodePosition.resize(nodes.size());
    cutMesh->nodeBinding.resize(nodes.size());
    cutMesh->nodeMeshVertex.resize(nodes.size());
    cutMesh->nodeSample.resize(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        cutMesh->nodePosition[i] = nodes[i].position;
        cutMesh->nodeMeshVertex[i] = nodes[i].meshVertex;
        cutMesh->nodeSample[i] = nodes[i].sample;
        RigExecMeshPoint binding;
        binding.face = nodes[i].face;
        binding.indexOffset =
            (nodes[i].face >= 0) ? view.faceBegin[nodes[i].face] : 0;
        binding.cornerBegin = int(cutMesh->bindingWeights.size());
        binding.cornerCount = int(nodes[i].weights.size());
        for (double w : nodes[i].weights) {
            cutMesh->bindingWeights.push_back(float(w));
        }
        cutMesh->nodeBinding[i] = binding;
    }
    for (size_t s = 0; s < sampleCount; ++s) {
        RigExecMeshPoint binding;
        binding.face = faceOfSample[s];
        binding.indexOffset =
            (faceOfSample[s] >= 0) ? view.faceBegin[faceOfSample[s]] : 0;
        binding.cornerBegin = int(cutMesh->bindingWeights.size());
        binding.cornerCount = int(sampleWeights[s].size());
        for (double w : sampleWeights[s]) {
            cutMesh->bindingWeights.push_back(float(w));
        }
        cutMesh->sampleBinding[s] = binding;
    }
    cutMesh->constraintCount = int(2 * sampleCount);

    // ---- unknown numbering ----------------------------------------------
    // §4.2 precedence: a cut-vertex that is both a mesh vertex and a curvenet
    // sample belongs to the curvenet, so it carries no unknown.
    cutMesh->vertexUnknown.assign(meshPoints.size(), -1);
    cutMesh->vertexConstraintNode.assign(meshPoints.size(), -1);
    std::vector<char> candidate(meshPoints.size(), 0);
    // A mesh vertex is an unknown when at least one of its CORNERS is
    // unconstrained -- decided from the corner assignment rather than from
    // whether a sample happened to land on it.
    //
    // Those are not the same test. A curve running along an edge loop passes
    // through vertices no sample lands on, and every corner there still reads
    // a curvenet constraint; giving such a vertex an unknown produces a row
    // of the system with nothing in it, and the factorization fails on a zero
    // pivot. §4.2's precedence rule is about corners, so this is too.
    for (size_t c = 0; c < cutMesh->cornerNode.size(); ++c) {
        if (cutMesh->cornerConstraintBegin[c] !=
            cutMesh->cornerConstraintBegin[c + 1]) {
            continue;
        }
        const int vertex = cutMesh->nodeMeshVertex[cutMesh->cornerNode[c]];
        if (vertex >= 0) {
            candidate[vertex] = 1;
        }
    }
    for (size_t i = 0; i < nodes.size(); ++i) {
        const int vertex = nodes[i].meshVertex;
        if (vertex >= 0 && nodes[i].sample >= 0) {
            cutMesh->vertexConstraintNode[vertex] = int(i);
        }
    }

    // Components the curvenet never constrains leave the system singular.
    // Find them explicitly rather than regularizing them into a made-up
    // answer: those vertices stay at rest and are reported.
    {
        _DisjointSet sets(int(meshPoints.size()));
        std::vector<char> constrained(meshPoints.size(), 0);
        for (size_t f = 0; f < cutMesh->GetCutFaceCount(); ++f) {
            const int begin = cutMesh->faceBegin[f];
            const int end = cutMesh->faceBegin[f + 1];
            int first = -1;
            bool touchesCurve = false;
            for (int c = begin; c < end; ++c) {
                if (cutMesh->cornerConstraintBegin[c] !=
                    cutMesh->cornerConstraintBegin[c + 1]) {
                    touchesCurve = true;
                }
                const int vertex = cutMesh->nodeMeshVertex[cutMesh->cornerNode[c]];
                if (vertex < 0 || !candidate[vertex]) {
                    continue;
                }
                if (first < 0) {
                    first = vertex;
                } else {
                    sets.Union(first, vertex);
                }
            }
            if (touchesCurve && first >= 0) {
                constrained[sets.Find(first)] = 1;
            }
        }
        // Propagate the flag to every root after all unions.
        std::vector<char> rootConstrained(meshPoints.size(), 0);
        for (size_t v = 0; v < meshPoints.size(); ++v) {
            if (constrained[v]) {
                rootConstrained[sets.Find(int(v))] = 1;
            }
        }
        for (size_t v = 0; v < meshPoints.size(); ++v) {
            if (candidate[v] && !rootConstrained[sets.Find(int(v))]) {
                candidate[v] = 0;
                cutMesh->unreachedVertices.push_back(int(v));
            }
        }
    }

    for (size_t v = 0; v < meshPoints.size(); ++v) {
        if (candidate[v]) {
            cutMesh->vertexUnknown[v] = int(cutMesh->unknownVertex.size());
            cutMesh->unknownVertex.push_back(int(v));
        }
    }
    cutMesh->unknownCount = int(cutMesh->unknownVertex.size());
    if (!cutMesh->unreachedVertices.empty()) {
        char buffer[224];
        std::snprintf(buffer, sizeof(buffer),
                      "%zu vertices lie in a mesh component this curvenet "
                      "does not reach; they are held at rest",
                      cutMesh->unreachedVertices.size());
        report->warnings.push_back(buffer);
    }

    // Per-corner unknown column.
    cutMesh->cornerVertexUnknown.assign(cutMesh->cornerNode.size(), -1);
    for (size_t c = 0; c < cutMesh->cornerNode.size(); ++c) {
        if (cutMesh->cornerConstraintBegin[c] !=
            cutMesh->cornerConstraintBegin[c + 1]) {
            continue;  // curvenet takes precedence
        }
        const int vertex = cutMesh->nodeMeshVertex[cutMesh->cornerNode[c]];
        if (vertex >= 0) {
            cutMesh->cornerVertexUnknown[c] = cutMesh->vertexUnknown[vertex];
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Appendix A: the polygonal Laplacian
// ---------------------------------------------------------------------------

void RigExecPolygonLaplacian(const std::vector<GfVec3d> &corners,
                             std::vector<double> *out)
{
    const int n = int(corners.size());
    out->assign(size_t(n) * size_t(n), 0.0);
    if (n < 3) {
        return;
    }

    // [a_f] = E^T A X, which is exactly skew because the X (x) X terms
    // telescope over the loop; its axial vector is the polygon's vector area.
    GfVec3d area(0.0);
    for (int i = 0; i < n; ++i) {
        area += GfCross(corners[i], corners[(i + 1) % n]);
    }
    area *= 0.5;
    const double magnitude = area.GetLength();
    if (magnitude <= kEps) {
        // A degenerate cut-face (zero area) contributes nothing; leaving it
        // at zero keeps the assembly PSD instead of injecting an infinity.
        return;
    }
    const GfVec3d normal = area / magnitude;

    // G = (-1/a) [n] E^T A, a 3 x n matrix.
    // (E^T A)_{k,j} = sum_i E_{i,k} A_{i,j}, with E_i = x_{i+1} - x_i and
    // A_{i,i} = A_{i,i+1} = 1/2, so column j receives half of edge j and half
    // of edge j-1.
    std::vector<GfVec3d> eta(n, GfVec3d(0.0));
    for (int i = 0; i < n; ++i) {
        const GfVec3d edge = corners[(i + 1) % n] - corners[i];
        eta[i] += edge * 0.5;
        eta[(i + 1) % n] += edge * 0.5;
    }
    std::vector<GfVec3d> gradient(n, GfVec3d(0.0));
    for (int j = 0; j < n; ++j) {
        gradient[j] = -GfCross(normal, eta[j]) / magnitude;
    }

    // Q = D - E G, an n x n matrix. Row i of D is -1 at i and +1 at i+1;
    // row i of E G is edge_i (dot) gradient_j.
    std::vector<double> q(size_t(n) * size_t(n), 0.0);
    for (int i = 0; i < n; ++i) {
        const GfVec3d edge = corners[(i + 1) % n] - corners[i];
        for (int j = 0; j < n; ++j) {
            double value = -GfDot(edge, gradient[j]);
            if (j == i) {
                value += -1.0;
            }
            if (j == (i + 1) % n) {
                value += 1.0;
            }
            q[size_t(i) * n + j] = value;
        }
    }

    // L = a G^T G + lambda Q^T Q, lambda = 1.
    const double lambda = 1.0;
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            double value = magnitude * GfDot(gradient[r], gradient[c]);
            double qq = 0.0;
            for (int i = 0; i < n; ++i) {
                qq += q[size_t(i) * n + r] * q[size_t(i) * n + c];
            }
            (*out)[size_t(r) * n + c] = value + lambda * qq;
        }
    }
}

// ---------------------------------------------------------------------------
// System assembly
// ---------------------------------------------------------------------------

void RigExecAssembleCutSystem(const RigExecCutMesh &cutMesh,
                              const std::vector<GfVec3d> &cornerPositions,
                              RigExecSparseBuilder *matrix,
                              std::vector<double> *faceLaplacians,
                              std::vector<int> *faceLaplacianBegin)
{
    faceLaplacians->clear();
    faceLaplacianBegin->assign(1, 0);
    std::vector<GfVec3d> polygon;
    std::vector<double> local;
    for (size_t f = 0; f < cutMesh.GetCutFaceCount(); ++f) {
        const int begin = cutMesh.faceBegin[f];
        const int n = cutMesh.faceBegin[f + 1] - begin;
        polygon.clear();
        for (int i = 0; i < n; ++i) {
            polygon.push_back(cornerPositions[begin + i]);
        }
        RigExecPolygonLaplacian(polygon, &local);
        faceLaplacians->insert(faceLaplacians->end(), local.begin(),
                               local.end());
        faceLaplacianBegin->push_back(int(faceLaplacians->size()));

        for (int a = 0; a < n; ++a) {
            const int rowVertex = cutMesh.cornerVertexUnknown[begin + a];
            if (rowVertex < 0) {
                continue;
            }
            for (int b = 0; b < n; ++b) {
                const int columnVertex =
                    cutMesh.cornerVertexUnknown[begin + b];
                if (columnVertex < 0) {
                    continue;
                }
                const double value = local[size_t(a) * n + b];
                if (value != 0.0) {
                    matrix->Add(rowVertex, columnVertex, value);
                }
            }
        }
    }
}

void RigExecAssembleCutRhs(const RigExecCutMesh &cutMesh,
                           const std::vector<double> &faceLaplacians,
                           const std::vector<int> &faceLaplacianBegin,
                           const std::vector<double> &constraintValues,
                           const std::vector<double> &cornerOffsets,
                           int columns, std::vector<double> *rhs)
{
    const int unknowns = cutMesh.unknownCount;
    rhs->assign(size_t(unknowns) * size_t(std::max(columns, 0)), 0.0);
    if (columns <= 0) {
        return;
    }
    const size_t cornerCount = cutMesh.cornerNode.size();
    const bool haveOffsets = !cornerOffsets.empty();

    // known[corner][column] = (C phi_c)[corner] - y_h[corner]
    std::vector<double> known(cornerCount * size_t(columns), 0.0);
    for (size_t c = 0; c < cornerCount; ++c) {
        for (int col = 0; col < columns; ++col) {
            double value = 0.0;
            for (int p = cutMesh.cornerConstraintBegin[c];
                 p < cutMesh.cornerConstraintBegin[c + 1]; ++p) {
                const int index = cutMesh.cornerConstraintIndex[p];
                const double weight = cutMesh.cornerConstraintWeight[p];
                value += weight *
                         constraintValues[size_t(col) *
                                              size_t(cutMesh.constraintCount) +
                                          size_t(index)];
            }
            if (haveOffsets) {
                value -= cornerOffsets[size_t(col) * cornerCount + c];
            }
            known[c * size_t(columns) + size_t(col)] = value;
        }
    }

    for (size_t f = 0; f < cutMesh.GetCutFaceCount(); ++f) {
        const int begin = cutMesh.faceBegin[f];
        const int n = cutMesh.faceBegin[f + 1] - begin;
        const double *local = faceLaplacians.data() + faceLaplacianBegin[f];
        for (int a = 0; a < n; ++a) {
            const int row = cutMesh.cornerVertexUnknown[begin + a];
            if (row < 0) {
                continue;
            }
            for (int b = 0; b < n; ++b) {
                const double value = local[size_t(a) * n + b];
                if (value == 0.0) {
                    continue;
                }
                const size_t corner = size_t(begin + b);
                for (int col = 0; col < columns; ++col) {
                    (*rhs)[size_t(col) * size_t(unknowns) + size_t(row)] -=
                        value * known[corner * size_t(columns) + size_t(col)];
                }
            }
        }
    }
}

}  // namespace rigExec
