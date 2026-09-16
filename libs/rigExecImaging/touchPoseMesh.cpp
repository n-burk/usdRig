//
// TouchPose geometry -- see touchPoseMesh.h.
//
#include "touchPoseMesh.h"

#include "pxr/base/work/loops.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

namespace {

// Leaves hold at most this many triangles. Four is the usual sweet spot for
// a ray caster: a leaf test is a few triangle tests, and a shallower tree
// means fewer box tests per ray.
constexpr uint32_t _kLeafSize = 4;
constexpr int _kBins = 16;

// Below this many elements a parallel loop costs more than it saves.
constexpr size_t _kParallelGrain = 2048;

// A refit that has made the tree this much worse than it was when built is
// rebuilt instead. The measure is the SAH proxy -- total node surface area
// relative to the root's -- which is what a ray actually pays for.
constexpr double _kRebuildRatio = 1.6;

// The same minimum hit distance the Python cast used, so a ray starting on
// the surface does not hit the face it starts on.
constexpr double _kMinT = 1e-4;

inline double _Area(const float lo[3], const float hi[3])
{
    const double x = std::max(0.0f, hi[0] - lo[0]);
    const double y = std::max(0.0f, hi[1] - lo[1]);
    const double z = std::max(0.0f, hi[2] - lo[2]);
    return 2.0 * (x * y + y * z + z * x);
}

inline void _Empty(float lo[3], float hi[3])
{
    for (int k = 0; k < 3; ++k) {
        lo[k] = std::numeric_limits<float>::max();
        hi[k] = -std::numeric_limits<float>::max();
    }
}

inline void _Grow(float lo[3], float hi[3], const float *blo, const float *bhi)
{
    for (int k = 0; k < 3; ++k) {
        lo[k] = std::min(lo[k], blo[k]);
        hi[k] = std::max(hi[k], bhi[k]);
    }
}

void _ParallelFor(size_t n, const std::function<void(size_t, size_t)> &body)
{
    if (n < _kParallelGrain) {
        body(0, n);
    } else {
        WorkParallelForN(n, body, _kParallelGrain);
    }
}

}  // namespace

bool
RigExecTouchPoseMesh::SetTopology(
    const VtIntArray &counts, const VtIntArray &indices, size_t pointCount)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _faceStarts.clear();
    _faceCounts.clear();
    _tris.clear();
    _triFace.clear();
    _indices = VtIntArray();
    _pointCount = 0;
    _nodes.clear();
    _order.clear();
    _bvhBuilt = false;
    _centroidVersion = ~uint64_t(0);

    size_t total = 0;
    size_t triangles = 0;
    for (int c : counts) {
        if (c < 3) {
            // A degenerate face is legal USD but draws nothing and can be
            // touched by nothing; it keeps its index so every face after it
            // still lines up with the authored face numbering.
            total += size_t(std::max(c, 0));
            continue;
        }
        total += size_t(c);
        triangles += size_t(c - 2);
    }
    if (total != indices.size()) {
        return false;
    }
    for (int index : indices) {
        if (index < 0 || size_t(index) >= pointCount) {
            return false;
        }
    }

    _faceStarts.resize(counts.size());
    _faceCounts.resize(counts.size());
    _tris.reserve(triangles * 3);
    _triFace.reserve(triangles);
    int start = 0;
    for (size_t f = 0; f < counts.size(); ++f) {
        const int c = std::max(counts[f], 0);
        _faceStarts[f] = start;
        _faceCounts[f] = c;
        for (int k = 1; k + 1 < c; ++k) {
            _tris.push_back(uint32_t(indices[start]));
            _tris.push_back(uint32_t(indices[start + k]));
            _tris.push_back(uint32_t(indices[start + k + 1]));
            _triFace.push_back(int(f));
        }
        start += c;
    }
    _indices = indices;
    _pointCount = pointCount;
    _faceRegion.assign(counts.size(), -1);
    ++_pointsVersion;
    return true;
}

bool
RigExecTouchPoseMesh::SetPoints(const VtVec3fArray &points)
{
    if (points.size() != _pointCount) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _points = points;
    ++_pointsVersion;
    return true;
}

void
RigExecTouchPoseMesh::SetTransform(const GfMatrix4d &localToWorld)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _xform = localToWorld;
    _identity = (localToWorld == GfMatrix4d(1.0));
    _inverse = _identity ? GfMatrix4d(1.0) : localToWorld.GetInverse();
}

void
RigExecTouchPoseMesh::SetFaceRegions(const int32_t *regionOf, size_t count)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _faceRegion.assign(_faceStarts.size(), -1);
    const size_t n = std::min(count, _faceRegion.size());
    if (regionOf) {
        std::copy(regionOf, regionOf + n, _faceRegion.begin());
    }
}

// ---------------------------------------------------------------------------
// acceleration
// ---------------------------------------------------------------------------

void
RigExecTouchPoseMesh::_ComputeTriangleBounds(std::vector<float> *bounds) const
{
    const size_t n = _triFace.size();
    bounds->resize(n * 6);
    const GfVec3f *p = _points.cdata();
    const uint32_t *tris = _tris.data();
    float *out = bounds->data();
    _ParallelFor(n, [p, tris, out](size_t b, size_t e) {
        for (size_t t = b; t < e; ++t) {
            const GfVec3f &a = p[tris[3 * t]];
            const GfVec3f &c = p[tris[3 * t + 1]];
            const GfVec3f &d = p[tris[3 * t + 2]];
            float *o = out + 6 * t;
            for (int k = 0; k < 3; ++k) {
                o[k] = std::min(a[k], std::min(c[k], d[k]));
                o[3 + k] = std::max(a[k], std::max(c[k], d[k]));
            }
        }
    });
}

void
RigExecTouchPoseMesh::_Build() const
{
    const size_t n = _triFace.size();
    _nodes.clear();
    _order.resize(n);
    for (size_t i = 0; i < n; ++i) {
        _order[i] = uint32_t(i);
    }
    if (!n) {
        _bvhBuilt = true;
        _builtCost = 0.0;
        return;
    }

    std::vector<float> bounds;
    _ComputeTriangleBounds(&bounds);
    std::vector<float> centroids(n * 3);
    {
        const float *bb = bounds.data();
        float *out = centroids.data();
        _ParallelFor(n, [bb, out](size_t b, size_t e) {
            for (size_t t = b; t < e; ++t) {
                for (int k = 0; k < 3; ++k) {
                    out[3 * t + k] = 0.5f * (bb[6 * t + k] + bb[6 * t + 3 + k]);
                }
            }
        });
    }

    _nodes.reserve(2 * n / _kLeafSize + 2);
    _nodes.push_back(_Node());
    _nodes[0].first = 0;
    _nodes[0].count = uint32_t(n);

    // Explicit stack: a biped's tree is ~20 deep, but a pathological mesh
    // should not be able to blow the thread's stack.
    std::vector<uint32_t> stack;
    stack.push_back(0);

    struct Bin {
        float lo[3], hi[3];
        uint32_t count;
    };

    while (!stack.empty()) {
        const uint32_t index = stack.back();
        stack.pop_back();
        const uint32_t first = _nodes[index].first;
        const uint32_t count = _nodes[index].count;

        float lo[3], hi[3], clo[3], chi[3];
        _Empty(lo, hi);
        _Empty(clo, chi);
        for (uint32_t i = first; i < first + count; ++i) {
            const uint32_t t = _order[i];
            _Grow(lo, hi, &bounds[6 * t], &bounds[6 * t + 3]);
            _Grow(clo, chi, &centroids[3 * t], &centroids[3 * t]);
        }
        std::copy(lo, lo + 3, _nodes[index].lo);
        std::copy(hi, hi + 3, _nodes[index].hi);

        if (count <= _kLeafSize) {
            continue;
        }

        // Binned SAH over the longest centroid axis.
        int axis = 0;
        for (int k = 1; k < 3; ++k) {
            if (chi[k] - clo[k] > chi[axis] - clo[axis]) {
                axis = k;
            }
        }
        const float extent = chi[axis] - clo[axis];
        uint32_t mid = first + count / 2;
        if (extent > 0.0f) {
            Bin bins[_kBins];
            for (Bin &bin : bins) {
                _Empty(bin.lo, bin.hi);
                bin.count = 0;
            }
            const float scale = float(_kBins) / extent;
            auto binOf = [&](uint32_t t) {
                int b = int((centroids[3 * t + axis] - clo[axis]) * scale);
                return std::min(std::max(b, 0), _kBins - 1);
            };
            for (uint32_t i = first; i < first + count; ++i) {
                const uint32_t t = _order[i];
                Bin &bin = bins[binOf(t)];
                _Grow(bin.lo, bin.hi, &bounds[6 * t], &bounds[6 * t + 3]);
                ++bin.count;
            }
            double leftArea[_kBins], rightArea[_kBins];
            uint32_t leftCount[_kBins], rightCount[_kBins];
            float alo[3], ahi[3];
            _Empty(alo, ahi);
            uint32_t acc = 0;
            for (int b = 0; b < _kBins; ++b) {
                if (bins[b].count) {
                    _Grow(alo, ahi, bins[b].lo, bins[b].hi);
                }
                acc += bins[b].count;
                leftArea[b] = acc ? _Area(alo, ahi) : 0.0;
                leftCount[b] = acc;
            }
            _Empty(alo, ahi);
            acc = 0;
            for (int b = _kBins - 1; b >= 0; --b) {
                if (bins[b].count) {
                    _Grow(alo, ahi, bins[b].lo, bins[b].hi);
                }
                acc += bins[b].count;
                rightArea[b] = acc ? _Area(alo, ahi) : 0.0;
                rightCount[b] = acc;
            }
            int best = -1;
            double bestCost = std::numeric_limits<double>::max();
            for (int b = 0; b + 1 < _kBins; ++b) {
                if (!leftCount[b] || !rightCount[b + 1]) {
                    continue;
                }
                const double cost = leftArea[b] * leftCount[b] +
                                    rightArea[b + 1] * rightCount[b + 1];
                if (cost < bestCost) {
                    bestCost = cost;
                    best = b;
                }
            }
            if (best >= 0) {
                auto split = std::partition(
                    _order.begin() + first, _order.begin() + first + count,
                    [&](uint32_t t) { return binOf(t) <= best; });
                mid = uint32_t(split - _order.begin());
            }
        }
        if (mid == first || mid == first + count) {
            // Every centroid in one bin (coincident triangles): split by
            // index, which is still a valid tree.
            mid = first + count / 2;
        }

        const uint32_t left = uint32_t(_nodes.size());
        _nodes.push_back(_Node());
        _nodes.push_back(_Node());
        _nodes[left].first = first;
        _nodes[left].count = mid - first;
        _nodes[left + 1].first = mid;
        _nodes[left + 1].count = first + count - mid;
        _nodes[index].first = left;
        _nodes[index].count = 0;
        stack.push_back(left + 1);
        stack.push_back(left);
    }

    double sum = 0.0;
    for (const _Node &node : _nodes) {
        sum += _Area(node.lo, node.hi);
    }
    const double root = std::max(_Area(_nodes[0].lo, _nodes[0].hi), 1e-30);
    _builtCost = sum / root;
    _bvhBuilt = true;
    ++_buildCount;
}

void
RigExecTouchPoseMesh::_Refit() const
{
    std::vector<float> bounds;
    _ComputeTriangleBounds(&bounds);

    // Leaves in parallel -- they are most of the work and independent.
    const size_t nodeCount = _nodes.size();
    {
        _Node *nodes = _nodes.data();
        const uint32_t *order = _order.data();
        const float *bb = bounds.data();
        _ParallelFor(nodeCount, [nodes, order, bb](size_t b, size_t e) {
            for (size_t i = b; i < e; ++i) {
                _Node &node = nodes[i];
                if (!node.count) {
                    continue;
                }
                _Empty(node.lo, node.hi);
                for (uint32_t j = node.first; j < node.first + node.count; ++j) {
                    const uint32_t t = order[j];
                    _Grow(node.lo, node.hi, bb + 6 * t, bb + 6 * t + 3);
                }
            }
        });
    }
    // Interiors bottom-up: children are always stored after their parent.
    double sum = 0.0;
    for (size_t i = nodeCount; i-- > 0;) {
        _Node &node = _nodes[i];
        if (!node.count) {
            const _Node &l = _nodes[node.first];
            const _Node &r = _nodes[node.first + 1];
            for (int k = 0; k < 3; ++k) {
                node.lo[k] = std::min(l.lo[k], r.lo[k]);
                node.hi[k] = std::max(l.hi[k], r.hi[k]);
            }
        }
        sum += _Area(node.lo, node.hi);
    }
    ++_refitCount;
    const double root = std::max(_Area(_nodes[0].lo, _nodes[0].hi), 1e-30);
    if (sum / root > _kRebuildRatio * _builtCost) {
        _Build();
    }
}

void
RigExecTouchPoseMesh::Prepare() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_bvhVersion == _pointsVersion) {
        return;
    }
    if (!HasPoints()) {
        _nodes.clear();
        _bvhBuilt = false;
        _bvhVersion = _pointsVersion;
        return;
    }
    if (!_bvhBuilt || _nodes.empty() || _order.size() != _triFace.size()) {
        _Build();
    } else {
        _Refit();
    }
    _bvhVersion = _pointsVersion;
}

// ---------------------------------------------------------------------------
// the cast
// ---------------------------------------------------------------------------

bool
RigExecTouchPoseMesh::_HitTriangle(
    size_t tri, const GfVec3d &o, const GfVec3d &d, double tMax,
    double *tOut) const
{
    // Moller-Trumbore in double: the triangle is in float, but a ray from a
    // camera a few metres away across a centimetre-scale face is exactly
    // where float edge products start to lose the hit.
    const GfVec3f *p = _points.cdata();
    const GfVec3d v0(p[_tris[3 * tri]]);
    const GfVec3d e1 = GfVec3d(p[_tris[3 * tri + 1]]) - v0;
    const GfVec3d e2 = GfVec3d(p[_tris[3 * tri + 2]]) - v0;
    const GfVec3d pv = GfCross(d, e2);
    const double det = GfDot(e1, pv);
    if (std::fabs(det) < 1e-18) {
        return false;
    }
    const double inv = 1.0 / det;
    const GfVec3d tv = o - v0;
    const double u = GfDot(tv, pv) * inv;
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    const GfVec3d qv = GfCross(tv, e1);
    const double v = GfDot(d, qv) * inv;
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }
    const double t = GfDot(e2, qv) * inv;
    if (t <= _kMinT || t >= tMax) {
        return false;
    }
    *tOut = t;
    return true;
}

int
RigExecTouchPoseMesh::Cast(
    const GfVec3d &worldOrigin, const GfVec3d &worldDirection,
    double *tOut) const
{
    if (tOut) {
        *tOut = -1.0;
    }
    const double length = worldDirection.GetLength();
    if (length < 1e-12 || !HasPoints()) {
        return -1;
    }
    Prepare();
    if (_nodes.empty()) {
        return -1;
    }

    // Into local space WITHOUT renormalizing, so the ray parameter stays the
    // world distance along the normalized world direction.
    const GfVec3d wd = worldDirection / length;
    const GfVec3d o = _identity ? worldOrigin : _inverse.Transform(worldOrigin);
    const GfVec3d d = _identity ? wd : _inverse.TransformDir(wd);

    const double big = 1e30;
    const double inv[3] = {
        std::fabs(d[0]) > 1e-30 ? 1.0 / d[0] : (d[0] >= 0 ? big : -big),
        std::fabs(d[1]) > 1e-30 ? 1.0 / d[1] : (d[1] >= 0 ? big : -big),
        std::fabs(d[2]) > 1e-30 ? 1.0 / d[2] : (d[2] >= 0 ? big : -big)};

    auto slab = [&](const _Node &node, double tMax, double *tNear) {
        double t0 = -std::numeric_limits<double>::max();
        double t1 = tMax;
        for (int k = 0; k < 3; ++k) {
            double a = (double(node.lo[k]) - o[k]) * inv[k];
            double b = (double(node.hi[k]) - o[k]) * inv[k];
            if (a > b) {
                std::swap(a, b);
            }
            // Pad a hair so a ray grazing a flat box (a planar region) is
            // never lost to rounding at the face of the box.
            a -= 1e-7 * std::fabs(a) + 1e-9;
            b += 1e-7 * std::fabs(b) + 1e-9;
            t0 = std::max(t0, a);
            t1 = std::min(t1, b);
            if (t0 > t1) {
                return false;
            }
        }
        *tNear = t0;
        return t1 >= 0.0;
    };

    double best = std::numeric_limits<double>::max();
    int bestTri = -1;
    uint32_t stack[128];
    int top = 0;
    double rootNear;
    if (!slab(_nodes[0], best, &rootNear)) {
        return -1;
    }
    stack[top++] = 0;
    while (top) {
        const _Node &node = _nodes[stack[--top]];
        double tNear;
        if (!slab(node, best, &tNear) || tNear > best) {
            continue;
        }
        if (node.count) {
            for (uint32_t j = node.first; j < node.first + node.count; ++j) {
                double t;
                if (_HitTriangle(_order[j], o, d, best, &t)) {
                    best = t;
                    bestTri = int(_order[j]);
                }
            }
            continue;
        }
        // Nearer child last, so it is popped first and tightens `best`
        // before the farther one is tested.
        const uint32_t l = node.first;
        const uint32_t r = node.first + 1;
        double tl, tr;
        const bool hl = slab(_nodes[l], best, &tl);
        const bool hr = slab(_nodes[r], best, &tr);
        if (top + 2 > int(sizeof(stack) / sizeof(stack[0]))) {
            // Never on a sane tree; fall back to the exact answer.
            return CastBruteForce(worldOrigin, worldDirection, tOut);
        }
        if (hl && hr) {
            if (tl < tr) {
                stack[top++] = r;
                stack[top++] = l;
            } else {
                stack[top++] = l;
                stack[top++] = r;
            }
        } else if (hl) {
            stack[top++] = l;
        } else if (hr) {
            stack[top++] = r;
        }
    }
    if (bestTri < 0) {
        return -1;
    }
    if (tOut) {
        *tOut = best;
    }
    return _triFace[bestTri];
}

int
RigExecTouchPoseMesh::CastBruteForce(
    const GfVec3d &worldOrigin, const GfVec3d &worldDirection,
    double *tOut) const
{
    if (tOut) {
        *tOut = -1.0;
    }
    const double length = worldDirection.GetLength();
    if (length < 1e-12 || !HasPoints()) {
        return -1;
    }
    const GfVec3d wd = worldDirection / length;
    const GfVec3d o = _identity ? worldOrigin : _inverse.Transform(worldOrigin);
    const GfVec3d d = _identity ? wd : _inverse.TransformDir(wd);
    double best = std::numeric_limits<double>::max();
    int bestTri = -1;
    for (size_t t = 0; t < _triFace.size(); ++t) {
        double hit;
        if (_HitTriangle(t, o, d, best, &hit)) {
            best = hit;
            bestTri = int(t);
        }
    }
    if (bestTri < 0) {
        return -1;
    }
    if (tOut) {
        *tOut = best;
    }
    return _triFace[bestTri];
}

// ---------------------------------------------------------------------------
// per-face queries
// ---------------------------------------------------------------------------

void
RigExecTouchPoseMesh::_ComputeCentroids() const
{
    // Called with _mutex held.
    if (_centroidVersion == _pointsVersion && _centroidXform == _xform) {
        return;
    }
    const size_t faces = _faceStarts.size();
    _centroids.resize(faces);
    _normals.resize(faces);
    const GfVec3f *p = _points.cdata();
    const int *idx = _indices.cdata();
    const int *starts = _faceStarts.data();
    const int *counts = _faceCounts.data();
    const bool identity = _identity;
    const GfMatrix4d xform = _xform;
    const GfMatrix4d normalXform = _inverse.GetTranspose();
    GfVec3f *cOut = _centroids.data();
    GfVec3f *nOut = _normals.data();
    _ParallelFor(faces, [=](size_t b, size_t e) {
        for (size_t f = b; f < e; ++f) {
            const int c = counts[f];
            if (c < 3) {
                cOut[f] = GfVec3f(std::numeric_limits<float>::quiet_NaN());
                nOut[f] = GfVec3f(0.0f);
                continue;
            }
            const int s = starts[f];
            GfVec3d sum(0.0);
            for (int k = 0; k < c; ++k) {
                sum += GfVec3d(p[idx[s + k]]);
            }
            sum /= double(c);
            // Newell's normal: right for a non-planar quad, where the first
            // corner triangle alone can point the wrong way.
            GfVec3d n(0.0);
            for (int k = 0; k < c; ++k) {
                const GfVec3f &a = p[idx[s + k]];
                const GfVec3f &d = p[idx[s + (k + 1) % c]];
                n[0] += (double(a[1]) - d[1]) * (double(a[2]) + d[2]);
                n[1] += (double(a[2]) - d[2]) * (double(a[0]) + d[0]);
                n[2] += (double(a[0]) - d[0]) * (double(a[1]) + d[1]);
            }
            if (!identity) {
                sum = xform.Transform(sum);
                n = normalXform.TransformDir(n);
            }
            cOut[f] = GfVec3f(sum);
            nOut[f] = GfVec3f(n);
        }
    });
    _centroidVersion = _pointsVersion;
    _centroidXform = _xform;
}

GfVec3d
RigExecTouchPoseMesh::FaceCentroid(int face) const
{
    if (face < 0 || size_t(face) >= _faceStarts.size() || !HasPoints()) {
        return GfVec3d(0.0);
    }
    const GfVec3f *p = _points.cdata();
    const int s = _faceStarts[face];
    const int c = _faceCounts[face];
    if (c <= 0) {
        return GfVec3d(0.0);
    }
    GfVec3d sum(0.0);
    for (int k = 0; k < c; ++k) {
        sum += GfVec3d(p[_indices[s + k]]);
    }
    sum /= double(c);
    return _identity ? sum : _xform.Transform(sum);
}

std::vector<int>
RigExecTouchPoseMesh::RegionsInRect(
    const GfMatrix4d &viewProjection, double width, double height,
    const GfVec3d &eye, double x0, double y0, double x1, double y1) const
{
    std::vector<int> result;
    if (!HasPoints() || width <= 0.0 || height <= 0.0) {
        return result;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _ComputeCentroids();

    const double loX = std::min(x0, x1), hiX = std::max(x0, x1);
    const double loY = std::min(y0, y1), hiY = std::max(y0, y1);
    int regionCount = 0;
    for (int r : _faceRegion) {
        regionCount = std::max(regionCount, r + 1);
    }
    if (!regionCount) {
        return result;
    }

    const size_t faces = _faceStarts.size();
    std::vector<std::vector<uint8_t>> perChunk;
    std::mutex merge;
    std::vector<uint8_t> caught(regionCount, 0);
    const GfVec3f *centroids = _centroids.data();
    const GfVec3f *normals = _normals.data();
    const int *regionOf = _faceRegion.data();
    const double m[4][4] = {
        {viewProjection[0][0], viewProjection[0][1], viewProjection[0][2], viewProjection[0][3]},
        {viewProjection[1][0], viewProjection[1][1], viewProjection[1][2], viewProjection[1][3]},
        {viewProjection[2][0], viewProjection[2][1], viewProjection[2][2], viewProjection[2][3]},
        {viewProjection[3][0], viewProjection[3][1], viewProjection[3][2], viewProjection[3][3]}};

    _ParallelFor(faces, [&](size_t b, size_t e) {
        std::vector<uint8_t> local(regionCount, 0);
        bool any = false;
        for (size_t f = b; f < e; ++f) {
            const int region = regionOf[f];
            if (region < 0 || local[region]) {
                continue;
            }
            const GfVec3f &c = centroids[f];
            // Row-vector convention: clip = [x y z 1] * M.
            const double cw = c[0] * m[0][3] + c[1] * m[1][3] + c[2] * m[2][3] + m[3][3];
            if (!(cw > 1e-9)) {
                continue;       // behind the eye: would project mirrored
            }
            const double cx = c[0] * m[0][0] + c[1] * m[1][0] + c[2] * m[2][0] + m[3][0];
            const double cy = c[0] * m[0][1] + c[1] * m[1][1] + c[2] * m[2][1] + m[3][1];
            const double px = (cx / cw * 0.5 + 0.5) * width - 0.5;
            const double py = (0.5 - cy / cw * 0.5) * height - 0.5;
            if (px < loX || px > hiX || py < loY || py > hiY) {
                continue;
            }
            const GfVec3f &n = normals[f];
            const double facing = n[0] * (eye[0] - c[0]) +
                                  n[1] * (eye[1] - c[1]) +
                                  n[2] * (eye[2] - c[2]);
            if (!(facing > 0.0)) {
                continue;
            }
            local[region] = 1;
            any = true;
        }
        if (any) {
            std::lock_guard<std::mutex> g(merge);
            for (int r = 0; r < regionCount; ++r) {
                caught[r] |= local[r];
            }
        }
    });
    for (int r = 0; r < regionCount; ++r) {
        if (caught[r]) {
            result.push_back(r);
        }
    }
    return result;
}

std::vector<int>
RigExecTouchPoseMesh::Brush(
    const GfVec3d &center, const GfVec3d &direction, double radius) const
{
    std::vector<int> result;
    if (!HasPoints() || !(radius > 0.0)) {
        return result;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _ComputeCentroids();
    const size_t faces = _faceStarts.size();
    const double r2 = radius * radius;
    const bool useDirection = direction.GetLengthSq() > 0.0;
    const GfVec3f *centroids = _centroids.data();
    const GfVec3f *normals = _normals.data();
    std::vector<uint8_t> inside(faces, 0);
    uint8_t *out = inside.data();
    _ParallelFor(faces, [=](size_t b, size_t e) {
        for (size_t f = b; f < e; ++f) {
            const GfVec3f &c = centroids[f];
            const double dx = c[0] - center[0];
            const double dy = c[1] - center[1];
            const double dz = c[2] - center[2];
            if (!(dx * dx + dy * dy + dz * dz <= r2)) {
                continue;
            }
            if (useDirection) {
                const GfVec3f &n = normals[f];
                if (!(n[0] * direction[0] + n[1] * direction[1] +
                      n[2] * direction[2] < 0.0)) {
                    continue;
                }
            }
            out[f] = 1;
        }
    });
    for (size_t f = 0; f < faces; ++f) {
        if (inside[f]) {
            result.push_back(int(f));
        }
    }
    return result;
}

bool
RigExecTouchPoseMesh::GetWorldBounds(GfVec3d *lo, GfVec3d *hi) const
{
    if (!HasPoints()) {
        return false;
    }
    Prepare();
    std::lock_guard<std::mutex> lock(_mutex);
    if (_nodes.empty()) {
        return false;
    }
    const _Node &root = _nodes[0];
    GfVec3d l(std::numeric_limits<double>::max());
    GfVec3d h(-std::numeric_limits<double>::max());
    for (int i = 0; i < 8; ++i) {
        GfVec3d corner((i & 1) ? root.hi[0] : root.lo[0],
                       (i & 2) ? root.hi[1] : root.lo[1],
                       (i & 4) ? root.hi[2] : root.lo[2]);
        if (!_identity) {
            corner = _xform.Transform(corner);
        }
        for (int k = 0; k < 3; ++k) {
            l[k] = std::min(l[k], corner[k]);
            h[k] = std::max(h[k], corner[k]);
        }
    }
    *lo = l;
    *hi = h;
    return true;
}

}  // namespace rigExec
