// Shared triangulation and nearest-primitive acceleration for the
// brute-force geometry queries (surface projection, projector raycasts,
// curve-volume weights). Header-only and USD-free so the USD evaluators
// (instantiated with Gf types) and the zero-USD runtime (Rr types) run
// one definition; every vector operation used here exists with the same
// arithmetic in both families.
//
// A Vec3 here needs const operator[], operator- and GetLengthSq().
//
// Equivalence contract with the brute-force loops this replaces:
//   * the fan builders enumerate triangles in face-major fan order and
//     reproduce each caller's validation verdicts, including the
//     H7 skip/truncate rules and the M41 strict rules;
//   * triangle queries retain all candidates in original fan order. The
//     caller's float closest-point kernel is not certified to remain within
//     the triangle's AABB; no triangle may be pruned by a geometric bound;
//   * segment queries prune only under the clamped-projection contract below
//     and keep primitive-ordinal ties, matching the strict-less-than loop;
//   * Build rejects (false) any input with an out-of-range index or a
//     non-finite referenced point, in which case the caller runs its
//     original loop verbatim; non-finite QUERY points need no fallback
//     because NaN bounds never prune and NaN distances never win.
#ifndef RIGEXEC_MATH_SPATIAL_ACCEL_H
#define RIGEXEC_MATH_SPATIAL_ACCEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace rigExec {

// One fan triangle: corner indices into the caller's point array, in the
// face-major fan order the brute-force loops enumerate.
struct RigExecFanTri {
    int a = -1, b = -1, c = -1;
};

// Maximum primitives in a leaf. Matches the touchPose mesh BVH leaf size
// (libs/rigExecImaging/touchPoseMesh.cpp _kLeafSize).
constexpr uint32_t kRigExecBvhLeafSize = 4;

// Below these sizes the callers keep the verbatim brute-force loop: a
// build does bound + partition work per primitive that only amortizes
// once queries outnumber it.
constexpr size_t kRigExecBvhMinTris = 64;
constexpr size_t kRigExecBvhMinSegments = 32;
constexpr size_t kRigExecBvhMinQueries = 4;

namespace spatialAccelDetail {

struct BvhNode {
    float lo[3];
    float hi[3];
    // Interior nodes come in pairs: `child` is the left child and
    // `child + 1` the right. A leaf holds an [order + child,
    // order + child + count) run and always has count > 0.
    uint32_t child = 0;
    uint32_t count = 0;
};

// Squared distance from p to the node box in double precision. Exact
// to double rounding; prune sites subtract the sound float-error pad.
template <class Vec3>
double BoxLowerSq(const BvhNode &node, const Vec3 &p)
{
    double lower = 0.0;
    for (int k = 0; k < 3; ++k) {
        const double v = double(p[k]);
        if (v < double(node.lo[k])) {
            const double d = double(node.lo[k]) - v;
            lower += d * d;
        } else if (v > double(node.hi[k])) {
            const double d = v - double(node.hi[k]);
            lower += d * d;
        }
    }
    return lower;
}

// Unit roundoff of the leaf float arithmetic.
constexpr double kRigExecBvhUnitRoundoff =
    double(std::numeric_limits<float>::epsilon()) * 0.5;

// Conservative arithmetic envelope for clamped segment projection and
// distance: the reconstructed point is a rounded convex combination of
// finite endpoints. This does not bound triangle barycentric reciprocals
// and must never be used to discard a triangle candidate.
constexpr double kRigExecBvhKernelFlops = 256.0;

// Largest input magnitude behind a prune decision: the node's bounds and
// the query point, which together bound every leaf-kernel input.
inline double BvhCoordMag(const BvhNode &node, double queryMag)
{
    double m = queryMag;
    for (int k = 0; k < 3; ++k) {
        m = std::max(m, std::fabs(double(node.lo[k])));
        m = std::max(m, std::fabs(double(node.hi[k])));
    }
    return m;
}

// Overestimate of the true distance behind a winning leaf value: the
// value itself plus relative slack for the length chain, plus four
// times the kernel envelope times coordinate magnitude (absolute slack
// for kernel error at zero distance, with margin for the circularity
// that the error itself scales with the distance).
inline double BvhDistStar(double bestDist, double coordMag)
{
    const double u = kRigExecBvhUnitRoundoff;
    return bestDist * (1.0 + 16.0 * u) +
           4.0 * kRigExecBvhKernelFlops * u * coordMag;
}

// Sound prune pad: an upper bound on how far a node's exact lower bound
// can exceed a leaf distance computed inside it, so pruning never drops
// a winner or an earlier-ordinal tie -- even under catastrophic
// cancellation, where the computed distance can land far outside the
// exact box. Terms: position error (kernel envelope times coordinate
// magnitude, plus the point subtraction) scaled by twice the distance;
// its square; the length-squared chain; double rounding of the bound;
// subnormal/underflow slack.
inline double BvhPrunePad(double coordMag, double distStar, double lower)
{
    const double u = kRigExecBvhUnitRoundoff;
    const double e1 = kRigExecBvhKernelFlops * u * coordMag +
                      2.0 * u * distStar;
    return 2.0 * distStar * e1 + e1 * e1 +
           6.0 * u * distStar * distStar + 1e-15 * lower + 1e-37;
}

// Median-split build over n items with 6-float bounds. Splits halve the
// run every level (even at zero centroid extent, by run midpoint), so
// depth stays under 33 for any item count and the explicit stack below
// never recurses. Split quality affects speed only: with strict pruning
// and ordinal ties the result equals brute force under any split.
inline void BuildBvh(size_t n, const float *bounds, std::vector<BvhNode> *nodes,
                     std::vector<uint32_t> *order)
{
    nodes->clear();
    order->resize(n);
    for (size_t i = 0; i < n; ++i) {
        (*order)[i] = uint32_t(i);
    }
    if (n == 0) {
        return;
    }
    nodes->reserve(2 * n / kRigExecBvhLeafSize + 2);
    BvhNode root;
    root.child = 0;
    root.count = uint32_t(n);
    for (int k = 0; k < 3; ++k) {
        root.lo[k] = std::numeric_limits<float>::infinity();
        root.hi[k] = -std::numeric_limits<float>::infinity();
    }
    nodes->push_back(root);
    std::vector<uint32_t> stack;
    stack.push_back(0);
    while (!stack.empty()) {
        const uint32_t index = stack.back();
        stack.pop_back();
        const uint32_t first = (*nodes)[index].child;
        const uint32_t count = (*nodes)[index].count;
        float lo[3], hi[3], clo[3], chi[3];
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::numeric_limits<float>::infinity();
            hi[k] = -std::numeric_limits<float>::infinity();
            clo[k] = std::numeric_limits<float>::infinity();
            chi[k] = -std::numeric_limits<float>::infinity();
        }
        for (uint32_t i = first; i < first + count; ++i) {
            const uint32_t t = (*order)[i];
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], bounds[6 * t + k]);
                hi[k] = std::max(hi[k], bounds[6 * t + 3 + k]);
                const float c = 0.5f * (bounds[6 * t + k] +
                                        bounds[6 * t + 3 + k]);
                clo[k] = std::min(clo[k], c);
                chi[k] = std::max(chi[k], c);
            }
        }
        for (int k = 0; k < 3; ++k) {
            (*nodes)[index].lo[k] = lo[k];
            (*nodes)[index].hi[k] = hi[k];
        }
        if (count <= kRigExecBvhLeafSize) {
            continue;
        }
        int axis = 0;
        for (int k = 1; k < 3; ++k) {
            if (chi[k] - clo[k] > chi[axis] - clo[axis]) {
                axis = k;
            }
        }
        const uint32_t mid = first + count / 2;
        std::nth_element(order->begin() + first, order->begin() + mid,
                         order->begin() + first + count,
                         [bounds, axis](uint32_t x, uint32_t y) {
                             const float cx =
                                 0.5f * (bounds[6 * x + axis] +
                                         bounds[6 * x + 3 + axis]);
                             const float cy =
                                 0.5f * (bounds[6 * y + axis] +
                                         bounds[6 * y + 3 + axis]);
                             return cx < cy;
                         });
        const uint32_t left = uint32_t(nodes->size());
        BvhNode leftNode, rightNode;
        leftNode.child = first;
        leftNode.count = mid - first;
        rightNode.child = mid;
        rightNode.count = first + count - mid;
        nodes->push_back(leftNode);
        nodes->push_back(rightNode);
        (*nodes)[index].child = left;
        (*nodes)[index].count = 0;
        stack.push_back(left);
        stack.push_back(left + 1);
    }
}

}  // namespace spatialAccelDetail

// H7 validation flavor: out-of-range index triples are skipped (the
// brute-force `continue`), and running past the index array truncates
// the walk (the brute-force early return, which always lands during the
// first point, so no point is modified). Offsets are signed: a wrapped
// size_t offset in the original loop is an out-of-bounds read, which
// the builder turns into a defined truncation instead.
struct RigExecFanTrisH7 {
    std::vector<RigExecFanTri> tris;
    bool truncated = false;
};

// The span form borrows both ranges for the call only; the caller keeps
// the storage (e.g. the VtArray behind cdata()) alive until it returns.
// A null range with a nonzero size truncates, as an overrun does; a null
// range with size zero is the empty range. Bit-identical to the vector
// form, which delegates to it.
inline RigExecFanTrisH7 RigExecBuildFanTrisH7(
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize,
    size_t pointCount)
{
    RigExecFanTrisH7 out;
    if ((faceVertexCountsSize > 0 && !faceVertexCounts) ||
        (faceVertexIndicesSize > 0 && !faceVertexIndices)) {
        out.truncated = true;
        return out;
    }
    size_t reserve = 0;
    for (size_t f = 0; f < faceVertexCountsSize; ++f) {
        const int faceCount = faceVertexCounts[f];
        if (faceCount > 2) {
            reserve += size_t(faceCount) - 2;
        }
    }
    out.tris.reserve(reserve);
    int64_t offset = 0;
    const int64_t size = int64_t(faceVertexIndicesSize);
    for (size_t f = 0; f < faceVertexCountsSize; ++f) {
        const int faceCount = faceVertexCounts[f];
        for (int c = 1; c + 1 < faceCount; ++c) {
            const int64_t i2 = offset + int64_t(c) + 1;
            if (offset < 0 || i2 < 0 || i2 >= size) {
                out.truncated = true;
                return out;
            }
            // 0 <= offset <= offset + c < i2 < size: every read below
            // is in range by construction.
            const int ia = faceVertexIndices[size_t(offset)];
            const int ib = faceVertexIndices[size_t(offset + int64_t(c))];
            const int ic = faceVertexIndices[size_t(i2)];
            if (ia < 0 || ib < 0 || ic < 0 ||
                size_t(ia) >= pointCount || size_t(ib) >= pointCount ||
                size_t(ic) >= pointCount) {
                continue;
            }
            out.tris.push_back(RigExecFanTri{ia, ib, ic});
        }
        if ((faceCount > 0 && offset > INT64_MAX - faceCount) ||
            (faceCount < 0 && offset < INT64_MIN - faceCount)) {
            out.truncated = true;
            return out;
        }
        offset += faceCount;
    }
    return out;
}

inline RigExecFanTrisH7 RigExecBuildFanTrisH7(
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices, size_t pointCount)
{
    return RigExecBuildFanTrisH7(
        faceVertexCounts.data(), faceVertexCounts.size(),
        faceVertexIndices.data(), faceVertexIndices.size(), pointCount);
}

// M41 validation flavor: any short face, overrun, out-of-range corner
// or trailing index fails the whole build, exactly like the raycast.
struct RigExecFanTrisStrict {
    std::vector<RigExecFanTri> tris;
    bool valid = false;
};

inline RigExecFanTrisStrict RigExecBuildFanTrisStrict(
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices, size_t pointCount)
{
    RigExecFanTrisStrict out;
    size_t reserve = 0;
    for (int faceCount : faceVertexCounts) {
        if (faceCount > 2) {
            reserve += size_t(faceCount) - 2;
        }
    }
    out.tris.reserve(reserve);
    int64_t offset = 0;
    const int64_t size = int64_t(faceVertexIndices.size());
    for (int faceCount : faceVertexCounts) {
        if (faceCount < 3 || offset < 0 || offset > size ||
            int64_t(faceCount) > size - offset) {
            return out;
        }
        for (int corner = 0; corner < faceCount; ++corner) {
            const int index =
                faceVertexIndices[size_t(offset + int64_t(corner))];
            if (index < 0 || size_t(index) >= pointCount) {
                return out;
            }
        }
        const int origin0 = faceVertexIndices[size_t(offset)];
        for (int corner = 1; corner + 1 < faceCount; ++corner) {
            out.tris.push_back(RigExecFanTri{
                origin0, faceVertexIndices[size_t(offset + int64_t(corner))],
                faceVertexIndices[size_t(offset + int64_t(corner) + 1)]});
        }
        offset += faceCount;
    }
    if (offset != size) {
        return out;
    }
    out.valid = true;
    return out;
}

// Cached triangle candidate index for surface projection. The legacy
// float closest-point callback can produce an ill-conditioned result outside
// its geometric AABB. All triangles therefore remain non-prunable until a
// callback supplies a certified computed-value bound. Keep the existing API
// so owner-retained fans and call sites can share one exact fallback.
template <class Vec3>
struct RigExecTriangleBvh {
    // Reserved for a future certified traversal; no triangle nodes are built.
    std::vector<spatialAccelDetail::BvhNode> nodes;
    std::vector<uint32_t> order;

    // Non-finite referenced points and invalid indices refuse this index,
    // leaving the caller to its original validation/fallback loop.
    bool Build(const Vec3 *points, size_t pointCount,
               const RigExecFanTri *tris, size_t triCount)
    {
        nodes.clear(); order.clear();
        if (!points || !tris || triCount == 0 || pointCount == 0 ||
            triCount > std::numeric_limits<uint32_t>::max()) return false;
        for (size_t t = 0; t < triCount; ++t) {
            const int corners[3] = {tris[t].a, tris[t].b, tris[t].c};
            for (int index : corners) {
                if (index < 0 || size_t(index) >= pointCount) return false;
                for (int k = 0; k < 3; ++k)
                    if (!std::isfinite(points[size_t(index)][k])) return false;
            }
        }
        order.resize(triCount);
        for (size_t t = 0; t < triCount; ++t) order[t] = uint32_t(t);
        return true;
    }

    // Exactly the original fan walk: every callback executes in ordinal
    // order, NaN distances never win, FLT_MAX never wins, and ties retain the
    // first candidate, including its signed-zero coordinates.
    template <class ClosestFn>
    Vec3 QueryNearest(const Vec3 &p, const Vec3 *points,
                      const RigExecFanTri *tris, ClosestFn closest) const
    {
        float best = std::numeric_limits<float>::max();
        Vec3 bestPoint = p;
        for (uint32_t t : order) {
            const Vec3 q = closest(p, points[size_t(tris[t].a)],
                                  points[size_t(tris[t].b)],
                                  points[size_t(tris[t].c)]);
            const float distance = (q - p).GetLengthSq();
            if (distance < best) { best = distance; bestPoint = q; }
        }
        return bestPoint;
    }
};

// Nearest-segment acceleration for curve-volume weights (M49). Segment
// i joins points[i] and points[i + 1]; the leaf calls the caller's own
// segment-distance kernel, so the arithmetic is the brute-force
// loop's verbatim. The callback contract is the project's segment
// distance: clamp its projection to [0,1], reconstruct a + (b-a)*t in float,
// then measure float Euclidean distance. An arbitrary/unbounded callback
// cannot use geometric pruning safely.
template <class Vec3>
struct RigExecSegmentBvh {
    std::vector<spatialAccelDetail::BvhNode> nodes;
    std::vector<uint32_t> order;

    // False (nodes left empty) for fewer than two points, or when a
    // point is non-finite; the caller then runs its original loop.
    bool Build(const Vec3 *points, size_t count)
    {
        nodes.clear();
        order.clear();
        if (!points || count < 2) {
            return false;
        }
        const size_t segments = count - 1;
        std::vector<float> bounds(segments * 6);
        for (size_t s = 0; s < segments; ++s) {
            for (int k = 0; k < 3; ++k) {
                const float v0 = points[s][k];
                const float v1 = points[s + 1][k];
                if (!std::isfinite(v0) || !std::isfinite(v1)) {
                    nodes.clear();
                    order.clear();
                    return false;
                }
                bounds[6 * s + k] = std::min(v0, v1);
                bounds[6 * s + 3 + k] = std::max(v0, v1);
            }
        }
        spatialAccelDetail::BuildBvh(segments, bounds.data(), &nodes,
                                     &order);
        return true;
    }

    // Smallest segment distance to p; +infinity when nothing wins.
    // Requires the same array Build accepted.
    template <class DistFn>
    float QueryNearest(const Vec3 &p, const Vec3 *points,
                       DistFn segDist) const
    {
        if (nodes.empty()) {
            return std::numeric_limits<float>::infinity();
        }
        uint32_t stack[64];
        int top = 0;
        stack[top++] = 0;
        double best = std::numeric_limits<double>::infinity();
        double queryMag = std::fabs(double(p[0]));
        queryMag = std::max(queryMag, std::fabs(double(p[1])));
        queryMag = std::max(queryMag, std::fabs(double(p[2])));
        uint32_t bestOrd = 0;
        bool haveBest = false;
        while (top > 0) {
            const spatialAccelDetail::BvhNode &node = nodes[stack[--top]];
            // BoxLowerSq is squared; best tracks distance, so compare
            // against best squared. best is +inf until the first leaf.
            const double coordMag =
                spatialAccelDetail::BvhCoordMag(node, queryMag);
            const double distStar =
                spatialAccelDetail::BvhDistStar(best, coordMag);
            const double lower =
                spatialAccelDetail::BoxLowerSq(node, p);
            if (haveBest &&
                lower - spatialAccelDetail::BvhPrunePad(coordMag, distStar,
                                                       lower) >
                    best * best) {
                continue;
            }
            if (node.count > 0) {
                for (uint32_t i = node.child;
                     i < node.child + node.count; ++i) {
                    const uint32_t s = order[i];
                    const float d =
                        segDist(p, points[s], points[s + 1]);
                    if (double(d) < best ||
                        (haveBest && double(d) == best &&
                         s < bestOrd)) {
                        best = double(d);
                        bestOrd = s;
                        haveBest = true;
                    }
                }
                continue;
            }
            const spatialAccelDetail::BvhNode &left = nodes[node.child];
            const spatialAccelDetail::BvhNode &right =
                nodes[node.child + 1];
            const double lowerLeft =
                spatialAccelDetail::BoxLowerSq(left, p);
            const double lowerRight =
                spatialAccelDetail::BoxLowerSq(right, p);
            if (lowerLeft < lowerRight) {
                stack[top++] = node.child;
                stack[top++] = node.child + 1;
            } else {
                stack[top++] = node.child + 1;
                stack[top++] = node.child;
            }
        }
        return haveBest ? float(best)
                        : std::numeric_limits<float>::infinity();
    }
};

}  // namespace rigExec

#endif  // RIGEXEC_MATH_SPATIAL_ACCEL_H
