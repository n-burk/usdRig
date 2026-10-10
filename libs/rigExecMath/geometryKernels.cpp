// RigExec geometry mover kernels implementation.
#include "geometryKernels.h"
#include "deltaMushKernel.h"
#include "latticeKernel.h"
#include "spatialAccel.h"
#include "wrinkleKernel.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/rotation.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace rigExec {

double
RigExecBoundVolume(const GfVec3f *points, size_t count)
{
    if (count < 2) {
        return 0.0;
    }
    GfVec3f lo = points[0], hi = points[0];
    for (size_t i = 1; i < count; ++i) {
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], points[i][a]);
            hi[a] = std::max(hi[a], points[i][a]);
        }
    }
    const GfVec3f d = hi - lo;
    return double(d[0]) * double(d[1]) * double(d[2]);
}

void
RigExecApplyVolumeCorrect(
    std::vector<GfVec3f> *points, double referenceVolume, double strength)
{
    if (points->empty() || referenceVolume <= 0.0 || strength <= 0.0) {
        return;
    }
    const double current = RigExecBoundVolume(points->data(), points->size());
    if (current <= 0.0) {
        return;  // degenerate bound: no deterministic correction exists
    }
    // Uniform scale about the centroid moving the bound volume toward the
    // reference; strength blends the correction.
    const double full = std::cbrt(referenceVolume / current);
    const double scale = 1.0 + std::min(std::max(strength, 0.0), 1.0) *
                                   (full - 1.0);
    GfVec3f centroid(0);
    for (const GfVec3f &p : *points) {
        centroid += p;
    }
    centroid /= float(points->size());
    for (GfVec3f &p : *points) {
        p = centroid + (p - centroid) * float(scale);
    }
}



bool
RigExecTransportSurfaceOffsets(
    const std::vector<GfVec3f> &rest,
    const std::vector<GfVec3f> &posed,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    const std::vector<GfVec3f> &deltas,
    std::vector<GfVec3f> *out)
{
    return RigExecTransportSurfaceOffsetsKernel<GfVec3f, GfVec3d>(
        rest, posed, faceCounts, faceIndices, deltas, out);
}

bool
RigExecRaycastSurface(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    const GfVec3d &origin,
    const GfVec3d &direction,
    RigExecSurfaceHit *hit)
{
    return RigExecRaycastSurfaceT(points, faceVertexCounts,
                                  faceVertexIndices, origin, direction, hit);
}

bool
RigExecSurfaceFrameAtHit(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    const RigExecSurfaceHit &hit,
    const GfVec3d &upHint,
    GfMatrix4d *frame)
{
    return RigExecSurfaceFrameAtHitT(
        points, faceVertexCounts, faceVertexIndices, hit, upHint,
        static_cast<RigExecVertexNormalsFn>(
            &RigExecComputeVertexNormals),
        frame);
}

bool
RigExecRaycastSurfaceFrame(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    const GfVec3d &origin,
    const GfVec3d &direction,
    const GfVec3d &upHint,
    GfMatrix4d *frame)
{
    if (!frame) return false;
    RigExecSurfaceHit hit;
    if (!RigExecRaycastSurface(points, faceVertexCounts, faceVertexIndices,
                               origin, direction, &hit)) {
        return false;
    }
    return RigExecSurfaceFrameAtHit(points, faceVertexCounts,
                                    faceVertexIndices, hit, upHint, frame);
}

RigExecPartialDecomposition
RigExecDecomposePartialTransform(const GfMatrix4d &transform)
{
    RigExecPartialDecomposition d;
    // Row lengths are the scale; dividing them out leaves the rotation.
    GfMatrix4d basis = transform;
    basis.SetTranslateOnly(GfVec3d(0));
    for (int axis = 0; axis < 3; ++axis) {
        const GfVec3d row(basis.GetRow3(axis));
        const double length = row.GetLength();
        if (length > 1e-12) {
            d.scale[axis] = length;
            basis.SetRow3(axis, row / length);
        }
    }

    // The same axis, a fraction of the angle, ABOUT THE SAME PIVOT. The
    // last part is what makes this radial rather than a rotation with a
    // separately-faded slide bolted on.
    //
    // A cluster's transform rotates about a pivot that is nowhere near
    // the origin -- a lid turning about an eyeball sits 165 units up --
    // and a rotation about a distant pivot carries an enormous
    // translation, P - R P. Scaling THAT by w while turning by w*theta
    // does not put the point on the arc at all: it lands inside, so a
    // half-weighted lid pulls back into the eye instead of sweeping
    // round it. That is the "the falloff is turning it linear" report,
    // and it is a weighting bug rather than a painting one.
    //
    // So recover the pivot the transform actually turns about -- its
    // screw axis -- and turn a fraction about that.
    const GfRotation rotation = basis.ExtractRotation();
    d.axis = rotation.GetAxis().GetNormalized();
    d.angle = rotation.GetAngle();
    d.translation = transform.ExtractTranslation();

    // Below about a tenth of a degree there is no meaningful axis to
    // turn about and the chord and the arc agree to within float noise,
    // so the straight blend is both correct and better conditioned.
    const double radians = GfDegreesToRadians(d.angle);
    d.smallAngle = std::abs(std::sin(0.5 * radians)) < 1e-4;
    if (d.smallAngle) {
        return d;
    }

    // Split the translation into the part along the axis (a screw's own
    // travel, which stays linear in w) and the part across it, which is
    // (I - R) applied to the pivot and so names where the pivot is.
    d.along = GfDot(d.translation, d.axis);
    const GfVec3d across = d.translation - d.axis * d.along;
    const double half = 0.5 / std::tan(0.5 * radians);
    d.pivot = across * 0.5 + GfCross(d.axis, across) * half;
    return d;
}

GfMatrix4d
RigExecApplyPartialDecomposition(
    const RigExecPartialDecomposition &d, double w)
{
    GfMatrix4d scaled(1.0);
    scaled.SetScale(GfVec3d(1.0 + (d.scale[0] - 1.0) * w,
                            1.0 + (d.scale[1] - 1.0) * w,
                            1.0 + (d.scale[2] - 1.0) * w));

    GfMatrix4d out(1.0);
    out.SetRotate(GfRotation(d.axis, d.angle * w));
    out = scaled * out;

    if (d.smallAngle) {
        out.SetTranslateOnly(d.translation * w);
        return out;
    }

    // p' = R_w (p - pivot) + pivot + w * along * axis.
    const GfMatrix4d partial = out;
    out.SetTranslateOnly(d.pivot - partial.TransformDir(d.pivot)
                         + d.axis * (d.along * w));
    return out;
}

GfMatrix4d
RigExecPartialTransform(const GfMatrix4d &transform, double weight)
{
    const double w = GfClamp(weight, 0.0, 1.0);
    if (w <= 0.0) {
        return GfMatrix4d(1.0);
    }
    if (w >= 1.0) {
        return transform;
    }
    return RigExecApplyPartialDecomposition(
        RigExecDecomposePartialTransform(transform), w);
}

void
RigExecApplyLaplacianSmoothWithAdjacency(
    std::vector<GfVec3f> *points,
    const RigExecMeshAdjacency &adjacency,
    double strength,
    const GfVec3f *source,
    size_t sourceCount)
{
    const double s = std::min(std::max(strength, 0.0), 1.0);
    if (points->empty() || s <= 0.0) {
        return;
    }
    if (!adjacency.Covers(points->size())) {
        return;  // invalid topology: pass through
    }
    std::vector<GfVec3f> owned;
    const GfVec3f *src = source;
    if (!src || sourceCount != points->size()) {
        if (src) {
            return;  // a short range reads OOB: pass through instead
        }
        owned = *points;
        src = owned.data();
    }
    for (size_t i = 0; i < points->size(); ++i) {
        if (adjacency.neighbors[i].empty()) {
            continue;
        }
        GfVec3f average(0);
        for (int n : adjacency.neighbors[i]) {
            average += src[n];
        }
        average /= float(adjacency.neighbors[i].size());
        (*points)[i] = src[i] + (average - src[i]) * float(s);
    }
}

void
RigExecApplyLaplacianSmooth(
    std::vector<GfVec3f> *points,
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize,
    double strength,
    const GfVec3f *source,
    size_t sourceCount)
{
    const double s = std::min(std::max(strength, 0.0), 1.0);
    if (points->empty() || s <= 0.0) {
        return;
    }
    RigExecMeshAdjacency adjacency;
    RigExecBuildMeshAdjacency(
        points->size(), faceVertexCounts, faceVertexCountsSize,
        faceVertexIndices, faceVertexIndicesSize, &adjacency);
    RigExecApplyLaplacianSmoothWithAdjacency(
        points, adjacency, strength, source, sourceCount);
}

void
RigExecApplyLaplacianSmooth(
    std::vector<GfVec3f> *points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    double strength,
    const GfVec3f *source,
    size_t sourceCount)
{
    RigExecApplyLaplacianSmooth(
        points, faceVertexCounts.data(), faceVertexCounts.size(),
        faceVertexIndices.data(), faceVertexIndices.size(), strength,
        source, sourceCount);
}

bool
RigExecApplyDeltaMush(
    std::vector<GfVec3f> *points, const std::vector<GfVec3f> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders,
    double distanceWeight, double displacement,
    const RigExecDeltaMushSettings &settings,
    const GfMatrix4d &computationToTarget,
    const RigExecDeltaMushRest *restData)
{
    if (restData) {
        return RigExecApplyDeltaMushInSpaceWithRestData<GfVec3f, GfVec3d,
                                                        GfMatrix4d>(
            points, *restData, displacement, computationToTarget);
    }
    return RigExecApplyDeltaMushInSpaceKernel<GfVec3f, GfVec3d, GfMatrix4d>(
        points, rest, counts, indices, iterations, step, pinBorders,
        distanceWeight, displacement, settings, computationToTarget);
}

bool
RigExecApplyWrinkle(
    std::vector<GfVec3f> *points, const std::vector<GfVec3f> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const RigExecWrinkleSettings &settings,
    const RigExecWrinkleMesh *topology)
{
    if (topology) {
        return RigExecApplyWrinkleWithTopology<GfVec3f, GfVec3d>(
            points, rest, counts, indices, settings, *topology);
    }
    return RigExecApplyWrinkleKernel<GfVec3f, GfVec3d>(
        points, rest, counts, indices, settings);
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(
    const GfVec3f *points, size_t pointsSize,
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize)
{
    // A null range with a nonzero size answers like a mesh with no
    // usable faces: zero normals. A null range with size zero is the
    // empty range, which the walk below already handles.
    if ((pointsSize > 0 && !points) ||
        (faceVertexCountsSize > 0 && !faceVertexCounts) ||
        (faceVertexIndicesSize > 0 && !faceVertexIndices)) {
        return std::vector<GfVec3f>(pointsSize, GfVec3f(0));
    }
    std::vector<GfVec3f> normals(pointsSize, GfVec3f(0));
    std::vector<int> ring;
    size_t offset = 0;
    for (size_t f = 0; f < faceVertexCountsSize; ++f) {
        const int faceCount = faceVertexCounts[f];
        if (faceCount < 3 ||
            offset + static_cast<size_t>(faceCount) >
                faceVertexIndicesSize) {
            return normals;
        }
        // Gather the face once, dropping indices that do not address a
        // point rather than letting them into the arithmetic.
        ring.clear();
        for (int k = 0; k < faceCount; ++k) {
            const int v = faceVertexIndices[offset + k];
            if (v >= 0 && static_cast<size_t>(v) < pointsSize) {
                ring.push_back(v);
            }
        }
        offset += faceCount;
        const size_t n = ring.size();
        if (n < 3) {
            continue;
        }

        // Newell's method for the face normal: the only construction that
        // is correct for a non-planar n-gon, and the only one that does
        // not depend on how the polygon happens to be triangulated.
        // Triangulating the face and accumulating per-triangle normals --
        // which this kernel used to do -- gives every vertex only the fan
        // triangles it happens to belong to. A vertex neighbouring the fan
        // anchor gets exactly one sliver triangle out of the whole face,
        // whose normal is the sliver's rather than the surface's; where two
        // faces meet along a symmetry seam the two slivers are mirror
        // images and cancel EXACTLY, leaving a valid manifold vertex with a
        // zero normal. That is observable on puppetA (chars/puppetA),
        // whose retopologised n-gons produce one such vertex.
        GfVec3f faceNormal(0);
        for (size_t k = 0; k < n; ++k) {
            const GfVec3f &p = points[ring[k]];
            const GfVec3f &q = points[ring[(k + 1) % n]];
            faceNormal += GfVec3f((p[1] - q[1]) * (p[2] + q[2]),
                                  (p[2] - q[2]) * (p[0] + q[0]),
                                  (p[0] - q[0]) * (p[1] + q[1]));
        }
        const float faceLength = faceNormal.GetLength();
        if (faceLength < 1e-20f) {
            continue;  // a fully degenerate face contributes nothing
        }
        faceNormal /= faceLength;

        // Weighted by the interior angle at each corner, so the result is
        // independent of tessellation and a vertex touching a face across
        // a wide corner is influenced by it more than one clipping a
        // corner. Zero-length edges fall back to an unweighted share
        // rather than dropping the corner's contribution entirely.
        for (size_t k = 0; k < n; ++k) {
            const GfVec3f &p = points[ring[k]];
            const GfVec3f a = points[ring[(k + 1) % n]] - p;
            const GfVec3f b = points[ring[(k + n - 1) % n]] - p;
            const float la = a.GetLength(), lb = b.GetLength();
            float weight = 1.0f;
            if (la > 1e-20f && lb > 1e-20f) {
                const GfVec3f ua = a / la, ub = b / lb;
                weight = std::atan2(GfCross(ua, ub).GetLength(),
                                    GfDot(ua, ub));
            }
            normals[ring[k]] += faceNormal * weight;
        }
    }
    for (GfVec3f &n : normals) {
        const float len = n.GetLength();
        if (len > 1e-12f) {
            n /= len;
        }
    }
    return normals;
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices)
{
    return RigExecComputeVertexNormals(
        points.data(), points.size(), faceVertexCounts.data(),
        faceVertexCounts.size(), faceVertexIndices.data(),
        faceVertexIndices.size());
}

std::vector<GfVec3f>
RigExecComputeExtent(
    const GfVec3f *points, size_t pointsSize,
    const float *widths, size_t widthsSize)
{
    if (pointsSize == 0 || !points) {
        return {};
    }
    // A null widths range reads as no widths, whatever its size claims.
    const size_t widthsCount = widths ? widthsSize : 0;
    GfVec3f lo = points[0], hi = points[0];
    for (size_t i = 0; i < pointsSize; ++i) {
        float pad = 0.0f;
        if (widthsCount == pointsSize) {
            pad = widths[i] * 0.5f;
        } else if (widthsCount == 1) {
            pad = widths[0] * 0.5f;
        }
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], points[i][a] - pad);
            hi[a] = std::max(hi[a], points[i][a] + pad);
        }
    }
    return {lo, hi};
}

std::vector<GfVec3f>
RigExecComputeExtent(
    const std::vector<GfVec3f> &points, const std::vector<float> &widths)
{
    return RigExecComputeExtent(
        points.data(), points.size(), widths.data(), widths.size());
}

void
RigExecApplyLattice(
    std::vector<GfVec3f> *points,
    const GfVec3f *restPoints, size_t restPointsSize,
    const GfVec3f *restCage, size_t restCageSize,
    const GfVec3f *posedCage, size_t posedCageSize,
    const GfVec3i &divisions)
{
    RigExecApplyLatticeKernel(
        points, restPoints, restPointsSize, restCage, restCageSize,
        posedCage, posedCageSize, divisions[0], divisions[1], divisions[2],
        static_cast<RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *>(nullptr));
}

void
RigExecApplyLattice(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &restPoints,
    const std::vector<GfVec3f> &restCage,
    const std::vector<GfVec3f> &posedCage,
    const GfVec3i &divisions)
{
    RigExecApplyLattice(
        points, restPoints.data(), restPoints.size(), restCage.data(),
        restCage.size(), posedCage.data(), posedCage.size(), divisions);
}

void
RigExecApplyLattice(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &restPoints,
    const std::vector<GfVec3f> &restCage,
    const std::vector<GfVec3f> &posedCage,
    const GfVec3i &divisions,
    RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache)
{
    RigExecApplyLatticeKernel(
        points, restPoints.data(), restPoints.size(), restCage.data(),
        restCage.size(), posedCage.data(), posedCage.size(), divisions[0],
        divisions[1], divisions[2], cache);
}

namespace {

GfVec3f
_ClosestPointOnTriangle(
    const GfVec3f &p, const GfVec3f &a, const GfVec3f &b, const GfVec3f &c)
{
    // Ericson, Real-Time Collision Detection, 5.1.5.
    const GfVec3f ab = b - a, ac = c - a, ap = p - a;
    const float d1 = GfDot(ab, ap), d2 = GfDot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const GfVec3f bp = p - b;
    const float d3 = GfDot(ab, bp), d4 = GfDot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const float v = d1 / (d1 - d3);
        return a + ab * v;
    }
    const GfVec3f cp = p - c;
    const float d5 = GfDot(ab, cp), d6 = GfDot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom, w = vc * denom;
    return a + ab * v + ac * w;
}

// Whether an owned key still describes a borrowed range: same size and
// same values. A null range only ever matches the empty key, and the
// projector returns before validation on a null range with a nonzero
// size, so every validated call below compares against real storage.
template <typename T>
bool
_RangeMatches(const std::vector<T> &key, const T *data, size_t size)
{
    return key.size() == size &&
        (size == 0 || std::equal(key.begin(), key.end(), data));
}

}  // namespace

void
RigExecBuildSurfaceAccel(
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize,
    const GfVec3f *surfacePoints, size_t surfacePointsSize,
    RigExecSurfaceAccel *accel)
{
    if (!accel) {
        return;
    }
    RigExecSurfaceAccel built;
    if (faceVertexCounts) {
        built.counts.assign(faceVertexCounts,
                            faceVertexCounts + faceVertexCountsSize);
    }
    if (faceVertexIndices) {
        built.indices.assign(faceVertexIndices,
                             faceVertexIndices + faceVertexIndicesSize);
    }
    if (surfacePoints) {
        built.points.assign(surfacePoints,
                            surfacePoints + surfacePointsSize);
    }
    built.fan = std::make_shared<const RigExecFanTrisH7>(
        RigExecBuildFanTrisH7(
            faceVertexCounts, faceVertexCountsSize,
            faceVertexIndices, faceVertexIndicesSize,
            surfacePointsSize));
    if (!built.fan->truncated &&
        built.fan->tris.size() >= kRigExecBvhMinTris &&
        (surfacePointsSize == 0 || surfacePoints)) {
        auto bvh = std::make_shared<RigExecTriangleBvh<GfVec3f>>();
        if (bvh->Build(surfacePoints, surfacePointsSize,
                       built.fan->tris.data(),
                       built.fan->tris.size())) {
            built.bvh = std::move(bvh);
        }
        // Build failed (non-finite surface points): the null bvh runs the
        // verbatim loop, whose NaN behavior is the defined one.
    }
    *accel = std::move(built);
}

void
RigExecBuildSurfaceAccel(
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    const std::vector<GfVec3f> &surfacePoints,
    RigExecSurfaceAccel *accel)
{
    RigExecBuildSurfaceAccel(
        faceVertexCounts.data(), faceVertexCounts.size(),
        faceVertexIndices.data(), faceVertexIndices.size(),
        surfacePoints.data(), surfacePoints.size(), accel);
}

void
RigExecApplySurfaceProject(
    std::vector<GfVec3f> *points,
    const GfVec3f *surfacePoints, size_t surfacePointsSize,
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize,
    double weight,
    const RigExecSurfaceAccel *accel)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (points->empty() || surfacePointsSize == 0 || w <= 0.0) {
        return;
    }
    if (!surfacePoints ||
        (faceVertexCountsSize > 0 && !faceVertexCounts) ||
        (faceVertexIndicesSize > 0 && !faceVertexIndices)) {
        return;  // a null range with a nonzero size: pass through
    }
    // One fan triangulation for the whole call instead of one per point.
    // A truncated walk returns with no point modified: the nested walk
    // below always meets the overrun during its first point.
    RigExecSurfaceAccel unretained;
    // A stale entry -- same cardinality but new topology or, on the
    // BVH path, new points -- rebuilds rather than answering wrong.
    // The fan only depends on the topology and the point count, so
    // below the gate the values are not compared: the verbatim loop
    // walks the current inputs either way.
    if (!accel || !accel->fan ||
        !_RangeMatches(accel->counts, faceVertexCounts,
                       faceVertexCountsSize) ||
        !_RangeMatches(accel->indices, faceVertexIndices,
                       faceVertexIndicesSize) ||
        accel->points.size() != surfacePointsSize ||
        (accel->bvh && !_RangeMatches(accel->points, surfacePoints,
                                      surfacePointsSize))) {
        RigExecBuildSurfaceAccel(
            faceVertexCounts, faceVertexCountsSize,
            faceVertexIndices, faceVertexIndicesSize,
            surfacePoints, surfacePointsSize, &unretained);
        accel = &unretained;
    }
    const RigExecFanTrisH7 &fan = *accel->fan;
    if (fan.truncated) {
        return;
    }
    if (fan.tris.size() >= kRigExecBvhMinTris &&
        points->size() >= kRigExecBvhMinQueries && accel->bvh) {
        for (GfVec3f &p : *points) {
            const GfVec3f best = accel->bvh->QueryNearest(
                p, surfacePoints, fan.tris.data(),
                _ClosestPointOnTriangle);
            p = p + (best - p) * float(w);
        }
        return;
    }
    for (GfVec3f &p : *points) {
        float bestDistSq = std::numeric_limits<float>::max();
        GfVec3f best = p;
        size_t offset = 0;
        for (size_t f = 0; f < faceVertexCountsSize; ++f) {
            const int faceCount = faceVertexCounts[f];
            for (int c = 1; c + 1 < faceCount; ++c) {
                const size_t i2 = offset + c + 1;
                if (i2 >= faceVertexIndicesSize) {
                    return;
                }
                const int ia = faceVertexIndices[offset];
                const int ib = faceVertexIndices[offset + c];
                const int ic = faceVertexIndices[i2];
                if (ia < 0 || ib < 0 || ic < 0 ||
                    size_t(ia) >= surfacePointsSize ||
                    size_t(ib) >= surfacePointsSize ||
                    size_t(ic) >= surfacePointsSize) {
                    continue;
                }
                const GfVec3f q = _ClosestPointOnTriangle(
                    p, surfacePoints[ia], surfacePoints[ib],
                    surfacePoints[ic]);
                const float distSq = (q - p).GetLengthSq();
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    best = q;
                }
            }
            offset += faceCount;
        }
        p = p + (best - p) * float(w);
    }
}

void
RigExecApplySurfaceProject(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &surfacePoints,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    double weight,
    const RigExecSurfaceAccel *accel)
{
    RigExecApplySurfaceProject(
        points, surfacePoints.data(), surfacePoints.size(),
        faceVertexCounts.data(), faceVertexCounts.size(),
        faceVertexIndices.data(), faceVertexIndices.size(), weight,
        accel);
}

namespace {

// Uniform cubic B-spline position on one nonperiodic segment span.
GfVec3f
_BsplinePoint(
    const GfVec3f &p0, const GfVec3f &p1, const GfVec3f &p2,
    const GfVec3f &p3, float t)
{
    const float t2 = t * t, t3 = t2 * t;
    const float b0 = (1 - 3 * t + 3 * t2 - t3) / 6.0f;
    const float b1 = (4 - 6 * t2 + 3 * t3) / 6.0f;
    const float b2 = (1 + 3 * t + 3 * t2 - 3 * t3) / 6.0f;
    const float b3 = t3 / 6.0f;
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

}  // namespace

RigExecCurveFrameSamples
RigExecSampleCurveRMF(
    const GfVec3f *controlPoints, size_t controlPointsSize,
    int sampleCount)
{
    RigExecCurveFrameSamples samples;
    // The short-curve branch below interpolates segments [i, i + 1].  A
    // single point has no segment; accepting it makes `size() - 2` equal -1
    // and indexes before the range.  Treat it like every other degenerate
    // driver and publish no frames.
    if (sampleCount < 2 || controlPointsSize < 2 || !controlPoints) {
        return samples;
    }

    // Dense presampling for arc-length parameterization. Under four
    // control points the stock cubic segment count is zero: the control
    // polygon interpolates linearly (deterministic short-curve rule).
    const int dense = std::max(sampleCount * 16, 64);
    std::vector<GfVec3f> densePoints;
    densePoints.reserve(dense + 1);
    const int spans = static_cast<int>(controlPointsSize) - 3;
    for (int i = 0; i <= dense; ++i) {
        const float u = float(i) / float(dense);
        if (spans >= 1) {
            const float s = u * spans;
            const int span = std::min(spans - 1, int(s));
            const float t = s - span;
            densePoints.push_back(_BsplinePoint(
                controlPoints[span], controlPoints[span + 1],
                controlPoints[span + 2], controlPoints[span + 3], t));
        } else {
            const float s = u * (controlPointsSize - 1);
            const int seg = std::min(
                int(controlPointsSize) - 2, int(s));
            densePoints.push_back(
                controlPoints[seg] +
                (controlPoints[seg + 1] - controlPoints[seg]) * (s - seg));
        }
    }
    std::vector<float> arcLength(densePoints.size(), 0.0f);
    for (size_t i = 1; i < densePoints.size(); ++i) {
        arcLength[i] = arcLength[i - 1] +
                       (densePoints[i] - densePoints[i - 1]).GetLength();
    }
    const float total = arcLength.back();
    if (total <= 1e-12f) {
        return samples;
    }

    // Sample at equal arc length.
    samples.positions.reserve(sampleCount);
    samples.parameters.reserve(sampleCount);
    size_t cursor = 0;
    for (int k = 0; k < sampleCount; ++k) {
        const float target = total * float(k) / float(sampleCount - 1);
        while (cursor + 1 < arcLength.size() &&
               arcLength[cursor + 1] < target) {
            ++cursor;
        }
        const float span = arcLength[cursor + 1] - arcLength[cursor];
        const float t = span > 1e-12f
            ? (target - arcLength[cursor]) / span : 0.0f;
        samples.positions.push_back(
            densePoints[cursor] +
            (densePoints[cursor + 1] - densePoints[cursor]) * t);
        samples.parameters.push_back(float(k) / float(sampleCount - 1));
    }

    // Tangents by central differences; rotation-minimizing normals by the
    // double-reflection method (Wang et al. 2008).
    samples.tangents.resize(sampleCount);
    for (int k = 0; k < sampleCount; ++k) {
        const GfVec3f &prev =
            samples.positions[std::max(0, k - 1)];
        const GfVec3f &next =
            samples.positions[std::min(sampleCount - 1, k + 1)];
        GfVec3f tangent = next - prev;
        const float len = tangent.GetLength();
        samples.tangents[k] =
            len > 1e-12f ? tangent / len : GfVec3f(1, 0, 0);
    }
    samples.normals.resize(sampleCount);
    samples.binormals.resize(sampleCount);
    // Deterministic initial normal: world axis least parallel to the
    // first tangent, projected (spec §5.3 ladder shape).
    {
        const GfVec3f t0 = samples.tangents[0];
        GfVec3f candidate(1, 0, 0);
        float best = 2.0f;
        for (const GfVec3f axis :
             {GfVec3f(1, 0, 0), GfVec3f(0, 1, 0), GfVec3f(0, 0, 1)}) {
            const float align = std::abs(GfDot(axis, t0));
            if (align < best) {
                best = align;
                candidate = axis;
            }
        }
        GfVec3f n0 = candidate - t0 * GfDot(t0, candidate);
        n0.Normalize();
        samples.normals[0] = n0;
        samples.binormals[0] = GfCross(t0, n0);
    }
    for (int k = 0; k + 1 < sampleCount; ++k) {
        const GfVec3f v1 = samples.positions[k + 1] - samples.positions[k];
        const float c1 = GfDot(v1, v1);
        if (c1 <= 1e-20f) {
            samples.normals[k + 1] = samples.normals[k];
            samples.binormals[k + 1] = samples.binormals[k];
            continue;
        }
        const GfVec3f nL =
            samples.normals[k] - v1 * (2.0f / c1) *
                GfDot(v1, samples.normals[k]);
        const GfVec3f tL =
            samples.tangents[k] - v1 * (2.0f / c1) *
                GfDot(v1, samples.tangents[k]);
        const GfVec3f v2 = samples.tangents[k + 1] - tL;
        const float c2 = GfDot(v2, v2);
        GfVec3f n = c2 > 1e-20f
            ? nL - v2 * (2.0f / c2) * GfDot(v2, nL) : nL;
        n -= samples.tangents[k + 1] * GfDot(samples.tangents[k + 1], n);
        const float len = n.GetLength();
        samples.normals[k + 1] =
            len > 1e-12f ? n / len : samples.normals[k];
        samples.binormals[k + 1] =
            GfCross(samples.tangents[k + 1], samples.normals[k + 1]);
    }
    return samples;
}

RigExecCurveFrameSamples
RigExecSampleCurveRMF(
    const std::vector<GfVec3f> &controlPoints, int sampleCount)
{
    return RigExecSampleCurveRMF(
        controlPoints.data(), controlPoints.size(), sampleCount);
}

void
RigExecApplyRibbonTransport(
    std::vector<GfVec3f> *points,
    const GfVec2f *bindCoords, size_t bindCoordsSize,
    const RigExecCurveFrameSamples &restSamples,
    const RigExecCurveFrameSamples &posedSamples)
{
    const size_t sampleCount = restSamples.GetSize();
    if (points->empty() || bindCoordsSize != points->size() ||
        sampleCount < 2 || posedSamples.GetSize() != sampleCount) {
        return;  // invalid binding: pass through
    }
    if (!bindCoords) {
        return;  // a null range with a nonzero size: pass through
    }

    // Per-sample rigid maps from the rest frame to the posed frame.
    std::vector<GfMatrix4d> maps(sampleCount);
    for (size_t k = 0; k < sampleCount; ++k) {
        const std::array<GfVec3d, 4> rest = {
            GfVec3d(restSamples.positions[k]),
            GfVec3d(restSamples.positions[k] + restSamples.tangents[k]),
            GfVec3d(restSamples.positions[k] + restSamples.normals[k]),
            GfVec3d(restSamples.positions[k] + restSamples.binormals[k])};
        const std::array<GfVec3d, 4> posed = {
            GfVec3d(posedSamples.positions[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.tangents[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.normals[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.binormals[k])};
        if (!RigExecPointsToMatrix(rest, posed, &maps[k])) {
            maps[k].SetIdentity();
        }
    }

    for (size_t i = 0; i < points->size(); ++i) {
        const float u = std::min(1.0f, std::max(0.0f, bindCoords[i][0]));
        const float s = u * float(sampleCount - 1);
        const size_t k = std::min(sampleCount - 2, size_t(s));
        const float t = s - float(k);
        const GfVec3d a = maps[k].TransformAffine(GfVec3d((*points)[i]));
        const GfVec3d b = maps[k + 1].TransformAffine(GfVec3d((*points)[i]));
        (*points)[i] = GfVec3f(a + (b - a) * double(t));
    }
}

void
RigExecApplyRibbonTransport(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec2f> &bindCoords,
    const RigExecCurveFrameSamples &restSamples,
    const RigExecCurveFrameSamples &posedSamples)
{
    RigExecApplyRibbonTransport(
        points, bindCoords.data(), bindCoords.size(), restSamples,
        posedSamples);
}

}  // namespace rigExec

namespace rigExec {

bool
RigExecNurbsCurve::IsValid() const
{
    if (!points || !knots || order < 2) {
        return false;
    }
    const size_t n = points->size();
    return order <= 16 && n >= size_t(order) &&
           knots->size() == n + size_t(order) &&
           (*knots)[size_t(order) - 1] < (*knots)[n];
}

double
RigExecNurbsCurve::DomainStart() const
{
    return (*knots)[size_t(order) - 1];
}

double
RigExecNurbsCurve::DomainEnd() const
{
    return (*knots)[points->size()];
}

GfVec3f
RigExecNurbsCurve::Evaluate(double u) const
{
    const std::vector<double> &k = *knots;
    const std::vector<GfVec3f> &cv = *points;
    const int p = order - 1;
    const size_t n = cv.size();
    u = std::min(std::max(u, DomainStart()), DomainEnd());
    // The span: largest s in [p, n-1] with k[s] <= u < k[s+1]; the domain
    // end belongs to the last non-empty span.
    size_t s = size_t(p);
    while (s + 1 < n && k[s + 1] <= u) {
        ++s;
    }
    // de Boor over the order control points of that span.
    GfVec3d d[16];
    const int count = std::min(order, 16);
    for (int j = 0; j < count; ++j) {
        d[j] = GfVec3d(cv[s - size_t(p) + size_t(j)]);
    }
    for (int r = 1; r <= p && r < 16; ++r) {
        for (int j = p; j >= r; --j) {
            const size_t i = s - size_t(p) + size_t(j);
            const double denom = k[i + size_t(p) + 1 - size_t(r)] - k[i];
            const double a = denom > 0.0 ? (u - k[i]) / denom : 0.0;
            d[j] = d[j - 1] * (1.0 - a) + d[j] * a;
        }
    }
    return GfVec3f(d[p]);
}

bool
RigExecBuildWireBasis(const GfVec2f *bindCoords, size_t bindCount,
                      size_t meshPointCount, const std::vector<int> &indices,
                      int order, const std::vector<double> &knots,
                      size_t controlPointCount, double dropoffDistance,
                      RigExecWireBasis *basis)
{
    const size_t n = controlPointCount;
    if (!basis ||
        !RigExecWireBasisInputsAreUsable(bindCoords, bindCount, meshPointCount,
                                         indices.size(), order, knots, n)) {
        return false;
    }
    const bool parallel = bindCount == indices.size();
    const int p = order - 1;
    const double u0 = knots[size_t(p)];
    const double u1 = knots[n];
    basis->byControlPoint.assign(n, {});
    double left[16], right[16], N[16];
    for (size_t k = 0; k < indices.size(); ++k) {
        const int index = indices[k];
        if (index < 0 || size_t(index) >= meshPointCount) {
            return false;
        }
        const GfVec2f &bind = bindCoords[parallel ? k : size_t(index)];
        double f = 1.0;
        if (dropoffDistance > 0.0) {
            const double s =
                std::min(std::max(double(bind[1]) / dropoffDistance, 0.0), 1.0);
            f = 1.0 - s * s * (3.0 - 2.0 * s);
        }
        if (f <= 0.0) {
            continue;
        }
        const double u = std::min(std::max(double(bind[0]), u0), u1);
        // The span exactly as RigExecNurbsCurve::Evaluate finds it.
        size_t span = size_t(p);
        while (span + 1 < n && knots[span + 1] <= u) {
            ++span;
        }
        // The order nonzero basis functions at u (Piegl and Tiller, A2.2).
        N[0] = 1.0;
        for (int j = 1; j <= p; ++j) {
            left[j] = u - knots[span + 1 - size_t(j)];
            right[j] = knots[span + size_t(j)] - u;
            double saved = 0.0;
            for (int r = 0; r < j; ++r) {
                const double denom = right[r + 1] + left[j - r];
                const double temp = denom != 0.0 ? N[r] / denom : 0.0;
                N[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            N[j] = saved;
        }
        for (int r = 0; r <= p; ++r) {
            const double c = f * N[r];
            if (c != 0.0) {
                basis->byControlPoint[span - size_t(p) + size_t(r)].emplace_back(
                    uint32_t(k), float(c));
            }
        }
    }
    return true;
}

bool
RigExecWireBasisInputsAreUsable(const GfVec2f *bindCoords, size_t bindCount,
                                size_t meshPointCount, size_t indexCount,
                                int order, const std::vector<double> &knots,
                                size_t controlPointCount)
{
    const size_t n = controlPointCount;
    return bindCoords && order >= 1 && order <= 16 && n >= size_t(order) &&
           knots.size() == n + size_t(order) &&
           knots[size_t(order - 1)] < knots[n] &&
           (bindCount == meshPointCount || bindCount == indexCount);
}

bool
RigExecApplyWireBasis(std::vector<GfVec3f> *points,
                      const RigExecWireBasis &basis,
                      const std::vector<int> &indices,
                      const std::vector<float> &weights,
                      const std::vector<GfVec3f> &restControlPoints,
                      const std::vector<GfVec3f> &posedControlPoints)
{
    return RigExecApplyWireBasisRange(points, basis, indices, weights,
                                      restControlPoints, posedControlPoints,
                                      0, points ? points->size() : 0);
}

bool
RigExecApplyWireBasisRange(std::vector<GfVec3f> *points,
                           const RigExecWireBasis &basis,
                           const std::vector<int> &indices,
                           const std::vector<float> &weights,
                           const std::vector<GfVec3f> &restControlPoints,
                           const std::vector<GfVec3f> &posedControlPoints,
                           size_t begin, size_t end)
{
    if (!points) {
        return false;
    }
    end = std::min(end, points->size());
    begin = std::min(begin, end);
    return RigExecApplyWireBasisGroup(points->data() + begin, begin, end,
                                      basis, indices, weights,
                                      restControlPoints, posedControlPoints);
}

bool
RigExecApplyWireBasisGroup(GfVec3f *out, size_t begin, size_t end,
                           const RigExecWireBasis &basis,
                           const std::vector<int> &indices,
                           const std::vector<float> &weights,
                           const std::vector<GfVec3f> &restControlPoints,
                           const std::vector<GfVec3f> &posedControlPoints)
{
    const size_t n = basis.byControlPoint.size();
    if ((!out && begin < end) || restControlPoints.size() != n ||
        posedControlPoints.size() != n || indices.size() != weights.size()) {
        return false;
    }
    // Control-point-major, as the whole call: a point's additions arrive in
    // control point order whichever range it is applied in.
    for (size_t j = 0; j < n; ++j) {
        const GfVec3f delta = posedControlPoints[j] - restControlPoints[j];
        if (delta == GfVec3f(0.0f)) {
            continue;  // a control point at rest moves nothing
        }
        for (const auto &[k, coefficient] : basis.byControlPoint[j]) {
            const int index = indices[k];
            if (index < 0 || size_t(index) < begin || size_t(index) >= end) {
                continue;
            }
            out[size_t(index) - begin] += delta * (coefficient * weights[k]);
        }
    }
    return true;
}

std::vector<GfVec2f>
RigExecBindWire(const std::vector<GfVec3f> &points,
                const RigExecNurbsCurve &restCurve)
{
    std::vector<GfVec2f> out(points.size(), GfVec2f(0.0f, 0.0f));
    if (!restCurve.IsValid()) {
        return out;
    }
    const double u0 = restCurve.DomainStart();
    const double u1 = restCurve.DomainEnd();
    const size_t spans = std::max<size_t>(
        1, restCurve.points->size() - size_t(restCurve.order) + 1);
    const size_t samples = spans * 32;
    std::vector<GfVec3f> table(samples + 1);
    for (size_t i = 0; i <= samples; ++i) {
        table[i] = restCurve.Evaluate(u0 + (u1 - u0) * double(i) / samples);
    }
    const double step = (u1 - u0) / double(samples);
    for (size_t pi = 0; pi < points.size(); ++pi) {
        const GfVec3f &p = points[pi];
        size_t best = 0;
        float bestSq = std::numeric_limits<float>::max();
        for (size_t i = 0; i <= samples; ++i) {
            const float dSq = (table[i] - p).GetLengthSq();
            if (dSq < bestSq) {
                bestSq = dSq;
                best = i;
            }
        }
        // Ternary refinement inside the neighbouring samples.
        double lo = std::max(u0, u0 + step * (double(best) - 1.0));
        double hi = std::min(u1, u0 + step * (double(best) + 1.0));
        for (int it = 0; it < 40; ++it) {
            const double m1 = lo + (hi - lo) / 3.0;
            const double m2 = hi - (hi - lo) / 3.0;
            if ((restCurve.Evaluate(m1) - p).GetLengthSq() <
                (restCurve.Evaluate(m2) - p).GetLengthSq()) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        const double u = 0.5 * (lo + hi);
        out[pi] = GfVec2f(float(u),
                          float((restCurve.Evaluate(u) - p).GetLength()));
    }
    return out;
}

bool
RigExecApplyWire(std::vector<GfVec3f> *points,
                 const RigExecNurbsCurve &restCurve,
                 const RigExecNurbsCurve &posedCurve,
                 const GfVec2f *bindCoords, size_t bindCount,
                 double dropoffDistance, size_t begin, size_t end,
                 const GfVec3f *restEvals, size_t restEvalCount)
{
    if (!points) {
        return false;
    }
    end = std::min(end, points->size());
    begin = std::min(begin, end);
    return RigExecApplyWireGroup(points->data() + begin, begin, end,
                                 points->size(), restCurve, posedCurve,
                                 bindCoords, bindCount, dropoffDistance,
                                 restEvals, restEvalCount);
}

bool
RigExecApplyWireGroup(GfVec3f *out, size_t begin, size_t end, size_t count,
                      const RigExecNurbsCurve &restCurve,
                      const RigExecNurbsCurve &posedCurve,
                      const GfVec2f *bindCoords, size_t bindCount,
                      double dropoffDistance, const GfVec3f *restEvals,
                      size_t restEvalCount)
{
    if (!RigExecWireInputsAreUsable(restCurve, posedCurve, bindCoords,
                                    bindCount, count) ||
        (restEvals && restEvalCount != count)) {
        return false;
    }
    end = std::min(end, count);
    if (begin < end && !out) {
        return false;
    }
    for (size_t i = begin; i < end; ++i) {
        const double u = bindCoords[i][0];
        const double d = bindCoords[i][1];
        double f = 1.0;
        if (dropoffDistance > 0.0) {
            const double s = std::min(std::max(d / dropoffDistance, 0.0), 1.0);
            f = 1.0 - s * s * (3.0 - 2.0 * s);
        }
        if (f <= 0.0) {
            continue;
        }
        const GfVec3f rest = restEvals ? restEvals[i] : restCurve.Evaluate(u);
        const GfVec3f delta = posedCurve.Evaluate(u) - rest;
        out[i - begin] += delta * float(f);
    }
    return true;
}

bool
RigExecWireInputsAreUsable(const RigExecNurbsCurve &restCurve,
                           const RigExecNurbsCurve &posedCurve,
                           const GfVec2f *bindCoords, size_t bindCount,
                           size_t pointCount)
{
    // The knot vectors compare by value, so a NaN knot fails even when both
    // curves share one vector.
    return bindCoords && restCurve.IsValid() && posedCurve.IsValid() &&
           restCurve.order == posedCurve.order &&
           restCurve.points->size() == posedCurve.points->size() &&
           !(*restCurve.knots != *posedCurve.knots) &&
           bindCount == pointCount;
}

bool
RigExecApplyWireSparse(std::vector<GfVec3f> *points,
                       const RigExecNurbsCurve &restCurve,
                       const RigExecNurbsCurve &posedCurve,
                       const GfVec2f *bindCoords, size_t bindCount,
                       double dropoffDistance,
                       const std::vector<int> &indices,
                       const std::vector<float> &weights)
{
    if (!points || !bindCoords || !restCurve.IsValid() ||
        !posedCurve.IsValid() ||
        restCurve.order != posedCurve.order ||
        restCurve.points->size() != posedCurve.points->size() ||
        *restCurve.knots != *posedCurve.knots ||
        (bindCount != points->size() && bindCount != indices.size()) ||
        indices.size() != weights.size()) {
        return false;
    }
    const bool parallel = bindCount == indices.size();
    for (size_t k = 0; k < indices.size(); ++k) {
        const size_t i = size_t(indices[k]);
        const GfVec2f &bind = bindCoords[parallel ? k : i];
        const double u = bind[0];
        const double d = bind[1];
        double f = weights[k];
        if (dropoffDistance > 0.0) {
            const double s = std::min(std::max(d / dropoffDistance, 0.0), 1.0);
            f *= 1.0 - s * s * (3.0 - 2.0 * s);
        }
        if (f <= 0.0) {
            continue;
        }
        const GfVec3f delta = posedCurve.Evaluate(u) - restCurve.Evaluate(u);
        (*points)[i] += delta * float(f);
    }
    return true;
}

}  // namespace rigExec
