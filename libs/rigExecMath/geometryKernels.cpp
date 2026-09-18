//
// RigExec geometry mover kernels implementation.
//
#include "geometryKernels.h"
#include "pxr/base/gf/vec3d.h"

#include <algorithm>
#include <cmath>
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

namespace {

// Edge adjacency (unique undirected edges) from standard polygon topology.
std::vector<std::vector<int>>
_BuildAdjacency(
    size_t pointCount,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices)
{
    std::vector<std::set<int>> adjacency(pointCount);
    size_t offset = 0;
    for (int faceCount : faceVertexCounts) {
        for (int c = 0; c < faceCount; ++c) {
            const size_t ia = offset + c;
            const size_t ib = offset + (c + 1) % faceCount;
            if (ia >= faceVertexIndices.size() ||
                ib >= faceVertexIndices.size()) {
                return {};
            }
            const int a = faceVertexIndices[ia];
            const int b = faceVertexIndices[ib];
            if (a < 0 || b < 0 ||
                static_cast<size_t>(a) >= pointCount ||
                static_cast<size_t>(b) >= pointCount) {
                return {};
            }
            adjacency[a].insert(b);
            adjacency[b].insert(a);
        }
        offset += faceCount;
    }
    std::vector<std::vector<int>> result(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        result[i].assign(adjacency[i].begin(), adjacency[i].end());
    }
    return result;
}

}  // namespace

bool
RigExecTransportSurfaceOffsets(
    const std::vector<GfVec3f> &rest,
    const std::vector<GfVec3f> &posed,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    const std::vector<GfVec3f> &deltas,
    std::vector<GfVec3f> *out)
{
    if (!out || rest.size() != posed.size() || rest.size() != deltas.size()) return false;
    for (size_t i = 0; i < rest.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(rest[i][axis]) || !std::isfinite(posed[i][axis]) ||
                !std::isfinite(deltas[i][axis])) return false;
        }
    }
    size_t offset = 0;
    std::vector<GfVec3d> restNormals(rest.size(), GfVec3d(0));
    std::vector<GfVec3d> posedNormals(rest.size(), GfVec3d(0));
    for (int count : faceCounts) {
        if (count < 3 || static_cast<size_t>(count) > faceIndices.size() - offset) return false;
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            if (index < 0 || static_cast<size_t>(index) >= rest.size()) return false;
        }
        // Newell's area vector, evaluated relative to the first corner to
        // avoid subtracting products of large translated coordinates.
        const int origin = faceIndices[offset];
        GfVec3d restArea(0), posedArea(0);
        for (int corner = 1; corner + 1 < count; ++corner) {
            const int a = faceIndices[offset + corner];
            const int b = faceIndices[offset + corner + 1];
            restArea += GfCross(GfVec3d(rest[a]) - GfVec3d(rest[origin]),
                                GfVec3d(rest[b]) - GfVec3d(rest[origin]));
            posedArea += GfCross(GfVec3d(posed[a]) - GfVec3d(posed[origin]),
                                 GfVec3d(posed[b]) - GfVec3d(posed[origin]));
        }
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            restNormals[index] += restArea;
            posedNormals[index] += posedArea;
        }
        offset += static_cast<size_t>(count);
    }
    if (offset != faceIndices.size()) return false;
    const auto adjacency = _BuildAdjacency(rest.size(), faceCounts, faceIndices);
    auto normalize = [](GfVec3d *value) {
        const double length = value->GetLength();
        if (!(length > 0) || !std::isfinite(length)) return false;
        *value /= length;
        return true;
    };
    std::vector<GfVec3f> result = deltas;
    for (size_t i = 0; i < rest.size(); ++i) {
        if (deltas[i] == GfVec3f(0)) continue;
        GfVec3d nr = restNormals[i], np = posedNormals[i];
        if (!normalize(&nr) || !normalize(&np)) return false;
        int neighbor = -1;
        double longest = 0;
        GfVec3d tr(0);
        // _BuildAdjacency orders neighbors by index, making equal-length
        // choices independent of face order and corner traversal direction.
        for (int candidate : adjacency[i]) {
            const GfVec3d edge = GfVec3d(rest[candidate]) - GfVec3d(rest[i]);
            const GfVec3d projected = edge - GfDot(edge, nr) * nr;
            const double length2 = projected.GetLengthSq();
            if (length2 > longest) {
                neighbor = candidate;
                longest = length2;
                tr = projected;
            }
        }
        if (neighbor < 0 || !normalize(&tr)) return false;
        const GfVec3d edge = GfVec3d(posed[neighbor]) - GfVec3d(posed[i]);
        GfVec3d tp = edge - GfDot(edge, np) * np;
        if (tp.GetLengthSq() <= edge.GetLengthSq() * 1e-24 || !normalize(&tp)) return false;
        const GfVec3d br = GfCross(nr, tr), bp = GfCross(np, tp);
        const GfVec3d delta(deltas[i]);
        const GfVec3d rotated = GfDot(delta, tr) * tp + GfDot(delta, br) * bp + GfDot(delta, nr) * np;
        result[i] = GfVec3f(rotated);
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(result[i][axis])) return false;
        }
    }
    *out = std::move(result);
    return true;
}

void
RigExecApplyLaplacianSmooth(
    std::vector<GfVec3f> *points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    double strength)
{
    const double s = std::min(std::max(strength, 0.0), 1.0);
    if (points->empty() || s <= 0.0) {
        return;
    }
    const std::vector<std::vector<int>> adjacency = _BuildAdjacency(
        points->size(), faceVertexCounts, faceVertexIndices);
    if (adjacency.empty()) {
        return;  // invalid topology: pass through
    }
    const std::vector<GfVec3f> source = *points;
    for (size_t i = 0; i < source.size(); ++i) {
        if (adjacency[i].empty()) {
            continue;
        }
        GfVec3f average(0);
        for (int n : adjacency[i]) {
            average += source[n];
        }
        average /= float(adjacency[i].size());
        (*points)[i] = source[i] + (average - source[i]) * float(s);
    }
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices)
{
    std::vector<GfVec3f> normals(points.size(), GfVec3f(0));
    std::vector<int> ring;
    size_t offset = 0;
    for (int faceCount : faceVertexCounts) {
        if (faceCount < 3 ||
            offset + static_cast<size_t>(faceCount) >
                faceVertexIndices.size()) {
            return normals;
        }
        // Gather the face once, dropping indices that do not address a
        // point rather than letting them into the arithmetic.
        ring.clear();
        for (int k = 0; k < faceCount; ++k) {
            const int v = faceVertexIndices[offset + k];
            if (v >= 0 && static_cast<size_t>(v) < points.size()) {
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
        //
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
RigExecComputeExtent(
    const std::vector<GfVec3f> &points, const std::vector<float> &widths)
{
    if (points.empty()) {
        return {};
    }
    GfVec3f lo = points[0], hi = points[0];
    for (size_t i = 0; i < points.size(); ++i) {
        float pad = 0.0f;
        if (widths.size() == points.size()) {
            pad = widths[i] * 0.5f;
        } else if (widths.size() == 1) {
            pad = widths[0] * 0.5f;
        }
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], points[i][a] - pad);
            hi[a] = std::max(hi[a], points[i][a] + pad);
        }
    }
    return {lo, hi};
}

namespace {

double
_Bernstein(int degree, int index, double t)
{
    double coefficient = 1.0;
    for (int k = 0; k < index; ++k) {
        coefficient *= double(degree - k) / double(index - k);
    }
    return coefficient * std::pow(t, index) *
           std::pow(1.0 - t, degree - index);
}

}  // namespace

void
RigExecApplyLattice(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &restPoints,
    const std::vector<GfVec3f> &restCage,
    const std::vector<GfVec3f> &posedCage,
    const GfVec3i &divisions)
{
    const size_t cageCount = size_t(divisions[0]) * size_t(divisions[1]) *
                             size_t(divisions[2]);
    if (points->empty() || restPoints.size() != points->size() ||
        restCage.size() != cageCount || posedCage.size() != cageCount ||
        divisions[0] < 2 || divisions[1] < 2 || divisions[2] < 2) {
        return;  // invalid cage description: pass through
    }

    // Rest cage bound defines the bind space.
    GfVec3f lo = restCage[0], hi = restCage[0];
    for (const GfVec3f &c : restCage) {
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], c[a]);
            hi[a] = std::max(hi[a], c[a]);
        }
    }
    const GfVec3f size = hi - lo;
    if (size[0] <= 0 || size[1] <= 0 || size[2] <= 0) {
        return;
    }

    // Cage deltas preserve identity when the cage is at rest.
    std::vector<GfVec3f> cageDeltas(cageCount);
    for (size_t i = 0; i < cageCount; ++i) {
        cageDeltas[i] = posedCage[i] - restCage[i];
    }

    const int dx = divisions[0], dy = divisions[1], dz = divisions[2];
    for (size_t i = 0; i < points->size(); ++i) {
        // Bind coordinates from the REST point, clamped into the cage.
        GfVec3f uvw;
        for (int a = 0; a < 3; ++a) {
            uvw[a] = std::min(
                1.0f, std::max(0.0f, (restPoints[i][a] - lo[a]) / size[a]));
        }
        GfVec3f delta(0);
        for (int c = 0; c < dz; ++c) {
            const double bc = _Bernstein(dz - 1, c, uvw[2]);
            for (int b = 0; b < dy; ++b) {
                const double bb = _Bernstein(dy - 1, b, uvw[1]);
                for (int a = 0; a < dx; ++a) {
                    const double ba = _Bernstein(dx - 1, a, uvw[0]);
                    delta += cageDeltas[(c * dy + b) * dx + a] *
                             float(ba * bb * bc);
                }
            }
        }
        (*points)[i] += delta;
    }
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

}  // namespace

void
RigExecApplySurfaceProject(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &surfacePoints,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    double weight)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (points->empty() || surfacePoints.empty() || w <= 0.0) {
        return;
    }
    for (GfVec3f &p : *points) {
        float bestDistSq = std::numeric_limits<float>::max();
        GfVec3f best = p;
        size_t offset = 0;
        for (int faceCount : faceVertexCounts) {
            for (int c = 1; c + 1 < faceCount; ++c) {
                const size_t i2 = offset + c + 1;
                if (i2 >= faceVertexIndices.size()) {
                    return;
                }
                const int ia = faceVertexIndices[offset];
                const int ib = faceVertexIndices[offset + c];
                const int ic = faceVertexIndices[i2];
                if (ia < 0 || ib < 0 || ic < 0 ||
                    size_t(ia) >= surfacePoints.size() ||
                    size_t(ib) >= surfacePoints.size() ||
                    size_t(ic) >= surfacePoints.size()) {
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
    const std::vector<GfVec3f> &controlPoints, int sampleCount)
{
    RigExecCurveFrameSamples samples;
    // The short-curve branch below interpolates segments [i, i + 1].  A
    // single point has no segment; accepting it makes `size() - 2` equal -1
    // and indexes before the vector.  Treat it like every other degenerate
    // driver and publish no frames.
    if (sampleCount < 2 || controlPoints.size() < 2) {
        return samples;
    }

    // Dense presampling for arc-length parameterization. Under four
    // control points the stock cubic segment count is zero: the control
    // polygon interpolates linearly (deterministic short-curve rule).
    const int dense = std::max(sampleCount * 16, 64);
    std::vector<GfVec3f> densePoints;
    densePoints.reserve(dense + 1);
    const int spans = static_cast<int>(controlPoints.size()) - 3;
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
            const float s = u * (controlPoints.size() - 1);
            const int seg = std::min(
                int(controlPoints.size()) - 2, int(s));
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

void
RigExecApplyRibbonTransport(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec2f> &bindCoords,
    const RigExecCurveFrameSamples &restSamples,
    const RigExecCurveFrameSamples &posedSamples)
{
    const size_t sampleCount = restSamples.GetSize();
    if (points->empty() || bindCoords.size() != points->size() ||
        sampleCount < 2 || posedSamples.GetSize() != sampleCount) {
        return;  // invalid binding: pass through
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
    if (!basis || !bindCoords || order < 1 || order > 16 || n < size_t(order) ||
        knots.size() != n + size_t(order) ||
        !(knots[size_t(order - 1)] < knots[n]) ||
        (bindCount != meshPointCount && bindCount != indices.size())) {
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
RigExecApplyWireBasis(std::vector<GfVec3f> *points,
                      const RigExecWireBasis &basis,
                      const std::vector<int> &indices,
                      const std::vector<float> &weights,
                      const std::vector<GfVec3f> &restControlPoints,
                      const std::vector<GfVec3f> &posedControlPoints)
{
    const size_t n = basis.byControlPoint.size();
    if (!points || restControlPoints.size() != n ||
        posedControlPoints.size() != n || indices.size() != weights.size()) {
        return false;
    }
    GfVec3f *data = points->data();
    for (size_t j = 0; j < n; ++j) {
        const GfVec3f delta = posedControlPoints[j] - restControlPoints[j];
        if (delta == GfVec3f(0.0f)) {
            continue;  // a control point at rest moves nothing
        }
        for (const auto &[k, coefficient] : basis.byControlPoint[j]) {
            data[size_t(indices[k])] += delta * (coefficient * weights[k]);
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
                 double dropoffDistance, size_t begin, size_t end)
{
    if (!points || !bindCoords || !restCurve.IsValid() ||
        !posedCurve.IsValid() ||
        restCurve.order != posedCurve.order ||
        restCurve.points->size() != posedCurve.points->size() ||
        *restCurve.knots != *posedCurve.knots ||
        bindCount != points->size()) {
        return false;
    }
    end = std::min(end, points->size());
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
        const GfVec3f delta = posedCurve.Evaluate(u) - restCurve.Evaluate(u);
        (*points)[i] += delta * float(f);
    }
    return true;
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
