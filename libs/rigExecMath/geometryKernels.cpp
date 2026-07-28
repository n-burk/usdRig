//
// RigExec geometry mover kernels implementation.
//
#include "geometryKernels.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

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
    size_t offset = 0;
    for (int faceCount : faceVertexCounts) {
        for (int c = 1; c + 1 < faceCount; ++c) {
            const size_t i0 = offset, i1 = offset + c, i2 = offset + c + 1;
            if (i2 >= faceVertexIndices.size()) {
                return normals;
            }
            const int a = faceVertexIndices[i0];
            const int b = faceVertexIndices[i1];
            const int d = faceVertexIndices[i2];
            if (a < 0 || b < 0 || d < 0 ||
                static_cast<size_t>(a) >= points.size() ||
                static_cast<size_t>(b) >= points.size() ||
                static_cast<size_t>(d) >= points.size()) {
                continue;
            }
            const GfVec3f n =
                GfCross(points[b] - points[a], points[d] - points[a]);
            normals[a] += n;
            normals[b] += n;
            normals[d] += n;
        }
        offset += faceCount;
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
    if (sampleCount < 2 || controlPoints.empty()) {
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
