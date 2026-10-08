// Method reference: Mancewicz et al. (2014), https://doi.org/10.1145/2614106.2614144
// Shared, USD-free geometry math. Both USD and binary adapters instantiate
// these kernels; Point/Wide only supply float/double vector storage and arithmetic.
#ifndef RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#define RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#include "deltaMushSettings.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {
namespace deltaMushDetail {
template<class To, class From> To Convert(const From &p) {
    return To(p[0], p[1], p[2]);
}
template<class V> double Dot(const V &a, const V &b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
template<class V> V Cross(const V &a, const V &b) {
    return V(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]);
}
inline std::vector<std::vector<int>>
BuildAdjacency(
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

} // namespace deltaMushDetail

template<class Point, class Wide>
bool
RigExecTransportSurfaceOffsetsKernel(
    const std::vector<Point> &rest,
    const std::vector<Point> &posed,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    const std::vector<Point> &deltas,
    std::vector<Point> *out)
{
    if (!out || rest.size() != posed.size() || rest.size() != deltas.size()) return false;
    for (size_t i = 0; i < rest.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(rest[i][axis]) || !std::isfinite(posed[i][axis]) ||
                !std::isfinite(deltas[i][axis])) return false;
        }
    }
    size_t offset = 0;
    std::vector<Wide> restNormals(rest.size(), Wide(0));
    std::vector<Wide> posedNormals(rest.size(), Wide(0));
    for (int count : faceCounts) {
        if (count < 3 || static_cast<size_t>(count) > faceIndices.size() - offset) return false;
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            if (index < 0 || static_cast<size_t>(index) >= rest.size()) return false;
        }
        // Newell's area vector, evaluated relative to the first corner to
        // avoid subtracting products of large translated coordinates.
        const int origin = faceIndices[offset];
        Wide restArea(0), posedArea(0);
        for (int corner = 1; corner + 1 < count; ++corner) {
            const int a = faceIndices[offset + corner];
            const int b = faceIndices[offset + corner + 1];
            restArea += deltaMushDetail::Cross(deltaMushDetail::Convert<Wide>(rest[a]) - deltaMushDetail::Convert<Wide>(rest[origin]),
                                deltaMushDetail::Convert<Wide>(rest[b]) - deltaMushDetail::Convert<Wide>(rest[origin]));
            posedArea += deltaMushDetail::Cross(deltaMushDetail::Convert<Wide>(posed[a]) - deltaMushDetail::Convert<Wide>(posed[origin]),
                                 deltaMushDetail::Convert<Wide>(posed[b]) - deltaMushDetail::Convert<Wide>(posed[origin]));
        }
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            restNormals[index] += restArea;
            posedNormals[index] += posedArea;
        }
        offset += static_cast<size_t>(count);
    }
    if (offset != faceIndices.size()) return false;
    const auto adjacency = deltaMushDetail::BuildAdjacency(rest.size(), faceCounts, faceIndices);
    auto normalize = [](Wide *value) {
        const double length = value->GetLength();
        if (!(length > 0) || !std::isfinite(length)) return false;
        *value /= length;
        return true;
    };
    std::vector<Point> result = deltas;
    for (size_t i = 0; i < rest.size(); ++i) {
        if (deltas[i] == Point(0)) continue;
        Wide nr = restNormals[i], np = posedNormals[i];
        if (!normalize(&nr) || !normalize(&np)) return false;
        int neighbor = -1;
        double longest = 0;
        Wide tr(0);
        // deltaMushDetail::BuildAdjacency orders neighbors by index, making equal-length
        // choices independent of face order and corner traversal direction.
        for (int candidate : adjacency[i]) {
            const Wide edge = deltaMushDetail::Convert<Wide>(rest[candidate]) - deltaMushDetail::Convert<Wide>(rest[i]);
            const Wide projected = edge - deltaMushDetail::Dot(edge, nr) * nr;
            const double length2 = projected.GetLengthSq();
            if (length2 > longest) {
                neighbor = candidate;
                longest = length2;
                tr = projected;
            }
        }
        if (neighbor < 0 || !normalize(&tr)) return false;
        const Wide edge = deltaMushDetail::Convert<Wide>(posed[neighbor]) - deltaMushDetail::Convert<Wide>(posed[i]);
        Wide tp = edge - deltaMushDetail::Dot(edge, np) * np;
        if (tp.GetLengthSq() <= edge.GetLengthSq() * 1e-24 || !normalize(&tp)) return false;
        const Wide br = deltaMushDetail::Cross(nr, tr), bp = deltaMushDetail::Cross(np, tp);
        const Wide delta = deltaMushDetail::Convert<Wide>(deltas[i]);
        const Wide rotated = deltaMushDetail::Dot(delta, tr) * tp + deltaMushDetail::Dot(delta, br) * bp + deltaMushDetail::Dot(delta, nr) * np;
        result[i] = deltaMushDetail::Convert<Point>(rotated);
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(result[i][axis])) return false;
        }
    }
    *out = std::move(result);
    return true;
}

// Corner tangent transport reference: Blender MOD_correctivesmooth.cc,
// calc_tangent_spaces; see docs/references.md for provenance.
template<class Point, class Wide>
bool
RigExecTransportCornerOffsetsKernel(
    const std::vector<Point> &rest, const std::vector<Point> &posed,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const std::vector<Point> &deltas, std::vector<Point> *out)
{
    if (!out || rest.size() != posed.size() || rest.size() != deltas.size()) return false;
    struct Basis { Wide x{0}, y{0}, z{0}; double weight = 0; };
    const auto basis = [](const std::vector<Point> &points, int previous, int vertex, int next) {
        const auto normalized = [](Wide v) {
            const double length = v.GetLength();
            return length > 0 ? v / length : Wide(0);
        };
        const Wide before = normalized(deltaMushDetail::Convert<Wide>(points[previous]) - deltaMushDetail::Convert<Wide>(points[vertex]));
        const Wide after = normalized(deltaMushDetail::Convert<Wide>(points[vertex]) - deltaMushDetail::Convert<Wide>(points[next]));
        Basis result;
        bool equal = true;
        for (int a = 0; a < 3; ++a)
            equal = equal && std::abs(before[a] - after[a]) <= std::numeric_limits<float>::epsilon() * 10;
        if (equal) { result.x = Wide(1,0,0); result.y = Wide(0,1,0); result.z = Wide(0,0,1); return result; }
        result.z = normalized(deltaMushDetail::Cross(before, after));
        result.x = normalized(before + after);
        result.y = deltaMushDetail::Cross(result.z, result.x);
        result.weight = std::acos(std::max(-1.0, std::min(1.0, deltaMushDetail::Dot(before, after))));
        return result;
    };
    std::vector<Basis> restBasis, posedBasis;
    std::vector<double> sums(rest.size(), 0);
    size_t offset = 0;
    for (int count : counts) {
        if (count < 3 || size_t(count) > indices.size() - offset) return false;
        for (int c = 0; c < count; ++c) {
            const int previous = indices[offset + (c+count-1)%count];
            const int vertex = indices[offset+c], next = indices[offset+(c+1)%count];
            if (previous < 0 || vertex < 0 || next < 0 || size_t(previous) >= rest.size() ||
                size_t(vertex) >= rest.size() || size_t(next) >= rest.size()) return false;
            restBasis.push_back(basis(rest, previous, vertex, next));
            posedBasis.push_back(basis(posed, previous, vertex, next));
            sums[vertex] += posedBasis.back().weight;
        }
        offset += count;
    }
    if (offset != indices.size()) return false;
    std::vector<Wide> result(rest.size(), Wide(0));
    for (size_t corner = 0; corner < indices.size(); ++corner) {
        const int vertex = indices[corner];
        const auto &r = restBasis[corner], &p = posedBasis[corner];
        if (!(sums[vertex] > 0) || !(p.weight > 0)) continue;
        const Wide delta = deltaMushDetail::Convert<Wide>(deltas[vertex]);
        result[vertex] += (deltaMushDetail::Dot(delta,r.x)*p.x + deltaMushDetail::Dot(delta,r.y)*p.y +
            deltaMushDetail::Dot(delta,r.z)*p.z) * (p.weight / sums[vertex]);
    }
    std::vector<Point> converted;
    converted.reserve(result.size());
    for (const auto &p : result) {
        for (int a = 0; a < 3; ++a) if (!std::isfinite(p[a])) return false;
        converted.push_back(deltaMushDetail::Convert<Point>(p));
    }
    *out = std::move(converted); return true;
}

template<class Point, class Wide>
bool
RigExecApplyDeltaMushKernel(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders,
    double distanceWeight, double displacement,
    const RigExecDeltaMushSettings &settings = {})
{
    if (!points || points->size() != rest.size() || iterations < 0 ||
        iterations > 1000 || !std::isfinite(step) ||
        (settings.smoothing == RigExecDeltaMushSmoothing::Rest && (step < 0 || step > 1)) ||
        !std::isfinite(distanceWeight) || distanceWeight < 0 || distanceWeight > 10 ||
        !std::isfinite(displacement) ||
        (settings.smoothing == RigExecDeltaMushSmoothing::Rest && (displacement < 0 || displacement > 1)) ||
        (settings.smoothing != RigExecDeltaMushSmoothing::Rest && settings.smoothing != RigExecDeltaMushSmoothing::Simple &&
         settings.smoothing != RigExecDeltaMushSmoothing::LengthWeighted) ||
        (settings.frameTransport != RigExecDeltaMushFrameTransport::Vertex &&
         settings.frameTransport != RigExecDeltaMushFrameTransport::Corner) ||
        (!settings.smoothWeights.empty() && settings.smoothWeights.size() != rest.size())) return false;
    for (float w : settings.smoothWeights) if (!std::isfinite(w) || w < 0 || w > 1) return false;
    for (size_t i = 0; i < rest.size(); ++i)
        for (int a = 0; a < 3; ++a)
            if (!std::isfinite(rest[i][a]) || !std::isfinite((*points)[i][a])) return false;
    std::map<std::pair<int,int>, int> edges;
    size_t offset = 0;
    for (int count : counts) {
        if (count < 3 || size_t(count) > indices.size() - offset) return false;
        for (int c = 0; c < count; ++c) {
            int a = indices[offset+c], b = indices[offset+(c+1)%count];
            if (a < 0 || b < 0 || size_t(a) >= rest.size() || size_t(b) >= rest.size() || a == b) return false;
            ++edges[std::minmax(a,b)];
        }
        offset += count;
    }
    if (offset != indices.size() || (counts.empty() && !rest.empty())) return false;
    if (settings.edges.size() % 2) return false;
    // Keep polygon incidence for pinning even when explicit mesh edges include loose edges.
    auto meshEdges = edges;
    if (!settings.edges.empty()) {
        meshEdges.clear();
        for (size_t i = 0; i < settings.edges.size(); i += 2) {
            const int a = settings.edges[i], b = settings.edges[i+1];
            if (a < 0 || b < 0 || size_t(a) >= rest.size() || size_t(b) >= rest.size() || a == b ||
                !meshEdges.emplace(std::minmax(a,b), 0).second) return false;
        }
        for (const auto &edge : edges) if (!meshEdges.count(edge.first)) return false;
    }
    if (!iterations || step == 0 || rest.empty()) return true;
    std::vector<std::vector<std::pair<int,double>>> neighbors(rest.size());
    std::vector<bool> pinned(rest.size(), false);
    for (const auto &edge : meshEdges) {
        int a = edge.first.first, b = edge.first.second;
        if (edges[edge.first] == 1 && pinBorders) pinned[a] = pinned[b] = true;
        double length = (deltaMushDetail::Convert<Wide>(rest[a])-deltaMushDetail::Convert<Wide>(rest[b])).GetLength();
        double w = distanceWeight == 0 ? 1 : std::pow(std::max(length, 1e-12), -distanceWeight);
        if (!std::isfinite(w)) return false;
        neighbors[a].emplace_back(b,w);neighbors[b].emplace_back(a,w);
    }
    const auto smooth = [&](const std::vector<Point> &input) {
        std::vector<Wide> current;
        current.reserve(input.size());
        for (const auto &p : input) current.push_back(deltaMushDetail::Convert<Wide>(p));
        auto next = current;
        for (int iteration = 0; iteration < iterations; ++iteration) {
            for (size_t i = 0; i < current.size(); ++i) {
                if (pinned[i] || neighbors[i].empty()) continue;
                const double influence = settings.smoothWeights.empty() ? 1 : settings.smoothWeights[i];
                Wide sum(0); double total=0;
                if (settings.smoothing == RigExecDeltaMushSmoothing::LengthWeighted) {
                    for (const auto &n : neighbors[i]) {
                        const Wide edge = current[n.first] - current[i];
                        const double length = edge.GetLength();
                        sum += edge * length; total += length;
                    }
                    const double divisor = total * neighbors[i].size();
                    next[i] = current[i];
                    if (divisor > std::numeric_limits<float>::epsilon() * 10)
                        next[i] += sum * (2 * step * influence / divisor);
                } else {
                    for (const auto &n : neighbors[i]) {
                        const double w = settings.smoothing == RigExecDeltaMushSmoothing::Simple ? 1 : n.second;
                        sum += current[n.first] * w; total += w;
                    }
                    next[i] = current[i] + step * influence * (sum / total - current[i]);
                }
            }
            current.swap(next);
        }
        std::vector<Point> result;
        result.reserve(current.size());
        for (const auto &p : current) result.push_back(deltaMushDetail::Convert<Point>(p));
        return result;
    };
    auto smoothPosed = smooth(*points);
    if (settings.onlySmooth) {
        for (const auto &p : smoothPosed) for (int a = 0; a < 3; ++a)
            if (!std::isfinite(p[a])) return false;
        *points = std::move(smoothPosed); return true;
    }
    auto smoothRest = smooth(rest);
    std::vector<Point> detail(rest.size()), rotated;
    for (size_t i=0; i<rest.size(); ++i) detail[i]=(rest[i]-smoothRest[i])*float(displacement);
    if (settings.frameTransport == RigExecDeltaMushFrameTransport::Corner) {
        if (!RigExecTransportCornerOffsetsKernel<Point, Wide>(smoothRest, smoothPosed, counts, indices, detail, &rotated)) return false;
    } else if (!RigExecTransportSurfaceOffsetsKernel<Point, Wide>(smoothRest,smoothPosed,counts,indices,detail,&rotated)) return false;
    for (size_t i=0; i<rest.size(); ++i) {
        smoothPosed[i] += rotated[i];
        for (int a=0;a<3;++a) if (!std::isfinite(smoothPosed[i][a])) return false;
    }
    *points = std::move(smoothPosed);
    return true;
}

/// Row-vector affine adapter. Reference points are already in computation space.
template<class Point, class Wide, class Matrix>
bool RigExecApplyDeltaMushInSpaceKernel(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders, double distanceWeight,
    double displacement, const RigExecDeltaMushSettings &settings,
    const Matrix &computationToTarget)
{
    if (!points) return false;
    for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col)
        if (!std::isfinite(computationToTarget[row][col])) return false;
    if (std::abs(computationToTarget[0][3]) > 1e-12 || std::abs(computationToTarget[1][3]) > 1e-12 ||
        std::abs(computationToTarget[2][3]) > 1e-12 || std::abs(computationToTarget[3][3]-1) > 1e-12 ||
        std::abs(computationToTarget.GetDeterminant()) < 1e-14) return false;
    // Preserve the exact legacy arithmetic for the identity adapter.
    if (computationToTarget == Matrix(1))
        return RigExecApplyDeltaMushKernel<Point, Wide>(points, rest, counts, indices,
            iterations, step, pinBorders, distanceWeight, displacement, settings);
    Matrix affine = computationToTarget;
    affine[0][3]=affine[1][3]=affine[2][3]=0; affine[3][3]=1;
    Matrix targetToComputation = affine.GetInverse();
    targetToComputation[0][3]=targetToComputation[1][3]=targetToComputation[2][3]=0; targetToComputation[3][3]=1;
    auto local = *points;
    for (auto &p : local)
        p = deltaMushDetail::Convert<Point>(targetToComputation.TransformAffine(deltaMushDetail::Convert<Wide>(p)));
    if (!RigExecApplyDeltaMushKernel<Point, Wide>(&local, rest, counts, indices,
        iterations, step, pinBorders, distanceWeight, displacement, settings)) return false;
    for (auto &p : local) {
        p = deltaMushDetail::Convert<Point>(affine.TransformAffine(deltaMushDetail::Convert<Wide>(p)));
        for (int axis = 0; axis < 3; ++axis) if (!std::isfinite(p[axis])) return false;
    }
    *points = std::move(local); return true;
}

} // namespace rigExec
#endif
