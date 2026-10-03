// Method reference: Mancewicz et al. (2014), https://doi.org/10.1145/2614106.2614144
// Shared, USD-free geometry math. Both USD and binary adapters instantiate
// these kernels; Point/Wide only supply float/double vector storage and arithmetic.
#ifndef RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#define RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#include <algorithm>
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

template<class Point, class Wide>
bool
RigExecApplyDeltaMushKernel(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders,
    double distanceWeight, double displacement)
{
    if (!points || points->size() != rest.size() || iterations < 0 ||
        iterations > 1000 || !std::isfinite(step) || step < 0 || step > 1 ||
        !std::isfinite(distanceWeight) || distanceWeight < 0 || distanceWeight > 10 ||
        !std::isfinite(displacement) || displacement < 0 || displacement > 1) return false;
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
    if (!iterations || step == 0 || rest.empty()) return true;
    std::vector<std::vector<std::pair<int,double>>> neighbors(rest.size());
    std::vector<bool> pinned(rest.size(), false);
    for (const auto &edge : edges) {
        int a = edge.first.first, b = edge.first.second;
        if (edge.second == 1 && pinBorders) pinned[a] = pinned[b] = true;
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
                Wide sum(0); double total=0;
                for (const auto &n : neighbors[i]) {sum += current[n.first]*n.second;total += n.second;}
                next[i] = current[i] + step*(sum/total-current[i]);
            }
            current.swap(next);
        }
        std::vector<Point> result;
        result.reserve(current.size());
        for (const auto &p : current) result.push_back(deltaMushDetail::Convert<Point>(p));
        return result;
    };
    auto smoothRest = smooth(rest), smoothPosed = smooth(*points);
    std::vector<Point> detail(rest.size()), rotated;
    for (size_t i=0; i<rest.size(); ++i) detail[i]=(rest[i]-smoothRest[i])*float(displacement);
    if (!RigExecTransportSurfaceOffsetsKernel<Point, Wide>(smoothRest,smoothPosed,counts,indices,detail,&rotated)) return false;
    for (size_t i=0; i<rest.size(); ++i) {
        smoothPosed[i] += rotated[i];
        for (int a=0;a<3;++a) if (!std::isfinite(smoothPosed[i][a])) return false;
    }
    *points = std::move(smoothPosed);
    return true;
}

} // namespace rigExec
#endif
