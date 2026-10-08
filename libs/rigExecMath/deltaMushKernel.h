// Method reference: Mancewicz et al. (2014), https://doi.org/10.1145/2614106.2614144
// Shared, USD-free geometry math. Both USD and binary adapters instantiate
// these kernels; Point/Wide only supply float/double vector storage and arithmetic.
#ifndef RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#define RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#include "meshConnectivity.h"
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
template<class Wide> bool Normalize(Wide *value) {
    const double length = value->GetLength();
    if (!(length > 0) || !std::isfinite(length)) return false;
    *value /= length;
    return true;
}
// One definition, shared with the smooth kernel: the same sorted neighbor
// lists the set-based build produced, without a node per edge endpoint.
inline std::vector<std::vector<int>>
BuildAdjacency(
    size_t pointCount,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices)
{
    RigExecMeshAdjacency built;
    if (!RigExecBuildMeshAdjacency(
            pointCount, faceVertexCounts, faceVertexIndices, &built)) {
        return {};
    }
    return built.neighbors;
}

} // namespace deltaMushDetail

/// The rest half of a surface-offset transport: everything the kernel
/// derives from (rest, faceCounts, faceIndices) rather than from the posed
/// surface or the offsets. Built once for rest state shared across frames
/// or nodes; the posed half reads it, it never re-derives it.
template<class Point, class Wide>
struct RigExecTransportRestData {
    size_t pointCount = 0;
    /// Unnormalized Newell area sums over the rest surface.
    std::vector<Wide> restNormals;
    std::vector<std::vector<int>> adjacency;
    /// Per-vertex longest projected rest edge (see the kernel contract)
    /// and the rest frame it spans. Meaningful only where restOk holds.
    std::vector<int> neighbor;
    std::vector<Wide> nr;
    std::vector<Wide> tr;
    std::vector<Wide> br;
    /// Per-vertex rest-frame validity. A zero offset never consults it --
    /// "a zero offset needs no valid frame" -- so recording a failure here
    /// instead of returning one preserves the kernel's skip-then-fail
    /// behavior exactly.
    std::vector<bool> restOk;
    /// Whether every member covers \p n vertices, so the apply half fails
    /// closed on a hand-built entry instead of indexing past an end.
    bool Covers(size_t n) const {
        return pointCount == n && restNormals.size() == n &&
               adjacency.size() == n && neighbor.size() == n &&
               nr.size() == n && tr.size() == n && br.size() == n &&
               restOk.size() == n;
    }
};

/// Build the rest half of a transport: validate, accumulate rest normals,
/// build adjacency, and choose every vertex's rest frame. False leaves
/// \p data untouched. Per-vertex frame failures are RECORDED, not returned
/// (see RigExecTransportRestData::restOk).
template<class Point, class Wide>
bool
RigExecBuildTransportRestData(
    const std::vector<Point> &rest,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    RigExecTransportRestData<Point, Wide> *data)
{
    if (!data) return false;
    RigExecTransportRestData<Point, Wide> built;
    built.pointCount = rest.size();
    for (size_t i = 0; i < rest.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(rest[i][axis])) return false;
        }
    }
    size_t offset = 0;
    built.restNormals.assign(rest.size(), Wide(0));
    for (int count : faceCounts) {
        if (count < 3 || static_cast<size_t>(count) > faceIndices.size() - offset) return false;
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            if (index < 0 || static_cast<size_t>(index) >= rest.size()) return false;
        }
        // Newell's area vector, evaluated relative to the first corner to
        // avoid subtracting products of large translated coordinates.
        const int origin = faceIndices[offset];
        Wide restArea(0);
        for (int corner = 1; corner + 1 < count; ++corner) {
            const int a = faceIndices[offset + corner];
            const int b = faceIndices[offset + corner + 1];
            restArea += deltaMushDetail::Cross(deltaMushDetail::Convert<Wide>(rest[a]) - deltaMushDetail::Convert<Wide>(rest[origin]),
                                deltaMushDetail::Convert<Wide>(rest[b]) - deltaMushDetail::Convert<Wide>(rest[origin]));
        }
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            built.restNormals[index] += restArea;
        }
        offset += static_cast<size_t>(count);
    }
    if (offset != faceIndices.size()) return false;
    built.adjacency = deltaMushDetail::BuildAdjacency(rest.size(), faceCounts, faceIndices);
    built.neighbor.assign(rest.size(), -1);
    built.nr.assign(rest.size(), Wide(0));
    built.tr.assign(rest.size(), Wide(0));
    built.br.assign(rest.size(), Wide(0));
    built.restOk.assign(rest.size(), false);
    for (size_t i = 0; i < rest.size(); ++i) {
        Wide nr = built.restNormals[i];
        if (!deltaMushDetail::Normalize(&nr)) continue;
        int neighbor = -1;
        double longest = 0;
        Wide tr(0);
        // deltaMushDetail::BuildAdjacency orders neighbors by index, making equal-length
        // choices independent of face order and corner traversal direction.
        for (int candidate : built.adjacency[i]) {
            const Wide edge = deltaMushDetail::Convert<Wide>(rest[candidate]) - deltaMushDetail::Convert<Wide>(rest[i]);
            const Wide projected = edge - deltaMushDetail::Dot(edge, nr) * nr;
            const double length2 = projected.GetLengthSq();
            if (length2 > longest) {
                neighbor = candidate;
                longest = length2;
                tr = projected;
            }
        }
        if (neighbor < 0 || !deltaMushDetail::Normalize(&tr)) continue;
        built.neighbor[i] = neighbor;
        built.nr[i] = nr;
        built.tr[i] = tr;
        built.br[i] = deltaMushDetail::Cross(nr, tr);
        built.restOk[i] = true;
    }
    *data = std::move(built);
    return true;
}

/// Apply a transport to \p posed with prebuilt rest state: accumulate
/// posed normals, then rotate each nonzero offset through its rest/posed
/// frame pair. The same arithmetic the single-call kernel performs, with
/// the same failure conditions; invalid input fails atomically, leaving
/// \p out unchanged.
template<class Point, class Wide>
bool
RigExecApplyTransportWithRestData(
    const std::vector<Point> &posed,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    const std::vector<Point> &deltas,
    const RigExecTransportRestData<Point, Wide> &restData,
    std::vector<Point> *out)
{
    if (!out || !restData.Covers(posed.size()) ||
        posed.size() != deltas.size()) return false;
    for (size_t i = 0; i < posed.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(posed[i][axis]) ||
                !std::isfinite(deltas[i][axis])) return false;
        }
    }
    size_t offset = 0;
    std::vector<Wide> posedNormals(posed.size(), Wide(0));
    for (int count : faceCounts) {
        if (count < 3 || static_cast<size_t>(count) > faceIndices.size() - offset) return false;
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            if (index < 0 || static_cast<size_t>(index) >= posed.size()) return false;
        }
        const int origin = faceIndices[offset];
        Wide posedArea(0);
        for (int corner = 1; corner + 1 < count; ++corner) {
            const int a = faceIndices[offset + corner];
            const int b = faceIndices[offset + corner + 1];
            posedArea += deltaMushDetail::Cross(deltaMushDetail::Convert<Wide>(posed[a]) - deltaMushDetail::Convert<Wide>(posed[origin]),
                                 deltaMushDetail::Convert<Wide>(posed[b]) - deltaMushDetail::Convert<Wide>(posed[origin]));
        }
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            posedNormals[index] += posedArea;
        }
        offset += static_cast<size_t>(count);
    }
    if (offset != faceIndices.size()) return false;
    std::vector<Point> result = deltas;
    for (size_t i = 0; i < posed.size(); ++i) {
        if (deltas[i] == Point(0)) continue;
        if (!restData.restOk[i]) return false;
        Wide np = posedNormals[i];
        if (!deltaMushDetail::Normalize(&np)) return false;
        const int neighbor = restData.neighbor[i];
        if (neighbor < 0 || static_cast<size_t>(neighbor) >= posed.size()) return false;
        const Wide edge = deltaMushDetail::Convert<Wide>(posed[neighbor]) - deltaMushDetail::Convert<Wide>(posed[i]);
        Wide tp = edge - deltaMushDetail::Dot(edge, np) * np;
        if (tp.GetLengthSq() <= edge.GetLengthSq() * 1e-24 || !deltaMushDetail::Normalize(&tp)) return false;
        const Wide bp = deltaMushDetail::Cross(np, tp);
        const Wide delta = deltaMushDetail::Convert<Wide>(deltas[i]);
        const Wide rotated = deltaMushDetail::Dot(delta, restData.tr[i]) * tp + deltaMushDetail::Dot(delta, restData.br[i]) * bp + deltaMushDetail::Dot(delta, restData.nr[i]) * np;
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
RigExecTransportSurfaceOffsetsKernel(
    const std::vector<Point> &rest,
    const std::vector<Point> &posed,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    const std::vector<Point> &deltas,
    std::vector<Point> *out)
{
    RigExecTransportRestData<Point, Wide> restData;
    if (!RigExecBuildTransportRestData(rest, faceCounts, faceIndices, &restData)) {
        return false;
    }
    return RigExecApplyTransportWithRestData(
        posed, faceCounts, faceIndices, deltas, restData, out);
}

/// The rest half of a delta-mush deformation: the smoothing table, the
/// border pins, the smoothed rest surface, and the transport rest state
/// over it. The inputs the derived state was built from are echoed so a
/// cache can compare candidates by value (see RigExecSkinTopology): the
/// derived state is a pure function of exactly the echo.
template<class Point, class Wide>
struct RigExecDeltaMushRestData {
    std::vector<Point> rest;
    std::vector<int> counts;
    std::vector<int> indices;
    int iterations = 0;
    double step = 0.0;
    bool pinBorders = false;
    double distanceWeight = 0.0;
    /// Per-vertex (neighbor, edge-distance weight) table.
    std::vector<std::vector<std::pair<int, double>>> neighbors;
    std::vector<bool> pinned;
    std::vector<Point> smoothRest;
    RigExecTransportRestData<Point, Wide> transport;
    /// The rest-side validation passed; only the posed points and the
    /// displacement are still frame business.
    bool validated = false;

    bool operator==(const RigExecDeltaMushRestData &o) const {
        return validated == o.validated && iterations == o.iterations &&
               step == o.step && pinBorders == o.pinBorders &&
               distanceWeight == o.distanceWeight && rest == o.rest &&
               counts == o.counts && indices == o.indices;
    }
    bool operator!=(const RigExecDeltaMushRestData &o) const {
        return !(*this == o);
    }
};

namespace deltaMushDetail {
// Edge-weighted Laplacian relaxation, shared by the rest build and the
// posed apply: smoothing the same input twice must smooth it the same
// way, which is only guaranteed in one definition.
template<class Point, class Wide>
std::vector<Point>
SmoothWithNeighbors(
    const std::vector<Point> &input,
    const std::vector<std::vector<std::pair<int, double>>> &neighbors,
    const std::vector<bool> &pinned,
    int iterations, double step)
{
    std::vector<Wide> current;
    current.reserve(input.size());
    for (const auto &p : input) current.push_back(Convert<Wide>(p));
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
    for (const auto &p : current) result.push_back(Convert<Point>(p));
    return result;
}
} // namespace deltaMushDetail

/// Build the rest half of a delta-mush deformation: validate the
/// rest-side inputs, derive the smoothing table, smooth the rest surface,
/// and build the transport rest state over it. The input echo is filled
/// whatever the outcome, so a cache can keep a rejected build as its
/// candidate; the derived state is meaningful only on true.
template<class Point, class Wide>
bool
RigExecBuildDeltaMushRestData(
    const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders,
    double distanceWeight,
    RigExecDeltaMushRestData<Point, Wide> *data)
{
    if (!data) return false;
    RigExecDeltaMushRestData<Point, Wide> built;
    built.rest = rest;
    built.counts = counts;
    built.indices = indices;
    built.iterations = iterations;
    built.step = step;
    built.pinBorders = pinBorders;
    built.distanceWeight = distanceWeight;
    if (iterations < 0 || iterations > 1000 ||
        !std::isfinite(step) || step < 0 || step > 1 ||
        !std::isfinite(distanceWeight) || distanceWeight < 0 ||
        distanceWeight > 10) {
        *data = std::move(built);
        return false;
    }
    for (size_t i = 0; i < rest.size(); ++i) {
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(rest[i][a])) {
                *data = std::move(built);
                return false;
            }
        }
    }
    // Sorted edges with run counts instead of the map: the same
    // lexicographic order the neighbor pushes below ran in, without a
    // node per edge. Connectivity only -- the distance weights stay
    // rest-derived, per the H1 split.
    std::vector<std::pair<int,int>> edges;
    size_t offset = 0;
    for (int count : counts) {
        if (count < 3 || size_t(count) > indices.size() - offset) {
            *data = std::move(built);
            return false;
        }
        for (int c = 0; c < count; ++c) {
            int a = indices[offset+c], b = indices[offset+(c+1)%count];
            if (a < 0 || b < 0 || size_t(a) >= rest.size() || size_t(b) >= rest.size() || a == b) {
                *data = std::move(built);
                return false;
            }
            edges.emplace_back(std::minmax(a,b));
        }
        offset += count;
    }
    std::sort(edges.begin(), edges.end());
    if (offset != indices.size() || (counts.empty() && !rest.empty())) {
        *data = std::move(built);
        return false;
    }
    if (!iterations || step == 0 || rest.empty()) {
        built.validated = true;
        *data = std::move(built);
        return true;
    }
    built.neighbors.assign(rest.size(), {});
    built.pinned.assign(rest.size(), false);
    for (size_t i = 0; i < edges.size();) {
        size_t j = i + 1;
        while (j < edges.size() && edges[j] == edges[i]) ++j;
        const int a = edges[i].first, b = edges[i].second;
        if (j - i == 1 && pinBorders) built.pinned[a] = built.pinned[b] = true;
        double length = (deltaMushDetail::Convert<Wide>(rest[a])-deltaMushDetail::Convert<Wide>(rest[b])).GetLength();
        double w = distanceWeight == 0 ? 1 : std::pow(std::max(length, 1e-12), -distanceWeight);
        if (!std::isfinite(w)) {
            *data = std::move(built);
            return false;
        }
        built.neighbors[a].emplace_back(b,w);built.neighbors[b].emplace_back(a,w);
        i = j;
    }
    built.smoothRest = deltaMushDetail::SmoothWithNeighbors<Point, Wide>(
        rest, built.neighbors, built.pinned, iterations, step);
    if (!RigExecBuildTransportRestData(
            built.smoothRest, counts, indices, &built.transport)) {
        *data = std::move(built);
        return false;
    }
    built.validated = true;
    *data = std::move(built);
    return true;
}

/// Apply a delta-mush deformation with prebuilt rest state: smooth the
/// posed points, transport the rest detail onto them, and add it. The
/// same arithmetic the single-call kernel performs, with the same failure
/// conditions; invalid input fails atomically, leaving \p points unchanged.
template<class Point, class Wide>
bool
RigExecApplyDeltaMushWithRestData(
    std::vector<Point> *points,
    const RigExecDeltaMushRestData<Point, Wide> &restData,
    double displacement)
{
    if (!points || !restData.validated) return false;
    if (points->size() != restData.rest.size()) return false;
    if (!std::isfinite(displacement) || displacement < 0 || displacement > 1) return false;
    for (size_t i = 0; i < points->size(); ++i)
        for (int a = 0; a < 3; ++a)
            if (!std::isfinite((*points)[i][a])) return false;
    if (!restData.iterations || restData.step == 0 || restData.rest.empty()) return true;
    if (restData.neighbors.size() != restData.rest.size() ||
        restData.pinned.size() != restData.rest.size() ||
        restData.smoothRest.size() != restData.rest.size() ||
        !restData.transport.Covers(restData.rest.size())) return false;
    std::vector<Point> smoothPosed = deltaMushDetail::SmoothWithNeighbors<Point, Wide>(
        *points, restData.neighbors, restData.pinned,
        restData.iterations, restData.step);
    std::vector<Point> detail(restData.rest.size()), rotated;
    for (size_t i=0; i<restData.rest.size(); ++i) detail[i]=(restData.rest[i]-restData.smoothRest[i])*float(displacement);
    if (!RigExecApplyTransportWithRestData(smoothPosed, restData.counts, restData.indices, detail, restData.transport, &rotated)) return false;
    for (size_t i=0; i<restData.rest.size(); ++i) {
        smoothPosed[i] += rotated[i];
        for (int a=0;a<3;++a) if (!std::isfinite(smoothPosed[i][a])) return false;
    }
    *points = std::move(smoothPosed);
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
    RigExecDeltaMushRestData<Point, Wide> restData;
    if (!RigExecBuildDeltaMushRestData(rest, counts, indices, iterations,
                                       step, pinBorders, distanceWeight,
                                       &restData)) {
        return false;
    }
    return RigExecApplyDeltaMushWithRestData(points, restData, displacement);
}

} // namespace rigExec
#endif
