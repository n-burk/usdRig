// Method reference: Mancewicz et al. (2014), https://doi.org/10.1145/2614106.2614144
// Shared, USD-free geometry math. Both USD and binary adapters instantiate
// these kernels; Point/Wide only supply float/double vector storage and arithmetic.
#ifndef RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#define RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#include "deltaMushSettings.h"
#include "meshConnectivity.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
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

/// The rest half of a delta-mush deformation: the smoothing table, the
/// border pins, the smoothed rest surface, and the vertex transport rest
/// state over it. The inputs the derived state was built from are echoed so
/// a cache can compare candidates by value (see RigExecSkinTopology): the
/// derived state is a pure function of exactly the echo. The smoothing
/// settings are part of it: the posed half smooths with the same mode and
/// per-vertex influence the rest half used.
template<class Point, class Wide>
struct RigExecDeltaMushRestData {
    std::vector<Point> rest;
    std::vector<int> counts;
    std::vector<int> indices;
    int iterations = 0;
    double step = 0.0;
    bool pinBorders = false;
    double distanceWeight = 0.0;
    RigExecDeltaMushSettings settings;
    /// Per-vertex (neighbor, edge-distance weight) table.
    std::vector<std::vector<std::pair<int, double>>> neighbors;
    std::vector<bool> pinned;
    /// Empty when the settings only smooth.
    std::vector<Point> smoothRest;
    /// Built for the vertex frame transport only.
    RigExecTransportRestData<Point, Wide> transport;
    /// The rest-side validation passed; only the posed points and the
    /// displacement are still frame business.
    bool validated = false;

    bool operator==(const RigExecDeltaMushRestData &o) const {
        return validated == o.validated && iterations == o.iterations &&
               step == o.step && pinBorders == o.pinBorders &&
               distanceWeight == o.distanceWeight && settings == o.settings &&
               rest == o.rest && counts == o.counts && indices == o.indices;
    }
    bool operator!=(const RigExecDeltaMushRestData &o) const {
        return !(*this == o);
    }
};

namespace deltaMushDetail {
// Laplacian relaxation, shared by the rest build and the posed apply:
// smoothing the same input twice must smooth it the same way, which is only
// guaranteed in one definition. A unit influence multiplies the step
// exactly, so the legacy rest-weighted relaxation keeps its bits.
template<class Point, class Wide>
std::vector<Point>
SmoothWithNeighbors(
    const std::vector<Point> &input,
    const std::vector<std::vector<std::pair<int, double>>> &neighbors,
    const std::vector<bool> &pinned,
    int iterations, double step,
    const RigExecDeltaMushSettings &settings)
{
    std::vector<Wide> current;
    current.reserve(input.size());
    for (const auto &p : input) current.push_back(Convert<Wide>(p));
    auto next = current;
    const bool weighted = !settings.smoothWeights.empty();
    for (int iteration = 0; iteration < iterations; ++iteration) {
        for (size_t i = 0; i < current.size(); ++i) {
            if (pinned[i] || neighbors[i].empty()) continue;
            const double influence = weighted ? double(settings.smoothWeights[i]) : 1.0;
            Wide sum(0); double total = 0;
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
    for (const auto &p : current) result.push_back(Convert<Point>(p));
    return result;
}

// The settings' own validation, independent of any mesh.
inline bool
SettingsValid(const RigExecDeltaMushSettings &settings, size_t pointCount)
{
    if ((settings.smoothing != RigExecDeltaMushSmoothing::Rest &&
         settings.smoothing != RigExecDeltaMushSmoothing::Simple &&
         settings.smoothing != RigExecDeltaMushSmoothing::LengthWeighted) ||
        (settings.frameTransport != RigExecDeltaMushFrameTransport::Vertex &&
         settings.frameTransport != RigExecDeltaMushFrameTransport::Corner) ||
        (!settings.smoothWeights.empty() && settings.smoothWeights.size() != pointCount) ||
        settings.edges.size() % 2) {
        return false;
    }
    for (float w : settings.smoothWeights) {
        if (!std::isfinite(w) || w < 0 || w > 1) return false;
    }
    return true;
}
} // namespace deltaMushDetail

/// Build the rest half of a delta-mush deformation: validate the
/// rest-side inputs, derive the smoothing table (from the polygon edges,
/// or from the explicit edge pairs, which must include every polygon edge),
/// smooth the rest surface, and build the vertex transport rest state over
/// it. The input echo is filled whatever the outcome, so a cache can keep a
/// rejected build as its candidate; the derived state is meaningful only on
/// true. A step outside [0, 1] is refused only for rest-weighted smoothing.
template<class Point, class Wide>
bool
RigExecBuildDeltaMushRestData(
    const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders,
    double distanceWeight, const RigExecDeltaMushSettings &settings,
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
    built.settings = settings;
    const bool restWeighted =
        settings.smoothing == RigExecDeltaMushSmoothing::Rest;
    if (iterations < 0 || iterations > 1000 || !std::isfinite(step) ||
        (restWeighted && (step < 0 || step > 1)) ||
        !std::isfinite(distanceWeight) || distanceWeight < 0 ||
        distanceWeight > 10 ||
        !deltaMushDetail::SettingsValid(settings, rest.size())) {
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
    // Sorted polygon edges with run counts instead of a map: the same
    // lexicographic order the neighbor pushes below run in, without a node
    // per edge. A run of one is a border edge. Connectivity only -- the
    // distance weights stay rest-derived.
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
    // Explicit edge pairs replace the polygon edges as the smoothing
    // connectivity; polygon incidence still decides the border pins.
    std::vector<std::pair<int,int>> explicitEdges;
    if (!settings.edges.empty()) {
        explicitEdges.reserve(settings.edges.size() / 2);
        for (size_t i = 0; i < settings.edges.size(); i += 2) {
            const int a = settings.edges[i], b = settings.edges[i+1];
            if (a < 0 || b < 0 || size_t(a) >= rest.size() || size_t(b) >= rest.size() || a == b) {
                *data = std::move(built);
                return false;
            }
            explicitEdges.emplace_back(std::minmax(a,b));
        }
        std::sort(explicitEdges.begin(), explicitEdges.end());
        if (std::adjacent_find(explicitEdges.begin(), explicitEdges.end()) != explicitEdges.end()) {
            *data = std::move(built);
            return false;
        }
        for (const auto &edge : edges) {
            if (!std::binary_search(explicitEdges.begin(), explicitEdges.end(), edge)) {
                *data = std::move(built);
                return false;
            }
        }
    }
    if (!iterations || step == 0 || rest.empty()) {
        built.validated = true;
        *data = std::move(built);
        return true;
    }
    built.neighbors.assign(rest.size(), {});
    built.pinned.assign(rest.size(), false);
    const auto connect = [&](int a, int b, size_t polygonUses) {
        if (polygonUses == 1 && pinBorders) built.pinned[a] = built.pinned[b] = true;
        double length = (deltaMushDetail::Convert<Wide>(rest[a])-deltaMushDetail::Convert<Wide>(rest[b])).GetLength();
        double w = distanceWeight == 0 ? 1 : std::pow(std::max(length, 1e-12), -distanceWeight);
        if (!std::isfinite(w)) return false;
        built.neighbors[a].emplace_back(b,w);built.neighbors[b].emplace_back(a,w);
        return true;
    };
    if (explicitEdges.empty()) {
        for (size_t i = 0; i < edges.size();) {
            size_t j = i + 1;
            while (j < edges.size() && edges[j] == edges[i]) ++j;
            if (!connect(edges[i].first, edges[i].second, j - i)) {
                *data = std::move(built);
                return false;
            }
            i = j;
        }
    } else {
        for (const auto &edge : explicitEdges) {
            const auto uses = std::equal_range(edges.begin(), edges.end(), edge);
            if (!connect(edge.first, edge.second, size_t(uses.second - uses.first))) {
                *data = std::move(built);
                return false;
            }
        }
    }
    if (settings.onlySmooth) {
        built.validated = true;
        *data = std::move(built);
        return true;
    }
    built.smoothRest = deltaMushDetail::SmoothWithNeighbors<Point, Wide>(
        rest, built.neighbors, built.pinned, iterations, step, settings);
    if (settings.frameTransport == RigExecDeltaMushFrameTransport::Vertex &&
        !RigExecBuildTransportRestData(
            built.smoothRest, counts, indices, &built.transport)) {
        *data = std::move(built);
        return false;
    }
    built.validated = true;
    *data = std::move(built);
    return true;
}

/// The legacy-settings build.
template<class Point, class Wide>
bool
RigExecBuildDeltaMushRestData(
    const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders,
    double distanceWeight,
    RigExecDeltaMushRestData<Point, Wide> *data)
{
    return RigExecBuildDeltaMushRestData<Point, Wide>(
        rest, counts, indices, iterations, step, pinBorders, distanceWeight,
        RigExecDeltaMushSettings(), data);
}

/// Apply a delta-mush deformation with prebuilt rest state: smooth the
/// posed points, transport the rest detail onto them, and add it. The
/// same arithmetic the single-call kernel performs, with the same failure
/// conditions; invalid input fails atomically, leaving \p points unchanged.
/// The displacement is limited to [0, 1] only for rest-weighted smoothing.
template<class Point, class Wide>
bool
RigExecApplyDeltaMushWithRestData(
    std::vector<Point> *points,
    const RigExecDeltaMushRestData<Point, Wide> &restData,
    double displacement)
{
    if (!points || !restData.validated) return false;
    if (points->size() != restData.rest.size()) return false;
    const RigExecDeltaMushSettings &settings = restData.settings;
    if (!std::isfinite(displacement) ||
        (settings.smoothing == RigExecDeltaMushSmoothing::Rest &&
         (displacement < 0 || displacement > 1))) return false;
    for (size_t i = 0; i < points->size(); ++i)
        for (int a = 0; a < 3; ++a)
            if (!std::isfinite((*points)[i][a])) return false;
    if (!restData.iterations || restData.step == 0 || restData.rest.empty()) return true;
    if (restData.neighbors.size() != restData.rest.size() ||
        restData.pinned.size() != restData.rest.size()) return false;
    std::vector<Point> smoothPosed = deltaMushDetail::SmoothWithNeighbors<Point, Wide>(
        *points, restData.neighbors, restData.pinned,
        restData.iterations, restData.step, settings);
    if (settings.onlySmooth) {
        for (const auto &p : smoothPosed)
            for (int a = 0; a < 3; ++a)
                if (!std::isfinite(p[a])) return false;
        *points = std::move(smoothPosed);
        return true;
    }
    if (restData.smoothRest.size() != restData.rest.size()) return false;
    std::vector<Point> detail(restData.rest.size()), rotated;
    for (size_t i=0; i<restData.rest.size(); ++i) detail[i]=(restData.rest[i]-restData.smoothRest[i])*float(displacement);
    if (settings.frameTransport == RigExecDeltaMushFrameTransport::Corner) {
        if (!RigExecTransportCornerOffsetsKernel<Point, Wide>(
                restData.smoothRest, smoothPosed, restData.counts,
                restData.indices, detail, &rotated)) return false;
    } else {
        if (!restData.transport.Covers(restData.rest.size())) return false;
        if (!RigExecApplyTransportWithRestData(smoothPosed, restData.counts, restData.indices, detail, restData.transport, &rotated)) return false;
    }
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
    double distanceWeight, double displacement,
    const RigExecDeltaMushSettings &settings = {})
{
    if (!points || points->size() != rest.size()) return false;
    RigExecDeltaMushRestData<Point, Wide> restData;
    if (!RigExecBuildDeltaMushRestData(rest, counts, indices, iterations,
                                       step, pinBorders, distanceWeight,
                                       settings, &restData)) {
        return false;
    }
    return RigExecApplyDeltaMushWithRestData(points, restData, displacement);
}

namespace deltaMushDetail {
// The computation-to-target map's own validation: finite, affine up to
// computed roundoff, and invertible.
template<class Matrix>
bool
ComputationMapValid(const Matrix &computationToTarget)
{
    for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col)
        if (!std::isfinite(computationToTarget[row][col])) return false;
    return !(std::abs(computationToTarget[0][3]) > 1e-12 || std::abs(computationToTarget[1][3]) > 1e-12 ||
             std::abs(computationToTarget[2][3]) > 1e-12 || std::abs(computationToTarget[3][3]-1) > 1e-12 ||
             std::abs(computationToTarget.GetDeterminant()) < 1e-14);
}

// Runs \p apply on \p points carried into computation space by the inverse
// of the canonicalized row-vector affine map, then carries the result back.
template<class Point, class Wide, class Matrix, class Apply>
bool
ApplyInComputationSpace(std::vector<Point> *points,
                        const Matrix &computationToTarget, Apply &&apply)
{
    Matrix affine = computationToTarget;
    affine[0][3]=affine[1][3]=affine[2][3]=0; affine[3][3]=1;
    Matrix targetToComputation = affine.GetInverse();
    targetToComputation[0][3]=targetToComputation[1][3]=targetToComputation[2][3]=0; targetToComputation[3][3]=1;
    auto local = *points;
    for (auto &p : local)
        p = Convert<Point>(targetToComputation.TransformAffine(Convert<Wide>(p)));
    if (!apply(&local)) return false;
    for (auto &p : local) {
        p = Convert<Point>(affine.TransformAffine(Convert<Wide>(p)));
        for (int axis = 0; axis < 3; ++axis) if (!std::isfinite(p[axis])) return false;
    }
    *points = std::move(local); return true;
}
} // namespace deltaMushDetail

/// Row-vector affine adapter. Reference points are already in computation
/// space; the identity map runs the plain kernel, bit for bit.
template<class Point, class Wide, class Matrix>
bool RigExecApplyDeltaMushInSpaceKernel(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders, double distanceWeight,
    double displacement, const RigExecDeltaMushSettings &settings,
    const Matrix &computationToTarget)
{
    if (!points || !deltaMushDetail::ComputationMapValid(computationToTarget)) return false;
    if (computationToTarget == Matrix(1))
        return RigExecApplyDeltaMushKernel<Point, Wide>(points, rest, counts, indices,
            iterations, step, pinBorders, distanceWeight, displacement, settings);
    return deltaMushDetail::ApplyInComputationSpace<Point, Wide>(
        points, computationToTarget, [&](std::vector<Point> *local) {
            return RigExecApplyDeltaMushKernel<Point, Wide>(local, rest, counts, indices,
                iterations, step, pinBorders, distanceWeight, displacement, settings);
        });
}

/// RigExecApplyDeltaMushInSpaceKernel over prebuilt rest state: the same
/// arithmetic and failure conditions.
template<class Point, class Wide, class Matrix>
bool RigExecApplyDeltaMushInSpaceWithRestData(
    std::vector<Point> *points,
    const RigExecDeltaMushRestData<Point, Wide> &restData,
    double displacement, const Matrix &computationToTarget)
{
    if (!points || !deltaMushDetail::ComputationMapValid(computationToTarget)) return false;
    if (computationToTarget == Matrix(1))
        return RigExecApplyDeltaMushWithRestData(points, restData, displacement);
    return deltaMushDetail::ApplyInComputationSpace<Point, Wide>(
        points, computationToTarget, [&](std::vector<Point> *local) {
            return RigExecApplyDeltaMushWithRestData(local, restData, displacement);
        });
}

} // namespace rigExec
#endif
