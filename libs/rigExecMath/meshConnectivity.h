// Shared topology-connectivity builders for the deformation kernels (H1).
// Header-only and USD-free so the USD evaluators and the zero-USD runtime
// run one definition: the smooth adjacency, the delta-mush neighbor table,
// and the wrinkle triangle topology are pure functions of the face counts
// and indices, built once per topology and shared across frames.
//
// Traversal order is semantic -- delta-mush breaks equal-length neighbor
// ties by index, wrinkle solves its constraints in edge order -- so every
// builder below is an order-preserving sorted-vector build: neighbors
// ascending, edges lexicographic, exactly the order the std::set/std::map
// builders they replace produced. A cached entry costs one allocation per
// vertex instead of one node per edge endpoint.
#ifndef RIGEXEC_MATH_MESH_CONNECTIVITY_H
#define RIGEXEC_MATH_MESH_CONNECTIVITY_H

#include "wrinkleSettings.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>
#include <vector>

namespace rigExec {

// Edge adjacency (unique undirected edges) from standard polygon topology:
// the neighbor lists RigExecApplyLaplacianSmooth and the delta-mush and
// transport kernels smooth over. Sorted ascending per vertex.
struct RigExecMeshAdjacency {
    size_t pointCount = 0;
    std::vector<std::vector<int>> neighbors;
    bool valid = false;

    /// Whether this entry answers a mesh of \p n points. Content trust
    /// stays with the retainer (which compares the topology arrays); this
    /// is the fail-closed shape check for a hand-built entry.
    bool Covers(size_t n) const {
        return valid && pointCount == n && neighbors.size() == n;
    }
};

// Builds the adjacency for \p pointCount points. False (\p out untouched)
// on the same inputs the legacy set-based builders rejected: a face corner
// past the index array or addressing no point. A repeated corner
// contributes once, exactly as the set insert did. No count<3 or
// trailing-index rule: the smooth kernel never had one.
//
// The span form borrows both ranges for the call only; the caller keeps
// the storage (e.g. the VtArray behind cdata()) alive until it returns.
// A null range with a nonzero size fails closed, as invalid topology
// does; a null range with size zero is the empty range. Bit-identical
// to the vector form, which delegates to it.
inline bool
RigExecBuildMeshAdjacency(
    size_t pointCount,
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize,
    RigExecMeshAdjacency *out)
{
    if (!out) return false;
    if ((faceVertexCountsSize > 0 && !faceVertexCounts) ||
        (faceVertexIndicesSize > 0 && !faceVertexIndices)) {
        return false;
    }
    RigExecMeshAdjacency built;
    built.pointCount = pointCount;
    built.neighbors.assign(pointCount, {});
    size_t offset = 0;
    for (size_t f = 0; f < faceVertexCountsSize; ++f) {
        const int faceCount = faceVertexCounts[f];
        for (int c = 0; c < faceCount; ++c) {
            const size_t ia = offset + c;
            const size_t ib = offset + (c + 1) % faceCount;
            if (ia >= faceVertexIndicesSize ||
                ib >= faceVertexIndicesSize) {
                return false;
            }
            const int a = faceVertexIndices[ia];
            const int b = faceVertexIndices[ib];
            if (a < 0 || b < 0 ||
                static_cast<size_t>(a) >= pointCount ||
                static_cast<size_t>(b) >= pointCount) {
                return false;
            }
            built.neighbors[a].push_back(b);
            if (b != a) {
                built.neighbors[b].push_back(a);
            }
        }
        offset += faceCount;
    }
    for (std::vector<int> &ring : built.neighbors) {
        std::sort(ring.begin(), ring.end());
        ring.erase(std::unique(ring.begin(), ring.end()), ring.end());
    }
    built.valid = true;
    *out = std::move(built);
    return true;
}

inline bool
RigExecBuildMeshAdjacency(
    size_t pointCount,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    RigExecMeshAdjacency *out)
{
    return RigExecBuildMeshAdjacency(
        pointCount, faceVertexCounts.data(), faceVertexCounts.size(),
        faceVertexIndices.data(), faceVertexIndices.size(), out);
}

// One triangle edge: its endpoints in minmax order with the opposite
// corner of every triangle using it, in triangle order -- the sorted key
// order std::map iteration produced, with the same per-key lists.
struct RigExecWrinkleEdge {
    int a = -1;
    int b = -1;
    std::vector<int> opposite;
};

// The wrinkle kernel's topology half: the fan triangulation (with the
// alternating quad diagonal), the sorted triangle edges, the sorted
// neighbor rings, the border vertices, and the bending pairs for one
// (topology mode, neighbor distance). Rest lengths, pins from degenerate
// normals, and everything posed stay per-frame in the apply half.
struct RigExecWrinkleMesh {
    size_t pointCount = 0;
    std::vector<int> counts;
    std::vector<int> indices;
    RigExecWrinkleTopology mode = RigExecWrinkleTopology::Cloth;
    int neighborDistance = 0;
    std::vector<std::array<int, 3>> triangles;
    std::vector<RigExecWrinkleEdge> edges;
    std::vector<std::vector<int>> neighbors;
    std::vector<int> borderVertices;
    std::vector<std::pair<int, int>> bendingPairs;
    bool valid = false;

    /// Whether this entry answers the given topology and bending rule.
    /// The full array compare (one memcmp per array, no allocation) is
    /// what lets the apply half walk the caller's faces without
    /// re-validating them: a mismatched entry fails closed here instead
    /// of reading out of range below.
    bool Covers(size_t n, const std::vector<int> &faceCounts,
                const std::vector<int> &faceIndices,
                RigExecWrinkleTopology wantMode,
                int wantNeighborDistance) const {
        return valid && pointCount == n && mode == wantMode &&
               neighborDistance == wantNeighborDistance &&
               neighbors.size() == n && counts == faceCounts &&
               indices == faceIndices;
    }
};

// Builds the wrinkle topology half. False (\p out untouched) on the same
// inputs the single-call kernel rejects: a short face, an index-array
// overrun or trailing indices, an out-of-range corner, or a face naming
// one corner twice. The bending pairs follow the same rule the kernel
// does: cloth pairs opposite corners across shared edges, surface struts
// pairs graph-hop rings at \p neighborDistance (which cloth ignores).
inline bool
RigExecBuildWrinkleMesh(
    size_t pointCount,
    const std::vector<int> &counts,
    const std::vector<int> &indices,
    RigExecWrinkleTopology mode,
    int neighborDistance,
    RigExecWrinkleMesh *out)
{
    if (!out) return false;
    RigExecWrinkleMesh built;
    built.pointCount = pointCount;
    built.counts = counts;
    built.indices = indices;
    built.mode = mode;
    built.neighborDistance = neighborDistance;
    std::vector<std::pair<int, int>> polygonEdges;
    std::vector<std::array<int, 3>> triangles;
    std::vector<int> corners;
    size_t offset = 0, faceIndex = 0;
    for (int count : counts) {
        if (count < 3 || static_cast<size_t>(count) > indices.size() - offset) {
            return false;
        }
        corners.clear();
        for (int corner = 0; corner < count; ++corner) {
            const int a = indices[offset + corner];
            const int b = indices[offset + (corner + 1) % count];
            if (a < 0 || b < 0 || static_cast<size_t>(a) >= pointCount ||
                static_cast<size_t>(b) >= pointCount) {
                return false;
            }
            corners.push_back(a);
            polygonEdges.emplace_back(std::minmax(a, b));
        }
        // The kernel's per-face duplicate-corner verdict, without the set:
        // every corner appears as `a` exactly once, so a sorted adjacent
        // match is the same rejection.
        std::sort(corners.begin(), corners.end());
        for (size_t i = 1; i < corners.size(); ++i) {
            if (corners[i] == corners[i - 1]) {
                return false;
            }
        }
        const int origin = indices[offset];
        if (count == 4 && faceIndex % 2) {
            triangles.push_back({indices[offset], indices[offset + 1], indices[offset + 3]});
            triangles.push_back({indices[offset + 1], indices[offset + 2], indices[offset + 3]});
        } else {
            for (int corner = 1; corner + 1 < count; ++corner) {
                triangles.push_back({origin, indices[offset + corner],
                                     indices[offset + corner + 1]});
            }
        }
        offset += static_cast<size_t>(count);
        ++faceIndex;
    }
    if (offset != indices.size()) {
        return false;
    }
    // Border vertices: polygon edges used exactly once, as the map count
    // the kernel consulted. Sorted runs give the same verdicts.
    std::sort(polygonEdges.begin(), polygonEdges.end());
    for (size_t i = 0; i < polygonEdges.size();) {
        size_t j = i + 1;
        while (j < polygonEdges.size() && polygonEdges[j] == polygonEdges[i]) {
            ++j;
        }
        if (j - i == 1) {
            built.borderVertices.push_back(polygonEdges[i].first);
            built.borderVertices.push_back(polygonEdges[i].second);
        }
        i = j;
    }
    std::sort(built.borderVertices.begin(), built.borderVertices.end());
    built.borderVertices.erase(
        std::unique(built.borderVertices.begin(), built.borderVertices.end()),
        built.borderVertices.end());
    // Triangle edges in map order with per-key opposites in triangle
    // order: the stable sort keeps each key's list in the order the
    // triangle walk pushed it.
    std::vector<std::pair<std::pair<int, int>, int>> keyed;
    keyed.reserve(triangles.size() * 3);
    for (const std::array<int, 3> &triangle : triangles) {
        for (int corner = 0; corner < 3; ++corner) {
            keyed.emplace_back(
                std::minmax(triangle[corner], triangle[(corner + 1) % 3]),
                triangle[(corner + 2) % 3]);
        }
    }
    std::stable_sort(keyed.begin(), keyed.end(),
                     [](const std::pair<std::pair<int, int>, int> &x,
                        const std::pair<std::pair<int, int>, int> &y) {
                         return x.first < y.first;
                     });
    built.triangles = triangles;
    built.edges.reserve(keyed.size());
    for (size_t i = 0; i < keyed.size();) {
        size_t j = i + 1;
        while (j < keyed.size() && keyed[j].first == keyed[i].first) {
            ++j;
        }
        RigExecWrinkleEdge edge;
        edge.a = keyed[i].first.first;
        edge.b = keyed[i].first.second;
        edge.opposite.reserve(j - i);
        for (size_t k = i; k < j; ++k) {
            edge.opposite.push_back(keyed[k].second);
        }
        built.edges.push_back(std::move(edge));
        i = j;
    }
    built.neighbors.assign(pointCount, {});
    for (const RigExecWrinkleEdge &edge : built.edges) {
        built.neighbors[edge.a].push_back(edge.b);
        built.neighbors[edge.b].push_back(edge.a);
    }
    for (std::vector<int> &ring : built.neighbors) {
        std::sort(ring.begin(), ring.end());
    }
    struct EdgeKeyLess {
        bool operator()(const RigExecWrinkleEdge &edge,
                        const std::pair<int, int> &key) const {
            return edge.a < key.first ||
                   (edge.a == key.first && edge.b < key.second);
        }
        bool operator()(const std::pair<int, int> &key,
                        const RigExecWrinkleEdge &edge) const {
            return key.first < edge.a ||
                   (key.first == edge.a && key.second < edge.b);
        }
    };
    const auto haveEdge = [&built](const std::pair<int, int> &pair) {
        return std::binary_search(built.edges.begin(), built.edges.end(),
                                  pair, EdgeKeyLess());
    };
    std::vector<std::pair<int, int>> bendingPairs;
    if (mode == RigExecWrinkleTopology::Cloth) {
        for (const RigExecWrinkleEdge &edge : built.edges) {
            if (edge.opposite.size() == 2 &&
                edge.opposite[0] != edge.opposite[1]) {
                const std::pair<int, int> pair =
                    std::minmax(edge.opposite[0], edge.opposite[1]);
                if (!haveEdge(pair)) {
                    bendingPairs.push_back(pair);
                }
            }
        }
    } else if (neighborDistance > 1) {
        // Exact graph-hop rings on the implicit triangle topology, as the
        // kernel walks them: per-start BFS over the sorted neighbor rings.
        std::vector<int> distance(pointCount, -1), visited;
        for (size_t start = 0; start < pointCount; ++start) {
            visited.clear();
            visited.push_back(int(start));
            distance[start] = 0;
            for (size_t cursor = 0; cursor < visited.size(); ++cursor) {
                const int a = visited[cursor];
                if (distance[a] == neighborDistance) continue;
                for (int b : built.neighbors[a]) {
                    if (distance[b] >= 0) continue;
                    distance[b] = distance[a] + 1;
                    visited.push_back(b);
                    if (distance[b] == neighborDistance && size_t(b) > start) {
                        bendingPairs.emplace_back(int(start), b);
                    }
                }
            }
            for (int index : visited) distance[index] = -1;
        }
    }
    std::sort(bendingPairs.begin(), bendingPairs.end());
    bendingPairs.erase(std::unique(bendingPairs.begin(), bendingPairs.end()),
                       bendingPairs.end());
    built.bendingPairs = std::move(bendingPairs);
    built.valid = true;
    *out = std::move(built);
    return true;
}

} // namespace rigExec
#endif // RIGEXEC_MATH_MESH_CONNECTIVITY_H
