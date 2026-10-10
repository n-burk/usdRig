// Guided quasistatic constraints based on Muller and Chentanez, Wrinkle Meshes
// (2010), https://matthias-research.github.io/pages/publications/wrinkleMeshes.pdf
// See docs/concepts/wrinkle-deformation.md for the stateless adaptation.
#ifndef RIGEXEC_MATH_WRINKLE_KERNEL_H
#define RIGEXEC_MATH_WRINKLE_KERNEL_H

#include "wrinkleSettings.h"
#include "meshConnectivity.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {
namespace wrinkleDetail {

template<class To, class From>
To Convert(const From &p) { return To(p[0], p[1], p[2]); }

template<class V>
bool IsFinite(const V &p)
{
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

template<class V>
double Length(const V &p)
{
    return std::hypot(double(p[0]), double(p[1]), double(p[2]));
}

template<class V>
double Dot(const V &a, const V &b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

template<class V>
V Cross(const V &a, const V &b)
{
    return V(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0]);
}

inline bool ValidSettings(const RigExecWrinkleSettings &s)
{
    const auto unit = [](float v) { return std::isfinite(v) && v >= 0 && v <= 1; };
    const auto positive = [](float v) { return std::isfinite(v) && v >= 0; };
    return s.iterations >= 0 && s.iterations <= 1000 &&
        (s.topology == RigExecWrinkleTopology::Cloth ||
         s.topology == RigExecWrinkleTopology::SurfaceStruts) &&
        s.neighborDistance >= 1 && s.neighborDistance <= 8 &&
        positive(s.restLengthScale) && s.restLengthScale > 0 &&
        unit(s.stretchStiffness) && unit(s.compressionStiffness) &&
        unit(s.bendStiffness) && positive(s.maxDisplacement) &&
        positive(s.tangentPlaneInset) && positive(s.wrinkleScale) &&
        s.smoothingIterations >= 0 && s.smoothingIterations <= 100;
}

inline double Seed(size_t index)
{
    uint32_t value = static_cast<uint32_t>(index) + 0x9e3779b9u;
    value = (value ^ (value >> 16)) * 0x85ebca6bu;
    value = (value ^ (value >> 13)) * 0xc2b2ae35u;
    value ^= value >> 16;
    return double(value >> 8) * (2.0 / 16777215.0) - 1.0;
}

struct Constraint {
    int a, b;
    double restLength;
    bool bending;
};

} // namespace wrinkleDetail

/// Full 3D affine edge projections, bounded by incoming attachment balls and
/// optional outward tangent halfspaces. Point/Wide provide storage
/// and arithmetic only; this shared implementation has no USD dependency.
///
/// The apply half of the split: \p topology carries the cached
/// triangulation, sorted edges, neighbor rings, border vertices, and
/// bending pairs RigExecBuildWrinkleMesh derived from (counts, indices),
/// and the same arithmetic the single-call kernel performs runs over it.
/// A topology built from anything else fails closed, exactly as its own
/// build would have verdict-ed.
template<class Point, class Wide>
bool
RigExecApplyWrinkleWithTopology(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const RigExecWrinkleSettings &settings,
    const RigExecWrinkleMesh &topology)
{
    using namespace wrinkleDetail;
    if (!points || points->size() != rest.size() || !ValidSettings(settings)) return false;
    const size_t pointCount = rest.size();
    std::vector<Wide> incoming, restWide;
    incoming.reserve(pointCount);
    restWide.reserve(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        if (!IsFinite(rest[i]) || !IsFinite((*points)[i])) return false;
        incoming.push_back(Convert<Wide>((*points)[i]));
        restWide.push_back(Convert<Wide>(rest[i]));
    }
    std::vector<bool> pinned(pointCount, false);
    for (int index : settings.pinPoints) {
        if (index < 0 || size_t(index) >= pointCount || pinned[index]) return false;
        pinned[index] = true;
    }
    if (!topology.Covers(pointCount, counts, indices,
                         settings.topology, settings.neighborDistance)) {
        return false;
    }

    // Posed face normals: the one face walk the apply half still makes, in
    // the single-call kernel's order. The shape verdicts (short faces,
    // overruns, range, duplicates) are the cached build's, covered above;
    // the area finiteness verdict is posed, so it stays here.
    std::vector<Wide> normals(pointCount, Wide(0));
    size_t offset = 0;
    for (int count : counts) {
        const int origin = indices[offset];
        Wide area(0);
        for (int corner = 1; corner + 1 < count; ++corner) {
            area += Cross(incoming[indices[offset + corner]] - incoming[origin],
                          incoming[indices[offset + corner + 1]] - incoming[origin]);
        }
        if (!IsFinite(area)) return false;
        for (int corner = 0; corner < count; ++corner) normals[indices[offset + corner]] += area;
        offset += size_t(count);
    }
    if (settings.pinBorders) {
        for (int index : topology.borderVertices) pinned[index] = true;
    }
    for (size_t i = 0; i < pointCount; ++i) {
        const double length = Length(normals[i]);
        if (!std::isfinite(length)) return false;
        if (length == 0) pinned[i] = true;
        else normals[i] /= length;
    }
    if (pointCount == 0 || settings.iterations == 0 || settings.maxDisplacement == 0 ||
        settings.wrinkleScale == 0) return true;

    // The cached triangle edges supply stable solve order and the cached
    // bending pairs the distance-based bending model; rest lengths stay
    // per-frame, exactly as the single-call kernel derived them.
    const std::vector<std::vector<int>> &neighbors = topology.neighbors;
    std::vector<Constraint> constraints;
    const auto addConstraint = [&](int a, int b, bool bending) {
        const double length = Length(restWide[a] - restWide[b]);
        if (!std::isfinite(length)) return false;
        if (length > 0) {
            const double scaledLength = length * double(settings.restLengthScale);
            if (!std::isfinite(scaledLength)) return false;
            constraints.push_back({a, b, scaledLength, bending});
        }
        return true;
    };
    for (const RigExecWrinkleEdge &edge : topology.edges) {
        if (!addConstraint(edge.a, edge.b, false)) return false;
    }
    for (const auto &pair : topology.bendingPairs) {
        if (!addConstraint(pair.first, pair.second, true)) return false;
    }

    const auto stiffness = [&](const Constraint &constraint, double length) {
        if (constraint.bending) return double(settings.bendStiffness);
        const double selected = length < constraint.restLength
            ? settings.compressionStiffness : settings.stretchStiffness;
        const double shared = std::min(settings.compressionStiffness, settings.stretchStiffness);
        const double strain = std::abs(length / constraint.restLength - 1.0);
        const double blend = std::min(1.0, strain / .02);
        // Unequal stiffnesses must not switch the entire linear system at
        // zero strain. This blend also preserves an exactly disabled branch.
        return shared + (selected - shared) * blend * blend * (3.0 - 2.0 * blend);
    };
    const double strainTolerance = 1e-6;
    bool active = false;
    std::vector<double> compression(pointCount, 0);
    for (const auto &constraint : constraints) {
        if (pinned[constraint.a] && pinned[constraint.b]) continue;
        const double length = Length(incoming[constraint.b] - incoming[constraint.a]);
        if (!std::isfinite(length)) return false;
        if (stiffness(constraint, length) == 0) continue;
        const double strain = 1.0 - length / constraint.restLength;
        if (std::abs(strain) <= strainTolerance) continue;
        active = true;
        if (strain > strainTolerance) {
            compression[constraint.a] = std::max(compression[constraint.a], strain);
            compression[constraint.b] = std::max(compression[constraint.b], strain);
        }
    }
    if (!active) return true;

    const auto project = [&](size_t i, Wide value) {
        if (pinned[i]) return incoming[i];
        Wide delta = value - incoming[i];
        if (settings.tangentPlaneCollisions) {
            const double height = Dot(delta, normals[i]);
            if (height < -double(settings.tangentPlaneInset))
                delta += (-double(settings.tangentPlaneInset) - height) * normals[i];
        }
        const double length = Length(delta);
        if (length > double(settings.maxDisplacement))
            delta *= double(settings.maxDisplacement) / length;
        return incoming[i] + delta;
    };

    // Transport a material-index phase field along relatively uncompressed
    // edges. Smooth weights keep the field continuous at compression onset.
    std::vector<double> phase(pointCount);
    std::vector<std::vector<double>> phaseWeights(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        phase[i] = Seed(i);
        phaseWeights[i].reserve(neighbors[i].size());
        for (int neighbor : neighbors[i]) {
            const double reference = Length(restWide[neighbor] - restWide[i]) *
                double(settings.restLengthScale);
            const double strain = reference > 0 ? std::max(0.0,
                1.0 - Length(incoming[neighbor] - incoming[i]) / reference) : 0;
            const double localCompression = std::max(compression[i], compression[neighbor]);
            phaseWeights[i].push_back(std::pow(std::max(0.0,
                1.0 - strain / (localCompression + .002)), 8));
        }
    }
    // One alternate buffer for all passes, swapped between them: each
    // pass writes every element (the else keeps the per-pass copy's
    // behavior for zero-weight vertices), so no pass reads stale data.
    std::vector<double> nextPhase = phase;
    for (int pass = 0; pass < 10; ++pass) {
        for (size_t i = 0; i < pointCount; ++i) {
            double sum = 0, weight = 0;
            for (size_t j = 0; j < neighbors[i].size(); ++j) {
                sum += phase[neighbors[i][j]] * phaseWeights[i][j];
                weight += phaseWeights[i][j];
            }
            nextPhase[i] = weight > 0 ? .5 * (phase[i] + sum / weight)
                                      : phase[i];
        }
        phase.swap(nextPhase);
    }
    const auto phaseRange = std::minmax_element(phase.begin(), phase.end());
    const double phaseWidth = .1 * (*phaseRange.second - *phaseRange.first) + 1e-6;

    // Fix each edge's lift branch before relaxation. A soft phase sign and
    // smooth missing-length onset avoid introducing threshold jumps. Unlike
    // nonlinear distance projection, the solve cannot change buckling basins.
    std::vector<Wide> targets;
    std::vector<double> strengths;
    targets.reserve(constraints.size());
    strengths.reserve(constraints.size());
    for (const auto &constraint : constraints) {
        const int a = constraint.a, b = constraint.b;
        const Wide baseEdge = incoming[b] - incoming[a];
        const double length = Length(baseEdge);
        Wide target = baseEdge;
        if (length > constraint.restLength) {
            target *= constraint.restLength / length;
        } else {
            Wide normal = normals[a] + normals[b];
            const double normalLength = Length(normal);
            if (normalLength > 0) {
                normal /= normalLength;
                const double ratio = length / constraint.restLength;
                const double deficit = std::max(0.0, 1.0 - ratio * ratio);
                const double difference = phase[b] - phase[a];
                const double height = difference / std::hypot(difference, phaseWidth) *
                    constraint.restLength * deficit / std::sqrt(deficit + .02);
                target += height * normal;
            }
        }
        if (!IsFinite(target)) return false;
        targets.push_back(target);
        strengths.push_back(stiffness(constraint, length));
    }

    std::vector<Wide> current = incoming;
    for (int iteration = 0; iteration < settings.iterations; ++iteration) {
        for (size_t cursor = 0; cursor < constraints.size(); ++cursor) {
            const size_t index = iteration % 2 ? constraints.size() - 1 - cursor : cursor;
            const auto &constraint = constraints[index];
            const int a = constraint.a, b = constraint.b;
            const double weightA = pinned[a] ? 0.0 : 1.0;
            const double weightB = pinned[b] ? 0.0 : 1.0;
            if (weightA + weightB == 0) continue;
            const Wide edge = current[b] - current[a];
            const Wide correction = (edge - targets[index]) *
                (strengths[index] / (weightA + weightB));
            if (weightA) current[a] += correction;
            if (weightB) current[b] -= correction;
        }
        for (size_t i = 0; i < pointCount; ++i) {
            if (!IsFinite(current[i])) return false;
            current[i] = project(i, current[i]);
        }
    }

    std::vector<Wide> deltas(pointCount);
    for (size_t i = 0; i < pointCount; ++i) deltas[i] = current[i] - incoming[i];
    // As above: pinned and isolated vertices keep their values
    // through the else, exactly as the per-iteration copy did.
    std::vector<Wide> nextDeltas = deltas;
    for (int iteration = 0; iteration < settings.smoothingIterations; ++iteration) {
        for (size_t i = 0; i < pointCount; ++i) {
            if (pinned[i] || neighbors[i].empty()) {
                nextDeltas[i] = deltas[i];
                continue;
            }
            Wide average(0);
            for (int neighbor : neighbors[i]) average += deltas[neighbor];
            average /= double(neighbors[i].size());
            nextDeltas[i] = .5 * (deltas[i] + average);
        }
        deltas.swap(nextDeltas);
    }
    std::vector<Point> result = *points;
    for (size_t i = 0; i < pointCount; ++i) {
        if (pinned[i]) continue;
        const Wide value = project(i, incoming[i] + double(settings.wrinkleScale) * deltas[i]);
        if (!IsFinite(value)) return false;
        result[i] = Convert<Point>(value);
        if (!IsFinite(result[i])) return false;
    }
    *points = std::move(result);
    return true;
}

/// Single-call form: builds the topology half, then applies it. Behavior
/// is bit-exact against the split by construction; revisions that deform
/// the same mesh every frame retain the topology and call the apply half.
template<class Point, class Wide>
bool
RigExecApplyWrinkleKernel(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const RigExecWrinkleSettings &settings)
{
    RigExecWrinkleMesh topology;
    if (!RigExecBuildWrinkleMesh(rest.size(), counts, indices,
                                 settings.topology,
                                 settings.neighborDistance, &topology)) {
        return false;
    }
    return RigExecApplyWrinkleWithTopology<Point, Wide>(
        points, rest, counts, indices, settings, topology);
}

} // namespace rigExec
#endif // RIGEXEC_MATH_WRINKLE_KERNEL_H
