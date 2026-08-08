//
// RigExec geometry mover kernels (spec §7.5–§7.6).
//
// Pure, deterministic CPU reference kernels over exact native property
// values. Callers collect the vectorized element stream into transient
// scratch, apply one kernel, and write the result back before the
// callback returns (spec §6.5 ephemeral-scratch rule).
//
#ifndef RIGEXEC_MATH_GEOMETRY_KERNELS_H
#define RIGEXEC_MATH_GEOMETRY_KERNELS_H

#include "pointFrame.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"

#include <vector>

namespace rigExec {

/// Axis-aligned bound volume of a point set (zero for < 2 points).
double RigExecBoundVolume(const GfVec3f *points, size_t count);

/// Volume/segment-length correction (RigExecPostMover "volumeCorrect",
/// spec §7.6): uniformly scales the point set about its centroid so its
/// bound volume moves toward the reference volume; strength in [0, 1]
/// blends between no correction (0) and full restoration (1).
void RigExecApplyVolumeCorrect(
    std::vector<GfVec3f> *points, double referenceVolume, double strength);

/// One iteration of uniform Laplacian smoothing with fixed adjacency
/// derived from standard mesh topology (RigExecPostMover "smooth",
/// spec §7.6): p' = lerp(p, average of edge-connected neighbors, strength).
/// Points without neighbors are unchanged.
void RigExecApplyLaplacianSmooth(
    std::vector<GfVec3f> *points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    double strength);

/// Angle-weighted vertex normals from standard polygon topology
/// (RigExecPostMover "recomputeNormals", spec §7.6).
///
/// Each face contributes its Newell normal to its own corners, weighted by
/// the interior angle there, so the result is correct for a non-planar
/// n-gon and independent of how that n-gon would be triangulated.
std::vector<GfVec3f> RigExecComputeVertexNormals(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices);

/// Two-element extent (min, max) from final local points, optionally
/// widened by the winning widths (curves/points rule, spec §7.6).
std::vector<GfVec3f> RigExecComputeExtent(
    const std::vector<GfVec3f> &points,
    const std::vector<float> &widths);

/// Tensor-product Bernstein lattice evaluation (RigExecLatticeMover,
/// spec §7.5): p'(u,v,w) = sum_abc B_a(u) B_b(v) B_c(w) C_abc.
///
/// The cage is divisions.x * divisions.y * divisions.z control points in
/// x-fastest lexicographic order. Bind coordinates derive from each rest
/// point normalized into the rest cage's bound; points outside the cage
/// bound clamp to it. restPoints/restCage define the bind; posedCage
/// drives the deformation. Identity when the cage is at rest.
void RigExecApplyLattice(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &restPoints,
    const std::vector<GfVec3f> &restCage,
    const std::vector<GfVec3f> &posedCage,
    const GfVec3i &divisions);

/// Closest point on a triangulated standard mesh (RigExecSurfaceMover
/// "project", spec §7.5): p' = lerp(p, closestSurfacePoint(p), weight).
void RigExecApplySurfaceProject(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &surfacePoints,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    double weight);

/// Sampled curve frames: positions plus a rotation-minimizing frame per
/// sample (spec §7.5), parameterized by normalized arc length.
struct RigExecCurveFrameSamples {
    std::vector<GfVec3f> positions;
    std::vector<GfVec3f> tangents;
    std::vector<GfVec3f> normals;    ///< rotation-minimizing
    std::vector<GfVec3f> binormals;
    std::vector<float> parameters;   ///< normalized arc length in [0, 1]

    size_t GetSize() const { return positions.size(); }
};

/// Samples a single nonperiodic cubic uniform B-spline (the stock
/// BasisCurves "cubic"/"bspline" contract) at sampleCount arc-length
/// parameters, transporting a rotation-minimizing frame by the
/// double-reflection method. Fewer than four control points fall back to
/// linear interpolation of the control polygon.
RigExecCurveFrameSamples RigExecSampleCurveRMF(
    const std::vector<GfVec3f> &controlPoints, int sampleCount);

/// Rest-relative ribbon transport (RigExecCurveMover "ribbon", spec
/// §7.5): each destination point binds to the curve parameter u from its
/// declared two-component bind coordinates; the point moves by the rigid
/// map taking its rest frame sample to its posed frame sample:
///   p' = M(u) p,  M(u) = PointsToMatrix(restFrame(u), posedFrame(u)).
/// Identity when the posed samples equal the rest samples.
void RigExecApplyRibbonTransport(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec2f> &bindCoords,
    const RigExecCurveFrameSamples &restSamples,
    const RigExecCurveFrameSamples &posedSamples);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_GEOMETRY_KERNELS_H
