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

/// Rotate target-local offsets through corresponding rest/posed surface frames.
/// Area-weighted polygon normals supply each vertex normal. The longest
/// projected incident REST edge supplies its tangent (lowest neighbor index
/// wins ties); the corresponding POSED edge supplies the posed tangent.
/// Orthonormal transport preserves offset magnitude, even under mesh scaling.
/// Invalid arrays/topology or a nonzero offset at a degenerate frame fail
/// atomically, leaving out unchanged. A zero offset needs no valid frame.
bool RigExecTransportSurfaceOffsets(
    const std::vector<GfVec3f> &rest,
    const std::vector<GfVec3f> &posed,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    const std::vector<GfVec3f> &deltas,
    std::vector<GfVec3f> *out);

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

/// A non-rational NURBS curve as UsdGeomNurbsCurves authors one: control
/// points, order (degree + 1) and a knot vector of points.size() + order
/// entries. Open and periodic curves differ only in their knots and in the
/// periodic curve's wrapped control points, so both evaluate the same way.
struct RigExecNurbsCurve {
    const std::vector<GfVec3f> *points = nullptr;
    int order = 0;
    const std::vector<double> *knots = nullptr;

    /// True when the arrays describe an evaluable curve.
    bool IsValid() const;
    /// The parameter domain [knots[order-1], knots[pointCount]].
    double DomainStart() const;
    double DomainEnd() const;
    /// The curve point at parameter u (clamped to the domain), by de Boor.
    GfVec3f Evaluate(double u) const;
};

/// Bind coordinates for RigExecApplyWire: per point, (u, d) with u the
/// parameter of the closest point on the REST curve and d the distance to
/// it. Found by sampling every knot span and refining the best sample.
std::vector<GfVec2f> RigExecBindWire(
    const std::vector<GfVec3f> &points, const RigExecNurbsCurve &restCurve);

/// Wire deformation (RigExecCurveMover "wire"): every point moves by the
/// driver curve's displacement at the parameter it was bound to,
///
///   p' = p + f(d) * (C(u) - C0(u))
///
/// with C the posed curve, C0 the rest curve, (u, d) the point's bind
/// coordinates and f = 1 - smoothstep(0, dropoff, d) (f = 1 when dropoff
/// <= 0). The two curves must share order, knots and point count. Only the
/// points in [begin, end) are written, so a caller can split the range
/// across threads. Returns false, writing nothing, on a shape mismatch.
bool RigExecApplyWire(
    std::vector<GfVec3f> *points,
    const RigExecNurbsCurve &restCurve,
    const RigExecNurbsCurve &posedCurve,
    const GfVec2f *bindCoords, size_t bindCount,
    double dropoffDistance,
    size_t begin, size_t end);

/// RigExecApplyWire weighted by a sparse field: only the named points move,
/// each by weight * f(d) * (C(u) - C0(u)). indices must be ascending and in
/// range and weights parallel to them. bindCoords either covers every point
/// or is itself sparse, one entry per index in the same order -- a wire
/// moves only the points its weights name, so that is all it needs bound.
bool RigExecApplyWireSparse(
    std::vector<GfVec3f> *points,
    const RigExecNurbsCurve &restCurve,
    const RigExecNurbsCurve &posedCurve,
    const GfVec2f *bindCoords, size_t bindCount,
    double dropoffDistance,
    const std::vector<int> &indices,
    const std::vector<float> &weights);

/// A sparse wire's deformation as a fixed linear map from control point
/// motion to point motion.
///
/// A wire moves point k by f(d_k) * sum_j N_j(u_k) (C_j - C0_j): u_k and d_k
/// are fixed at bind, the basis functions N_j depend only on the knots and
/// order, so every coefficient f(d_k) N_j(u_k) is a constant until the bind,
/// the knots or the weighted point set change. Stored per control point, so
/// a frame in which one tweak moves touches only the points that control
/// point can reach, with no curve evaluation at all.
struct RigExecWireBasis {
    /// For control point j: (position in the weight index list, coefficient).
    std::vector<std::vector<std::pair<uint32_t, float>>> byControlPoint;
};

/// Builds the basis for the points named by \p indices. \p bindCoords is
/// either one entry per mesh point (\p meshPointCount of them) or one per
/// index. Returns false for an invalid curve layout.
bool RigExecBuildWireBasis(
    const GfVec2f *bindCoords, size_t bindCount, size_t meshPointCount,
    const std::vector<int> &indices, int order,
    const std::vector<double> &knots, size_t controlPointCount,
    double dropoffDistance, RigExecWireBasis *basis);

/// Applies a prebuilt basis: point indices[k] moves by
/// weights[k] * sum_j coefficient * (posed[j] - rest[j]), summed over only
/// the control points that moved.
bool RigExecApplyWireBasis(
    std::vector<GfVec3f> *points, const RigExecWireBasis &basis,
    const std::vector<int> &indices, const std::vector<float> &weights,
    const std::vector<GfVec3f> &restControlPoints,
    const std::vector<GfVec3f> &posedControlPoints);

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
