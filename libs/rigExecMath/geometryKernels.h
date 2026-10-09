// RigExec geometry mover kernels (spec §7.5–§7.6).
// Pure, deterministic CPU reference kernels over exact native property
// values. Callers collect the vectorized element stream into transient
// scratch, apply one kernel, and write the result back before the
// callback returns (spec §6.5 ephemeral-scratch rule).
#ifndef RIGEXEC_MATH_GEOMETRY_KERNELS_H
#define RIGEXEC_MATH_GEOMETRY_KERNELS_H

#include "pointFrame.h"
#include "wrinkleSettings.h"
#include "meshConnectivity.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"

#include <memory>
#include <vector>
#include "rigExecMath/deltaMushKernel.h"
#include "rigExecMath/spatialAccel.h"
#include "rigExecMath/surfaceProjectorKernel.h"

namespace rigExec {

/// The concrete delta-mush rest state the evaluator caches: GfVec3f
/// points, GfVec3d working frames -- the instantiation every USD-side
/// caller shares. (The binary runtime instantiates the same templates
/// over its own vector types; the alias lives here, with USD, rather
/// than in the USD-free kernel header.)
using RigExecDeltaMushRest = RigExecDeltaMushRestData<GfVec3f, GfVec3d>;

/// The concrete transport rest state every USD-side caller shares:
/// GfVec3f points, GfVec3d working frames.
using RigExecTransportRest = RigExecTransportRestData<GfVec3f, GfVec3d>;

/// Smooth rest and posed meshes with identical rest-derived weights, then
/// transport the rest detail onto the smoothed posed surface. No host runtime.
/// Invalid input fails atomically; isolated vertices and pinned borders stay put.
/// \p restData supplies prebuilt rest state; null builds it from the inputs.
bool RigExecApplyDeltaMush(
    std::vector<GfVec3f> *points, const std::vector<GfVec3f> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations = 10, double step = 0.5, bool pinBorders = true,
    double distanceWeight = 0.0, double displacement = 1.0,
    const RigExecDeltaMushRest *restData = nullptr);

/// Resolve phase-guided rest-length constraints inside attachment balls about
/// incoming points. A material phase field stabilizes the folds. Quasistatic:
/// no history, velocity, or authored waveform. Invalid inputs fail atomically;
/// pins, isolated vertices, and degenerate normals stay put.
/// See docs/concepts/wrinkle-deformation.md and the cited Wrinkle Meshes paper.
/// \p topology supplies prebuilt connectivity; null builds it from the inputs.
bool RigExecApplyWrinkle(
    std::vector<GfVec3f> *points, const std::vector<GfVec3f> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const RigExecWrinkleSettings &settings = {},
    const RigExecWrinkleMesh *topology = nullptr);

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
///
/// \p source lends the read side: a caller that seeded \p points from a
/// stable buffer passes it so the kernel reads there instead of copying.
/// Null (the default) keeps the owned copy; a non-null range covering
/// anything but the points passes through, as invalid topology does.
void RigExecApplyLaplacianSmooth(
    std::vector<GfVec3f> *points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    double strength,
    const GfVec3f *source = nullptr,
    size_t sourceCount = 0);

/// The span form borrows every input range for the call only: the
/// caller keeps the storage (e.g. the VtArray behind cdata()) alive
/// until it returns. Bit-identical to the vector form, which
/// delegates to it.
void RigExecApplyLaplacianSmooth(
    std::vector<GfVec3f> *points,
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize,
    double strength,
    const GfVec3f *source = nullptr,
    size_t sourceCount = 0);

/// The smoothing above over prebuilt connectivity: the same lerp against
/// the same neighbor lists, without rebuilding them. An entry covering
/// anything but the points passes through, as invalid topology does.
void RigExecApplyLaplacianSmoothWithAdjacency(
    std::vector<GfVec3f> *points,
    const RigExecMeshAdjacency &adjacency,
    double strength,
    const GfVec3f *source = nullptr,
    size_t sourceCount = 0);


/// Cast a ray at a mesh: Moller-Trumbore against each face fan-triangulated
/// about its first corner, two-sided, nearest hit strictly in front of the
/// origin. Returns false, leaving hit untouched, when nothing is met, the
/// direction is degenerate, or the topology is invalid.
bool RigExecRaycastSurface(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    const GfVec3d &origin,
    const GfVec3d &direction,
    RigExecSurfaceHit *hit);

/// The surface frame at a hit, evaluated on the given points: the hit's
/// barycentric position as the translation and the surface normal there as
/// +Z. The normal is the barycentric blend of RigExecComputeVertexNormals,
/// so it varies smoothly across a face rather than stepping from triangle
/// to triangle.
///
/// This is what puts a projector on a deforming eyeball. The ray is cast
/// ONCE, at the rest points, and picks a material point on the cornea; the
/// posed points then say where that point went and which way the surface
/// faces there. Re-casting at the posed surface would measure something
/// else -- where the surface happens to cross a fixed line -- and a
/// surface sliding across the ray would read as standing still while its
/// material moved. A rigid motion of the mesh comes back exactly: the frame
/// on the moved points is the rest frame carried by that motion.
///
/// upHint fixes the roll, which the normal alone cannot: it is orthogonalized
/// against the normal, and a hint parallel to it (or degenerate) falls back
/// to the least-aligned principal axis, so a frame always comes back rather
/// than failing at a pole. Returns false only when the hit or the topology
/// is invalid, leaving frame untouched.
bool RigExecSurfaceFrameAtHit(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    const RigExecSurfaceHit &hit,
    const GfVec3d &upHint,
    GfMatrix4d *frame);

/// RigExecRaycastSurface followed by RigExecSurfaceFrameAtHit on the same
/// points: the frame where the ray meets THIS surface. Returns false only
/// when nothing is hit or the topology is invalid, leaving frame untouched.
bool RigExecRaycastSurfaceFrame(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    const GfVec3d &origin,
    const GfVec3d &direction,
    const GfVec3d &upHint,
    GfMatrix4d *frame);

/// A fraction of a transform, taken as a fraction of its ROTATION.
///
/// The point-domain matrix mover blends linearly -- p' = q + w(Tq - q) --
/// which is exact at both ends and a straight line in between. Under a
/// rotation that straight line is the CHORD, and a chord always falls
/// inside its arc, toward the axis. For a cluster that rotates a lid
/// about an eyeball that is the difference between the lid sweeping over
/// the eye and the lid cutting into it: the sag at weight one half is
/// r(1 - cos(theta/2)), which on a 2.48 radius eye at a 56 degree blink
/// is 0.29 -- a tenth of the eyeball.
///
/// This takes the same axis and a fraction of the ANGLE, so every point
/// keeps its distance from the axis however it is weighted. Translation
/// and scale scale linearly, which is what they mean. Endpoints agree
/// with the linear form exactly, so weight 0 and weight 1 are unchanged.
GfMatrix4d RigExecPartialTransform(const GfMatrix4d &transform,
                                   double weight);

/// The weight-independent half of RigExecPartialTransform: scale
/// factors, screw axis and angle, translation, and the recovered
/// pivot. Decomposing once and applying per weight keeps an
/// alternating fractional falloff from paying a decomposition per
/// distinct weight; endpoints need no decomposition at all.
struct RigExecPartialDecomposition {
    GfVec3d scale{1.0};
    GfVec3d axis{1.0, 0.0, 0.0};
    double angle = 0.0;
    GfVec3d translation{0.0};
    double along = 0.0;
    GfVec3d pivot{0.0};
    bool smallAngle = true;
};

/// The decomposition behind RigExecPartialTransform, for callers
/// that apply several weights to one transform.
RigExecPartialDecomposition RigExecDecomposePartialTransform(
    const GfMatrix4d &transform);

/// The weight-dependent half for a clamped \p w in (0, 1), exactly
/// the fractional matrix RigExecPartialTransform returns.
GfMatrix4d RigExecApplyPartialDecomposition(
    const RigExecPartialDecomposition &decomposition, double w);

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

/// The vector form's signature, for callers taking its address as the
/// normals callback (the span overload below would make &Name ambiguous).
using RigExecVertexNormalsFn = std::vector<GfVec3f> (*)(
    const std::vector<GfVec3f> &, const std::vector<int> &,
    const std::vector<int> &);

/// The span form borrows every input range for the call only: the
/// caller keeps the storage (e.g. the VtArray behind cdata()) alive
/// until it returns. Bit-identical to the vector form, which
/// delegates to it.
std::vector<GfVec3f> RigExecComputeVertexNormals(
    const GfVec3f *points, size_t pointsSize,
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize);

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

/// The span form borrows every input range for the call only: the
/// caller keeps the storage (e.g. the VtArray behind cdata()) alive
/// until it returns. Bit-identical to the vector form, which
/// delegates to it.
std::vector<GfVec3f> RigExecComputeExtent(
    const GfVec3f *points, size_t pointsSize,
    const float *widths, size_t widthsSize);

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

/// The span form borrows every input range for the call only: the
/// caller keeps the storage (e.g. the VtArray behind cdata()) alive
/// until it returns. Bit-identical to the vector form, which
/// delegates to it.
void RigExecApplyLattice(
    std::vector<GfVec3f> *points,
    const GfVec3f *restPoints, size_t restPointsSize,
    const GfVec3f *restCage, size_t restCageSize,
    const GfVec3f *posedCage, size_t posedCageSize,
    const GfVec3i &divisions);

/// The vector form with the bind's per-point Bernstein factors retained in
/// \p cache, the caller's own per-revision cache, within its budget. Null,
/// like the forms above, streams them one point at a time. Bit-identical to
/// the forms above (latticeKernel.h).
void RigExecApplyLattice(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &restPoints,
    const std::vector<GfVec3f> &restCage,
    const std::vector<GfVec3f> &posedCage,
    const GfVec3i &divisions,
    RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache);

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
/// across threads. restEvals, when given, supplies C0(u) for every point
/// and must cover the points; a null table evaluates the rest curve per
/// point instead. Returns false, writing nothing, on a shape mismatch.
bool RigExecApplyWire(
    std::vector<GfVec3f> *points,
    const RigExecNurbsCurve &restCurve,
    const RigExecNurbsCurve &posedCurve,
    const GfVec2f *bindCoords, size_t bindCount,
    double dropoffDistance,
    size_t begin, size_t end,
    const GfVec3f *restEvals = nullptr, size_t restEvalCount = 0);

/// RigExecApplyWire's validation prefix over \p pointCount points, apart
/// from the rest-evaluation table: both curves evaluable and alike (order,
/// control point count, knots) and one bind coordinate per point. The one
/// definition RigExecApplyWire checks and a caller deciding before the
/// kernel runs reads.
bool RigExecWireInputsAreUsable(
    const RigExecNurbsCurve &restCurve,
    const RigExecNurbsCurve &posedCurve,
    const GfVec2f *bindCoords, size_t bindCount, size_t pointCount);

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

/// RigExecBuildWireBasis's validation of its inputs before it builds
/// anything, with \p indexCount weighted points: the bind table's storage
/// and size and the curve layout. The per-index range check stays in the
/// build; a caller that already proved the indices in range has its whole
/// answer here.
bool RigExecWireBasisInputsAreUsable(
    const GfVec2f *bindCoords, size_t bindCount, size_t meshPointCount,
    size_t indexCount, int order, const std::vector<double> &knots,
    size_t controlPointCount);

/// Applies a prebuilt basis: point indices[k] moves by
/// weights[k] * sum_j coefficient * (posed[j] - rest[j]), summed over only
/// the control points that moved.
bool RigExecApplyWireBasis(
    std::vector<GfVec3f> *points, const RigExecWireBasis &basis,
    const std::vector<int> &indices, const std::vector<float> &weights,
    const std::vector<GfVec3f> &restControlPoints,
    const std::vector<GfVec3f> &posedControlPoints);

/// The surface projector's cached query data: the fan triangulation and
/// the candidate index, built once and shared across frames. Triangle
/// candidates remain non-prunable to preserve legacy float arithmetic.
/// A null bvh runs the direct fan loop: either the fan is below the gate
/// or the build met non-finite surface points. The owned
/// keys are the build inputs: the projector rebuilds unretained rather
/// than answering a stale entry, so a mismatched accel falls back to
/// the per-call build exactly as a null one does.
struct RigExecSurfaceAccel {
    std::shared_ptr<const RigExecFanTrisH7> fan;
    std::shared_ptr<const RigExecTriangleBvh<GfVec3f>> bvh;
    std::vector<int> counts;
    std::vector<int> indices;
    std::vector<GfVec3f> points;
};

/// Builds query data for a surface: always the fan, plus its candidate
/// index when the fan reaches the gate and every referenced point is
/// finite. Always fills \p accel; the bvh member carries the verdict.
void RigExecBuildSurfaceAccel(
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    const std::vector<GfVec3f> &surfacePoints,
    RigExecSurfaceAccel *accel);

/// The span form borrows every input range for the call only: the
/// caller keeps the storage (e.g. the VtArray behind cdata()) alive
/// until it returns. Bit-identical to the vector form, which
/// delegates to it.
void RigExecBuildSurfaceAccel(
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize,
    const GfVec3f *surfacePoints, size_t surfacePointsSize,
    RigExecSurfaceAccel *accel);

/// Closest point on a triangulated standard mesh (RigExecSurfaceMover
/// "project", spec §7.5): p' = lerp(p, closestSurfacePoint(p), weight).
/// \p accel supplies prebuilt triangulation; null builds it per call.
/// An entry built for another surface falls back to the per-call build.
void RigExecApplySurfaceProject(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &surfacePoints,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    double weight,
    const RigExecSurfaceAccel *accel = nullptr);

/// The span form borrows every input range for the call only: the
/// caller keeps the storage (e.g. the VtArray behind cdata()) alive
/// until it returns. Bit-identical to the vector form, which
/// delegates to it.
void RigExecApplySurfaceProject(
    std::vector<GfVec3f> *points,
    const GfVec3f *surfacePoints, size_t surfacePointsSize,
    const int *faceVertexCounts, size_t faceVertexCountsSize,
    const int *faceVertexIndices, size_t faceVertexIndicesSize,
    double weight,
    const RigExecSurfaceAccel *accel = nullptr);

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

/// The span form borrows every input range for the call only: the
/// caller keeps the storage (e.g. the VtArray behind cdata()) alive
/// until it returns. Bit-identical to the vector form, which
/// delegates to it.
RigExecCurveFrameSamples RigExecSampleCurveRMF(
    const GfVec3f *controlPoints, size_t controlPointsSize,
    int sampleCount);

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

/// The span form borrows every input range for the call only: the
/// caller keeps the storage (e.g. the VtArray behind cdata()) alive
/// until it returns. Bit-identical to the vector form, which
/// delegates to it.
void RigExecApplyRibbonTransport(
    std::vector<GfVec3f> *points,
    const GfVec2f *bindCoords, size_t bindCoordsSize,
    const RigExecCurveFrameSamples &restSamples,
    const RigExecCurveFrameSamples &posedSamples);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_GEOMETRY_KERNELS_H
