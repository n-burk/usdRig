//
// Curvenet representation and scaled frames (de Goes, Sheffler & Fleischer,
// "Character Articulation through Profile Curves", ACM TOG 41(4) 2022, §3).
// See docs/curvenet.md; section numbers below are that paper's.
//
// A curvenet is a pool of control points plus a list of four-index tuples
// naming cubic splines over it. Connectivity is index SHARING -- two splines
// meet because they name the same pool entry -- which is what detaches the
// rig from the surface tessellation.
//
// Everything here is pure geometry over that encoding: no USD, no exec, no
// mesh. The mesh only enters in cutMesh.h.
//
#ifndef RIGEXEC_MATH_CURVENET_H
#define RIGEXEC_MATH_CURVENET_H

#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"

#include <functional>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Curve basis for a four-control-point spline tuple.
enum class RigExecCurvenetBasis {
    Bezier,      ///< cubic Bezier: p0, h0, h1, p1 (knots at 0 and 3)
    CatmullRom,  ///< centripetal Catmull-Rom: the curve runs p1 -> p2
};

/// How a pool point participates in the net (§3 "Intersections, anchors &
/// curves"). Valence counts SPLINE ENDPOINTS only; tangent handles are not
/// endpoints of anything.
enum class RigExecCurvenetKnotKind {
    Unused,        ///< named by no spline
    Handle,        ///< only ever an interior control point
    Anchor,        ///< endpoint of exactly one spline
    Interior,      ///< endpoint of exactly two splines: inside a curve
    Intersection,  ///< endpoint of three or more splines
};

/// One curve: a maximal chain of splines bridging intersections and/or
/// anchors, or an isolated cycle of interior knots.
struct RigExecCurvenetCurve {
    std::vector<int> splines;    ///< spline indices in traversal order
    std::vector<bool> reversed;  ///< spline traversed from its p1 to its p0
    int startKnot = -1;          ///< pool index of the first knot
    int endKnot = -1;            ///< pool index of the last knot
    bool closed = false;
    bool startIsIntersection = false;
    bool endIsIntersection = false;

    /// §3 "In the case of isolated curves, either closed or with both ends
    /// at anchors, there is no curvenet structure that complements the
    /// tangent and the length of the curve segments."
    bool IsIsolated() const {
        return !startIsIntersection && !endIsIntersection;
    }
};

/// One segment emanating from an intersection, in the intersection's
/// counter-clockwise order.
struct RigExecCurvenetSpoke {
    int curve = -1;      ///< curve index
    bool atCurveEnd = false;  ///< spoke leaves from the curve's LAST sample
};

/// An intersection and its counter-clockwise fan of curves.
struct RigExecCurvenetIntersection {
    int knot = -1;                              ///< pool index
    std::vector<RigExecCurvenetSpoke> spokes;   ///< sorted counter-clockwise
    GfVec3d referenceNormal{0.0, 0.0, 1.0};     ///< surface normal used to sort
};

/// The derived structure of a curvenet layout (§3). Computed once per
/// layout, in the neutral pose; independent of any later posing.
struct RigExecCurvenetTopology {
    RigExecCurvenetBasis basis = RigExecCurvenetBasis::Bezier;
    int pointCount = 0;
    std::vector<int> splineIndices;  ///< 4 per spline, into the point pool
    std::vector<RigExecCurvenetKnotKind> knotKinds;  ///< per pool point
    std::vector<int> knotValence;                    ///< per pool point
    std::vector<RigExecCurvenetCurve> curves;
    std::vector<RigExecCurvenetIntersection> intersections;
    /// intersections index by pool index, or -1.
    std::vector<int> intersectionOfKnot;

    size_t GetSplineCount() const { return splineIndices.size() / 4; }

    /// The two knot pool indices of spline \p s (its endpoints).
    int GetSplineStartKnot(size_t s) const { return splineIndices[4 * s]; }
    int GetSplineEndKnot(size_t s) const { return splineIndices[4 * s + 3]; }
};

/// Builds the derived structure from a spline-index array.
///
/// \p normalAt supplies the surface normal used to sort each intersection's
/// fan counter-clockwise (§3: "projected orthogonal to the normal of the
/// closest surface point in a neutral pose"). It may be null, in which case
/// the fan is sorted about the best-fit plane of its own tangents, which is
/// stable but not surface-aware.
///
/// Returns false and fills \p error on a malformed layout: an index count
/// that is not a multiple of four, or an out-of-range index.
bool RigExecBuildCurvenetTopology(
    const std::vector<int> &splineIndices,
    size_t pointCount,
    RigExecCurvenetBasis basis,
    const std::vector<GfVec3f> &neutralPoints,
    const std::function<GfVec3d(const GfVec3d &)> *normalAt,
    RigExecCurvenetTopology *topology,
    std::string *error);

/// Recomputes each intersection's reference normal and re-sorts its fan.
///
/// §3 sorts a fan about "the normal of the closest surface point in a neutral
/// pose", and that is not a convenience: the surface normal is what orients
/// every intersection CONSISTENTLY. Orientation derived from a fan's own
/// geometry is sign-arbitrary, so two intersections can disagree by a half
/// turn -- and a curve running between them then reads its two end normals as
/// opposed, invents a 180-degree torsion to reconcile them, and twists the
/// surface through a right angle in the middle. Any caller that has the
/// surface should call this.
void RigExecOrientCurvenetIntersections(
    const std::vector<GfVec3f> &points,
    const std::function<GfVec3d(const GfVec3d &)> &normalAt,
    RigExecCurvenetTopology *topology);

/// Per-spline sample counts, fixed once in the neutral pose (§3 "Sampling").
///
/// count(spline) = round(samplesPerSpline * controlPolygonLength / meanEdgeLength),
/// clamped to at least one, so the discretization is proportional to the
/// surface's element size and adaptive to the undeformed spline's length.
/// Fixing the counts is what makes sample i of a spline correspond to sample
/// i of the same spline in every other pose.
std::vector<int> RigExecPlanCurvenetSamples(
    const RigExecCurvenetTopology &topology,
    const std::vector<GfVec3f> &neutralPoints,
    double meshMeanEdgeLength,
    int samplesPerSpline);

/// The sampled (piecewise-linear) curvenet in one pose.
///
/// Samples are grouped per curve; \p curveBegin has curves+1 entries so
/// curve c owns [curveBegin[c], curveBegin[c+1]). A closed curve does not
/// repeat its first sample.
struct RigExecCurvenetSampling {
    std::vector<GfVec3d> positions;
    std::vector<int> curveBegin;
    /// Pool point index a sample coincides with, or -1. Knots land exactly
    /// on samples, which is what lets adjustments address them.
    std::vector<int> knotOfSample;

    size_t GetSampleCount() const { return positions.size(); }
    size_t GetCurveCount() const {
        return curveBegin.empty() ? 0 : curveBegin.size() - 1;
    }
    /// Samples in curve \p c.
    int GetCurveSampleCount(size_t c) const {
        return curveBegin[c + 1] - curveBegin[c];
    }
};

/// Evaluates the sampled curvenet for \p points using sample counts fixed by
/// RigExecPlanCurvenetSamples. Uniform refinement in parametric space
/// followed by even arc-length resampling, per §3.
RigExecCurvenetSampling RigExecSampleCurvenet(
    const RigExecCurvenetTopology &topology,
    const std::vector<GfVec3f> &points,
    const std::vector<int> &samplesPerSpline);

/// Segment-side scaled frames (§3 "Normal & width interpolation").
///
/// Segments are indexed globally: curve c's segments run
/// [segmentBegin[c], segmentBegin[c+1]), where an open curve with n samples
/// has n-1 segments and a closed one has n.
///
/// Every per-segment array is indexed by that global segment index. The
/// left/right pair is the whole point: a curve is a hinge, and the two sides
/// carry different normals, widths and therefore different deformation
/// gradients.
struct RigExecCurvenetFrames {
    std::vector<int> segmentBegin;    ///< curves+1
    std::vector<GfVec3d> tangent;     ///< unit, shared by both sides
    std::vector<double> length;       ///< l_s
    std::vector<GfVec3d> normalLeft;  ///< n+
    std::vector<GfVec3d> normalRight; ///< n-
    std::vector<double> widthLeft;    ///< w+
    std::vector<double> widthRight;   ///< w-
    /// False where the curve is isolated: no net structure supplies a
    /// normal or a width, and §3 falls back to a rotation-only gradient.
    std::vector<bool> curveFramed;

    size_t GetSegmentCount() const { return tangent.size(); }
    size_t GetCurveCount() const {
        return segmentBegin.empty() ? 0 : segmentBegin.size() - 1;
    }

    /// B_i S_i for one side: [t b n] * diag(l, w, h), h = sqrt(l w).
    GfMatrix3d GetScaledFrame(size_t segment, bool left) const;
};

/// Computes scaled frames for a sampled pose.
///
/// Degenerate segments (zero length) leave the tangent zero and mark the
/// frame unusable; callers reject the bind rather than laundering them into
/// an identity, matching how the engine treats degenerate joint frames.
RigExecCurvenetFrames RigExecComputeCurvenetFrames(
    const RigExecCurvenetTopology &topology,
    const RigExecCurvenetSampling &sampling);

/// True when every segment has a usable (non-degenerate) tangent.
bool RigExecCurvenetFramesAreValid(
    const RigExecCurvenetFrames &frames, std::string *error);

/// Per-segment-side deformation gradients F = (B S)(B̆ S̆)^-1 (§3, Eq. 1).
///
/// Isolated curves take the rotation-only branch: the smallest rotation
/// from the rest tangent to the posed tangent, scaled by the length ratio.
struct RigExecCurvenetGradients {
    std::vector<GfMatrix3d> left;
    std::vector<GfMatrix3d> right;
};

RigExecCurvenetGradients RigExecComputeCurvenetGradients(
    const RigExecCurvenetFrames &restFrames,
    const RigExecCurvenetFrames &posedFrames);

/// Remaps per-segment gradients onto per-sample values (§4.3).
///
/// "For a sample inside a curve, we set its left and right matrices by
/// averaging the values from the previous and next segments along the curve.
/// When the sample is a curve endpoint, we make a copy of the sample for
/// every incident segment."
///
/// Within one curve an endpoint has exactly one incident segment, and a knot
/// shared by several curves is already a separate sample in each of them --
/// so "a copy per incident segment" comes out as exactly two values per
/// sample, one per side. That is why §4.2 can state n_c = 2 * (total
/// samples), and why this is a pair of plain per-sample arrays.
struct RigExecCurvenetSampleGradients {
    std::vector<GfMatrix3d> left;
    std::vector<GfMatrix3d> right;
};

RigExecCurvenetSampleGradients RigExecRemapGradientsToSamples(
    const RigExecCurvenetTopology &topology,
    const RigExecCurvenetSampling &sampling,
    const RigExecCurvenetFrames &frames,
    const RigExecCurvenetGradients &gradients);

/// Smallest rotation taking unit \p from to unit \p to. Antipodal inputs
/// pick an arbitrary but deterministic perpendicular axis.
GfMatrix3d RigExecSmallestRotation(const GfVec3d &from, const GfVec3d &to);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_CURVENET_H
