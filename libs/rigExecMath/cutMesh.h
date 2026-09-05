//
// Curvenet-to-surface binding: the cut-mesh and its cut-aware polygonal
// Laplacian (2022 paper §4.1, §4.2, Appendix A).
//
// The cut-mesh conforms the character surface to the curvenet by splitting
// the faces the profile curves cross, so a value can be DISCONTINUOUS across
// a curve. That discontinuity is the whole technique: a curve is a hinge, and
// Fig. 11 shows what averaging the two sides instead produces.
//
// Structure of the implementation, and where it departs from the paper's
// description:
//
//   * The paper builds one global halfedge mesh with cracks and traces
//     segments across it. Here every segment is first split at its crossings
//     with mesh edges -- which the paper does too -- and the arrangement is
//     then solved INDEPENDENTLY PER INPUT FACE. The result is the same
//     cut-face set, because a sub-segment that lies inside one face cannot
//     affect another; what it avoids is a global halfedge structure with
//     crack semantics, which is the part that is hard to get right.
//   * Segment tracing seeks the target rather than shooting a fixed
//     straightest direction (Polthier & Schmies). At the sample spacing §3
//     prescribes -- one sample per mesh edge length -- the two agree, and the
//     target-seeking form is guaranteed to arrive.
//
// Both are recorded in docs/curvenet.md.
//
#ifndef RIGEXEC_MATH_CUT_MESH_H
#define RIGEXEC_MATH_CUT_MESH_H

#include "curvenet.h"
#include "sparseSolve.h"

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"

#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// A point on the projection mesh, as weights over one face's corners.
///
/// Every cut-vertex is such a point, which is what makes §5's warping to a
/// different rest pose a linear evaluation rather than a second projection.
struct RigExecMeshPoint {
    int face = -1;
    /// Absolute offset of the face's first corner in faceVertexIndices, so
    /// evaluating a binding never has to re-scan faceVertexCounts.
    int indexOffset = 0;
    int cornerBegin = 0;   ///< into RigExecCutMesh::bindingWeights
    int cornerCount = 0;
};

/// One oriented curvenet crossing recorded on a cut-edge, so a corner can
/// name which sample side it reads.
struct RigExecCutCurveRef {
    int sampleA = -1;   ///< sample the parent segment leaves
    int sampleB = -1;   ///< sample the parent segment reaches
    double tA = 0.0;    ///< parameter of this cut-edge's first endpoint
    double tB = 1.0;    ///< parameter of its second endpoint
};

/// The cut-mesh and everything derived from it that does not depend on pose.
struct RigExecCutMesh {
    // ---- nodes -------------------------------------------------------
    std::vector<GfVec3d> nodePosition;      ///< in the projection pose
    std::vector<RigExecMeshPoint> nodeBinding;
    std::vector<float> bindingWeights;      ///< pool for nodeBinding
    std::vector<int> nodeMeshVertex;        ///< mesh vertex index, or -1
    std::vector<int> nodeSample;            ///< curvenet sample index, or -1

    // ---- cut faces ---------------------------------------------------
    /// Corner loops. Face f owns corners [faceBegin[f], faceBegin[f+1]).
    /// A crack appears as the same node twice in one loop, which is exactly
    /// the "simple polygon of arbitrary shape" Appendix A wants.
    std::vector<int> faceBegin;
    std::vector<int> cornerNode;
    /// Vertex unknown column for this corner, or -1 when the corner reads a
    /// curvenet constraint instead.
    std::vector<int> cornerVertexUnknown;
    /// Constraint entries per corner: corner c owns
    /// [cornerConstraintBegin[c], cornerConstraintBegin[c+1]). A corner on a
    /// curvenet sample has one entry; one at a segment/mesh-edge crossing has
    /// two, interpolating the segment's two samples -- which is how C stays a
    /// map from SAMPLE values to halfedges even though the cut introduces
    /// vertices between samples.
    std::vector<int> cornerConstraintBegin;   ///< corners+1
    std::vector<int> cornerConstraintIndex;   ///< 2*sample + (left ? 0 : 1)
    std::vector<double> cornerConstraintWeight;
    /// Input face each cut-face came from, for reporting.
    std::vector<int> faceSource;

    // ---- unknown numbering -------------------------------------------
    /// Mesh vertex -> unknown column, or -1 when the curvenet took it over
    /// (§4.2 precedence) or when it lies in a component no curve reaches.
    std::vector<int> vertexUnknown;
    std::vector<int> unknownVertex;   ///< inverse
    int unknownCount = 0;
    /// Mesh vertices the curvenet passes exactly through: they carry no
    /// unknown, and their posed position is read from the constraint.
    std::vector<int> vertexConstraintNode;  ///< node index, or -1
    /// Mesh vertices in a component with no curvenet constraint at all.
    /// Left at rest and reported rather than given an arbitrary solution.
    std::vector<int> unreachedVertices;

    int constraintCount = 0;  ///< 2 * curvenet sample count

    // ---- per-sample surface binding ----------------------------------
    /// Closest point on the projection mesh for each curvenet sample, and
    /// the residual q - p the deformation transports (§4.3).
    std::vector<RigExecMeshPoint> sampleBinding;
    std::vector<GfVec3d> sampleResidual;

    size_t GetCutFaceCount() const {
        return faceBegin.empty() ? 0 : faceBegin.size() - 1;
    }
    size_t GetNodeCount() const { return nodePosition.size(); }

    /// Evaluates a mesh point against an arbitrary pose of the input mesh.
    GfVec3d EvaluateBinding(const RigExecMeshPoint &point,
                            const std::vector<GfVec3f> &meshPoints,
                            const std::vector<int> &faceVertexCounts,
                            const std::vector<int> &faceVertexIndices) const;
};

/// Diagnostics from the bind, surfaced to the artist rather than swallowed.
struct RigExecCutMeshReport {
    int sampleCount = 0;
    int cutFaceCount = 0;
    int crackCount = 0;
    int tracedSegments = 0;   ///< segments that crossed a mesh edge
    int failedTraces = 0;     ///< segments the walk could not route
    /// Input faces that produced NO cut-face. Cutting can only ever add
    /// faces, so this is always a defect: those vertices lose their support
    /// in the Laplacian and the solve goes soft around them.
    int lostFaces = 0;
    double maxResidual = 0.0; ///< furthest a curve floats from the surface
    double meanEdgeLength = 0.0;
    std::vector<std::string> warnings;
};

/// Builds the cut-mesh binding a sampled curvenet to a polygonal mesh.
///
/// \p meshPoints, \p faceVertexCounts and \p faceVertexIndices are the
/// surface in its PROJECTION pose -- the pose the curvenet was drawn in.
/// \p sampling must be the curvenet in that same pose.
///
/// Returns false and fills \p error only for inputs that cannot yield a
/// cut-mesh at all; recoverable trouble lands in \p report.
bool RigExecBuildCutMesh(const RigExecCurvenetTopology &topology,
                         const RigExecCurvenetSampling &sampling,
                         const std::vector<GfVec3f> &meshPoints,
                         const std::vector<int> &faceVertexCounts,
                         const std::vector<int> &faceVertexIndices,
                         RigExecCutMesh *cutMesh,
                         RigExecCutMeshReport *report, std::string *error);

/// Closest-point normal lookup against a polygonal mesh.
///
/// Exists so a curvenet can be oriented the way §3 specifies -- each
/// intersection's fan sorted about the normal of the closest surface point.
/// That is the only globally consistent orientation available, and without it
/// two intersections can disagree about which way is "up".
class RigExecMeshSurfaceQuery
{
public:
    RigExecMeshSurfaceQuery(const std::vector<GfVec3f> &meshPoints,
                            const std::vector<int> &faceVertexCounts,
                            const std::vector<int> &faceVertexIndices);
    ~RigExecMeshSurfaceQuery();

    RigExecMeshSurfaceQuery(const RigExecMeshSurfaceQuery &) = delete;
    RigExecMeshSurfaceQuery &operator=(const RigExecMeshSurfaceQuery &) =
        delete;

    /// Unit normal of the face closest to \p point, or zero when the mesh is
    /// empty.
    GfVec3d Normal(const GfVec3d &point) const;

    /// Closest-point triangle's sparse barycentric coordinates in mesh vertices.
    bool Project(const GfVec3d &point,
                 std::vector<std::pair<int, double>> *weights) const;

private:
    struct _Impl;
    std::unique_ptr<_Impl> _impl;
};

/// Mean edge length of a polygonal mesh (§3 sampling, §4.1 tolerance).
double RigExecMeshMeanEdgeLength(const std::vector<GfVec3f> &meshPoints,
                                 const std::vector<int> &faceVertexCounts,
                                 const std::vector<int> &faceVertexIndices);

/// The per-cut-face Laplacian of Appendix A, for a polygon given by its
/// corner positions. \p out is sized n*n, row-major.
///
/// L_f = a_f G_f^T G_f + lambda Q_f^T Q_f, with lambda = 1. Symmetric, PSD,
/// scale invariant, exactly cotan on a triangle, and correct for the cracked
/// polygons the cutting produces because a crack simply appears as two
/// coincident corners of an otherwise simple polygon.
void RigExecPolygonLaplacian(const std::vector<GfVec3d> &corners,
                             std::vector<double> *out);

/// The system matrix Vt L V, assembled over the cut-faces in a given pose.
///
/// The paper assembles this once, in the projection pose, and reuses the
/// factorization for every frame -- so \p restCorners is the projection-pose
/// geometry even when §5's separate rest pose is in play.
void RigExecAssembleCutSystem(const RigExecCutMesh &cutMesh,
                              const std::vector<GfVec3d> &cornerPositions,
                              RigExecSparseBuilder *matrix,
                              std::vector<double> *faceLaplacians,
                              std::vector<int> *faceLaplacianBegin);

/// Right-hand side of Eq. (6) for \p columns simultaneous unknowns.
///
/// \p constraintValues is column-major over the constraint columns
/// (constraintCount x columns) and \p cornerOffsets, when non-empty, is the
/// per-corner y_h term of Eq. (5), column-major over corners. Produces
/// -Vt L (C phi_c - y_h).
void RigExecAssembleCutRhs(const RigExecCutMesh &cutMesh,
                           const std::vector<double> &faceLaplacians,
                           const std::vector<int> &faceLaplacianBegin,
                           const std::vector<double> &constraintValues,
                           const std::vector<double> &cornerOffsets,
                           int columns, std::vector<double> *rhs);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_CUT_MESH_H
