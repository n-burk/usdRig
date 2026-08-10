//
// The Profile Mover: propagating a rigged curvenet's articulation over a
// surface mesh (2022 paper §4, Algorithm 1).
//
//   precompute (projection pose):  cut-mesh, {L, C, V}, factorize Vt L V
//   runtime:                       curvenet frames and gradients (§3)
//                                  -> harmonic interpolation of the gradients
//                                  -> Poisson reconstruction of positions
//
// The bind is expensive and the solve is not, which is the whole point: one
// factorization serves every frame of animation.
//
#ifndef RIGEXEC_MATH_PROFILE_MOVER_H
#define RIGEXEC_MATH_PROFILE_MOVER_H

#include "curvenet.h"
#include "cutMesh.h"
#include "sparseSolve.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// Everything the runtime solve needs, computed once per binding epoch.
///
/// Copyable only by rebuilding: it owns a factorization, which is large and
/// has no meaning apart from the layout that produced it.
struct RigExecProfileMoverBinding {
    RigExecCurvenetTopology topology;
    std::vector<int> samplesPerSpline;
    RigExecCutMesh cutMesh;
    RigExecCutMeshReport report;

    /// The projection pose: the surface the curvenet was drawn on and the
    /// cut was computed against. Never re-derived.
    std::vector<GfVec3f> projectionMeshPoints;
    std::vector<int> faceVertexCounts;
    std::vector<int> faceVertexIndices;
    RigExecCurvenetSampling projectionSampling;

    /// L per cut-face, assembled in the projection pose exactly once.
    std::vector<double> faceLaplacians;
    std::vector<int> faceLaplacianBegin;
    std::shared_ptr<RigExecSparseCholesky> solver;

    bool IsValid() const { return solver && solver->IsFactorized(); }
};

/// Precomputes the binding (Algorithm 1, lines 1-3).
///
/// \p curvenetPoints and \p meshPoints are both in the PROJECTION pose.
/// \p samplesPerSpline defaults from RigExecPlanCurvenetSamples when empty.
bool RigExecBindProfileMover(const RigExecCurvenetTopology &topology,
                             const std::vector<GfVec3f> &curvenetPoints,
                             const std::vector<GfVec3f> &meshPoints,
                             const std::vector<int> &faceVertexCounts,
                             const std::vector<int> &faceVertexIndices,
                             int samplesPerSpline,
                             RigExecProfileMoverBinding *binding,
                             std::string *error);

/// Runs the two solves (Algorithm 1, lines 4-9).
///
/// \p posedCurvenetPoints is the articulated control-point pool.
/// \p restMeshPoints is the surface the curvenet deforms FROM. Passing the
/// projection points is the plain §4 formulation; passing the result of an
/// earlier deformer is §5's layered form, and the curvenet and cut-mesh are
/// then warped onto it through the cached binding.
///
/// \p strength blends the result back toward the rest surface, which the
/// paper has no equivalent for -- it is the engine's usual per-mover control
/// and is applied after the solve, so 0 is exactly a no-op.
bool RigExecEvaluateProfileMover(const RigExecProfileMoverBinding &binding,
                                 const std::vector<GfVec3f> &posedCurvenetPoints,
                                 const std::vector<GfVec3f> &restMeshPoints,
                                 double strength,
                                 std::vector<GfVec3f> *outPoints,
                                 std::string *error);

/// The scaled frames a UI draws as the paper's deformed boxes (Fig. 5),
/// evaluated for one pose without running a solve.
bool RigExecComputeCurvenetDisplayFrames(
    const RigExecCurvenetTopology &topology,
    const std::vector<int> &samplesPerSpline,
    const std::vector<GfVec3f> &points, RigExecCurvenetSampling *sampling,
    RigExecCurvenetFrames *frames, std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_PROFILE_MOVER_H
