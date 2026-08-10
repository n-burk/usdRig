//
// The Profile Mover (2022 paper §4.3, Algorithm 1).
//
#include "profileMover.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

namespace {

constexpr double kEps = 1e-12;

GfVec3d _ToD(const GfVec3f &v) { return GfVec3d(v[0], v[1], v[2]); }
GfVec3f _ToF(const GfVec3d &v) {
    return GfVec3f(float(v[0]), float(v[1]), float(v[2]));
}

/// Row-major flatten of a 3x3 into nine columns, and back.
void _Flatten(const GfMatrix3d &m, double *out, size_t stride)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out[size_t(r * 3 + c) * stride] = m[r][c];
        }
    }
}

GfMatrix3d _Unflatten(const double *values)
{
    GfMatrix3d m;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            m[r][c] = values[r * 3 + c];
        }
    }
    return m;
}

}  // namespace

bool RigExecBindProfileMover(const RigExecCurvenetTopology &topology,
                             const std::vector<GfVec3f> &curvenetPoints,
                             const std::vector<GfVec3f> &meshPoints,
                             const std::vector<int> &faceVertexCounts,
                             const std::vector<int> &faceVertexIndices,
                             int samplesPerSpline,
                             RigExecProfileMoverBinding *binding,
                             std::string *error)
{
    *binding = RigExecProfileMoverBinding();
    binding->topology = topology;
    binding->projectionMeshPoints = meshPoints;
    binding->faceVertexCounts = faceVertexCounts;
    binding->faceVertexIndices = faceVertexIndices;

    // Re-orient the net against the surface it is about to deform (§3).
    // Callers build the topology before they have a mesh in hand, and an
    // orientation derived from the fans alone is sign-arbitrary per
    // intersection -- which shows up as a spurious half turn of torsion along
    // any curve whose two ends happened to disagree.
    {
        const RigExecMeshSurfaceQuery query(meshPoints, faceVertexCounts,
                                            faceVertexIndices);
        RigExecOrientCurvenetIntersections(
            curvenetPoints,
            [&query](const GfVec3d &p) { return query.Normal(p); },
            &binding->topology);
    }
    // Everything below uses the ORIENTED copy, never the caller's.
    const RigExecCurvenetTopology &oriented = binding->topology;

    const double meanEdge = RigExecMeshMeanEdgeLength(
        meshPoints, faceVertexCounts, faceVertexIndices);
    binding->samplesPerSpline = RigExecPlanCurvenetSamples(
        oriented, curvenetPoints, meanEdge, samplesPerSpline);
    binding->projectionSampling = RigExecSampleCurvenet(
        oriented, curvenetPoints, binding->samplesPerSpline);
    if (binding->projectionSampling.GetSampleCount() < 2) {
        if (error) {
            *error = "the curvenet produced fewer than two samples; it has no "
                     "usable curves";
        }
        return false;
    }

    // Reject degenerate geometry at BIND, naming the curve -- a zero-length
    // segment has no tangent, and every frame downstream would silently
    // inherit an identity from it.
    {
        const RigExecCurvenetFrames frames = RigExecComputeCurvenetFrames(
            oriented, binding->projectionSampling);
        std::string reason;
        if (!RigExecCurvenetFramesAreValid(frames, &reason)) {
            if (error) {
                *error = "curvenet is degenerate in the projection pose: " +
                         reason;
            }
            return false;
        }
    }

    if (!RigExecBuildCutMesh(oriented, binding->projectionSampling, meshPoints,
                             faceVertexCounts, faceVertexIndices,
                             &binding->cutMesh, &binding->report, error)) {
        return false;
    }

    // L is assembled in the projection pose and never rebuilt (§4.3).
    std::vector<GfVec3d> cornerPositions(binding->cutMesh.cornerNode.size());
    for (size_t c = 0; c < cornerPositions.size(); ++c) {
        cornerPositions[c] =
            binding->cutMesh.nodePosition[binding->cutMesh.cornerNode[c]];
    }
    RigExecSparseBuilder matrix(binding->cutMesh.unknownCount);
    RigExecAssembleCutSystem(binding->cutMesh, cornerPositions, &matrix,
                             &binding->faceLaplacians,
                             &binding->faceLaplacianBegin);

    binding->solver = std::make_shared<RigExecSparseCholesky>();
    if (!binding->solver->Factorize(matrix, 0.0, error)) {
        binding->solver.reset();
        return false;
    }
    return true;
}

bool RigExecEvaluateProfileMover(
    const RigExecProfileMoverBinding &binding,
    const std::vector<GfVec3f> &posedCurvenetPoints,
    const std::vector<GfVec3f> &restMeshPoints, double strength,
    std::vector<GfVec3f> *outPoints, std::string *error)
{
    if (!binding.IsValid()) {
        if (error) {
            *error = "the curvenet binding was never built";
        }
        return false;
    }
    const RigExecCutMesh &cut = binding.cutMesh;
    const size_t vertexCount = binding.projectionMeshPoints.size();
    if (restMeshPoints.size() != vertexCount) {
        if (error) {
            *error = "the incoming points array has " +
                     std::to_string(restMeshPoints.size()) +
                     " entries but the curvenet was bound against " +
                     std::to_string(vertexCount);
        }
        return false;
    }

    // Whether the rest surface differs from the projection surface decides
    // whether anything needs warping at all (§5).
    bool layered = false;
    for (size_t v = 0; v < vertexCount && !layered; ++v) {
        layered = (restMeshPoints[v] != binding.projectionMeshPoints[v]);
    }

    // ---- rest configuration --------------------------------------------
    RigExecCurvenetSampling restSampling = binding.projectionSampling;
    std::vector<GfVec3d> restCorner(cut.cornerNode.size());

    if (!layered) {
        for (size_t c = 0; c < restCorner.size(); ++c) {
            restCorner[c] = cut.nodePosition[cut.cornerNode[c]];
        }
    } else {
        // Warp the cut-mesh and the curvenet onto the rest surface through
        // the cached closest-point binding, then read the rest frames off
        // that configuration (§5).
        std::vector<GfVec3d> warpedNode(cut.GetNodeCount());
        for (size_t n = 0; n < warpedNode.size(); ++n) {
            warpedNode[n] = cut.EvaluateBinding(
                cut.nodeBinding[n], restMeshPoints, binding.faceVertexCounts,
                binding.faceVertexIndices);
        }
        for (size_t c = 0; c < restCorner.size(); ++c) {
            restCorner[c] = warpedNode[cut.cornerNode[c]];
        }
        for (size_t s = 0; s < cut.sampleBinding.size(); ++s) {
            const GfVec3d projected = cut.EvaluateBinding(
                cut.sampleBinding[s], restMeshPoints, binding.faceVertexCounts,
                binding.faceVertexIndices);
            restSampling.positions[s] = projected + cut.sampleResidual[s];
        }
    }

    // ---- §3: frames and gradients ---------------------------------------
    const RigExecCurvenetSampling posedSampling = RigExecSampleCurvenet(
        binding.topology, posedCurvenetPoints, binding.samplesPerSpline);
    if (posedSampling.GetSampleCount() != restSampling.GetSampleCount()) {
        if (error) {
            *error = "the posed curvenet sampled to a different number of "
                     "points than the bind did";
        }
        return false;
    }
    const RigExecCurvenetFrames restFrames =
        RigExecComputeCurvenetFrames(binding.topology, restSampling);
    const RigExecCurvenetFrames posedFrames =
        RigExecComputeCurvenetFrames(binding.topology, posedSampling);
    std::string reason;
    if (!RigExecCurvenetFramesAreValid(posedFrames, &reason)) {
        if (error) {
            *error = "the posed curvenet is degenerate: " + reason;
        }
        return false;
    }
    const RigExecCurvenetGradients gradients =
        RigExecComputeCurvenetGradients(restFrames, posedFrames);
    const RigExecCurvenetSampleGradients sampleGradients =
        RigExecRemapGradientsToSamples(binding.topology, posedSampling,
                                       posedFrames, gradients);

    // ---- Eq. (4): interpolate the gradients ------------------------------
    const int constraintCount = cut.constraintCount;
    const int unknowns = cut.unknownCount;
    std::vector<double> gradientConstraints(size_t(constraintCount) * 9, 0.0);
    for (size_t s = 0; s < sampleGradients.left.size(); ++s) {
        _Flatten(sampleGradients.left[s],
                 gradientConstraints.data() + size_t(2 * s),
                 size_t(constraintCount));
        _Flatten(sampleGradients.right[s],
                 gradientConstraints.data() + size_t(2 * s + 1),
                 size_t(constraintCount));
    }

    std::vector<double> rhs, gradientUnknowns;
    RigExecAssembleCutRhs(cut, binding.faceLaplacians,
                          binding.faceLaplacianBegin, gradientConstraints,
                          std::vector<double>(), 9, &rhs);
    binding.solver->Solve(rhs, 9, &gradientUnknowns);

    // ---- per-corner gradients, then the deformed cut-face polygons -------
    const size_t cornerCount = cut.cornerNode.size();
    std::vector<double> cornerGradient(cornerCount * 9, 0.0);
    for (size_t c = 0; c < cornerCount; ++c) {
        const int unknown = cut.cornerVertexUnknown[c];
        if (unknown >= 0) {
            for (int k = 0; k < 9; ++k) {
                cornerGradient[c * 9 + k] =
                    gradientUnknowns[size_t(k) * size_t(unknowns) +
                                     size_t(unknown)];
            }
            continue;
        }
        for (int p = cut.cornerConstraintBegin[c];
             p < cut.cornerConstraintBegin[c + 1]; ++p) {
            const int index = cut.cornerConstraintIndex[p];
            const double weight = cut.cornerConstraintWeight[p];
            for (int k = 0; k < 9; ++k) {
                cornerGradient[c * 9 + k] +=
                    weight * gradientConstraints[size_t(k) *
                                                     size_t(constraintCount) +
                                                 size_t(index)];
            }
        }
    }

    std::vector<double> cornerOffsets(cornerCount * 3, 0.0);
    std::vector<GfVec3d> cornerTarget(cornerCount, GfVec3d(0.0));
    for (size_t f = 0; f < cut.GetCutFaceCount(); ++f) {
        const int begin = cut.faceBegin[f];
        const int end = cut.faceBegin[f + 1];
        const int n = end - begin;
        if (n <= 0) {
            continue;
        }
        double average[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        for (int c = begin; c < end; ++c) {
            for (int k = 0; k < 9; ++k) {
                average[k] += cornerGradient[size_t(c) * 9 + k];
            }
        }
        GfMatrix3d faceGradient;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                faceGradient[r][c] = average[r * 3 + c] / double(n);
            }
        }
        // y_h = rows of Xrest_f * F_f^T, i.e. F applied to each rest corner.
        for (int c = begin; c < end; ++c) {
            const GfVec3d transformed = faceGradient * restCorner[c];
            for (int k = 0; k < 3; ++k) {
                cornerOffsets[size_t(k) * cornerCount + size_t(c)] =
                    transformed[k];
            }
        }
    }

    // ---- Eq. (5): positional constraints and the second solve ------------
    //
    // §4.3 writes the positional constraint as p_i = q_i - F_i (q_i - p_i)
    // per SAMPLE: the surface keeps the offset the curve floats at,
    // transported by the same gradient. Every cut-vertex the cut introduced
    // BETWEEN samples -- a segment crossing a mesh edge -- needs the same
    // thing, and interpolating the finished p_i between the two samples is
    // not it: on a curved surface the traced crossing does not lie on the
    // straight chord between the two samples' projections, so at rest the
    // interpolated constraint pulls the vertex off itself and the mover is
    // no longer the identity.
    //
    // Measuring the residual from the CUT-VERTEX'S OWN rest position instead
    // is exact for both. At a sample the node IS that sample's projection and
    // this reduces to the paper's formula unchanged; between samples it is
    // the honest generalization, and at rest (q = q_rest, F = I) it returns
    // the node exactly whatever the surface is doing.
    std::vector<double> positionConstraints(size_t(constraintCount) * 3, 0.0);
    for (size_t s = 0; s < posedSampling.GetSampleCount(); ++s) {
        for (int side = 0; side < 2; ++side) {
            const size_t index = 2 * s + size_t(side);
            for (int k = 0; k < 3; ++k) {
                positionConstraints[size_t(k) * size_t(constraintCount) +
                                    index] = posedSampling.positions[s][k];
            }
        }
    }

    // The correction rides in the y_h term, which the right-hand side already
    // subtracts per corner: known = C phi_c - offsets, so offsetting by
    // (interpolated q) - target makes the corner read `target`.
    for (size_t c = 0; c < cornerCount; ++c) {
        // Unknown corners need no target; nor do corners that are NEITHER --
        // a vertex in a component the curvenet never reached has no unknown
        // and no constraint, and must simply keep its rest position.
        if (cut.cornerVertexUnknown[c] >= 0 ||
            cut.cornerConstraintBegin[c] == cut.cornerConstraintBegin[c + 1]) {
            continue;
        }
        GfVec3d posed(0.0), rest(0.0);
        for (int p = cut.cornerConstraintBegin[c];
             p < cut.cornerConstraintBegin[c + 1]; ++p) {
            const int index = cut.cornerConstraintIndex[p];
            const double weight = cut.cornerConstraintWeight[p];
            const size_t sample = size_t(index) / 2;
            posed += posedSampling.positions[sample] * weight;
            rest += restSampling.positions[sample] * weight;
        }
        const GfMatrix3d gradient =
            _Unflatten(cornerGradient.data() + c * 9);
        cornerTarget[c] = posed - gradient * (rest - restCorner[c]);
        for (int k = 0; k < 3; ++k) {
            cornerOffsets[size_t(k) * cornerCount + c] +=
                posed[k] - cornerTarget[c][k];
        }
    }

    std::vector<double> positionUnknowns;
    RigExecAssembleCutRhs(cut, binding.faceLaplacians,
                          binding.faceLaplacianBegin, positionConstraints,
                          cornerOffsets, 3, &rhs);
    binding.solver->Solve(rhs, 3, &positionUnknowns);

    // ---- write out --------------------------------------------------------
    outPoints->assign(restMeshPoints.begin(), restMeshPoints.end());
    for (int u = 0; u < unknowns; ++u) {
        const int vertex = cut.unknownVertex[u];
        GfVec3d p;
        for (int k = 0; k < 3; ++k) {
            p[k] = positionUnknowns[size_t(k) * size_t(unknowns) + size_t(u)];
        }
        // A non-finite solution means the system was singular in a way the
        // factorization's pivot test did not catch. Fail the mover rather
        // than write NaN into geometry, where it propagates silently through
        // every downstream revision and into the render.
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
            !std::isfinite(p[2])) {
            if (error) {
                *error = "the curvenet solve produced a non-finite position "
                         "for mesh vertex " + std::to_string(vertex) +
                         "; the cut left that region unsupported";
            }
            return false;
        }
        (*outPoints)[vertex] = _ToF(p);
    }
    // A mesh vertex every one of whose corners is constrained carries no
    // unknown (§4.2 precedence). Its position is what the curvenet says
    // there, averaged over its corners: each corner names one side, and both
    // sides describe the same point, so the average is the point they agree
    // on rather than an arbitrary pick.
    {
        std::vector<GfVec3d> accumulated(vertexCount, GfVec3d(0.0));
        std::vector<int> counted(vertexCount, 0);
        for (size_t c = 0; c < cut.cornerNode.size(); ++c) {
            const int vertex = cut.nodeMeshVertex[cut.cornerNode[c]];
            if (vertex < 0 || cut.vertexUnknown[vertex] >= 0 ||
                cut.cornerConstraintBegin[c] ==
                    cut.cornerConstraintBegin[c + 1]) {
                continue;
            }
            accumulated[vertex] += cornerTarget[c];
            ++counted[vertex];
        }
        for (size_t v = 0; v < vertexCount; ++v) {
            if (counted[v] > 0) {
                (*outPoints)[v] = _ToF(accumulated[v] / double(counted[v]));
            }
        }
    }

    if (strength != 1.0) {
        const double s = strength;
        for (size_t v = 0; v < vertexCount; ++v) {
            const GfVec3d rest = _ToD(restMeshPoints[v]);
            const GfVec3d moved = _ToD((*outPoints)[v]);
            (*outPoints)[v] = _ToF(rest + (moved - rest) * s);
        }
    }
    // Vertices in a component no curve reaches were never solved for and are
    // already sitting at their rest value from the assign above.
    return true;
}

bool RigExecComputeCurvenetDisplayFrames(
    const RigExecCurvenetTopology &topology,
    const std::vector<int> &samplesPerSpline,
    const std::vector<GfVec3f> &points, RigExecCurvenetSampling *sampling,
    RigExecCurvenetFrames *frames, std::string *error)
{
    *sampling = RigExecSampleCurvenet(topology, points, samplesPerSpline);
    if (sampling->GetSampleCount() < 2) {
        if (error) {
            *error = "the curvenet has no sampled curves";
        }
        return false;
    }
    *frames = RigExecComputeCurvenetFrames(topology, *sampling);
    return RigExecCurvenetFramesAreValid(*frames, error);
}

}  // namespace rigExec
