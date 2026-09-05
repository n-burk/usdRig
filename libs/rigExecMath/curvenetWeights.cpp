#include "curvenetWeights.h"
#include "cutMesh.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>

namespace rigExec {

bool RigExecBindCurvenetWeights(
    const RigExecCurvenetTopology &topology,
    const std::vector<GfVec3f> &controlPoints,
    const std::vector<GfVec3f> &meshPoints,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const std::vector<int> &autoSmooth, int samplesPerSpline,
    RigExecCurvenetWeightBinding *binding, std::string *error)
{
    auto fail = [&](const char *message) { if (error) *error = message; return false; };
    if (!binding || meshPoints.empty() || counts.empty() || controlPoints.empty() ||
        topology.pointCount != int(controlPoints.size()) || samplesPerSpline < 1)
        return fail("curvenet parametrization requires nonempty valid geometry");
    size_t corners = 0;
    for (int count : counts) { if (count < 3) return fail("mesh face has fewer than three vertices"); corners += count; }
    if (corners != indices.size()) return fail("mesh topology cardinality mismatch");
    for (int v : indices) if (v < 0 || size_t(v) >= meshPoints.size()) return fail("mesh index out of range");
    for (const auto &points : {&meshPoints, &controlPoints})
        for (const auto &p : *points) for (int k = 0; k < 3; ++k)
            if (!std::isfinite(p[k])) return fail("non-finite parametrization geometry");
    *binding = RigExecCurvenetWeightBinding();
    auto &b = *binding;
    b.controlCount = controlPoints.size();
    const double edge = RigExecMeshMeanEdgeLength(meshPoints, counts, indices);
    if (!(edge > 1e-12)) return fail("degenerate parametrization surface");
    b.stiffness = 100.0 * edge;
    b.sampling = RigExecSampleCurvenet(topology, controlPoints,
        RigExecPlanCurvenetSamples(topology, controlPoints, edge, samplesPerSpline));
    if (b.sampling.positions.empty()) return fail("curvenet has no samples");
    RigExecMeshSurfaceQuery query(meshPoints, counts, indices);
    std::vector<int> component(meshPoints.size());
    std::iota(component.begin(), component.end(), 0);
    auto root = [&](int v) { while (component[v] != v) { component[v] = component[component[v]]; v = component[v]; } return v; };
    size_t cursor = 0;
    for (int count : counts) {
        for (int k = 1; k < count; ++k) component[root(indices[cursor+k])] = root(indices[cursor]);
        cursor += count;
    }
    std::set<int> reached;
    for (const auto &point : b.sampling.positions) {
        std::vector<std::pair<int,double>> projection;
        if (!query.Project(point, &projection)) return fail("sample cannot be projected to surface");
        for (const auto &[v, w] : projection) if (w != 0) reached.insert(root(v));
        b.projections.push_back(std::move(projection));
    }
    b.surfaceRows.assign(meshPoints.size(), -1);
    int size = 0;
    for (size_t v = 0; v < meshPoints.size(); ++v)
        if (reached.count(root(int(v)))) b.surfaceRows[v] = size++;
    if (!size) return fail("curvenet reaches no mesh component");
    RigExecSparseBuilder matrix(size);
    cursor = 0;
    for (int count : counts) {
        std::vector<GfVec3d> polygon;
        for (int k = 0; k < count; ++k) polygon.emplace_back(meshPoints[indices[cursor+k]]);
        std::vector<double> laplacian;
        RigExecPolygonLaplacian(polygon, &laplacian);
        for (int i = 0; i < count; ++i) for (int j = 0; j < count; ++j) {
            const int a = b.surfaceRows[indices[cursor+i]], c = b.surfaceRows[indices[cursor+j]];
            if (a >= 0 && c >= 0) matrix.Add(a, c, laplacian[i*count+j]);
        }
        cursor += count;
    }
    for (const auto &projection : b.projections)
        for (const auto &[a, wa] : projection) for (const auto &[c, wc] : projection)
            matrix.Add(b.surfaceRows[a], b.surfaceRows[c], b.stiffness*wa*wc);
    b.surfaceSolver = std::make_shared<RigExecSparseCholesky>();
    if (!b.surfaceSolver->Factorize(matrix, 0.0, error)) return false;

    // Discrete harmonic interpolation over the control-net connectivity.
    b.controlRows.assign(controlPoints.size(), -1);
    std::set<int> smooth(autoSmooth.begin(), autoSmooth.end());
    int smoothCount = 0;
    for (int v : smooth) {
        if (v < 0 || size_t(v) >= controlPoints.size()) return fail("auto-smooth index out of range");
        b.controlRows[v] = smoothCount++;
    }
    if (smoothCount) {
        std::vector<std::set<int>> neighbors(controlPoints.size());
        for (size_t s = 0; s < topology.GetSplineCount(); ++s) for (int k = 0; k < 3; ++k) {
            const int a = topology.splineIndices[4*s+k], c = topology.splineIndices[4*s+k+1];
            if (a != c) { neighbors[a].insert(c); neighbors[c].insert(a); }
        }
        // Every unknown component needs an authored boundary. Detect it
        // explicitly; a nearly zero numerical pivot can hide a nullspace.
        std::vector<bool> anchored(controlPoints.size(), false);
        std::vector<int> pending;
        for (size_t v = 0; v < controlPoints.size(); ++v) {
            if (b.controlRows[v] < 0) { anchored[v] = true; pending.push_back(int(v)); }
        }
        for (size_t i = 0; i < pending.size(); ++i) {
            for (int v : neighbors[pending[i]]) {
                if (!anchored[v]) { anchored[v] = true; pending.push_back(v); }
            }
        }
        for (int v : smooth) if (!anchored[v])
            return fail("auto-smooth component has no authored weight anchor");
        RigExecSparseBuilder smoothMatrix(smoothCount);
        b.smoothTerms.resize(smoothCount);
        for (int v : smooth) {
            const int row = b.controlRows[v];
            if (neighbors[v].empty()) return fail("auto-smooth point has no anchored neighbors");
            for (int other : neighbors[v]) {
                const double w = 1.0/std::max(1e-8, double((controlPoints[v]-controlPoints[other]).GetLength()));
                smoothMatrix.Add(row, row, w);
                const int col = b.controlRows[other];
                if (col >= 0) smoothMatrix.Add(row, col, -w);
                else b.smoothTerms[row].emplace_back(other, w);
            }
        }
        b.smoothSolver = std::make_shared<RigExecSparseCholesky>();
        if (!b.smoothSolver->Factorize(smoothMatrix, 0.0, error)) return false;
    }
    return true;
}

bool RigExecEvaluateCurvenetWeights(const RigExecCurvenetWeightBinding &b,
    const std::vector<float> &weights, int columns, float fallback,
    std::vector<float> *output, std::string *error)
{
    auto fail = [&](const char *message) { if (error) *error = message; return false; };
    if (!output || !b.surfaceSolver || !b.surfaceSolver->IsFactorized() || columns < 1 ||
        weights.size() != b.controlCount*size_t(columns) || !std::isfinite(fallback))
        return fail("invalid curvenet weight cardinality or binding");
    std::vector<double> values(weights.begin(), weights.end());
    for (int c = 0; c < columns; ++c) for (size_t i = 0; i < b.controlCount; ++i)
        if (b.controlRows[i] < 0 && !std::isfinite(values[c*b.controlCount+i]))
            return fail("non-finite authored curvenet weight");
    if (b.smoothSolver) {
        const size_t rows = b.smoothTerms.size();
        std::vector<double> rhs(rows*columns, 0), smoothed;
        for (int c = 0; c < columns; ++c) for (size_t r = 0; r < rows; ++r)
            for (const auto &[point, weight] : b.smoothTerms[r])
                rhs[c*rows+r] += weight * values[c*b.controlCount+point];
        b.smoothSolver->Solve(rhs, columns, &smoothed);
        for (int c = 0; c < columns; ++c) for (size_t p = 0; p < b.controlCount; ++p)
            if (b.controlRows[p] >= 0) values[c*b.controlCount+p] = smoothed[c*rows+b.controlRows[p]];
    }
    const size_t rows = b.surfaceSolver->GetSize();
    std::vector<double> rhs(rows*columns, 0), solved;
    for (int c = 0; c < columns; ++c) for (size_t s = 0; s < b.projections.size(); ++s) {
        double sample = 0;
        for (int k = 0; k < 4; ++k) sample += b.sampling.stencilWeights[s][k] *
            values[c*b.controlCount + b.sampling.stencilIndices[s][k]];
        for (const auto &[v,w] : b.projections[s]) rhs[c*rows+b.surfaceRows[v]] += b.stiffness*w*sample;
    }
    b.surfaceSolver->Solve(rhs, columns, &solved);
    std::vector<float> result(b.surfaceRows.size()*columns, fallback);
    for (int c = 0; c < columns; ++c) for (size_t v = 0; v < b.surfaceRows.size(); ++v) {
        const int row = b.surfaceRows[v];
        if (row < 0) continue;
        const double value = solved[c*rows+row];
        if (!std::isfinite(value)) return fail("parametrization solve produced a non-finite weight");
        result[c*b.surfaceRows.size()+v] = float(value);
    }
    *output = std::move(result);
    return true;
}
} // namespace rigExec
