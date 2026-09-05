#ifndef RIGEXEC_MATH_CURVENET_WEIGHTS_H
#define RIGEXEC_MATH_CURVENET_WEIGHTS_H

#include "curvenet.h"
#include "sparseSolve.h"
#include <memory>

namespace rigExec {

/// Factorized surface and optional control-net smoothing systems. The fixed
/// geometry determines L, B and S; animating weights only changes right sides.
struct RigExecCurvenetWeightBinding {
    RigExecCurvenetSampling sampling;
    std::vector<std::vector<std::pair<int,double>>> projections;
    std::vector<int> surfaceRows, controlRows;
    std::vector<std::vector<std::pair<int,double>>> smoothTerms;
    std::shared_ptr<RigExecSparseCholesky> surfaceSolver, smoothSolver;
    size_t controlCount = 0;
    double stiffness = 1.0;
};

bool RigExecBindCurvenetWeights(
    const RigExecCurvenetTopology &topology,
    const std::vector<GfVec3f> &controlPoints,
    const std::vector<GfVec3f> &meshPoints,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const std::vector<int> &autoSmooth, int samplesPerSpline,
    RigExecCurvenetWeightBinding *binding, std::string *error);

/// Multiple maps are column-major (controlCount values per map). Unreached
/// mesh components receive the explicit fallback; unanchored auto-smoothing
/// components are rejected at bind time.
bool RigExecEvaluateCurvenetWeights(
    const RigExecCurvenetWeightBinding &binding,
    const std::vector<float> &weights, int columns, float fallback,
    std::vector<float> *output, std::string *error);

} // namespace rigExec
#endif
