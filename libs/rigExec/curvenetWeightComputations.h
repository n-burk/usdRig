#ifndef RIGEXEC_CURVENET_WEIGHT_COMPUTATIONS_H
#define RIGEXEC_CURVENET_WEIGHT_COMPUTATIONS_H
#include "types.h"

#include "rigExecMath/curvenetWeights.h"

#include <memory>
#include <string>

namespace rigExec {

/// Whether \p basis and \p rangePolicy are tokens this weight can be built
/// from at all.
///
/// The structural half of the packet, asked before anything is cut: both
/// entry points below ask it, so the two cannot come to differ about which
/// tokens are legal.
bool RigExecCurvenetWeightTokensAreValid(const TfToken &basis,
                                        const TfToken &rangePolicy);

/// The BIND: cut the mesh and factorize its Laplacian for one LAYOUT.
///
/// Split out of RigExecComputeCurvenetWeightPacket, which now calls it, so
/// that a caller who must not take a lock can hold the binding itself. It
/// depends on nothing but the geometry -- which is why the computation's own
/// cache is keyed by exactly these arguments and why animating the weights
/// never re-cuts anything.
std::shared_ptr<RigExecCurvenetWeightBinding> RigExecBindCurvenetWeightPacket(
    const std::vector<GfVec3f> &mesh, const std::vector<int> &counts,
    const std::vector<int> &indices, const std::vector<GfVec3f> &net,
    const std::vector<int> &splines, const TfToken &basis, int samples,
    const std::vector<int> &autoSmooth, std::string *error);

/// The FIELD: solve \p binding for one right-hand side.
///
/// The half that runs per frame, and the half an animated inputs:weights
/// moves. Range policy and the unreached fallback are applied here because
/// they are properties of the VALUES rather than of the layout.
RigExecWeightPacket RigExecCurvenetWeightPacketFromBinding(
    const RigExecCurvenetWeightBinding &binding,
    const std::vector<float> &weights, const TfToken &rangePolicy,
    float fallback, std::string *error);

RigExecWeightPacket RigExecComputeCurvenetWeightPacket(
    const std::vector<GfVec3f> &mesh, const std::vector<int> &counts,
    const std::vector<int> &indices, const std::vector<GfVec3f> &net,
    const std::vector<int> &splines, const TfToken &basis, int samples,
    const std::vector<int> &autoSmooth, const std::vector<float> &weights,
    const TfToken &rangePolicy, float fallback, std::string *error);
}
#endif
