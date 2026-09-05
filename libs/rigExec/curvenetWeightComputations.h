#ifndef RIGEXEC_CURVENET_WEIGHT_COMPUTATIONS_H
#define RIGEXEC_CURVENET_WEIGHT_COMPUTATIONS_H
#include "types.h"

namespace rigExec {
RigExecWeightPacket RigExecComputeCurvenetWeightPacket(
    const std::vector<GfVec3f> &mesh, const std::vector<int> &counts,
    const std::vector<int> &indices, const std::vector<GfVec3f> &net,
    const std::vector<int> &splines, const TfToken &basis, int samples,
    const std::vector<int> &autoSmooth, const std::vector<float> &weights,
    const TfToken &rangePolicy, float fallback, std::string *error);
}
#endif
