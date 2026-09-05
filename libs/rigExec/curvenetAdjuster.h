#ifndef RIGEXEC_CURVENET_ADJUSTER_H
#define RIGEXEC_CURVENET_ADJUSTER_H
#include "moverGraph.h"
namespace rigExec {
std::vector<SdfPath> RigExecCurvenetAdjustmentPaths(const UsdPrim &mover);
bool RigExecValidateCurvenetAdjuster(const UsdPrim &mover,
    const SdfPath &target, std::string *error);
RigExecMoverParameters RigExecAssembleCurvenetAdjusterParameters(
    const UsdPrim &mover, const SdfPath &target,
    const RigExecWeightPacket *weights, UsdTimeCode time,
    const RigExecResolvedInputs *resolved);
} // namespace rigExec
#endif
