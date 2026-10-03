#ifndef RIGEXEC_MATH_CURVENET_ADJUSTMENTS_H
#define RIGEXEC_MATH_CURVENET_ADJUSTMENTS_H

#include "curvenet.h"
#include "pxr/base/gf/matrix4d.h"

namespace rigExec {

/// A local animation delta on one knot, or a tangent parented to an earlier
/// knot command. Matrices use USD's row-vector convention.
struct RigExecCurvenetAdjustmentCommand {
    int pointIndex = -1;
    int parentCommand = -1;
    bool includeTangents = true;
    GfMatrix4d localTransform{1.0};

    bool operator==(const RigExecCurvenetAdjustmentCommand &o) const {
        return pointIndex == o.pointIndex && parentCommand == o.parentCommand &&
               includeTangents == o.includeTangents && localTransform == o.localTransform;
    }
};

/// Computes deformation-relative knot frames. Intersections use the best-fit
/// incident-tangent rotation; other knots transport nearby intersection
/// rotations along the curves, blending by inverse arc distance. Isolated
/// curves use their best-fit tangent rotation as the transport seed.
bool RigExecComputeCurvenetAdjustmentFrames(
    const RigExecCurvenetTopology &topology,
    const std::vector<GfVec3f> &restPoints,
    const std::vector<GfVec3f> &posedPoints,
    std::vector<GfMatrix4d> *frames,
    std::string *error = nullptr);

/// Applies controls after preceding point revisions. Invalid topology,
/// controls, or nonfinite data fail atomically. Optional commandFrames are
/// the fully adjusted control frames, in the same space as the point pool.
bool RigExecApplyCurvenetAdjustments(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &restPoints,
    const std::vector<int> &splineIndices,
    RigExecCurvenetBasis basis,
    const std::vector<RigExecCurvenetAdjustmentCommand> &commands,
    std::vector<GfMatrix4d> *commandFrames = nullptr,
    std::string *error = nullptr);

} // namespace rigExec
#endif
