//
// Shared normalization for RigExecControl/RigExecJoint avars:sx/sy/sz.
// Kept in the common math layer so evaluation, authoring, and imaging apply
// one identical scale policy. Other avars require no analogous normalization.
//
#ifndef RIGEXEC_MATH_AVAR_SCALE_H
#define RIGEXEC_MATH_AVAR_SCALE_H

#include <cmath>

namespace rigExec {

/// Smallest supported magnitude for one control/joint avar scale channel.
///
/// This remains well above the point-frame degeneracy epsilon (1e-10 for a
/// unit rest frame), while still representing a visually collapsed axis.
inline constexpr double RigExecAvarScaleFloor = 1e-4;

/// True when strict authoring may accept this scale value.
inline bool
RigExecIsFiniteAvarScale(double value)
{
    return std::isfinite(value);
}

/// Apply the runtime avar-scale contract.
///
/// Finite magnitudes below RigExecAvarScaleFloor are raised to the floor with
/// their sign intact (including negative zero), because negative scale is a
/// supported reflection. Non-finite raw USD values cannot describe a usable
/// transform and resolve to identity scale. Strict authoring rejects those
/// values before they reach the stage.
inline double
RigExecNormalizeAvarScale(double value)
{
    if (!RigExecIsFiniteAvarScale(value)) {
        return 1.0;
    }
    return std::abs(value) < RigExecAvarScaleFloor
        ? std::copysign(RigExecAvarScaleFloor, value)
        : value;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_AVAR_SCALE_H
