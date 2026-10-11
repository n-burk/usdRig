#ifndef RIGEXEC_MATH_VOLUME_FIELD_CHECK_H
#define RIGEXEC_MATH_VOLUME_FIELD_CHECK_H

// Shared volume-field validation. The reference oracle and the runtime
// oracle emit the same sentences. The scale transform that follows a
// passing check stays with the caller.

#include <cmath>
#include <string>

namespace rigExec {

inline bool RigExecVolumeExtentsOk(const std::string &who, float extentU,
                                   float extentV, std::string *error)
{
    for (const float e : {extentU, extentV}) {
        if (!std::isfinite(e) || e <= 0.0f) {
            *error = who +
                     ": inputs:extentU/V must be finite and positive "
                     "when rigExec:planeBounds is `bounded`";
            return false;
        }
    }
    return true;
}

inline std::string RigExecUnknownPlaneBoundsMessage(const std::string &who,
                                                    const std::string &mode)
{
    return who + ": unknown rigExec:planeBounds " + mode;
}

inline bool RigExecVolumeAxisScalesOk(const std::string &who, float sx,
                                      float sy, float sz, std::string *error)
{
    for (const float s : {sx, sy, sz}) {
        if (!std::isfinite(s) || s <= 0.0f) {
            *error = who + ": inputs:scaleX/Y/Z must be finite and positive";
            return false;
        }
    }
    return true;
}

template <class Vec>
inline bool RigExecSignedAxisScalesOk(const std::string &who,
                                      const Vec &positive, const Vec &negative,
                                      std::string *error)
{
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(positive[axis]) || positive[axis] <= 0.0f ||
            !std::isfinite(negative[axis]) || negative[axis] <= 0.0f) {
            *error = who + ": signed axis scales must be finite and positive";
            return false;
        }
    }
    return true;
}

} // namespace rigExec

#endif
