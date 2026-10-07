// Shared, USD-free arithmetic for a two-bone limb's segment lengths and for
// scaling a solved joint along its bone. The USD solver (solvers.cpp) and the
// binary runtime (rigExecRuntime/poseSolvers.cpp) both call these, so the
// two agree by construction.
//
// Conventions: points are 3 doubles; a frame is four points (origin, then
// the three handle tips), as RigExecPointFrame and the runtime publish them.
#ifndef RIGEXEC_MATH_LIMB_STRETCH_KERNEL_H
#define RIGEXEC_MATH_LIMB_STRETCH_KERNEL_H

#include <algorithm>
#include <cmath>

namespace rigExec {

/// The per-frame limb inputs of RigExecTwoBoneIk under stretchPolicy
/// "softDistance". Defaults reproduce the unpinned, unscaled limb.
struct RigExecLimbStretch {
    /// inputs:stretch: blends the auto-stretched lengths over the authored
    /// ones, in [0, 1].
    double stretch = 1.0;
    /// inputs:softDistance: how far short of full reach the soft ease
    /// begins, in the chain's own (space-carried) units. Zero is a hard
    /// reach: the chain stretches as soon as the goal is past it.
    double softDistance = 0.0;
    /// inputs:upperScale / inputs:lowerScale: multipliers on each bone's
    /// rest length before anything else.
    double upperScale = 1.0;
    double lowerScale = 1.0;
    /// inputs:pin: blends both bones toward the lengths that put the middle
    /// joint exactly on the pole (root to pole, pole to goal), in [0, 1].
    double pin = 0.0;
    /// rigExec:scaleCalibration: subtracted from both scales inside the soft
    /// stretch only, so a chain that sits inside the soft zone at its bind
    /// pose still solves to its rest lengths there. The unstretched lengths
    /// use the scales as given.
    double scaleCalibration = 0.0;
};

/// The two segment lengths a limb solves with.
///
/// \p restUpper and \p restLower are the bones' rest lengths, \p dist the
/// root-to-goal distance, and \p poleUpper / \p poleLower the root-to-pole
/// and pole-to-goal distances, all in the same units. With
/// a = (upperScale - c) * restUpper and b = (lowerScale - c) * restLower,
/// c the scale calibration:
///
///   past the soft start (a + b - soft), both bones scale by
///     k = dist / (a + b - soft * exp(-(dist - (a + b - soft)) / soft)),
///   which eases the reach in rather than snapping it straight;
///   stretched = stretch * k * (a, b) + (1 - stretch) * (a, b);
///   unpinned  = stretch * stretched
///               + (1 - stretch) * (upperScale * restUpper,
///                                  lowerScale * restLower);
///   result    = pin * (poleUpper, poleLower) + (1 - pin) * unpinned.
///
/// The stretch weight appears twice, as in the network this reproduces; the
/// two forms agree at 0 and 1, which are the values a limb is set to.
inline void
RigExecLimbSegmentLengths(double restUpper, double restLower, double dist,
                          double poleUpper, double poleLower,
                          const RigExecLimbStretch &p, double *upper,
                          double *lower)
{
    const double stretch = std::min(std::max(p.stretch, 0.0), 1.0);
    const double pin = std::min(std::max(p.pin, 0.0), 1.0);
    const double a = restUpper * (p.upperScale - p.scaleCalibration);
    const double b = restLower * (p.lowerScale - p.scaleCalibration);
    const double chain = a + b;
    double k = 1.0;
    const double soft = std::max(p.softDistance, 0.0);
    if (chain > 1e-12) {
        if (soft > 1e-12 && dist > chain - soft) {
            const double eased =
                chain - soft * std::exp(-(dist - (chain - soft)) / soft);
            if (eased > 1e-12) {
                k = dist / eased;
            }
        } else if (soft <= 1e-12 && dist > chain) {
            k = dist / chain;
        }
    }
    const double sa = stretch * k * a + (1.0 - stretch) * a;
    const double sb = stretch * k * b + (1.0 - stretch) * b;
    const double ua =
        stretch * sa + (1.0 - stretch) * restUpper * p.upperScale;
    const double ub =
        stretch * sb + (1.0 - stretch) * restLower * p.lowerScale;
    if (upper) {
        *upper = pin * poleUpper + (1.0 - pin) * ua;
    }
    if (lower) {
        *lower = pin * poleLower + (1.0 - pin) * ub;
    }
}

/// Scales a solved frame along its bone: the component of every handle
/// along the unit direction \p dir is multiplied by \p factor, the rest is
/// left alone. That is a joint whose bone stretched to \p factor times its
/// rest length while its cross-section did not, which is what lets the
/// skin follow the length rather than only the child joint moving.
///
/// \p points is a frame's four points (origin first); V is any 3-vector
/// type with operator[].
template <class V>
inline void
RigExecScaleFrameAlong(V *points, const V &dir, double factor)
{
    if (!std::isfinite(factor) || std::abs(factor - 1.0) < 1e-15) {
        return;
    }
    const double dl = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] +
                                dir[2] * dir[2]);
    if (dl < 1e-12) {
        return;
    }
    const double u[3] = {dir[0] / dl, dir[1] / dl, dir[2] / dl};
    for (int a = 1; a < 4; ++a) {
        double h[3] = {points[a][0] - points[0][0],
                       points[a][1] - points[0][1],
                       points[a][2] - points[0][2]};
        const double along = h[0] * u[0] + h[1] * u[1] + h[2] * u[2];
        for (int c = 0; c < 3; ++c) {
            points[a][c] = points[0][c] + h[c] + (factor - 1.0) * along * u[c];
        }
    }
}

/// The length a rest bone \p restBone (child origin minus joint origin, at
/// rest) has once carried by the joint's own frame: \p restPoints to
/// \p posedPoints, the joint's four points at rest and posed. A frame that
/// already carries a scale (a master's) lengthens the bone by it, and only
/// a stretch beyond that is the joint's own. Falls back to the plain rest
/// length when the rest frame is degenerate.
template <class V>
inline double
RigExecBoneLengthUnderFrame(const V *restPoints, const V *posedPoints,
                            const V &restBone)
{
    double r[3][3], p[3][3];
    for (int a = 0; a < 3; ++a) {
        for (int c = 0; c < 3; ++c) {
            r[a][c] = restPoints[a + 1][c] - restPoints[0][c];
            p[a][c] = posedPoints[a + 1][c] - posedPoints[0][c];
        }
    }
    const double b[3] = {restBone[0], restBone[1], restBone[2]};
    const double plain = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    // Solve b = sum_a k_a r_a (Cramer), then carry k onto the posed axes.
    const auto det3 = [](const double *x, const double *y, const double *z) {
        return x[0] * (y[1] * z[2] - y[2] * z[1]) -
               x[1] * (y[0] * z[2] - y[2] * z[0]) +
               x[2] * (y[0] * z[1] - y[1] * z[0]);
    };
    const double det = det3(r[0], r[1], r[2]);
    if (!(std::abs(det) > 1e-18)) {
        return plain;
    }
    const double k[3] = {det3(b, r[1], r[2]) / det,
                         det3(r[0], b, r[2]) / det,
                         det3(r[0], r[1], b) / det};
    double out[3] = {0.0, 0.0, 0.0};
    for (int a = 0; a < 3; ++a) {
        for (int c = 0; c < 3; ++c) {
            out[c] += k[a] * p[a][c];
        }
    }
    const double len =
        std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    return std::isfinite(len) ? len : plain;
}

}  // namespace rigExec

#endif
