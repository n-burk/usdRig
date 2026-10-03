// Runtime radial-basis interpolation over solved wire tables.

#include "poseInternal.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

constexpr double _RrRbfNormalizeFloor = 1.0e-6;
constexpr double _RrRbfSameRotation = 1.0e-6;
constexpr double _RrRbfSameTranslation = 1.0e-9;

// Radial-basis pose interpolation (rbf.cpp): the per-frame evaluation
// half plus the solved-table reconstitution. The width fitting and the
// matrix inverse ran once at conversion time; what travels on the wire
// is their answer.

RrQuatd
_RrRbfMakeQuat(double w, double x, double y, double z)
{
    return RrQuatd(w, RrVec3d(x, y, z));
}

// The 4-vector dot, real-first then x, y, z -- NOT the imaginary dot,
// which sums in a different order.
double
_RrRbfQuatDot(const RrQuatd &a, const RrQuatd &b)
{
    const RrVec3d &ai = a.GetImaginary();
    const RrVec3d &bi = b.GetImaginary();
    return a.GetReal() * b.GetReal() + ai[0] * bi[0] + ai[1] * bi[1] +
           ai[2] * bi[2];
}

double
_RrRbfGaussian(double distance, double radius)
{
    if (radius <= 0.0) {
        return distance <= 0.0 ? 1.0 : 0.0;
    }
    const double ratio = distance / radius;
    return std::exp(-(ratio * ratio));
}

double
_RrRbfLinear(double distance, double radius)
{
    if (radius <= 0.0) {
        return distance <= 0.0 ? 1.0 : 0.0;
    }
    return std::max(0.0, 1.0 - distance / radius);
}

} // namespace

namespace runtimePoseDetail {

double
_RrRbfEvalKernel(_RrRbfKernel kernel, double distance, double radius)
{
    return kernel == _RrRbfKernel::Linear
               ? _RrRbfLinear(distance, radius)
               : _RrRbfGaussian(distance, radius);
}

double
_RrRbfCombine(double angle, double width, double gap,
              double translationWidth, bool enableRotation,
              bool enableTranslation)
{
    if (!enableTranslation) {
        if (!enableRotation) {
            return 0.0;
        }
        if (width <= 0.0) {
            return angle <= 0.0
                ? 0.0
                : std::numeric_limits<double>::infinity();
        }
        return angle / width;
    }

    double total = 0.0;
    if (enableRotation) {
        if (width <= 0.0) {
            if (angle > 0.0) {
                return std::numeric_limits<double>::infinity();
            }
        } else {
            const double share = angle / width;
            total += share * share;
        }
    }
    if (translationWidth <= 0.0) {
        if (gap > 0.0) {
            return std::numeric_limits<double>::infinity();
        }
    } else {
        const double share = gap / translationWidth;
        total += share * share;
    }
    return std::sqrt(total);
}

// The closed-form euler -> quaternion, term for term, NOT a GfRotation
// composition (rbf.py:1009-1027).
RrQuatd
_RrRbfQuaternionFromEuler(const RrVec3d &euler)
{
    const double hx = euler[0] * 0.5;
    const double hy = euler[1] * 0.5;
    const double hz = euler[2] * 0.5;
    const double cx = std::cos(hx), cy = std::cos(hy), cz = std::cos(hz);
    const double sx = std::sin(hx), sy = std::sin(hy), sz = std::sin(hz);
    return _RrRbfMakeQuat(cx * cy * cz + sx * sy * sz,
                          sx * cy * cz - cx * sy * sz,
                          cx * sy * cz + sx * cy * sz,
                          cx * cy * sz - sx * sy * cz);
}

RrVec3d
_RrRbfEulerFromQuaternion(const RrQuatd &quaternion)
{
    const double w = quaternion.GetReal();
    const RrVec3d &v = quaternion.GetImaginary();
    const double x = v[0], y = v[1], z = v[2];

    const double sinr = 2.0 * (w * x + y * z);
    const double cosr = 1.0 - 2.0 * (x * x + y * y);
    const double sinp = 2.0 * (w * y - z * x);
    const double pitch = std::abs(sinp) >= 1.0
                             ? std::copysign(_RrPi / 2.0, sinp)
                             : std::asin(sinp);
    const double siny = 2.0 * (w * z + x * y);
    const double cosy = 1.0 - 2.0 * (y * y + z * z);
    return RrVec3d(std::atan2(sinr, cosr), pitch,
                   std::atan2(siny, cosy));
}

double
_RrRbfAngleBetween(const RrQuatd &first, const RrQuatd &second)
{
    const double dot = _RrRbfQuatDot(first, second);
    return 2.0 * std::acos(std::min(1.0, std::abs(dot)));
}

void
_RrRbfSwingTwist(const RrQuatd &quaternion, const RrVec3d &axis,
                 RrQuatd *swing, RrQuatd *twist)
{
    const double w = quaternion.GetReal();
    const RrVec3d &v = quaternion.GetImaginary();
    const double x = v[0], y = v[1], z = v[2];

    double ax = axis[0], ay = axis[1], az = axis[2];
    double length = std::sqrt(ax * ax + ay * ay + az * az);
    if (length == 0.0) {
        length = 1.0;
    }
    ax /= length;
    ay /= length;
    az /= length;

    const double dot = x * ax + y * ay + z * az;
    double tw[4] = {w, ax * dot, ay * dot, az * dot};
    const double size = std::sqrt(tw[0] * tw[0] + tw[1] * tw[1] +
                                  tw[2] * tw[2] + tw[3] * tw[3]);
    if (size < 1.0e-9) {
        tw[0] = 1.0;
        tw[1] = tw[2] = tw[3] = 0.0;
    } else {
        tw[0] /= size;
        tw[1] /= size;
        tw[2] /= size;
        tw[3] /= size;
    }

    const double tw_w = tw[0], tw_x = -tw[1], tw_y = -tw[2],
                 tw_z = -tw[3];
    const RrQuatd sw =
        _RrRbfMakeQuat(w * tw_w - x * tw_x - y * tw_y - z * tw_z,
                       w * tw_x + x * tw_w + y * tw_z - z * tw_y,
                       w * tw_y - x * tw_z + y * tw_w + z * tw_x,
                       w * tw_z + x * tw_y - y * tw_x + z * tw_w);

    if (swing) {
        *swing = sw;
    }
    if (twist) {
        *twist = _RrRbfMakeQuat(tw[0], tw[1], tw[2], tw[3]);
    }
}

_RrRbfSolver::_RrRbfSolver(const std::vector<RrVec3d> &poses,
             const std::vector<RrVec3d> &translations,
             const std::vector<_RrRbfPoseType> &poseTypes,
             const RrVec3d &twistAxis, _RrRbfKernel kernel,
             double radius, double translationRadius,
             bool normalize,
             bool enableRotation, bool enableTranslation)
    : _poses(poses),
      _translations(translations),
      _poseTypes(poseTypes),
      _twistAxis(twistAxis),
      _kernel(kernel),
      _normalize(normalize),
      _enableRotation(enableRotation)
{
    _enableTranslation =
        enableTranslation && !_translations.empty();

    _parts.resize(_poses.size());
    for (size_t i = 0; i < _poses.size(); ++i) {
        const RrQuatd whole = _RrRbfQuaternionFromEuler(_poses[i]);
        RrQuatd swing, twist;
        _RrRbfSwingTwist(whole, _twistAxis, &swing, &twist);
        _parts[i] = {whole, swing, twist};
    }

    _radius = radius > 0.0 ? radius : _MeasureRadius();
    _translationRadius = translationRadius > 0.0
                             ? translationRadius
                             : _MeasureTranslationRadius();
}

void _RrRbfSolver::SetSolvedTable(const std::vector<double> &radii,
                    const std::vector<double> &translationRadii,
                    const std::vector<std::vector<double>> &weights)
{
    _radii = radii;
    _translationRadii = translationRadii;
    _weights = weights;
}

void _RrRbfSolver::Kernels(const RrVec3d &euler, const RrVec3d *translation,
             std::vector<double> *out) const
{
    const RrQuatd here = _RrRbfQuaternionFromEuler(euler);
    out->resize(_poses.size());
    for (size_t index = 0; index < _poses.size(); ++index) {
        (*out)[index] = _Kernel(Ratio(here, index, translation));
    }
}

void _RrRbfSolver::Normalize(std::vector<double> *weights) const
{
    if (!_normalize) {
        return;
    }
    double total = 0.0;
    for (double value : *weights) {
        total += value;
    }
    if (std::abs(total) < _RrRbfNormalizeFloor) {
        return;
    }
    for (double &value : *weights) {
        value /= total;
    }
}

void _RrRbfSolver::Evaluate(const RrVec3d &euler, const RrVec3d *translation,
              std::vector<double> *out,
              bool allowNegativeWeights) const
{
    out->clear();
    if (_weights.empty()) {
        return;
    }
    std::vector<double> found;
    Kernels(euler, translation, &found);
    out->resize(_weights.size());
    for (size_t row = 0; row < _weights.size(); ++row) {
        double total = 0.0;
        for (size_t index = 0; index < found.size(); ++index) {
            total += _weights[row][index] * found[index];
        }
        (*out)[row] = total;
    }
    Normalize(out);
    if (!allowNegativeWeights) {
        for (double &value : *out) {
            value = std::max(0.0, value);
        }
    }
}

double _RrRbfSolver::Distance(const RrQuatd &quaternion, size_t index) const
{
    const _RrRbfPoseType kind =
        index < _poseTypes.size() ? _poseTypes[index]
                                  : _RrRbfPoseType::Whole;
    if (kind != _RrRbfPoseType::Swing &&
        kind != _RrRbfPoseType::Twist) {
        return _RrRbfAngleBetween(quaternion, _parts[index][0]);
    }
    const size_t part =
        kind == _RrRbfPoseType::Swing ? 1 : 2;
    RrQuatd swing, twist;
    _RrRbfSwingTwist(quaternion, _twistAxis, &swing, &twist);
    return _RrRbfAngleBetween(part == 1 ? swing : twist,
                              _parts[index][part]);
}

double _RrRbfSolver::TranslationDistance(const RrVec3d *translation,
                           size_t index) const
{
    if (index >= _translations.size()) {
        return 0.0;
    }
    static const RrVec3d rest(0.0);
    const RrVec3d &here = translation ? *translation : rest;
    const RrVec3d &there = _translations[index];
    double total = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double delta = here[i] - there[i];
        total += delta * delta;
    }
    return std::sqrt(total);
}

double _RrRbfSolver::Ratio(const RrQuatd &quaternion, size_t index,
             const RrVec3d *translation) const
{
    const double angle =
        _enableRotation ? Distance(quaternion, index) : 0.0;
    const double gap = _enableTranslation
        ? TranslationDistance(translation, index)
        : 0.0;
    return _RrRbfCombine(angle, _Width(index), gap,
                         _TranslationWidth(index), _enableRotation,
                         _enableTranslation);
}

double _RrRbfSolver::_Kernel(double ratio) const
{
    return _RrRbfEvalKernel(_kernel, ratio, 1.0);
}

double _RrRbfSolver::_Width(size_t index) const
{
    if (!_radii.empty() && index < _radii.size()) {
        return _radii[index];
    }
    return _radius;
}

double _RrRbfSolver::_TranslationWidth(size_t index) const
{
    if (!_translationRadii.empty() &&
        index < _translationRadii.size()) {
        return _translationRadii[index];
    }
    return _translationRadius;
}

double _RrRbfSolver::_MeasureRadius() const
{
    if (_poses.size() < 2) {
        return _RrPi / 2.0;
    }
    std::vector<double> nearest;
    for (size_t index = 0; index < _poses.size(); ++index) {
        const RrQuatd &here = _parts[index][0];
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _poses.size(); ++other) {
            if (other == index) {
                continue;
            }
            const double value = Distance(here, other);
            if (value <= _RrRbfSameRotation) {
                continue;
            }
            if (!found || value < best) {
                best = value;
                found = true;
            }
        }
        if (found) {
            nearest.push_back(best);
        }
    }
    double total = 0.0;
    for (double value : nearest) {
        total += value;
    }
    if (nearest.empty() || total <= 0.0) {
        return _RrPi / 2.0;
    }
    return total / static_cast<double>(nearest.size());
}

double _RrRbfSolver::_MeasureTranslationRadius() const
{
    if (_translations.size() < 2) {
        return 0.0;
    }
    std::vector<double> nearest;
    for (size_t index = 0; index < _translations.size(); ++index) {
        const RrVec3d &here = _translations[index];
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _translations.size();
             ++other) {
            if (other == index) {
                continue;
            }
            const double value =
                TranslationDistance(&here, other);
            if (value <= _RrRbfSameTranslation) {
                continue;
            }
            if (!found || value < best) {
                best = value;
                found = true;
            }
        }
        if (found) {
            nearest.push_back(best);
        }
    }
    double total = 0.0;
    for (double value : nearest) {
        total += value;
    }
    if (nearest.empty() || total <= 0.0) {
        return 0.0;
    }
    return total / static_cast<double>(nearest.size());
}

} // namespace runtimePoseDetail

} // namespace rigExec
