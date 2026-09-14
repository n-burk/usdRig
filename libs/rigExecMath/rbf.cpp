//
// RigExec radial-basis pose interpolation. See rbf.h for the conventions and
// for why this is native rather than a call into the reference implementation.
//
// THE ARITHMETIC IS DELIBERATELY LITERAL. Every sum below accumulates in the
// Python's own left-to-right order, every division is spelled the way the
// Python spells it, and the Gauss-Jordan pivot search picks the FIRST maximal
// row exactly as Python's `max(..., key=...)` does. That is not fussiness:
// tests/testRigExecRbf.cpp compares this against the Python at 1e-6 THROUGH A
// MATRIX INVERSE, and several of the biped's interpolators are ill enough
// conditioned that a reassociated sum moves the answer well past that. Where
// a rewrite would obviously be faster, the fast spelling belongs in a
// separate evaluation path with its own tolerance, not here.
//
#include "rigExecMath/rbf.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rigExec {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

/// The ratio a pose gets when a channel it is measured on has no width at
/// all and the driver is not sitting exactly on it. Both kernels take it to
/// zero -- exp(-inf) and max(0, 1-inf) -- which is the same answer a
/// zero-width branch would give by hand, without the branch. rbf.py:71-75.
const double kInfinite = std::numeric_limits<double>::infinity();

/// the conventional default poseFalloff, and the divisor that turns a painted share
/// into a multiple of the nearest-neighbour spacing. rbf.py:629-630.
constexpr double kDefaultFalloff = 0.3;

/// The floor under that share, so a pose painted at zero still has SOME
/// width rather than an infinite ratio. rbf.py:630.
constexpr double kMinimumShare = 0.05;

/// Poses closer than this are the same pose, for the purpose of measuring
/// spacing. rbf.py:571, 625, 800.
constexpr double kSameRotation = 1.0e-6;

/// And the same for translations, in metres. rbf.py:601, 649, 732.
constexpr double kSameTranslation = 1.0e-9;

GfQuatd
MakeQuat(double w, double x, double y, double z)
{
    return GfQuatd(w, GfVec3d(x, y, z));
}

/// The 4-vector dot, accumulated real-first then x, y, z -- the order
/// `sum(a * b for a, b in zip(...))` produces over a (w, x, y, z) tuple.
/// GfDot() sums the imaginary part first, which is a different double.
double
QuatDot(const GfQuatd &a, const GfQuatd &b)
{
    const GfVec3d &ai = a.GetImaginary();
    const GfVec3d &bi = b.GetImaginary();
    return a.GetReal() * b.GetReal() + ai[0] * bi[0] + ai[1] * bi[1] +
           ai[2] * bi[2];
}

}  // namespace

// ---------------------------------------------------------------------------
// Kernels and the metric
// ---------------------------------------------------------------------------

double
RigExecRbfGaussian(double distance, double radius)
{
    if (radius <= 0.0) {
        return distance <= 0.0 ? 1.0 : 0.0;
    }
    const double ratio = distance / radius;
    return std::exp(-(ratio * ratio));
}

double
RigExecRbfLinear(double distance, double radius)
{
    if (radius <= 0.0) {
        return distance <= 0.0 ? 1.0 : 0.0;
    }
    return std::max(0.0, 1.0 - distance / radius);
}

double
RigExecRbfEvalKernel(RigExecRbfKernel kernel, double distance, double radius)
{
    return kernel == RigExecRbfKernel::Linear
               ? RigExecRbfLinear(distance, radius)
               : RigExecRbfGaussian(distance, radius);
}

double
RigExecRbfCombine(double angle, double width, double gap,
                  double translationWidth, bool enableRotation,
                  bool enableTranslation)
{
    // The rotation-only case returns the BARE ratio, with no square and no
    // square root. That is the float the kernels were handed before the
    // translation channel existed, and every table already shipped was
    // solved with it; sqrt(x * x) would be right to within an ulp and wrong
    // as a promise (rbf.py:161-169).
    if (!enableTranslation) {
        if (!enableRotation) {
            return 0.0;
        }
        if (width <= 0.0) {
            return angle <= 0.0 ? 0.0 : kInfinite;
        }
        return angle / width;
    }

    double total = 0.0;
    if (enableRotation) {
        if (width <= 0.0) {
            if (angle > 0.0) {
                return kInfinite;
            }
        } else {
            const double share = angle / width;
            total += share * share;
        }
    }
    if (translationWidth <= 0.0) {
        if (gap > 0.0) {
            return kInfinite;
        }
    } else {
        const double share = gap / translationWidth;
        total += share * share;
    }
    return std::sqrt(total);
}

// ---------------------------------------------------------------------------
// Rotations
// ---------------------------------------------------------------------------

GfQuatd
RigExecRbfQuaternionFromEuler(const GfVec3d &euler)
{
    // rbf.py:1009-1027, term for term. Not GfRotation: see the conventions
    // note in rbf.h -- the composition order reverses under this codebase's
    // row-vector convention, and only the closed form is the same double.
    const double hx = euler[0] * 0.5;
    const double hy = euler[1] * 0.5;
    const double hz = euler[2] * 0.5;
    const double cx = std::cos(hx), cy = std::cos(hy), cz = std::cos(hz);
    const double sx = std::sin(hx), sy = std::sin(hy), sz = std::sin(hz);
    return MakeQuat(cx * cy * cz + sx * sy * sz,
                    sx * cy * cz - cx * sy * sz,
                    cx * sy * cz + sx * cy * sz,
                    cx * cy * sz - sx * sy * cz);
}

GfVec3d
RigExecRbfEulerFromQuaternion(const GfQuatd &quaternion)
{
    // rbf.py:1130-1146.
    const double w = quaternion.GetReal();
    const GfVec3d &v = quaternion.GetImaginary();
    const double x = v[0], y = v[1], z = v[2];

    const double sinr = 2.0 * (w * x + y * z);
    const double cosr = 1.0 - 2.0 * (x * x + y * y);
    const double sinp = 2.0 * (w * y - z * x);
    // At the gimbal pole asin() is out of domain by a rounding error, so the
    // pitch is taken as +-pi/2 with the sign of sinp, the way math.copysign
    // does it.
    const double pitch = std::abs(sinp) >= 1.0
                             ? std::copysign(kPi / 2.0, sinp)
                             : std::asin(sinp);
    const double siny = 2.0 * (w * z + x * y);
    const double cosy = 1.0 - 2.0 * (y * y + z * z);
    return GfVec3d(std::atan2(sinr, cosr), pitch, std::atan2(siny, cosy));
}

double
RigExecRbfAngleBetween(const GfQuatd &first, const GfQuatd &second)
{
    // rbf.py:1080-1091. The |dot| is the short way round the double cover --
    // a quaternion and its negative are the same rotation, and without the
    // absolute value poses more than a half turn apart measure as though
    // they were close.
    const double dot = QuatDot(first, second);
    return 2.0 * std::acos(std::min(1.0, std::abs(dot)));
}

void
RigExecRbfSwingTwist(const GfQuatd &quaternion, const GfVec3d &axis,
                     GfQuatd *swing, GfQuatd *twist)
{
    // rbf.py:1041-1077.
    const double w = quaternion.GetReal();
    const GfVec3d &v = quaternion.GetImaginary();
    const double x = v[0], y = v[1], z = v[2];

    double ax = axis[0], ay = axis[1], az = axis[2];
    double length = std::sqrt(ax * ax + ay * ay + az * az);
    if (length == 0.0) {
        length = 1.0;  // Python's `or 1.0`
    }
    ax /= length;
    ay /= length;
    az /= length;

    // The rotation vector projected onto the axis IS the twist.
    const double dot = x * ax + y * ay + z * az;
    double tw[4] = {w, ax * dot, ay * dot, az * dot};
    const double size = std::sqrt(tw[0] * tw[0] + tw[1] * tw[1] +
                                  tw[2] * tw[2] + tw[3] * tw[3]);
    if (size < 1.0e-9) {
        // A half turn perpendicular to the axis: no twist at all.
        tw[0] = 1.0;
        tw[1] = tw[2] = tw[3] = 0.0;
    } else {
        tw[0] /= size;
        tw[1] /= size;
        tw[2] /= size;
        tw[3] /= size;
    }

    // swing = quaternion * twist.conjugated(). Written out rather than as
    // `quaternion * GfQuatd(...).GetConjugate()`: Gf's operator* is the same
    // Hamilton product term for term (verified against rbf.py:1073-1076),
    // but it forms the imaginary part as r1*i2 + r2*i1 + cross(i1, i2),
    // which reassociates the six products into a different double.
    const double tw_w = tw[0], tw_x = -tw[1], tw_y = -tw[2], tw_z = -tw[3];
    const GfQuatd sw =
        MakeQuat(w * tw_w - x * tw_x - y * tw_y - z * tw_z,
                 w * tw_x + x * tw_w + y * tw_z - z * tw_y,
                 w * tw_y - x * tw_z + y * tw_w + z * tw_x,
                 w * tw_z + x * tw_y - y * tw_x + z * tw_w);

    if (swing) {
        *swing = sw;
    }
    if (twist) {
        *twist = MakeQuat(tw[0], tw[1], tw[2], tw[3]);
    }
}

GfVec3d
RigExecRbfSlerpEuler(const GfVec3d &first, const GfVec3d &second,
                     double amount)
{
    // rbf.py:1094-1127. GfSlerp would do the same job and not the same
    // doubles; this is sampled by the width fitter, so its output feeds the
    // coverage and overshoot numbers the fixture pins.
    const GfQuatd start = RigExecRbfQuaternionFromEuler(first);
    GfQuatd end = RigExecRbfQuaternionFromEuler(second);
    double dot = QuatDot(start, end);
    if (dot < 0.0) {
        end = MakeQuat(-end.GetReal(), -end.GetImaginary()[0],
                       -end.GetImaginary()[1], -end.GetImaginary()[2]);
        dot = -dot;
    }
    dot = std::min(1.0, std::max(-1.0, dot));
    const double theta = std::acos(dot);

    double b[4];
    if (theta < 1.0e-6) {
        b[0] = start.GetReal();
        b[1] = start.GetImaginary()[0];
        b[2] = start.GetImaginary()[1];
        b[3] = start.GetImaginary()[2];
    } else {
        const double sine = std::sin(theta);
        const double firstShare = std::sin((1.0 - amount) * theta) / sine;
        const double secondShare = std::sin(amount * theta) / sine;
        b[0] = start.GetReal() * firstShare + end.GetReal() * secondShare;
        for (int i = 0; i < 3; ++i) {
            b[i + 1] = start.GetImaginary()[i] * firstShare +
                       end.GetImaginary()[i] * secondShare;
        }
    }
    double size =
        std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + b[3] * b[3]);
    if (size == 0.0) {
        size = 1.0;  // Python's `or 1.0`
    }
    return RigExecRbfEulerFromQuaternion(
        MakeQuat(b[0] / size, b[1] / size, b[2] / size, b[3] / size));
}

bool
RigExecRbfInvert(const std::vector<std::vector<double>> &matrix,
                 std::vector<std::vector<double>> *inverse)
{
    // rbf.py:1149-1187, Gauss-Jordan with partial pivoting.
    const size_t size = matrix.size();
    std::vector<std::vector<double>> work(size);
    for (size_t i = 0; i < size; ++i) {
        work[i].resize(size * 2, 0.0);
        for (size_t j = 0; j < size; ++j) {
            work[i][j] = matrix[i][j];
        }
        work[i][size + i] = 1.0;
    }

    for (size_t column = 0; column < size; ++column) {
        // Python's max(range(column, size), key=...) keeps the FIRST maximal
        // row, so the comparison is strict.
        size_t pivot = column;
        double best = std::abs(work[column][column]);
        for (size_t row = column + 1; row < size; ++row) {
            const double value = std::abs(work[row][column]);
            if (value > best) {
                best = value;
                pivot = row;
            }
        }
        if (std::abs(work[pivot][column]) < RigExecRbfSingular) {
            return false;
        }
        std::swap(work[column], work[pivot]);

        const double divisor = work[column][column];
        for (size_t j = 0; j < size * 2; ++j) {
            work[column][j] /= divisor;
        }

        for (size_t row = 0; row < size; ++row) {
            if (row == column) {
                continue;
            }
            const double factor = work[row][column];
            if (factor == 0.0) {
                continue;
            }
            for (size_t j = 0; j < size * 2; ++j) {
                work[row][j] -= factor * work[column][j];
            }
        }
    }

    inverse->assign(size, std::vector<double>(size, 0.0));
    for (size_t i = 0; i < size; ++i) {
        for (size_t j = 0; j < size; ++j) {
            (*inverse)[i][j] = work[i][size + j];
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// RigExecRbfSolver
// ---------------------------------------------------------------------------

RigExecRbfSolver::RigExecRbfSolver(const RigExecRbfSolverDesc &desc)
    : _poses(desc.poses),
      _translations(desc.translations),
      _poseTypes(desc.poseTypes),
      _twistAxis(desc.twistAxis),
      _kernel(desc.kernel),
      _regularization(desc.regularization),
      _normalize(desc.normalize),
      _enableRotation(desc.enableRotation)
{
    // Asked for but not given is OFF, not a crash and not a channel that
    // silently measures zero: an interpolator with no translations has
    // nothing to say about translation, and saying "every pose is equally
    // close" would peg its weights at 1/n. rbf.py:431-436.
    _enableTranslation = desc.enableTranslation && !_translations.empty();

    // Each pose as (whole, swing, twist), split once rather than per call.
    _parts.resize(_poses.size());
    for (size_t i = 0; i < _poses.size(); ++i) {
        const GfQuatd whole = RigExecRbfQuaternionFromEuler(_poses[i]);
        GfQuatd swing, twist;
        RigExecRbfSwingTwist(whole, _twistAxis, &swing, &twist);
        _parts[i] = {whole, swing, twist};
    }

    // ORDER MATTERS HERE, the way it does in rbf.py:439-447: _MeasureRadii
    // falls back on _radius when a pose has no separable neighbour, so the
    // shared radii have to be settled first.
    _radius = desc.radius > 0.0 ? desc.radius : _MeasureRadius();
    _translationRadius = desc.translationRadius > 0.0
                             ? desc.translationRadius
                             : _MeasureTranslationRadius();
    if (!desc.falloffs.empty()) {
        _radii = _MeasureRadii(desc.falloffs);
        if (!_translations.empty()) {
            _translationRadii = _MeasureTranslationRadii(desc.falloffs);
        }
    }
}

double
RigExecRbfSolver::_Kernel(double ratio) const
{
    // With a radius of exactly ONE, because the ratio is already in units of
    // the pose's own falloff. Dividing by 1.0 is exact in binary floating
    // point, so what comes back is the same float the kernel produced when
    // it was handed an angle and a radius (rbf.py:535-550).
    return RigExecRbfEvalKernel(_kernel, ratio, 1.0);
}

double
RigExecRbfSolver::_Width(size_t index) const
{
    if (!_radii.empty() && index < _radii.size()) {
        return _radii[index];
    }
    return _radius;
}

double
RigExecRbfSolver::_TranslationWidth(size_t index) const
{
    if (!_translationRadii.empty() && index < _translationRadii.size()) {
        return _translationRadii[index];
    }
    return _translationRadius;
}

const GfVec3d *
RigExecRbfSolver::_Translation(size_t index) const
{
    return index < _translations.size() ? &_translations[index] : nullptr;
}

double
RigExecRbfSolver::Distance(const GfQuatd &quaternion, size_t index) const
{
    // The metric belongs to the POSE, not to the interpolator: the conventional tool measures
    // a swing pose by the swing part and a twist pose by the twist part, so
    // a driver carrying both is judged on the half each pose is about.
    // rbf.py:462-483.
    const RigExecRbfPoseType kind = index < _poseTypes.size()
                                        ? _poseTypes[index]
                                        : RigExecRbfPoseType::Whole;
    if (kind != RigExecRbfPoseType::Swing &&
        kind != RigExecRbfPoseType::Twist) {
        return RigExecRbfAngleBetween(quaternion, _parts[index][0]);
    }
    const size_t part = kind == RigExecRbfPoseType::Swing ? 1 : 2;
    GfQuatd swing, twist;
    RigExecRbfSwingTwist(quaternion, _twistAxis, &swing, &twist);
    return RigExecRbfAngleBetween(part == 1 ? swing : twist,
                                  _parts[index][part]);
}

double
RigExecRbfSolver::TranslationDistance(const GfVec3d *translation,
                                      size_t index) const
{
    // Straight euclidean distance. the conventional tool has no per-axis weighting here and
    // neither does this: a brow driver pushed up and one pushed sideways by
    // the same amount are equally far from neutral. rbf.py:485-508.
    if (index >= _translations.size()) {
        return 0.0;
    }
    static const GfVec3d rest(0.0);
    const GfVec3d &here = translation ? *translation : rest;
    const GfVec3d &there = _translations[index];
    double total = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double delta = here[i] - there[i];
        total += delta * delta;
    }
    return std::sqrt(total);
}

double
RigExecRbfSolver::Ratio(const GfQuatd &quaternion, size_t index,
                        const GfVec3d *translation) const
{
    // The channels the conventional tool has turned off are NOT MEASURED AT ALL -- not
    // measured and weighted zero, not measured. rbf.py:510-533.
    const double angle = _enableRotation ? Distance(quaternion, index) : 0.0;
    const double gap =
        _enableTranslation ? TranslationDistance(translation, index) : 0.0;
    return RigExecRbfCombine(angle, _Width(index), gap,
                             _TranslationWidth(index), _enableRotation,
                             _enableTranslation);
}

double
RigExecRbfSolver::_MeasureRadius() const
{
    // The MEAN distance from each pose to its nearest neighbour, which is
    // the width at which neighbours just meet: wider and every pose bleeds
    // into the next, narrower and there are gaps between them where nothing
    // is driving anything. rbf.py:552-576.
    if (_poses.size() < 2) {
        return kPi / 2.0;
    }
    std::vector<double> nearest;
    for (size_t index = 0; index < _poses.size(); ++index) {
        const GfQuatd &here = _parts[index][0];
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _poses.size(); ++other) {
            if (other == index) {
                continue;
            }
            const double value = Distance(here, other);
            if (value <= kSameRotation) {
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
        // Half a turn is a sensible guess for a joint.
        return kPi / 2.0;
    }
    return total / static_cast<double>(nearest.size());
}

double
RigExecRbfSolver::_MeasureTranslationRadius() const
{
    // Zero when there is nothing to measure. That is not a fallback value,
    // it is the honest answer: a channel with no spread should not be handed
    // an invented width. Deliberately NOT _MeasureRadius's pi/2 -- half a
    // turn is a sensible guess for a joint; half a metre is not a sensible
    // guess for a brow. rbf.py:578-606.
    if (_translations.size() < 2) {
        return 0.0;
    }
    std::vector<double> nearest;
    for (size_t index = 0; index < _translations.size(); ++index) {
        const GfVec3d &here = _translations[index];
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _translations.size(); ++other) {
            if (other == index) {
                continue;
            }
            const double value = TranslationDistance(&here, other);
            if (value <= kSameTranslation) {
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

std::vector<double>
RigExecRbfSolver::_MeasureRadii(const std::vector<double> &falloffs) const
{
    // One falloff width per pose, from its OWN nearest neighbour. This is
    // what a shared radius cannot do: a thigh with poses at 25, 90 and 120
    // degrees has a mean spacing too wide for the close pair and too narrow
    // for the far one, and the weights overshoot 0..1 in between, which
    // reads as a pop. rbf.py:608-631.
    std::vector<double> out;
    out.reserve(_poses.size());
    for (size_t index = 0; index < _poses.size(); ++index) {
        // NOTE THE DIRECTION: the OTHER pose is measured against THIS pose's
        // metric -- Distance(other, index), not Distance(this, other). A
        // twist pose's neighbours are near in twist even when they are far
        // as whole rotations, and Spacing() below measures the other way
        // round on purpose. rbf.py:618-624 against rbf.py:789-802.
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _poses.size(); ++other) {
            if (other == index) {
                continue;
            }
            const double value = Distance(_parts[other][0], index);
            if (value <= kSameRotation) {
                continue;
            }
            if (!found || value < best) {
                best = value;
                found = true;
            }
        }
        double nearest = found ? best : _radius;
        if (nearest <= 0.0) {
            nearest = _radius;
        }
        const double share =
            index < falloffs.size() ? falloffs[index] : kDefaultFalloff;
        out.push_back(nearest *
                      std::max(share / kDefaultFalloff, kMinimumShare));
    }
    return out;
}

std::vector<double>
RigExecRbfSolver::_MeasureTranslationRadii(
    const std::vector<double> &falloffs) const
{
    // The same share drives both channels -- the conventional tool paints one number per
    // pose, not one per channel. rbf.py:633-655.
    std::vector<double> out;
    out.reserve(_translations.size());
    for (size_t index = 0; index < _translations.size(); ++index) {
        const GfVec3d &here = _translations[index];
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _translations.size(); ++other) {
            if (other == index) {
                continue;
            }
            const double value = TranslationDistance(&here, other);
            if (value <= kSameTranslation) {
                continue;
            }
            if (!found || value < best) {
                best = value;
                found = true;
            }
        }
        double nearest = found ? best : _translationRadius;
        if (nearest <= 0.0) {
            nearest = _translationRadius;
        }
        const double share =
            index < falloffs.size() ? falloffs[index] : kDefaultFalloff;
        out.push_back(nearest *
                      std::max(share / kDefaultFalloff, kMinimumShare));
    }
    return out;
}

std::vector<double>
RigExecRbfSolver::Spacing() const
{
    // rbf.py:789-802. THIS pose measured against every OTHER pose's metric,
    // which is the opposite direction from _MeasureRadii above.
    std::vector<double> out;
    out.reserve(_poses.size());
    for (size_t index = 0; index < _poses.size(); ++index) {
        const GfQuatd &here = _parts[index][0];
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _poses.size(); ++other) {
            if (other == index) {
                continue;
            }
            const double value = Distance(here, other);
            if (value <= kSameRotation) {
                continue;
            }
            if (!found || value < best) {
                best = value;
                found = true;
            }
        }
        out.push_back(found ? best : 0.0);
    }
    return out;
}

std::vector<double>
RigExecRbfSolver::TranslationSpacing() const
{
    std::vector<double> out;
    out.reserve(_translations.size());
    for (size_t index = 0; index < _translations.size(); ++index) {
        const GfVec3d *here = _Translation(index);
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _translations.size(); ++other) {
            if (other == index) {
                continue;
            }
            const double value = TranslationDistance(here, other);
            if (value <= kSameTranslation) {
                continue;
            }
            if (!found || value < best) {
                best = value;
                found = true;
            }
        }
        out.push_back(found ? best : 0.0);
    }
    return out;
}

bool
RigExecRbfSolver::Degenerate() const
{
    // Not "nearly singular" -- actually coincident, under the channels the conventional tool
    // has turned ON. That last part is the whole of it: an interpolator the conventional tool
    // drives by TRANSLATION has identity rotations, so measuring it as a
    // rotation makes every distance zero, the kernel matrix all ones and
    // rank one, and no regularisation makes it meaningful. Regularising
    // anyway "succeeds", returns weights around 1e11, and evaluates to a
    // flat 1/n for every input. rbf.py:804-837.
    for (size_t index = 0; index < _poses.size(); ++index) {
        const GfQuatd &here = _parts[index][0];
        const GfVec3d *moved = _Translation(index);
        for (size_t other = 0; other < _poses.size(); ++other) {
            if (other == index) {
                continue;
            }
            if (_enableRotation && Distance(here, other) > kSameRotation) {
                return false;
            }
            if (_enableTranslation &&
                TranslationDistance(moved, other) > kSameTranslation) {
                return false;
            }
        }
    }
    return _poses.size() > 1;
}

std::vector<std::vector<double>>
RigExecRbfSolver::Matrix() const
{
    const size_t size = _poses.size();
    std::vector<std::vector<double>> built(size);
    for (size_t row = 0; row < size; ++row) {
        const GfQuatd &here = _parts[row][0];
        const GfVec3d *moved = _Translation(row);
        built[row].resize(size);
        for (size_t column = 0; column < size; ++column) {
            // The COLUMN's width: each pose carries its own falloff, so the
            // matrix stops being symmetric. Gauss-Jordan does not mind.
            double value = _Kernel(Ratio(here, column, moved));
            if (row == column) {
                value += _regularization;
            }
            built[row][column] = value;
        }
    }
    return built;
}

bool
RigExecRbfSolver::Solve()
{
    // THE TRANSPOSE IS NOT COSMETIC.
    //
    // Evaluate() forms w[c] = sum_i W[c][i] * phi_i(x), where phi_i is the
    // kernel carrying POSE i's width, and asks that it come out as 1 at pose
    // c and 0 at the others. Since Matrix()[r][i] is phi_i(pose_r), that
    // reads W @ M.T == I, so what has to be inverted is M TRANSPOSED.
    //
    // With one shared radius M is symmetric and the two are the same matrix,
    // which is why this went unnoticed in the Python for as long as every
    // interpolator had one. Per-pose widths make it asymmetric --
    // phi_far(near) is not phi_near(far) -- and inverting the UNTRANSPOSED
    // matrix then produces weights that are wrong AT THE POSES, which is the
    // one place an interpolator is supposed to be exact. Measured on a three
    // pose chain at 0, 25 and 120 degrees with per-pose widths: the far pose
    // read 1.716 of itself and -0.582 of its neighbour. On the shipped biped
    // it cost the head, the neck, both elbows, both wrists, both ankles and
    // both toes between 0.001 and 0.130 at their own poses -- small enough
    // to read as a soft corrective rather than as a bug. rbf.py:868-920.
    //
    // 33 of the biped's 67 interpolators fit to per-pose widths, so this is
    // the common case here, not the exotic one.
    const std::vector<std::vector<double>> rows = Matrix();
    const size_t size = rows.size();
    std::vector<std::vector<double>> built(size,
                                           std::vector<double>(size, 0.0));
    for (size_t row = 0; row < size; ++row) {
        for (size_t column = 0; column < size; ++column) {
            built[row][column] = rows[column][row];
        }
    }

    _regularizedSingular = false;
    if (size == 0) {
        _weights.clear();
        return false;
    }
    if (RigExecRbfInvert(built, &_weights)) {
        return true;
    }
    // Poses on top of each other make a singular matrix. A nudge on the
    // diagonal separates them; regularization does the same job, and
    // this is the fallback when it is zero. rbf.py:908-919.
    _regularizedSingular = true;
    for (size_t index = 0; index < size; ++index) {
        built[index][index] += RigExecRbfSingular;
    }
    if (!RigExecRbfInvert(built, &_weights)) {
        _weights.clear();
        return false;
    }
    return true;
}

void
RigExecRbfSolver::Kernels(const GfVec3d &euler, const GfVec3d *translation,
                          std::vector<double> *out) const
{
    const GfQuatd here = RigExecRbfQuaternionFromEuler(euler);
    out->resize(_poses.size());
    for (size_t index = 0; index < _poses.size(); ++index) {
        (*out)[index] = _Kernel(Ratio(here, index, translation));
    }
}

void
RigExecRbfSolver::Normalize(std::vector<double> *weights) const
{
    if (!_normalize) {
        return;
    }
    // The SUM rather than the sum of magnitudes: a negative weight is a real
    // instruction to lean away from a pose, and dividing by the absolute sum
    // would quietly turn a strong lean into a weak one. rbf.py:964-987.
    double total = 0.0;
    for (double value : *weights) {
        total += value;
    }
    if (std::abs(total) < RigExecRbfNormalizeFloor) {
        // Nothing to normalise against -- every pose is far away -- and
        // scaling by an almost-zero divisor would turn rounding into a huge
        // weight. Left alone, which is what falling away to nothing outside
        // the poses should look like.
        return;
    }
    for (double &value : *weights) {
        value /= total;
    }
}

void
RigExecRbfSolver::Evaluate(const GfVec3d &euler, const GfVec3d *translation,
                           std::vector<double> *out,
                           bool allowNegativeWeights) const
{
    out->clear();
    if (_weights.empty()) {
        // The Python solves lazily on first evaluate (rbf.py:954-957); a
        // const method cannot, so an unsolved solver answers nothing rather
        // than answering wrongly. Callers on the per-frame path have solved
        // at build time by construction.
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
        // AFTER normalisation, never before and never inside the solve.
        // the reference implementation.
        for (double &value : *out) {
            value = std::max(0.0, value);
        }
    }
}

void
RigExecRbfSolver::Walk(size_t first, size_t second, double amount,
                       GfVec3d *euler, GfVec3d *translation) const
{
    // The rotation slerps and the translation lerps, which is the path a
    // driver actually takes. Both are needed because an interpolator may be
    // watching either, and sampling only the rotation between two
    // translation poses walks a straight line through a single point.
    // rbf.py:693-718.
    *euler = RigExecRbfSlerpEuler(_poses[first], _poses[second], amount);
    if (_translations.empty()) {
        *translation = GfVec3d(0.0);
        return;
    }
    static const GfVec3d rest(0.0);
    const GfVec3d *a = _Translation(first);
    const GfVec3d *b = _Translation(second);
    const GfVec3d &here = a ? *a : rest;
    const GfVec3d &there = b ? *b : rest;
    for (int i = 0; i < 3; ++i) {
        (*translation)[i] = here[i] + (there[i] - here[i]) * amount;
    }
}

double
RigExecRbfSolver::Coverage(int steps) const
{
    // Zero is a DEAD ZONE, and it is the worst thing an interpolator can do:
    // every shape switches off there and snaps back on as the driver leaves.
    // It is what popping usually is, and it is invisible to any check made
    // AT the poses, where the answer is always right. It happens with the
    // linear kernel, whose support is compact -- the left thigh had poses 25
    // to 150 degrees apart under a 30 degree radius, so the whole middle of
    // every swing was uncovered. rbf.py:736-765.
    const size_t size = _poses.size();
    if (size < 2) {
        return 1.0;
    }
    bool found = false;
    double worst = 0.0;
    std::vector<double> kernels;
    GfVec3d euler, translation;
    const bool hasTranslations = !_translations.empty();
    for (size_t first = 0; first < size; ++first) {
        for (size_t second = 0; second < size; ++second) {
            if (first == second) {
                continue;
            }
            for (int step = 0; step <= steps; ++step) {
                Walk(first, second,
                     static_cast<double>(step) / static_cast<double>(steps),
                     &euler, &translation);
                Kernels(euler, hasTranslations ? &translation : nullptr,
                        &kernels);
                double total = 0.0;
                for (double value : kernels) {
                    total += value;
                }
                if (!found || total < worst) {
                    worst = total;
                    found = true;
                }
            }
        }
    }
    return found ? worst : 1.0;
}

double
RigExecRbfSolver::Overshoot(int steps) const
{
    // rbf.py:767-787.
    const size_t size = _poses.size();
    if (size < 2 || _weights.empty()) {
        return 0.0;
    }
    double worst = 0.0;
    std::vector<double> values;
    GfVec3d euler, translation;
    const bool hasTranslations = !_translations.empty();
    for (size_t first = 0; first < size; ++first) {
        for (size_t second = 0; second < size; ++second) {
            if (first == second) {
                continue;
            }
            for (int step = 0; step <= steps; ++step) {
                Walk(first, second,
                     static_cast<double>(step) / static_cast<double>(steps),
                     &euler, &translation);
                Evaluate(euler, hasTranslations ? &translation : nullptr,
                         &values);
                if (values.empty()) {
                    continue;
                }
                double high = values[0];
                double low = values[0];
                for (double value : values) {
                    high = std::max(high, value);
                    low = std::min(low, value);
                }
                worst = std::max(worst, std::max(high - 1.0, -low));
            }
        }
    }
    return worst;
}

void
RigExecRbfSolver::SetSolvedTable(
    const std::vector<double> &radii,
    const std::vector<double> &translationRadii,
    const std::vector<std::vector<double>> &weights)
{
    _radii = radii;
    _translationRadii = translationRadii;
    _weights = weights;
    _regularizedSingular = false;
}

void
RigExecRbfSolver::ScaleWidths(double factor)
{
    // Both channels, or a painted falloff would widen the brow's ROTATION
    // width -- which it does not use -- and leave the translation width it
    // does use alone.
    for (double &value : _radii) {
        value *= factor;
    }
    _radius *= factor;
    for (double &value : _translationRadii) {
        value *= factor;
    }
    _translationRadius *= factor;
    _weights.clear();
}

// ---------------------------------------------------------------------------
// The width fitter
// ---------------------------------------------------------------------------

RigExecRbfSolver
RigExecRbfFitWidth(const RigExecRbfSolverDesc &desc,
                   RigExecRbfFitReport *report, double floor)
{
    // rbf.py:223-352.
    RigExecRbfSolverDesc common = desc;
    common.radius = 0.0;
    common.translationRadius = 0.0;
    common.falloffs.clear();
    common.normalize = true;

    RigExecRbfSolver measure(common);
    const std::vector<double> spacing =
        measure.GetEnableRotation() ? measure.Spacing()
                                    : std::vector<double>();
    const std::vector<double> gaps = measure.GetEnableTranslation()
                                         ? measure.TranslationSpacing()
                                         : std::vector<double>();
    double widest = 0.0;
    for (double value : spacing) {
        widest = std::max(widest, value);
    }
    double widestGap = 0.0;
    for (double value : gaps) {
        widestGap = std::max(widestGap, value);
    }
    // Python takes max() of the list without a zero seed, but every entry is
    // a distance or the 0.0 placeholder Spacing() writes for an isolated
    // pose, so a zero seed cannot change the answer.

    auto fill = [](RigExecRbfFitReport *out, const RigExecRbfSolver &solver,
                   bool perPose, double scale, double coverage,
                   double overshoot) {
        if (!out) {
            return;
        }
        out->width = solver.GetRadius();
        out->translationWidth = solver.GetTranslationRadius();
        out->perPose = perPose;
        out->scale = scale;
        out->coverage = coverage;
        out->overshoot = overshoot;
    };

    if (widest <= 0.0 && widestGap <= 0.0) {
        // Nothing to fit against: no spread on either measured channel. The
        // Python returns the unsolved probe here; Solve() is called so every
        // path out of this function hands back something evaluable.
        fill(report, measure, false, 0.0, 1.0, 0.0);
        measure.Solve();
        return measure;
    }

    bool haveBest = false, haveFallback = false;
    RigExecRbfSolver best, fallback;
    RigExecRbfFitReport bestReport, fallbackReport;

    for (int perPoseStep = 0; perPoseStep < 2; ++perPoseStep) {
        const bool perPose = perPoseStep == 1;
        for (int step = 4; step < 27; ++step) {
            const double scale = step * 0.1;
            RigExecRbfSolverDesc attempt = common;
            if (perPose) {
                // Each pose measured against its OWN neighbour, which is
                // what a chain of unevenly spaced poses needs. The falloff
                // handed in is 0.3 * scale, so share / 0.3 is exactly the
                // swept scale (rbf.py:320-323).
                attempt.radius = 0.0;
                attempt.translationRadius = 0.0;
                attempt.falloffs.assign(desc.poses.size(),
                                        kDefaultFalloff * scale);
            } else {
                attempt.radius = widest * scale;
                attempt.translationRadius = widestGap * scale;
            }
            RigExecRbfSolver built(attempt);
            if (built.Degenerate()) {
                continue;
            }
            built.Solve();
            if (built.GetWeights().empty()) {
                continue;
            }
            const double covered = built.Coverage();
            const double over = built.Overshoot();

            if (covered >= floor) {
                // Least overshoot wins; a wider width (higher coverage)
                // breaks the tie, which is Python's (over, -covered) tuple
                // comparison at rbf.py:341-343.
                const bool better =
                    !haveBest ||
                    over < bestReport.overshoot ||
                    (over == bestReport.overshoot &&
                     -covered < -bestReport.coverage);
                if (better) {
                    best = built;
                    fill(&bestReport, built, perPose, scale, covered, over);
                    haveBest = true;
                }
            }
            if (!haveFallback || covered > fallbackReport.coverage) {
                fallback = built;
                fill(&fallbackReport, built, perPose, scale, covered, over);
                haveFallback = true;
            }
        }
    }

    if (haveBest) {
        if (report) {
            *report = bestReport;
        }
        return best;
    }
    if (haveFallback) {
        if (report) {
            *report = fallbackReport;
        }
        return fallback;
    }
    fill(report, measure, false, 0.0, 0.0, 0.0);
    measure.Solve();
    return measure;
}

}  // namespace rigExec
