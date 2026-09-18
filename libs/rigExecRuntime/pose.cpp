//
// rigExecRuntime pose family (M2): ComposeSubtree, Solve, SolverCommit,
// Constraint, CommitDelta, PropagateChunk, CommitApply, ProviderMatrix,
// SnapshotFinals, PoseInterpolator. A zero-USD port of the baked pose
// loop (libs/rigExec/bakedPose.cpp) over the RrStore domains.
//
// Fidelity rule: the same operations in the same order as the baked
// path; double stays double, float stays float. This TU includes ONLY
// store.h plus STL and <cmath>: no USD headers, no USD library.
//

#include "rigExecRuntime/store.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

// Smallest supported avar scale magnitude, in avarScale.h terms.
constexpr double _RrAvarScaleFloor = 1e-4;

// RigExecNormalizeAvarScale, verbatim: non-finite resolves to identity,
// magnitudes below the floor are raised to it with the sign intact.
double
_RrNormalizeAvarScale(double value)
{
    if (!std::isfinite(value)) {
        return 1.0;
    }
    return std::abs(value) < _RrAvarScaleFloor
        ? std::copysign(_RrAvarScaleFloor, value)
        : value;
}

RrMat4d
_RrIdentity()
{
    RrMat4d m;
    m.SetIdentity();
    return m;
}

// std::acos(-1) rather than M_PI, as the kernels do.
const double _RrPi = std::acos(-1);

RrMat4d
_RrWireMatrix(const RigExecWireMatrix4d &wire)
{
    RrMat4d m;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            m[r][c] = wire[r * 4 + c];
        }
    }
    return m;
}

std::array<RrVec3d, 4>
_RrWireLandmarks(const std::array<RigExecWireVec3d, 4> &wire)
{
    std::array<RrVec3d, 4> out;
    for (size_t i = 0; i < 4; ++i) {
        out[i] = RrVec3d(wire[i][0], wire[i][1], wire[i][2]);
    }
    return out;
}

// RigExecBakedComposeAvars (bakedProgramImpl.h), written the same way
// rather than algebraically simplified. `order` is the resolved token
// text; anything but a 3-letter sequence composes XYZ.
RrMat4d
_RrComposeAvars(double tx, double ty, double tz, double sx, double sy,
                double sz, double rx, double ry, double rz, double rspin,
                const std::string &order)
{
    static const RrVec3d axes[3] = {
        RrVec3d(1, 0, 0), RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
    const double angles[3] = {rx, ry, rz};
    std::string sequence = order;
    if (sequence.size() != 3) {
        sequence = "XYZ";
    }
    RrMat4d m;
    m.SetIdentity();
    m[0][0] = _RrNormalizeAvarScale(sx);
    m[1][1] = _RrNormalizeAvarScale(sy);
    m[2][2] = _RrNormalizeAvarScale(sz);
    for (const char axis : sequence) {
        const int index = axis == 'X' ? 0 : axis == 'Y' ? 1 : 2;
        if (angles[index] != 0.0) {
            m = m * RrMat4d(RrRotation(axes[index], angles[index]),
                            RrVec3d(0));
        }
    }
    if (rspin != 0.0) {
        m = m * RrMat4d(RrRotation(axes[0], rspin), RrVec3d(0));
    }
    RrMat4d t;
    t.SetIdentity();
    t.SetTranslateOnly(RrVec3d(tx, ty, tz));
    return m * t;
}

// The step body's `live`: whether a baked parameter has to be re-read
// at all (bakedPose.cpp). `uid` is the input's routed uid, or -1.
bool
_RrLive(const RrProgram *program, const RigExecWireInput &input,
        int32_t uid)
{
    if (input.varying) {
        return true;
    }
    if (input.overrideIndex >= 0 &&
        size_t(input.overrideIndex) < program->store.overridden.size() &&
        program->store.overridden[size_t(input.overrideIndex)]) {
        return true;
    }
    (void)uid;
    return false;
}

bool
_RrLiveLadder(const RrProgram *program, size_t slot, int field)
{
    return _RrLive(program, program->LadderInput(slot, field),
                   program->ladderUid[slot][size_t(field)]);
}

bool
_RrLiveSolver(const RrProgram *program, size_t solver, int field)
{
    return _RrLive(program, program->SolverInput(solver, field),
                   program->solverUid[solver][size_t(field)]);
}

bool
_RrLiveConstraint(const RrProgram *program, size_t constraint, int field)
{
    return _RrLive(program, program->ConstraintInput(constraint, field),
                   program->constraintUid[constraint][size_t(field)]);
}

}  // namespace

// ---------------------------------------------------------------------------
// Radial-basis pose interpolation (rbf.cpp): the per-frame evaluation
// half plus the solved-table reconstitution. The width fitting and the
// matrix inverse ran once at conversion time; what travels on the wire
// is their answer.
// ---------------------------------------------------------------------------

constexpr double _RrRbfSingular = 1.0e-12;
constexpr double _RrRbfNormalizeFloor = 1.0e-6;
constexpr double _RrRbfSameRotation = 1.0e-6;
constexpr double _RrRbfSameTranslation = 1.0e-9;
constexpr double _RrRbfDefaultFalloff = 0.3;
constexpr double _RrRbfMinimumShare = 0.05;

enum class _RrRbfKernel : uint8_t {
    Gaussian = 0,
    Linear = 1,
};

enum class _RrRbfPoseType : uint8_t {
    Whole = 0,
    Swing = 1,
    Twist = 2,
};

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

// A pose interpolator reconstituted from its solved wire table.
class _RrRbfSolver
{
public:
    _RrRbfSolver() = default;

    _RrRbfSolver(const std::vector<RrVec3d> &poses,
                 const std::vector<RrVec3d> &translations,
                 const std::vector<_RrRbfPoseType> &poseTypes,
                 const RrVec3d &twistAxis, _RrRbfKernel kernel,
                 double radius, double translationRadius,
                 double regularization, bool normalize,
                 bool enableRotation, bool enableTranslation)
        : _poses(poses),
          _translations(translations),
          _poseTypes(poseTypes),
          _twistAxis(twistAxis),
          _kernel(kernel),
          _regularization(regularization),
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

    void SetSolvedTable(const std::vector<double> &radii,
                        const std::vector<double> &translationRadii,
                        const std::vector<std::vector<double>> &weights)
    {
        _radii = radii;
        _translationRadii = translationRadii;
        _weights = weights;
        _regularizedSingular = false;
    }

    void Kernels(const RrVec3d &euler, const RrVec3d *translation,
                 std::vector<double> *out) const
    {
        const RrQuatd here = _RrRbfQuaternionFromEuler(euler);
        out->resize(_poses.size());
        for (size_t index = 0; index < _poses.size(); ++index) {
            (*out)[index] = _Kernel(Ratio(here, index, translation));
        }
    }

    void Normalize(std::vector<double> *weights) const
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

    void Evaluate(const RrVec3d &euler, const RrVec3d *translation,
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

    double Distance(const RrQuatd &quaternion, size_t index) const
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

    double TranslationDistance(const RrVec3d *translation,
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

    double Ratio(const RrQuatd &quaternion, size_t index,
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

private:
    double _Kernel(double ratio) const
    {
        return _RrRbfEvalKernel(_kernel, ratio, 1.0);
    }

    double _Width(size_t index) const
    {
        if (!_radii.empty() && index < _radii.size()) {
            return _radii[index];
        }
        return _radius;
    }

    double _TranslationWidth(size_t index) const
    {
        if (!_translationRadii.empty() &&
            index < _translationRadii.size()) {
            return _translationRadii[index];
        }
        return _translationRadius;
    }

    double _MeasureRadius() const
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

    double _MeasureTranslationRadius() const
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

    std::vector<RrVec3d> _poses;
    std::vector<RrVec3d> _translations;
    std::vector<std::array<RrQuatd, 3>> _parts;
    std::vector<_RrRbfPoseType> _poseTypes;
    RrVec3d _twistAxis{0.0, 1.0, 0.0};
    _RrRbfKernel _kernel = _RrRbfKernel::Gaussian;
    double _radius = 0.0;
    double _translationRadius = 0.0;
    std::vector<double> _radii;
    std::vector<double> _translationRadii;
    double _regularization = 0.0;
    bool _normalize = true;
    bool _enableRotation = true;
    bool _enableTranslation = false;
    bool _regularizedSingular = false;
    std::vector<std::vector<double>> _weights;
};

// One FkChain element's per-run inputs, mirroring RigExecFkChainElement.
struct RrPoseFkElement {
    std::array<RrVec3d, 4> restPoints;
    std::array<RrVec3d, 4> posePoints;
    bool hasOutRest = false;
    std::array<RrVec3d, 4> outRestPoints;
    int parentIndex = -1;
};

// RigExecTwoBoneIkParams, field for field.
struct RrPoseTwoBoneIkParams {
    double upperLength = 1;
    double lowerLength = 1;
    double stretch = 1;
    double softness = 0;
    double preferredBendRadians = 0;
};

// RigExecSplineIkRest, field for field.
struct RrPoseSplineIkRest {
    std::array<RrVec3d, 4> cvs;
    RrPointFrame rootControl;
    RrPointFrame midControl;
    RrPointFrame endControl;
    std::vector<RrPointFrame> joints;
    std::vector<double> segmentLengths;
    double restArcLength = 0;
    std::vector<double> volumeWeights;
};

// One solver's mutable rest description: what RefreshSolverRests and the
// Solve step rewrite every run. The wire solver is the epoch half.
struct RrPoseSolverState {
    std::vector<std::array<RrVec3d, 4>> jointRests;
    std::vector<std::array<RrVec3d, 4>> controlRests;
    std::vector<RrPoseFkElement> elements;
    std::array<std::array<RrVec3d, 4>, 3> ikRests;
    RrPoseTwoBoneIkParams ikParams;
    double upperLengthBase = 0;
    double lowerLengthBase = 0;
    RrPoseSplineIkRest splineRest;
    std::vector<std::array<RrVec3d, 4>> splineJointRests;
    std::array<RrVec3d, 4> twistStartRest;
    std::array<RrVec3d, 4> twistEndRest;
    std::vector<double> twistWeights;
    std::vector<RrVec3f> ribbonRestPoints;
};

struct RrPoseScratch {
    // The composed ladder and its move-comparison lasts, mirroring the
    // baked program's restM/restPts/restFrames/selfD/parentDinv/rotOrder/
    // posedAuthored(M)/restRoundTrip/defaultRoundTrip tables.
    std::vector<RrMat4d> restM, selfD, parentDinv, posedAuthoredM;
    std::vector<RrMat4d> restRoundTrip, defaultRoundTrip;
    std::vector<std::array<RrVec3d, 4>> restPts;
    std::vector<RrPointFrame> restFrames;
    std::vector<uint32_t> rotOrder;
    std::vector<char> posedAuthored;
    std::vector<char> noScaleAvars;
    std::vector<RrMat4d> lastRestM, lastSelfD, lastParentDinv;
    std::vector<RrMat4d> lastPosedAuthoredM;
    std::vector<char> lastPosedAuthored;
    std::vector<uint32_t> lastRotOrder;
    bool ladderRecomputed = false;
    bool ladderDisturbed = false;
    // Interpolator enables, read by the prologue so the step reads no
    // input table.
    std::vector<char> interpEnabled;
    // The reconstituted RBF solvers plus one step's own scratch.
    std::vector<_RrRbfSolver> interpSolvers;
    std::vector<std::vector<double>> interpScratch;
    // Solver live rests and per-run elements.
    std::vector<RrPoseSolverState> solvers;
    // Commit record flags, written by the head and read by CommitApply.
    std::vector<char> recordAfter, recordEveryTarget;
    // Constraint envelope scratch, one step's own storage.
    std::vector<std::vector<float>> weightScratch;
    std::vector<std::string> weightError;
    // Weight-object path id -> index into the geometry weight objects.
    std::unordered_map<uint32_t, size_t> weightIndex;
    // The captured constraint envelopes, for the resolve arm the
    // weight packets cannot serve (masked weights, failed capture).
    std::vector<float> constraintWeights;
    std::vector<char> constraintHaveWeight;
    // Geometry-domain constraint deltas. Conceptually framework-visible
    // (the Matrix revision reads them), but RrStore has no home for
    // them, so they live here until the framework grows one.
    std::vector<RrMat4d> deltaValues;
    std::vector<char> deltaPresent;
};

namespace {

RrPoseScratch *
_RrScratch(RrProgram *program)
{
    return static_cast<RrPoseScratch *>(program->pose.get());
}

// RigExecBakedComposeLadder (bakedPose.cpp), over the scratch tables.
// Reads through ReadLadder, the runtime form of RigExecBakedRead.
void
_RrComposeLadder(RrProgram *program, bool trackMoves)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const size_t slots = meta.paths.size();
    const RrMat4d identity = _RrIdentity();
    if (trackMoves) {
        store.ladderMovedSlots.clear();
    }
    for (size_t i = 0; i < slots; ++i) {
        if (i >= meta.slotKind.size() ||
            meta.slotKind[i] != RigExecWireSlotKind::FirstFramePose) {
            continue;
        }
        const RrMat4d posed =
            program->ReadLadder(i, RrLadderPosedSpace).matrix;
        scratch->posedAuthored[i] = posed != identity ? 1 : 0;
        scratch->posedAuthoredM[i] = posed;

        const RrInputValue restAvar0 =
            program->ReadLadder(i, RrLadderRestAvar0 + 0);
        const RrInputValue restAvar1 =
            program->ReadLadder(i, RrLadderRestAvar0 + 1);
        const RrInputValue restAvar2 =
            program->ReadLadder(i, RrLadderRestAvar0 + 2);
        const RrInputValue restAvar3 =
            program->ReadLadder(i, RrLadderRestAvar0 + 3);
        const RrInputValue restAvar4 =
            program->ReadLadder(i, RrLadderRestAvar0 + 4);
        const RrInputValue restAvar5 =
            program->ReadLadder(i, RrLadderRestAvar0 + 5);
        RrMat4d rest =
            _RrComposeAvars(restAvar0.f64, restAvar1.f64, restAvar2.f64,
                            1, 1, 1, restAvar3.f64, restAvar4.f64,
                            restAvar5.f64, 0, "XYZ") *
            program->ReadLadder(i, RrLadderRestSpace).matrix;
        rest.Orthonormalize(false);
        const int parent =
            i < meta.parent.size() ? meta.parent[i] : -1;
        const RrMat4d parentRest =
            parent >= 0 ? scratch->restRoundTrip[size_t(parent)]
                        : identity;
        scratch->restM[i] = rest * parentRest;
        scratch->restFrames[i] = RrFrameFromMatrix(scratch->restM[i]);
        scratch->restPts[i] = scratch->restFrames[i].points;
        scratch->restRoundTrip[i] = RrRoundTrip(scratch->restM[i]);

        const RrMat4d authoredDefault =
            program->ReadLadder(i, RrLadderDefaultSpace).matrix;
        const RrMat4d parentDefault =
            parent >= 0 ? scratch->defaultRoundTrip[size_t(parent)]
                        : identity;
        if (authoredDefault != identity) {
            scratch->selfD[i] = authoredDefault;
        } else {
            const RrInputValue defAvar0 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 0);
            const RrInputValue defAvar1 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 1);
            const RrInputValue defAvar2 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 2);
            const RrInputValue defAvar3 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 3);
            const RrInputValue defAvar4 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 4);
            const RrInputValue defAvar5 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 5);
            const RrMat4d offset = _RrComposeAvars(
                defAvar0.f64, defAvar1.f64, defAvar2.f64, 1, 1, 1,
                defAvar3.f64, defAvar4.f64, defAvar5.f64, 0, "XYZ");
            scratch->selfD[i] = offset * scratch->restRoundTrip[i] *
                                parentRest.GetInverse() * parentDefault;
        }
        scratch->defaultRoundTrip[i] = RrRoundTrip(scratch->selfD[i]);
        scratch->parentDinv[i] = parentDefault.GetInverse();
        const uint32_t orderId =
            program->ReadLadder(i, RrLadderRotationOrder).token;
        scratch->rotOrder[i] = orderId;

        if (i < store.ladders.size()) {
            RrLadderLive &live = store.ladders[i];
            live.restSpace =
                program->ReadLadder(i, RrLadderRestSpace).matrix;
            live.defaultSpace =
                program->ReadLadder(i, RrLadderDefaultSpace).matrix;
            live.posedSpace = posed;
            live.restAvars[0] = restAvar0.f64;
            live.restAvars[1] = restAvar1.f64;
            live.restAvars[2] = restAvar2.f64;
            live.restAvars[3] = restAvar3.f64;
            live.restAvars[4] = restAvar4.f64;
            live.restAvars[5] = restAvar5.f64;
            live.defaultAvars[0] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 0).f64;
            live.defaultAvars[1] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 1).f64;
            live.defaultAvars[2] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 2).f64;
            live.defaultAvars[3] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 3).f64;
            live.defaultAvars[4] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 4).f64;
            live.defaultAvars[5] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 5).f64;
            live.rotationOrder = orderId;
        }

        if (!trackMoves) {
            continue;
        }
        if (scratch->restM[i] != scratch->lastRestM[i] ||
            scratch->selfD[i] != scratch->lastSelfD[i] ||
            scratch->parentDinv[i] != scratch->lastParentDinv[i] ||
            scratch->posedAuthored[i] != scratch->lastPosedAuthored[i] ||
            scratch->posedAuthoredM[i] != scratch->lastPosedAuthoredM[i] ||
            scratch->rotOrder[i] != scratch->lastRotOrder[i]) {
            store.ladderMovedSlots.push_back(int(i));
            scratch->lastRestM[i] = scratch->restM[i];
            scratch->lastSelfD[i] = scratch->selfD[i];
            scratch->lastParentDinv[i] = scratch->parentDinv[i];
            scratch->lastPosedAuthored[i] = scratch->posedAuthored[i];
            scratch->lastPosedAuthoredM[i] = scratch->posedAuthoredM[i];
            scratch->lastRotOrder[i] = scratch->rotOrder[i];
        }
    }
}

}  // namespace

bool
RrPoseSizeScratch(RrProgram *program, std::string *error)
{
    if (!program || !program->slotMeta || !program->constants ||
        !program->poses || !program->geometry) {
        if (error) {
            *error = "pose scratch needs the program tables";
        }
        return false;
    }
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const RigExecWireConstants &constants = *program->constants;
    const RigExecWireDomainPose &poses = *program->poses;
    const size_t slots = meta.paths.size();
    if (constants.restM.size() != slots ||
        constants.restPts.size() != slots ||
        constants.restFrames.size() != slots ||
        constants.selfD.size() != slots ||
        constants.parentDinv.size() != slots ||
        constants.rotOrder.size() != slots ||
        constants.restRoundTrip.size() != slots ||
        constants.defaultRoundTrip.size() != slots ||
        constants.posedAuthored.size() != slots ||
        constants.posedAuthoredM.size() != slots ||
        constants.noScaleAvars.size() != slots) {
        if (error) {
            *error = "pose constants do not cover the slots";
        }
        return false;
    }
    RrPoseScratch *scratch = new RrPoseScratch();
    const RrMat4d identity = _RrIdentity();
    scratch->restM.assign(slots, identity);
    scratch->selfD.assign(slots, identity);
    scratch->parentDinv.assign(slots, identity);
    scratch->posedAuthoredM.assign(slots, identity);
    scratch->restRoundTrip.assign(slots, identity);
    scratch->defaultRoundTrip.assign(slots, identity);
    scratch->restPts.assign(slots, RrIdentityLandmarks());
    scratch->restFrames.assign(slots, RrPointFrame());
    scratch->rotOrder.assign(slots, 0);
    scratch->posedAuthored.assign(slots, 0);
    scratch->noScaleAvars.assign(slots, 0);
    for (size_t i = 0; i < slots; ++i) {
        scratch->restM[i] = _RrWireMatrix(constants.restM[i]);
        scratch->selfD[i] = _RrWireMatrix(constants.selfD[i]);
        scratch->parentDinv[i] = _RrWireMatrix(constants.parentDinv[i]);
        scratch->posedAuthoredM[i] =
            _RrWireMatrix(constants.posedAuthoredM[i]);
        scratch->restRoundTrip[i] =
            _RrWireMatrix(constants.restRoundTrip[i]);
        scratch->defaultRoundTrip[i] =
            _RrWireMatrix(constants.defaultRoundTrip[i]);
        scratch->restPts[i] = _RrWireLandmarks(constants.restPts[i]);
        scratch->restFrames[i] = RrWireToFrame(constants.restFrames[i]);
        scratch->rotOrder[i] = constants.rotOrder[i];
        scratch->posedAuthored[i] =
            constants.posedAuthored[i] ? 1 : 0;
        scratch->noScaleAvars[i] =
            constants.noScaleAvars[i] ? 1 : 0;
    }
    scratch->lastRestM = scratch->restM;
    scratch->lastSelfD = scratch->selfD;
    scratch->lastParentDinv = scratch->parentDinv;
    scratch->lastPosedAuthoredM = scratch->posedAuthoredM;
    scratch->lastPosedAuthored = scratch->posedAuthored;
    scratch->lastRotOrder = scratch->rotOrder;
    scratch->interpEnabled.assign(poses.poseInterpolators.size(), 0);
    scratch->interpSolvers.assign(poses.poseInterpolators.size(),
                                  _RrRbfSolver());
    scratch->interpScratch.assign(poses.poseInterpolators.size(), {});
    for (size_t i = 0; i < poses.poseInterpolators.size(); ++i) {
        const RigExecWireRbf &wire =
            poses.poseInterpolators[i].solver;
        std::vector<RrVec3d> rbfPoses;
        for (const RigExecWireVec3d &p : wire.poses) {
            rbfPoses.push_back(RrVec3d(p[0], p[1], p[2]));
        }
        std::vector<RrVec3d> rbfTranslations;
        for (const RigExecWireVec3d &p : wire.translations) {
            rbfTranslations.push_back(RrVec3d(p[0], p[1], p[2]));
        }
        std::vector<_RrRbfPoseType> rbfPoseTypes;
        for (uint8_t type : wire.poseTypes) {
            rbfPoseTypes.push_back(_RrRbfPoseType(type));
        }
        scratch->interpSolvers[i] = _RrRbfSolver(
            rbfPoses, rbfTranslations, rbfPoseTypes,
            RrVec3d(wire.twistAxis[0], wire.twistAxis[1],
                    wire.twistAxis[2]),
            _RrRbfKernel(wire.kernel), wire.radius,
            wire.translationRadius, wire.regularization,
            wire.normalize, wire.enableRotation,
            wire.enableTranslation);
        scratch->interpSolvers[i].SetSolvedTable(
            wire.radii, wire.translationRadii, wire.weights);
    }
    scratch->solvers.resize(poses.solvers.size());
    for (size_t s = 0; s < poses.solvers.size(); ++s) {
        const RigExecWireSolver &wire = poses.solvers[s];
        RrPoseSolverState &state = scratch->solvers[s];
        for (const auto &rest : wire.jointRests) {
            state.jointRests.push_back(_RrWireLandmarks(rest));
        }
        for (const auto &rest : wire.controlRests) {
            state.controlRests.push_back(_RrWireLandmarks(rest));
        }
        state.elements.resize(wire.controls.size());
        for (size_t k = 0; k < wire.controls.size(); ++k) {
            if (k < state.controlRests.size()) {
                state.elements[k].restPoints = state.controlRests[k];
            } else {
                state.elements[k].restPoints = RrIdentityLandmarks();
            }
            state.elements[k].posePoints = RrIdentityLandmarks();
            state.elements[k].outRestPoints = RrIdentityLandmarks();
        }
        for (size_t k = 0; k < 3; ++k) {
            state.ikRests[k] = _RrWireLandmarks(wire.ikRests[k]);
        }
        state.ikParams.upperLength = wire.ikParams.upperLength;
        state.ikParams.lowerLength = wire.ikParams.lowerLength;
        state.ikParams.stretch = wire.ikParams.stretch;
        state.ikParams.softness = wire.ikParams.softness;
        state.ikParams.preferredBendRadians =
            wire.ikParams.preferredBendRadians;
        state.upperLengthBase = wire.upperLengthBase;
        state.lowerLengthBase = wire.lowerLengthBase;
        state.splineRest.cvs = _RrWireLandmarks(wire.splineRest.cvs);
        state.splineRest.rootControl =
            RrWireToFrame(wire.splineRest.rootControl);
        state.splineRest.midControl =
            RrWireToFrame(wire.splineRest.midControl);
        state.splineRest.endControl =
            RrWireToFrame(wire.splineRest.endControl);
        for (const RigExecWireFrame &joint : wire.splineRest.joints) {
            state.splineRest.joints.push_back(RrWireToFrame(joint));
        }
        state.splineRest.segmentLengths = wire.splineRest.segmentLengths;
        state.splineRest.restArcLength = wire.splineRest.restArcLength;
        state.splineRest.volumeWeights = wire.splineRest.volumeWeights;
        for (const auto &rest : wire.splineJointRests) {
            state.splineJointRests.push_back(_RrWireLandmarks(rest));
        }
        state.twistStartRest = _RrWireLandmarks(wire.twistStartRest);
        state.twistEndRest = _RrWireLandmarks(wire.twistEndRest);
        state.twistWeights = wire.twistWeights;
        for (const RigExecWireVec3f &p : wire.ribbonRestPoints) {
            state.ribbonRestPoints.push_back(
                RrVec3f(p[0], p[1], p[2]));
        }
    }
    scratch->recordAfter.assign(poses.commits.size(), 1);
    scratch->recordEveryTarget.assign(poses.commits.size(), 1);
    scratch->constraintWeights.assign(poses.constraints.size(), 0.0f);
    scratch->constraintHaveWeight.assign(poses.constraints.size(), 0);
    scratch->weightScratch.assign(poses.constraints.size(), {});
    scratch->weightError.assign(poses.constraints.size(), {});
    for (size_t o = 0; o < program->geometry->weightObjects.size();
         ++o) {
        scratch->weightIndex.emplace(
            program->geometry->weightObjects[o].path, o);
    }
    scratch->deltaValues.assign(
        program->geometry->deltaBasePaths.size(), identity);
    scratch->deltaPresent.assign(
        program->geometry->deltaBasePaths.size(), 0);
    // Per-commit IK chains, sized from the wire commit the way Build
    // sizes them: the IK write-back and the atomic check walk these.
    RrStore &store = program->store;
    for (size_t c = 0; c < poses.commits.size() &&
         c < store.commits.size(); ++c) {
        const size_t targets =
            poses.commits[c].targetReads.size();
        store.commits[c].ikChain.assign(targets, RrPointFrame());
        store.commits[c].ikPrepared.assign(targets, RrPointFrame());
        store.commits[c].ikRest.assign(targets, RrPointFrame());
        store.commits[c].ikSolved.clear();
    }
    program->pose = std::shared_ptr<void>(scratch);
    return true;
}

bool
RrProloguePose(RrProgram *program,
               const RigExecWireFrameInputs &record,
               std::vector<std::string> *poseDiagnostics,
               std::string *error)
{
    (void)poseDiagnostics;
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    // Every record table is validated before anything is mutated.
    if (record.solverRibbonPoints.size() != poses.solvers.size()) {
        if (error) {
            *error = "frame record ribbon tables do not match the solvers";
        }
        return false;
    }
    if (record.xformBase.size() != meta.xformSlots.size()) {
        if (error) {
            *error = "frame record xform tables do not match the slots";
        }
        return false;
    }
    if (record.nativeFrames.size() != poses.nativeSources.size()) {
        if (error) {
            *error =
                "frame record native frames do not match the sources";
        }
        return false;
    }
    if (record.deltaBaseMatrix.size() != geometry.deltaBasePaths.size() ||
        record.deltaBaseOk.size() != geometry.deltaBasePaths.size()) {
        if (error) {
            *error = "frame record delta tables do not match the paths";
        }
        return false;
    }
    if (record.arrayWeights.size() != poses.constraintArrays.size() ||
        record.arrayTranslationOffsets.size() !=
            poses.constraintArrays.size() ||
        record.arrayRotationOffsets.size() !=
            poses.constraintArrays.size() ||
        record.arrayOk.size() != poses.constraintArrays.size() ||
        record.arrayPoleWeights.size() !=
            poses.constraintArrays.size() ||
        record.arrayPoleOk.size() != poses.constraintArrays.size()) {
        if (error) {
            *error =
                "frame record array tables do not match the constraints";
        }
        return false;
    }
    if (record.constraintWeights.size() != poses.constraints.size() ||
        record.constraintHaveWeight.size() != poses.constraints.size()) {
        if (error) {
            *error =
                "frame record envelope tables do not match the constraints";
        }
        return false;
    }
    // The avar table: every uid the capture routed to an avar binding
    // carries this frame's value in its holder.
    for (size_t uid = 0; uid < store.inputHolders.size(); ++uid) {
        if (uid >= program->avarUidTarget.size()) {
            break;
        }
        const int32_t target = program->avarUidTarget[uid];
        if (target >= 0 && size_t(target) < store.avars.size()) {
            store.avars[size_t(target)] =
                store.inputHolders[uid].f64;
        }
    }
    // The provider ladder, before the avars that compose against it.
    // No overrides yet, so the dragged arm stays out: the recompute
    // gate is the varying flag plus the disturbed one-more-pass.
    scratch->ladderRecomputed =
        poses.ladderVarying || scratch->ladderDisturbed;
    if (scratch->ladderRecomputed) {
        _RrComposeLadder(program, /* trackMoves = */ true);
        scratch->ladderDisturbed = false;
    } else if (!store.ladderMovedSlots.empty()) {
        store.ladderMovedSlots.clear();
    }
    // The pose interpolators' enables, read here so their step reads
    // no input table.
    for (size_t i = 0; i < poses.poseInterpolators.size() &&
         i < scratch->interpEnabled.size(); ++i) {
        scratch->interpEnabled[i] =
            program->ReadInterp(i).boolean ? 1 : 0;
    }
    // Live ribbon driver points, swapped against last run's by value.
    for (size_t s = 0; s < poses.solvers.size(); ++s) {
        if (!store.ribbonVarying[s]) {
            continue;
        }
        store.ribbonLast[s].swap(store.ribbonPoints[s]);
        std::vector<RrVec3f> &points = store.ribbonPoints[s];
        points.clear();
        points.reserve(record.solverRibbonPoints[s].size());
        for (const RigExecWireVec3f &p : record.solverRibbonPoints[s]) {
            points.push_back(RrVec3f(p[0], p[1], p[2]));
        }
        store.ribbonDirty[s] = points != store.ribbonLast[s] ? 1 : 0;
    }
    // Xform-derived slots, seeded from the stage through the record.
    for (size_t k = 0; k < meta.xformSlots.size(); ++k) {
        const int slot = meta.xformSlots[k];
        if (slot < 0 || size_t(slot) >= store.fin.size() ||
            size_t(slot) >= store.base.size() ||
            k >= store.xformBase.size()) {
            if (error) {
                *error = "frame record xform tables do not match the slots";
            }
            return false;
        }
        const RrMat4d matrix = _RrWireMatrix(record.xformBase[k]);
        store.xformBase[k] = matrix;
        const RrPointFrame frame = RrFrameFromMatrix(matrix);
        store.base[size_t(slot)] = frame;
        store.fin[size_t(slot)] = frame;
    }
    // The target transform each geometry-domain constraint measures
    // its delta against.
    for (size_t k = 0; k < geometry.deltaBasePaths.size(); ++k) {
        store.deltaBaseOk[k] = record.deltaBaseOk[k] ? 1 : 0;
        store.deltaBaseMatrix[k] = _RrWireMatrix(record.deltaBaseMatrix[k]);
    }
    // The plain Xformables a constraint reads as a source. The record
    // carries frames only, no ok array, so ok is the frame's own
    // usability: a store read that failed has no frame to seed.
    for (size_t k = 0; k < poses.nativeSources.size(); ++k) {
        const RrPointFrame frame = RrWireToFrame(record.nativeFrames[k]);
        store.nativeFrames[k] = frame;
        store.nativeFrameOk[k] = RrFrameUsable(frame) ? 1 : 0;
    }
    // The captured envelopes, for the resolve arm below. Sizes were
    // validated against the constraints at the head.
    for (size_t k = 0; k < poses.constraints.size(); ++k) {
        scratch->constraintHaveWeight[k] =
            k < record.constraintHaveWeight.size() &&
                    record.constraintHaveWeight[k]
                ? 1
                : 0;
        scratch->constraintWeights[k] =
            k < record.constraintWeights.size()
                ? record.constraintWeights[k]
                : 0.0f;
    }
    // A constraint's own authored tables, as the prologue read them at
    // this frame's time. Capture folded every read semantic into the
    // record; the cardinality lines it cannot carry stay empty.
    for (size_t k = 0; k < poses.constraintArrays.size(); ++k) {
        RrConstraintArraysLive &live = store.arrays[k];
        live.weights = record.arrayWeights[k];
        live.translationOffsets.clear();
        live.translationOffsets.reserve(
            record.arrayTranslationOffsets[k].size());
        for (const RigExecWireVec3d &v :
             record.arrayTranslationOffsets[k]) {
            live.translationOffsets.push_back(
                RrVec3d(v[0], v[1], v[2]));
        }
        live.rotationOffsets.clear();
        live.rotationOffsets.reserve(
            record.arrayRotationOffsets[k].size());
        for (const RigExecWireVec3d &v : record.arrayRotationOffsets[k]) {
            live.rotationOffsets.push_back(
                RrVec3d(v[0], v[1], v[2]));
        }
        live.ok = record.arrayOk[k] != 0;
        live.poleWeights = record.arrayPoleWeights[k];
        live.poleOk = record.arrayPoleOk[k] != 0;
    }
    return true;
}

namespace {

// Propagation outcomes, in RigExecBakedPropagateOutcome order.
enum _RrPropagateOutcome : uint8_t {
    _RrStaged = 0,
    _RrSkipped = 1,
    _RrNoCandidate = 2,
    _RrUnusableDescendant = 3,
    _RrSingularDelta = 4,
    _RrInvalidResult = 5,
};

// The split constants of the commit walk (bakedPose.cpp).
enum : size_t {
    _RrPropagateChunkSize = 64,
};

std::string
_RrKindName(RigExecWireStepKind kind)
{
    switch (kind) {
    case RigExecWireStepKind::ComposeSubtree:
        return "ComposeSubtree";
    case RigExecWireStepKind::Solve:
        return "Solve";
    case RigExecWireStepKind::SolverCommit:
        return "SolverCommit";
    case RigExecWireStepKind::Constraint:
        return "Constraint";
    case RigExecWireStepKind::CommitDelta:
        return "CommitDelta";
    case RigExecWireStepKind::PropagateChunk:
        return "PropagateChunk";
    case RigExecWireStepKind::CommitApply:
        return "CommitApply";
    case RigExecWireStepKind::ProviderMatrix:
        return "ProviderMatrix";
    case RigExecWireStepKind::SnapshotFinals:
        return "SnapshotFinals";
    case RigExecWireStepKind::PoseInterpolator:
        return "PoseInterpolator";
    default:
        return "Unknown";
    }
}

std::string
_RrStepHead(const RrProgram *program, size_t step)
{
    const RigExecWireStep &wire = (*program->steps)[step];
    return "pose step " + _RrKindName(wire.kind) + " object " +
           std::to_string(wire.object) + " part " +
           std::to_string(wire.part);
}

// The hierarchy delta each candidate of the commit carries, once
// (ComputeCommitDeltas, bakedPose.cpp).
bool
_RrComputeCommitDeltas(RrProgram *program, size_t step,
                       const RigExecWireCommit &commit,
                       RrCommitScratch *scratch, std::string *error)
{
    RrStore &store = program->store;
    for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
        if (pos >= scratch->present.size() ||
            pos >= scratch->frames.size() ||
            pos >= scratch->deltas.size() ||
            pos >= scratch->deltaOk.size() ||
            pos >= commit.slotReads.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no candidate slot";
            }
            return false;
        }
        if (!scratch->present[pos]) {
            scratch->deltaOk[pos] = 0;
            continue;
        }
        scratch->deltas[pos] = _RrIdentity();
        const uint32_t read = commit.slotReads[pos];
        if (size_t(read) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        scratch->deltaOk[pos] =
            RrPointsToMatrix(store.fin[size_t(read)].points,
                             scratch->frames[pos].points,
                             &scratch->deltas[pos])
                ? 1
                : 2;
    }
    return true;
}

// Stages descendants [begin, end) of the commit (StageCommitPairs,
// bakedPose.cpp): each pair decided exactly where the walk decides it.
bool
_RrStageCommitPairs(RrProgram *program, size_t step,
                    const RigExecWireCommit &commit,
                    RrCommitScratch *scratch, size_t begin, size_t end,
                    std::string *error)
{
    RrStore &store = program->store;
    for (size_t k = begin;
         k < end && k < commit.propagate.size(); ++k) {
        if (k >= commit.closestPos.size() ||
            k >= commit.descendantReads.size() ||
            k >= commit.closestReads.size() ||
            k >= scratch->staged.size() ||
            k >= scratch->outcome.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no propagation pair";
            }
            return false;
        }
        const int pos = commit.closestPos[k];
        if (pos < 0 || size_t(pos) >= scratch->present.size() ||
            !scratch->present[size_t(pos)]) {
            scratch->outcome[k] = _RrNoCandidate;
            continue;
        }
        const uint32_t descendantRead = commit.descendantReads[k];
        const uint32_t closestRead = commit.closestReads[k];
        if (size_t(descendantRead) >= store.fin.size() ||
            size_t(closestRead) >= store.fin.size() ||
            size_t(pos) >= scratch->frames.size() ||
            size_t(pos) >= scratch->deltaOk.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        const RrPointFrame &current =
            store.fin[size_t(descendantRead)];
        const RrPointFrame &before = store.fin[size_t(closestRead)];
        if (commit.solverOutput &&
            (!RrFrameUsable(current) || !RrFrameUsable(before) ||
             !RrFrameUsable(scratch->frames[size_t(pos)]))) {
            scratch->outcome[k] = _RrSkipped;
            continue;
        }
        if (!RrFrameUsable(current)) {
            scratch->outcome[k] = _RrUnusableDescendant;
            continue;
        }
        if (scratch->deltaOk[size_t(pos)] != 1) {
            scratch->outcome[k] = _RrSingularDelta;
            continue;
        }
        const RrPointFrame frame = RrMatrixToPoints(
            current.points, scratch->deltas[size_t(pos)]);
        if (!RrFrameUsable(frame)) {
            scratch->outcome[k] = _RrInvalidResult;
            continue;
        }
        scratch->staged[k] = frame;
        scratch->outcome[k] = _RrStaged;
    }
    return true;
}

// The phased-read store's pose-half record (RecordFrame, bakedPose.cpp).
bool
_RrRecordFrame(RrProgram *program, size_t step,
               const RigExecWireConstraint &constraint,
               const RigExecWireCommit &commit, RrStepOutput *output,
               std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    if (!constraint.snapshotAfter) {
        return true;
    }
    const size_t commitIndex = size_t((*program->steps)[step].object);
    const bool everyTarget =
        commitIndex < scratch->recordEveryTarget.size() &&
        scratch->recordEveryTarget[commitIndex];
    const size_t count =
        everyTarget ? constraint.targetSlots.size()
                    : std::min<size_t>(1, constraint.targetSlots.size());
    for (size_t k = 0; k < count; ++k) {
        if (k >= constraint.snapshotTargets.size() ||
            !constraint.snapshotTargets[k]) {
            continue;
        }
        if (k >= constraint.targetSlots.size() ||
            k >= commit.targetReads.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no record target";
            }
            return false;
        }
        const int32_t target = constraint.targetSlots[k];
        const size_t slot = size_t(target);
        const auto found = std::lower_bound(commit.slots.begin(),
                                            commit.slots.end(), target);
        const uint32_t version =
            found != commit.slots.end() && *found == target
                ? (size_t(found - commit.slots.begin()) <
                           commit.slotWrites.size()
                       ? commit.slotWrites[size_t(found -
                                                  commit.slots.begin())]
                       : commit.targetReads[k])
                : commit.targetReads[k];
        if (size_t(version) >= store.fin.size() ||
            slot >= scratch->restFrames.size() ||
            slot >= program->slotMeta->paths.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no record target";
            }
            return false;
        }
        const RrPointFrame &frame = store.fin[size_t(version)];
        if (!frame.IsValid()) {
            continue;
        }
        const RrPointFrame &rest = scratch->restFrames[slot];
        const std::array<RrVec3d, 4> landmarks =
            rest.IsValid() ? rest.points : RrIdentityLandmarks();
        RrMat4d matrix = _RrIdentity();
        if (RrPointsToMatrix(landmarks, frame.points, &matrix)) {
            RrSnapshotValue value;
            value.tag = RrSnapshotValue::Tag::Matrix;
            value.matrix = matrix;
            output->snapshots.Record(
                program->slotMeta->paths[slot], constraint.path, value);
        }
    }
    return true;
}

// Decides the commit and writes it back, or says why it passed through
// (FinishCommit, bakedPose.cpp).
bool
_RrFinishCommit(RrProgram *program, size_t step, size_t commitIndex,
                std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireCommit &commit =
        program->poses->commits[commitIndex];
    RrCommitScratch &live = store.commits[commitIndex];
    RrStepOutput &output = store.stepOutputs[step];
    const RigExecWireConstraint *constraint = nullptr;
    if (!commit.solverOutput) {
        const size_t walk = size_t((*program->steps)[step].object);
        if (walk >= program->poses->walkSteps.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no walk step";
            }
            return false;
        }
        const int index = program->poses->walkSteps[walk].index;
        if (index < 0 ||
            size_t(index) >= program->poses->constraints.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no constraint";
            }
            return false;
        }
        constraint = &program->poses->constraints[size_t(index)];
    }
    const auto carry = [&](size_t pos, size_t k, bool candidates,
                           bool descendants, std::string *fail) {
        if (candidates) {
            if (pos >= commit.slotWrites.size() ||
                pos >= commit.slotCarry.size() ||
                size_t(commit.slotWrites[pos]) >= store.fin.size() ||
                size_t(commit.slotCarry[pos]) >= store.fin.size()) {
                if (fail) {
                    *fail = _RrStepHead(program, step) +
                            " names no fin version";
                }
                return false;
            }
            store.fin[size_t(commit.slotWrites[pos])] =
                store.fin[size_t(commit.slotCarry[pos])];
            if (commit.solverOutput) {
                if (pos >= commit.slotBaseWrites.size() ||
                    pos >= commit.slotBaseCarry.size() ||
                    size_t(commit.slotBaseWrites[pos]) >=
                        store.base.size() ||
                    size_t(commit.slotBaseCarry[pos]) >=
                        store.base.size()) {
                    if (fail) {
                        *fail = _RrStepHead(program, step) +
                                " names no base version";
                    }
                    return false;
                }
                store.base[size_t(commit.slotBaseWrites[pos])] =
                    store.base[size_t(commit.slotBaseCarry[pos])];
            }
        }
        if (descendants) {
            if (k >= commit.descendantWrites.size() ||
                k >= commit.descendantCarry.size() ||
                size_t(commit.descendantWrites[k]) >=
                    store.fin.size() ||
                size_t(commit.descendantCarry[k]) >= store.fin.size()) {
                if (fail) {
                    *fail = _RrStepHead(program, step) +
                            " names no fin version";
                }
                return false;
            }
            store.fin[size_t(commit.descendantWrites[k])] =
                store.fin[size_t(commit.descendantCarry[k])];
            if (commit.solverOutput) {
                if (k >= commit.descendantBaseWrites.size() ||
                    k >= commit.descendantBaseCarry.size() ||
                    size_t(commit.descendantBaseWrites[k]) >=
                        store.base.size() ||
                    size_t(commit.descendantBaseCarry[k]) >=
                        store.base.size()) {
                    if (fail) {
                        *fail = _RrStepHead(program, step) +
                                " names no base version";
                    }
                    return false;
                }
                store.base[size_t(commit.descendantBaseWrites[k])] =
                    store.base[size_t(commit.descendantBaseCarry[k])];
            }
        }
        return true;
    };
    const auto carryEverything = [&](std::string *fail) {
        for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
            if (!carry(pos, 0, true, false, fail)) {
                return false;
            }
        }
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            if (!carry(0, k, false, true, fail)) {
                return false;
            }
        }
        return true;
    };
    const auto record = [&]() {
        if (constraint && commitIndex < scratch->recordAfter.size() &&
            scratch->recordAfter[commitIndex]) {
            return _RrRecordFrame(program, step, *constraint, commit,
                                  &output, error);
        }
        return true;
    };
    if (live.abandoned) {
        if (!carryEverything(error)) {
            return false;
        }
        return record();
    }
    const std::string mover = program->TextOrEmpty(commit.moverPath);
    for (size_t k = 0; k < commit.propagate.size(); ++k) {
        if (k >= live.outcome.size() ||
            k >= commit.propagate.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no propagation pair";
            }
            return false;
        }
        const _RrPropagateOutcome outcome =
            _RrPropagateOutcome(live.outcome[k]);
        if (outcome == _RrStaged || outcome == _RrSkipped) {
            continue;
        }
        switch (outcome) {
        case _RrNoCandidate:
            if (!carryEverything(error)) {
                return false;
            }
            output.bail = true;
            return true;
        case _RrUnusableDescendant:
            output.diagnostics.push_back(
                mover + " could not propagate its pose revision through " +
                program->TextOrEmpty(
                    program->slotMeta->paths[size_t(
                        commit.propagate[k].first)]) +
                "; constraint passed through");
            break;
        case _RrSingularDelta:
            output.diagnostics.push_back(
                mover + " produced a singular hierarchy delta; constraint "
                "passed through");
            break;
        default:
            output.diagnostics.push_back(
                mover + " produced an invalid descendant frame for " +
                program->TextOrEmpty(
                    program->slotMeta->paths[size_t(
                        commit.propagate[k].first)]) +
                "; constraint passed through");
            break;
        }
        if (!carryEverything(error)) {
            return false;
        }
        return record();
    }
    for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
        if (pos >= live.present.size() || pos >= live.frames.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no candidate slot";
            }
            return false;
        }
        if (!live.present[pos]) {
            if (!carry(pos, 0, true, false, error)) {
                return false;
            }
            continue;
        }
        if (pos >= commit.slotWrites.size() ||
            size_t(commit.slotWrites[pos]) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        store.fin[size_t(commit.slotWrites[pos])] = live.frames[pos];
        if (commit.solverOutput) {
            if (pos >= commit.slotBaseWrites.size() ||
                size_t(commit.slotBaseWrites[pos]) >=
                    store.base.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no base version";
                }
                return false;
            }
            store.base[size_t(commit.slotBaseWrites[pos])] =
                live.frames[pos];
        }
    }
    for (size_t k = 0; k < commit.propagate.size(); ++k) {
        if (k >= live.outcome.size() || k >= live.staged.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no propagation pair";
            }
            return false;
        }
        if (_RrPropagateOutcome(live.outcome[k]) != _RrStaged) {
            if (!carry(0, k, false, true, error)) {
                return false;
            }
            continue;
        }
        if (k >= commit.descendantWrites.size() ||
            size_t(commit.descendantWrites[k]) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        store.fin[size_t(commit.descendantWrites[k])] = live.staged[k];
        if (commit.solverOutput) {
            if (k >= commit.descendantBaseWrites.size() ||
                size_t(commit.descendantBaseWrites[k]) >=
                    store.base.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no base version";
                }
                return false;
            }
            store.base[size_t(commit.descendantBaseWrites[k])] =
                live.staged[k];
        }
    }
    return record();
}

}  // namespace

namespace _RrPoseSteps {

bool _RrRunSolveStep(RrProgram *program, size_t step,
                     std::string *error);
bool _RrRunSolverCommitStep(RrProgram *program, size_t step,
                            std::string *error);
bool _RrRunConstraintStep(RrProgram *program, size_t step,
                          std::string *error);

}  // namespace _RrPoseSteps

bool
RrRunPoseStep(RrProgram *program, size_t step, double time,
              std::string *error)
{
    (void)time;
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    if (step >= program->steps->size() ||
        step >= store.stepOutputs.size()) {
        if (error) {
            *error = "pose step " + std::to_string(step) +
                     " names no step";
        }
        return false;
    }
    const RigExecWireStep &wire = (*program->steps)[step];
    RrStepOutput &output = store.stepOutputs[step];
    (void)output;

    switch (wire.kind) {
    case RigExecWireStepKind::ComposeSubtree: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->composeGroups.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no compose group";
            }
            return false;
        }
        const RigExecWireComposeGroup &group =
            program->poses->composeGroups[size_t(wire.object)];
        const RigExecWireSlotMeta &meta = *program->slotMeta;
        for (int i = group.begin; i < group.end; ++i) {
            if (i < 0 || size_t(i) >= meta.slotKind.size() ||
                size_t(i) >= store.base.size() ||
                size_t(i) >= store.fin.size() ||
                size_t(i) >= store.posedM.size() ||
                size_t(i) >= scratch->restM.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no slot";
                }
                return false;
            }
            if (meta.slotKind[size_t(i)] !=
                RigExecWireSlotKind::FirstFramePose) {
                continue;
            }
            if (scratch->posedAuthored[size_t(i)]) {
                store.base[size_t(i)] = RrFrameFromMatrix(
                    scratch->posedAuthoredM[size_t(i)]);
            } else {
                if (size_t(i) * 11 + 11 > store.avars.size() ||
                    size_t(i) >= scratch->noScaleAvars.size() ||
                    size_t(i) >= scratch->rotOrder.size() ||
                    size_t(i) >= scratch->selfD.size() ||
                    size_t(i) >= scratch->parentDinv.size() ||
                    size_t(i) >= meta.parent.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no slot";
                    }
                    return false;
                }
                const double *a = &store.avars[size_t(i) * 11];
                const double units = a[10];
                const bool noScale =
                    scratch->noScaleAvars[size_t(i)] != 0;
                const RrMat4d avars = _RrComposeAvars(
                    a[0] * units, a[1] * units, a[2] * units,
                    noScale ? 1.0 : a[3], noScale ? 1.0 : a[4],
                    noScale ? 1.0 : a[5], a[6], a[7], a[8], a[9],
                    program->TextOrEmpty(
                        scratch->rotOrder[size_t(i)]));
                const int parent = meta.parent[size_t(i)];
                if (parent >= 0 &&
                    size_t(parent) >= store.posedM.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no slot";
                    }
                    return false;
                }
                const RrMat4d parentPosed =
                    parent >= 0 ? store.posedM[size_t(parent)]
                                : _RrIdentity();
                store.base[size_t(i)] = RrFrameFromMatrix(
                    avars * scratch->selfD[size_t(i)] *
                    scratch->parentDinv[size_t(i)] * parentPosed);
            }
            store.fin[size_t(i)] = store.base[size_t(i)];
            RrMat4d space = _RrIdentity();
            if (!store.base[size_t(i)].IsValid() ||
                store.base[size_t(i)].IsDegenerate() ||
                !RrPointsToMatrix(RrIdentityLandmarks(),
                                  store.base[size_t(i)].points,
                                  &space)) {
                space = _RrIdentity();
                space[3][0] = std::numeric_limits<double>::quiet_NaN();
            }
            store.posedM[size_t(i)] = space;
        }
        return true;
    }

    case RigExecWireStepKind::Solve: {
        return _RrPoseSteps::_RrRunSolveStep(program, step, error);
    }

    case RigExecWireStepKind::SolverCommit: {
        return _RrPoseSteps::_RrRunSolverCommitStep(program, step,
                                                    error);
    }

    case RigExecWireStepKind::Constraint: {
        return _RrPoseSteps::_RrRunConstraintStep(program, step, error);
    }

    case RigExecWireStepKind::CommitDelta: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->commits.size() ||
            size_t(wire.object) >= store.commits.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no commit";
            }
            return false;
        }
        RrCommitScratch &commit = store.commits[size_t(wire.object)];
        if (!commit.abandoned) {
            return _RrComputeCommitDeltas(
                program, step,
                program->poses->commits[size_t(wire.object)],
                &commit, error);
        }
        return true;
    }

    case RigExecWireStepKind::PropagateChunk: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->commits.size() ||
            size_t(wire.object) >= store.commits.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no commit";
            }
            return false;
        }
        RrCommitScratch &commit = store.commits[size_t(wire.object)];
        if (commit.abandoned) {
            return true;
        }
        if (wire.part < 0) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no chunk";
            }
            return false;
        }
        const RigExecWireCommit &wireCommit =
            program->poses->commits[size_t(wire.object)];
        const size_t begin =
            size_t(wire.part) * _RrPropagateChunkSize;
        const size_t end =
            std::min(begin + _RrPropagateChunkSize,
                     wireCommit.propagate.size());
        return _RrStageCommitPairs(program, step, wireCommit, &commit,
                                   begin, end, error);
    }

    case RigExecWireStepKind::CommitApply: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->commits.size() ||
            size_t(wire.object) >= store.commits.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no commit";
            }
            return false;
        }
        return _RrFinishCommit(program, step, size_t(wire.object),
                               error);
    }

    case RigExecWireStepKind::ProviderMatrix: {
        if (wire.object < 0 ||
            size_t(wire.object) >= store.finalMatrix.size() ||
            size_t(wire.object) >= store.baseMatrix.size() ||
            size_t(wire.object) >= store.finLast.size() ||
            size_t(wire.object) >= store.baseLast.size() ||
            size_t(wire.object) >= scratch->restFrames.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no slot";
            }
            return false;
        }
        const size_t slot = size_t(wire.object);
        RrMat4d matrix = _RrIdentity();
        if (wire.part) {
            if (size_t(store.finLast[slot]) >= store.fin.size() ||
                slot >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no fin version";
                }
                return false;
            }
            const RrPointFrame &frame =
                store.fin[size_t(store.finLast[slot])];
            if (RrFrameUsable(scratch->restFrames[slot]) &&
                RrFrameUsable(frame)) {
                RrPointsToMatrix(scratch->restPts[slot], frame.points,
                                 &matrix);
            }
            store.finalMatrix[slot] = matrix;
        } else {
            if (size_t(store.baseLast[slot]) >= store.base.size() ||
                slot >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no base version";
                }
                return false;
            }
            const RrPointFrame &frame =
                store.base[size_t(store.baseLast[slot])];
            if (RrFrameUsable(scratch->restFrames[slot]) &&
                RrFrameUsable(frame)) {
                RrPointsToMatrix(scratch->restPts[slot], frame.points,
                                 &matrix);
            }
            store.baseMatrix[slot] = matrix;
        }
        return true;
    }

    case RigExecWireStepKind::PoseInterpolator: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->poseInterpolators
                                        .size() ||
            size_t(wire.object) >= scratch->interpEnabled.size() ||
            size_t(wire.object) >= scratch->interpSolvers.size() ||
            size_t(wire.object) >= scratch->interpScratch.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no pose interpolator";
            }
            return false;
        }
        const RigExecWirePoseInterpolator &interp =
            program->poses->poseInterpolators[size_t(wire.object)];
        for (int slot = interp.weightBegin; slot < interp.weightEnd;
             ++slot) {
            if (slot < 0 ||
                size_t(slot) >= store.poseWeights.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no pose weight";
                }
                return false;
            }
            store.poseWeights[size_t(slot)] = 0.0f;
        }
        if (!scratch->interpEnabled[size_t(wire.object)] ||
            interp.poseSlots.empty()) {
            return true;
        }
        if (interp.driverSlot < 0 ||
            size_t(interp.driverSlot) >= store.finLast.size() ||
            size_t(interp.driverSlot) >= scratch->restFrames.size() ||
            size_t(interp.driverSlot) >=
                program->slotMeta->paths.size() ||
            (interp.parentSlot >= 0 &&
             (size_t(interp.parentSlot) >= store.finLast.size() ||
              size_t(interp.parentSlot) >=
                  scratch->restFrames.size()))) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no driver slot";
            }
            return false;
        }
        const size_t d = size_t(interp.driverSlot);
        if (size_t(store.finLast[d]) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        RrQuatd driverFinal(1.0), driverRest(1.0);
        RrQuatd parentFinal(1.0), parentRest(1.0);
        bool usable =
            RrFrameRotation(store.fin[size_t(store.finLast[d])],
                            &driverFinal) &&
            RrFrameRotation(scratch->restFrames[d], &driverRest);
        if (usable && interp.parentSlot >= 0) {
            const size_t p = size_t(interp.parentSlot);
            if (size_t(store.finLast[p]) >= store.fin.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no fin version";
                }
                return false;
            }
            usable = RrFrameRotation(store.fin[size_t(store.finLast[p])],
                                     &parentFinal) &&
                     RrFrameRotation(scratch->restFrames[p], &parentRest);
        }
        if (!usable) {
            output.diagnostics.push_back(
                "pose interpolator " +
                program->TextOrEmpty(interp.path) +
                " has no usable frame for its driver " +
                program->TextOrEmpty(program->slotMeta->paths[d]) +
                " after the pose walk; its weights are zero this "
                "generation");
            return true;
        }
        const RrQuatd local = parentFinal.GetInverse() * driverFinal;
        const RrQuatd restLocal = parentRest.GetInverse() * driverRest;
        const RrQuatd delta =
            (restLocal.GetInverse() * local).GetNormalized();
        std::vector<double> &interpScratch =
            scratch->interpScratch[size_t(wire.object)];
        scratch->interpSolvers[size_t(wire.object)].Evaluate(
            _RrRbfEulerFromQuaternion(delta), nullptr, &interpScratch,
            interp.allowNegativeWeights);
        if (interpScratch.size() != interp.poseSlots.size()) {
            output.diagnostics.push_back(
                "pose interpolator " +
                program->TextOrEmpty(interp.path) + " solved " +
                std::to_string(interpScratch.size()) + " weights for " +
                std::to_string(interp.poseSlots.size()) + " poses");
            return true;
        }
        for (size_t i = 0; i < interp.poseSlots.size(); ++i) {
            if (interp.poseSlots[i] < 0 ||
                size_t(interp.poseSlots[i]) >= store.poseWeights.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no pose weight";
                }
                return false;
            }
            store.poseWeights[size_t(interp.poseSlots[i])] =
                static_cast<float>(interpScratch[i]);
        }
        return true;
    }

    case RigExecWireStepKind::SnapshotFinals: {
        const size_t slots = program->slotMeta->paths.size();
        for (size_t i = 0; i < slots; ++i) {
            if (i >= store.finLast.size() ||
                size_t(store.finLast[i]) >= store.fin.size() ||
                i >= scratch->restFrames.size() ||
                i >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no slot";
                }
                return false;
            }
            const RrPointFrame &frame =
                store.fin[size_t(store.finLast[i])];
            if (!RrFrameUsable(scratch->restFrames[i]) ||
                !RrFrameUsable(frame)) {
                continue;
            }
            RrMat4d matrix = _RrIdentity();
            if (RrPointsToMatrix(scratch->restPts[i], frame.points,
                                 &matrix)) {
                RrSnapshotValue value;
                value.tag = RrSnapshotValue::Tag::Matrix;
                value.matrix = matrix;
                output.snapshots.RecordFinal(
                    program->slotMeta->paths[i], value);
            }
        }
        return true;
    }

    default:
        break;
    }

    if (error) {
        *error = _RrStepHead(program, step) + " not implemented yet";
    }
    return false;
}

namespace _RrPoseSteps {

// ---------------------------------------------------------------------------
// Solver kernels (rigExecMath/solvers.cpp, splineIk.cpp): Gf -> Rr, the
// arithmetic untouched.
// ---------------------------------------------------------------------------

// Rotates v about unit axis by angle (Rodrigues).
RrVec3d
_RrRotate(const RrVec3d &v, const RrVec3d &axis, double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return v * c + RrCross(axis, v) * s + axis * RrDot(axis, v) * (1 - c);
}

// RigExecSolveFkChain: each control's own rest->pose map composes with
// its parent's accumulated map, applied to the output basis.
std::vector<RrPointFrame>
_RrSolveFkChain(const std::vector<RrPoseFkElement> &elements)
{
    std::vector<RrPointFrame> result;
    result.reserve(elements.size());
    std::vector<RrMat4d> accumulated;
    accumulated.reserve(elements.size());
    for (size_t i = 0; i < elements.size(); ++i) {
        const RrPoseFkElement &e = elements[i];
        RrMat4d own = _RrIdentity();
        RrPointsToMatrix(e.restPoints, e.posePoints, &own);
        RrMat4d w = own;
        if (e.parentIndex >= 0 &&
            size_t(e.parentIndex) < accumulated.size()) {
            w = own * accumulated[size_t(e.parentIndex)];
        }
        accumulated.push_back(w);
        result.push_back(RrMatrixToPoints(
            e.hasOutRest ? e.outRestPoints : e.restPoints, w));
    }
    return result;
}

// A posed frame from an origin plus orthonormal aim/up directions,
// preserving the rest landmark handle lengths.
RrPointFrame
_RrFrameFromAxes(const std::array<RrVec3d, 4> &restPoints,
                 const RrVec3d &origin, const RrVec3d &ex,
                 const RrVec3d &eyCandidate)
{
    RrVec3d ey = eyCandidate - ex * RrDot(ex, eyCandidate);
    const double eyLen = ey.GetLength();
    if (eyLen < 1e-12) {
        const RrVec3d axes[3] = {
            RrVec3d(0, 1, 0), RrVec3d(0, 0, 1), RrVec3d(1, 0, 0)};
        for (const RrVec3d &axis : axes) {
            ey = axis - ex * RrDot(ex, axis);
            if (ey.GetLength() >= 1e-12) {
                break;
            }
        }
    }
    ey.Normalize();
    RrVec3d ez = RrCross(ex, ey);
    ez.Normalize();

    const double lx = (restPoints[1] - restPoints[0]).GetLength();
    const double ly = (restPoints[2] - restPoints[0]).GetLength();
    const double lz = (restPoints[3] - restPoints[0]).GetLength();

    RrPointFrame f;
    f.points[0] = origin;
    f.points[1] = origin + ex * lx;
    f.points[2] = origin + ey * ly;
    f.points[3] = origin + ez * lz;
    f.flags = RrPointFrameValid;
    return f;
}

// RigExecSolveTwoBoneIk: analytic two-bone IK with pole vector.
std::array<RrPointFrame, 3>
_RrSolveTwoBoneIk(
    const RrPointFrame &rootFrame, const RrPointFrame &effectorFrame,
    const RrPointFrame &poleFrame,
    const std::array<std::array<RrVec3d, 4>, 3> &restPoints,
    const RrPoseTwoBoneIkParams &params)
{
    const RrVec3d root = rootFrame.points[0];
    const RrVec3d goal = effectorFrame.points[0];
    const RrVec3d pole = poleFrame.points[0];

    const double l1 = std::max(params.upperLength, 1e-9);
    const double l2 = std::max(params.lowerLength, 1e-9);
    const double chain = l1 + l2;

    RrVec3d toGoal = goal - root;
    double dist = toGoal.GetLength();
    RrVec3d aim;
    if (dist > 1e-12) {
        aim = toGoal / dist;
    } else {
        const RrVec3d rootAim = rootFrame.points[1] - root;
        if (rootAim.GetLength() < 1e-12) {
            std::array<RrPointFrame, 3> failed = {
                rootFrame, rootFrame, effectorFrame};
            for (auto &f : failed) {
                f.flags |= RrPointFrameDegenerate;
            }
            return failed;
        }
        aim = rootAim.GetNormalized();
    }

    const double soft = std::max(params.softness, 0.0) * chain;
    double reach = dist;
    if (soft > 1e-12 && dist > chain - soft) {
        reach = chain - soft * std::exp(-(dist - (chain - soft)) / soft);
    } else if (dist > chain) {
        reach = chain;
    }

    double s1 = l1, s2 = l2;
    if (dist > reach && params.stretch > 0.0) {
        const double factor =
            1.0 + (dist / chain - 1.0) *
                std::min(std::max(params.stretch, 0.0), 1.0);
        if (factor > 1.0) {
            s1 = l1 * factor;
            s2 = l2 * factor;
            reach = std::min(dist, s1 + s2);
        }
    }
    reach = std::min(reach, s1 + s2);
    reach = std::max(reach, std::abs(s1 - s2) + 1e-12);

    RrVec3d poleDir = pole - root;
    poleDir -= aim * RrDot(aim, poleDir);
    RrVec3d bendUp;
    if (poleDir.GetLength() > 1e-12) {
        bendUp = poleDir.GetNormalized();
    } else {
        RrVec3d candidate = restPoints[0][2] - restPoints[0][0];
        candidate -= aim * RrDot(aim, candidate);
        if (candidate.GetLength() < 1e-12) {
            const RrVec3d axes[3] = {
                RrVec3d(1, 0, 0), RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
            double best = 2.0;
            for (const RrVec3d &axis : axes) {
                const double align = std::abs(RrDot(axis, aim));
                if (align < best) {
                    best = align;
                    candidate = axis - aim * RrDot(aim, axis);
                }
            }
        }
        bendUp = _RrRotate(candidate.GetNormalized(), aim,
                           params.preferredBendRadians);
    }
    RrVec3d bendAxis = RrCross(aim, bendUp);
    if (bendAxis.GetLength() < 1e-12) {
        bendAxis = RrCross(aim, _RrRotate(bendUp, aim, 0.5 * _RrPi));
    }
    bendAxis.Normalize();

    const double cosAlpha =
        (s1 * s1 + reach * reach - s2 * s2) / (2 * s1 * reach);
    const double alpha =
        std::acos(std::min(std::max(cosAlpha, -1.0), 1.0));

    const RrVec3d upperDir = _RrRotate(aim, bendAxis, alpha);
    const RrVec3d mid = root + upperDir * s1;
    const RrVec3d endPos = root + aim * reach;
    const RrVec3d lowerDir = (endPos - mid).GetNormalized();

    std::array<RrPointFrame, 3> out;
    out[0] = _RrFrameFromAxes(restPoints[0], root, upperDir, bendUp);
    out[1] = _RrFrameFromAxes(restPoints[1], mid, lowerDir, bendUp);

    RrPointFrame end = effectorFrame;
    const RrVec3d offset = endPos - effectorFrame.points[0];
    for (auto &p : end.points) {
        p += offset;
    }
    out[2] = end;
    return out;
}

// The open degree-2, four-CV B-spline with its arc-length table
// (RigExecSplineIkCurve, splineIk.cpp).
constexpr double _RrSplineIkEpsilon = 1e-10;

class _RrSplineIkCurve
{
public:
    _RrSplineIkCurve() : _RrSplineIkCurve(_ZeroCvs()) {}

    explicit _RrSplineIkCurve(const std::array<RrVec3d, 4> &cvs)
        : _cvs(cvs)
    {
        const int cells = 2 * 32;
        _u.resize(size_t(cells) + 1);
        _cumulative.resize(size_t(cells) + 1);
        _u[0] = 0.0;
        _cumulative[0] = 0.0;
        for (int i = 0; i < cells; ++i) {
            const double a = double(i) / 32;
            const double b = double(i + 1) / 32;
            const double half = 0.5 * (b - a);
            const double mid = 0.5 * (a + b);
            double sum = 0.0;
            for (int g = 0; g < 8; ++g) {
                RrVec3d d(0.0);
                Evaluate(mid + half * _GaussNodes[g], nullptr, &d);
                sum += _GaussWeights[g] * d.GetLength();
            }
            _u[size_t(i) + 1] = b;
            _cumulative[size_t(i) + 1] = _cumulative[size_t(i)] +
                                        sum * half;
        }
        _length = _cumulative[size_t(cells)];
        if (!std::isfinite(_length)) {
            _length = 0.0;
        }
    }

    double ArcLength() const { return _length; }

    const std::array<RrVec3d, 4> &Cvs() const { return _cvs; }

    bool IsDegenerate() const
    {
        return _length <= _RrSplineIkEpsilon;
    }

    double ParamAtArcLength(double distance) const
    {
        if (IsDegenerate()) {
            return 0.0;
        }
        distance = std::clamp(distance, 0.0, _length);
        const auto it = std::lower_bound(
            _cumulative.begin(), _cumulative.end(), distance);
        size_t hi = size_t(it - _cumulative.begin());
        if (hi == 0) {
            return _u[0];
        }
        if (hi >= _cumulative.size()) {
            hi = _cumulative.size() - 1;
        }
        const size_t lo = hi - 1;
        const double a = _u[lo], b = _u[hi];
        const double cellLength = _cumulative[hi] - _cumulative[lo];
        if (cellLength <=
            _RrSplineIkEpsilon * std::max(1.0, _length)) {
            return a;
        }
        const double target = distance - _cumulative[lo];

        auto partial = [&](double u) {
            const double half = 0.5 * (u - a);
            const double mid = 0.5 * (u + a);
            double sum = 0.0;
            for (int g = 0; g < 8; ++g) {
                RrVec3d d(0.0);
                Evaluate(mid + half * _GaussNodes[g], nullptr, &d);
                sum += _GaussWeights[g] * d.GetLength();
            }
            return sum * half;
        };

        double bl = a, bh = b;
        double u = a + (b - a) * (target / cellLength);
        const double tolerance = 1e-15 * std::max(1.0, _length);
        for (int iteration = 0; iteration < 32; ++iteration) {
            const double g = partial(u) - target;
            if (std::abs(g) <= tolerance) {
                break;
            }
            if (g > 0.0) {
                bh = u;
            } else {
                bl = u;
            }
            RrVec3d d(0.0);
            Evaluate(u, nullptr, &d);
            const double speed = d.GetLength();
            double next = (speed > _RrSplineIkEpsilon) ? u - g / speed
                                                       : 0.5 * (bl + bh);
            if (!(next > bl && next < bh)) {
                next = 0.5 * (bl + bh);
            }
            if (std::abs(next - u) <= 1e-16 * std::max(1.0, u)) {
                u = next;
                break;
            }
            u = next;
        }
        return u;
    }

    bool PointAtArcLength(double distance, RrVec3d *position,
                          RrVec3d *tangent) const
    {
        if (IsDegenerate()) {
            if (position) {
                *position = _cvs[0];
            }
            if (tangent) {
                *tangent = RrVec3d(0.0);
            }
            return false;
        }
        const double clamped = std::clamp(distance, 0.0, _length);
        const double u = ParamAtArcLength(clamped);
        RrVec3d p(0.0), d(0.0);
        Evaluate(u, &p, &d);
        RrVec3d dir = d;
        double speed = dir.GetLength();
        if (speed <= _RrSplineIkEpsilon) {
            const double step = 1.0 / 32;
            for (int i = 1;
                 i <= 2 * 32 && speed <= _RrSplineIkEpsilon; ++i) {
                RrVec3d ahead(0.0), behind(0.0);
                Evaluate(u + i * step, &ahead, nullptr);
                Evaluate(u - i * step, &behind, nullptr);
                dir = ahead - p;
                speed = dir.GetLength();
                if (speed <= _RrSplineIkEpsilon) {
                    dir = p - behind;
                    speed = dir.GetLength();
                }
            }
        }
        if (speed <= _RrSplineIkEpsilon) {
            if (position) {
                *position = p;
            }
            if (tangent) {
                *tangent = RrVec3d(0.0);
            }
            return false;
        }
        dir /= speed;
        if (distance != clamped) {
            p += dir * (distance - clamped);
        }
        if (position) {
            *position = p;
        }
        if (tangent) {
            *tangent = dir;
        }
        return true;
    }

    void Evaluate(double u, RrVec3d *position,
                  RrVec3d *derivative) const
    {
        const double clamped = RrClamp(u, 0.0, 2.0);
        const int span = clamped < 1.0 ? 0 : 1;
        const double t = clamped - span;
        const double s = 1.0 - t;
        RrVec3d q[3];
        const RrVec3d mid = (_cvs[1] + _cvs[2]) * 0.5;
        if (span == 0) {
            q[0] = _cvs[0];
            q[1] = _cvs[1];
            q[2] = mid;
        } else {
            q[0] = mid;
            q[1] = _cvs[2];
            q[2] = _cvs[3];
        }
        if (position) {
            *position = q[0] * (s * s) + q[1] * (2.0 * s * t) +
                        q[2] * (t * t);
        }
        if (derivative) {
            // dB/dt = 2[(1-t)(q1-q0) + t(q2-q1)], and du = dt within
            // a span.
            *derivative = ((q[1] - q[0]) * s + (q[2] - q[1]) * t) * 2.0;
        }
    }

private:
    static std::array<RrVec3d, 4> _ZeroCvs()
    {
        std::array<RrVec3d, 4> cvs;
        cvs[0] = RrVec3d(0.0);
        cvs[1] = RrVec3d(0.0);
        cvs[2] = RrVec3d(0.0);
        cvs[3] = RrVec3d(0.0);
        return cvs;
    }

    static constexpr double _GaussNodes[8] = {
        -0.9602898564975363, -0.7966664774136267, -0.5255324099163290,
        -0.1834346424956498, 0.1834346424956498, 0.5255324099163290,
        0.7966664774136267, 0.9602898564975363,
    };
    static constexpr double _GaussWeights[8] = {
        0.1012285362903763, 0.2223810344533745, 0.3137066458778873,
        0.3626837833783620, 0.3626837833783620, 0.3137066458778873,
        0.2223810344533745, 0.1012285362903763,
    };

    std::array<RrVec3d, 4> _cvs;
    std::vector<double> _u;
    std::vector<double> _cumulative;
    double _length = 0.0;
};

// RigExecSplineIkMakeRest: rest CVs from joints [0],[1],[N-2],[N-1],
// segment lengths, and the reference length.
RrPoseSplineIkRest
_RrSplineIkMakeRest(const std::vector<RrPointFrame> &joints,
                    const RrPointFrame &rootControl,
                    const RrPointFrame &midControl,
                    const RrPointFrame &endControl,
                    const std::vector<double> &volumeWeights,
                    uint8_t restLengthChain)
{
    RrPoseSplineIkRest rest;
    rest.rootControl = rootControl;
    rest.midControl = midControl;
    rest.endControl = endControl;
    rest.joints = joints;
    rest.volumeWeights = volumeWeights;

    const size_t n = joints.size();
    if (n == 0) {
        return rest;
    }
    const auto origin = [&](size_t i) {
        return joints[std::min(i, n - 1)].points[0];
    };
    rest.cvs = {origin(0), origin(1), origin(n >= 2 ? n - 2 : 0),
                origin(n - 1)};

    double chainLength = 0.0;
    rest.segmentLengths.resize(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
        rest.segmentLengths[i] = (origin(i + 1) - origin(i)).GetLength();
        chainLength += rest.segmentLengths[i];
    }
    rest.restArcLength =
        restLengthChain ? chainLength
                        : _RrSplineIkCurve(rest.cvs).ArcLength();
    return rest;
}

// Rebuilds one solver's rest description from the ladder this run
// composed (RefreshSolverRests, bakedPose.cpp).
bool
_RrRefreshSolverRests(RrProgram *program, size_t step, size_t solver,
                      std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSolver &wire = program->poses->solvers[solver];
    RrPoseSolverState &s = scratch->solvers[solver];
    const auto liveRest = [&](size_t k) {
        return k < wire.restIsLive.size() && wire.restIsLive[k] &&
               k < wire.restReads.size() &&
               size_t(wire.restReads[k]) < store.fin.size();
    };
    for (size_t k = 0; k < wire.restRefs.size() &&
         k < s.jointRests.size(); ++k) {
        const int slot = wire.restRefs[k].first;
        if (liveRest(k)) {
            s.jointRests[k] =
                store.fin[size_t(wire.restReads[k])].points;
        } else if (slot >= 0 &&
                   size_t(slot) < scratch->restPts.size()) {
            s.jointRests[k] = scratch->restPts[size_t(slot)];
        } else {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no rest slot";
            }
            return false;
        }
    }
    const std::string type = program->TextOrEmpty(wire.type);
    if (type == "RigExecFkChain") {
        for (size_t k = 0; k < wire.controls.size(); ++k) {
            if (k >= s.controlRests.size() || wire.controls[k] < 0 ||
                size_t(wire.controls[k]) >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no rest slot";
                }
                return false;
            }
            s.controlRests[k] =
                scratch->restPts[size_t(wire.controls[k])];
        }
    } else if (type == "RigExecTwoBoneIk") {
        for (size_t k = 0; k < wire.restRefs.size(); ++k) {
            const int slot = wire.restRefs[k].first;
            const int element = wire.restRefs[k].second;
            if (element >= 0 && element < 3) {
                if (liveRest(k)) {
                    s.ikRests[size_t(element)] =
                        store.fin[size_t(wire.restReads[k])].points;
                } else if (slot >= 0 &&
                           size_t(slot) < scratch->restPts.size()) {
                    s.ikRests[size_t(element)] =
                        scratch->restPts[size_t(slot)];
                } else {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no rest slot";
                    }
                    return false;
                }
            }
        }
        s.upperLengthBase =
            (s.ikRests[1][0] - s.ikRests[0][0]).GetLength();
        s.lowerLengthBase =
            (s.ikRests[2][0] - s.ikRests[1][0]).GetLength();
        s.ikParams.upperLength =
            s.upperLengthBase + wire.upperOffset.f64;
        s.ikParams.lowerLength =
            s.lowerLengthBase + wire.lowerOffset.f64;
    } else if (type == "RigExecSplineIk") {
        std::vector<RrPointFrame> restJoints(size_t(wire.splineCount));
        for (size_t k = 0; k < wire.restRefs.size(); ++k) {
            const int slot = wire.restRefs[k].first;
            const int element = wire.restRefs[k].second;
            if (element >= 0 &&
                size_t(element) < size_t(wire.splineCount)) {
                if (liveRest(k)) {
                    restJoints[size_t(element)] =
                        store.fin[size_t(wire.restReads[k])];
                    if (size_t(element) < s.splineJointRests.size()) {
                        s.splineJointRests[size_t(element)] =
                            store.fin[size_t(wire.restReads[k])].points;
                    }
                } else if (slot >= 0 &&
                           size_t(slot) < scratch->restPts.size() &&
                           size_t(slot) < scratch->restFrames.size()) {
                    restJoints[size_t(element)] =
                        scratch->restFrames[size_t(slot)];
                    if (size_t(element) < s.splineJointRests.size()) {
                        s.splineJointRests[size_t(element)] =
                            scratch->restPts[size_t(slot)];
                    }
                } else {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no rest slot";
                    }
                    return false;
                }
            }
        }
        const RrPointFrame noFrame;
        s.splineRest = _RrSplineIkMakeRest(
            restJoints,
            wire.root >= 0 ? scratch->restFrames[size_t(wire.root)]
                           : noFrame,
            wire.mid >= 0 ? scratch->restFrames[size_t(wire.mid)]
                          : noFrame,
            wire.end >= 0 ? scratch->restFrames[size_t(wire.end)]
                          : noFrame,
            wire.splineRestWeights, wire.splineRestMode);
    } else if (type == "RigExecTwistDistribution") {
        if (wire.root >= 0 && wire.end >= 0 &&
            size_t(wire.root) < scratch->restPts.size() &&
            size_t(wire.end) < scratch->restPts.size()) {
            s.twistStartRest =
                scratch->restPts[size_t(wire.root)];
            s.twistEndRest = scratch->restPts[size_t(wire.end)];
        }
    }
    return true;
}

RrPointFrame _RrBlendFrames(
    const RrPointFrame &a, const RrPointFrame &b,
    const std::array<RrVec3d, 4> &restPoints, double weight,
    bool logScale, const std::array<RrVec3d, 4> *outRestPoints);
RrPointFrameArray _RrSolveTwistDistribution(
    const RrPointFrame &start, const RrPointFrame &end,
    const std::array<RrVec3d, 4> &startRest,
    const std::array<RrVec3d, 4> &endRest,
    const std::vector<double> &weights, double twistTurns,
    const std::vector<std::array<RrVec3d, 4>> &jointRests,
    const std::vector<char> &jointRestLive);
RrPointFrameArray _RrSampleRibbonFrames(
    const std::vector<RrVec3f> &posed,
    const std::vector<RrVec3f> &rest, int sampleCount,
    const std::vector<std::array<RrVec3d, 4>> &jointRests,
    const std::vector<char> &jointRestLive);
struct _RrSplineIkControls {
    RrPointFrame root;
    RrPointFrame mid;
    RrPointFrame end;
};

struct _RrSplineIkParams {
    double preserveVolume = 1.0;
    double midFollowWeight = 0.5;
    double roll = 0.0;
    double twist = 0.0;
    double minLengthRatio = 0.0;
    bool aimRootTangent = false;
};

struct _RrSplineIkJoint {
    RrPointFrame frame;
    RrVec3d scale{1.0, 1.0, 1.0};
    double arcDistance = 0.0;
    double arcParam = 0.0;
    double twist = 0.0;
};

struct _RrSplineIkResult {
    std::array<RrVec3d, 4> cvs{};
    double arcLength = 0.0;
    double ratio = 1.0;
    double roll = 0.0;
    double twist = 0.0;
    std::vector<_RrSplineIkJoint> joints;
};

bool _RrSolveSplineIk(const RrPoseSplineIkRest &rest,
                      const _RrSplineIkControls &controls,
                      const _RrSplineIkParams &params,
                      _RrSplineIkResult *result);

bool
_RrRunSolveStep(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->solvers.size() ||
        size_t(wire.object) >= scratch->solvers.size() ||
        size_t(wire.object) >= store.aggregates.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    const RigExecWireSolver &ws =
        program->poses->solvers[size_t(wire.object)];
    RrPoseSolverState &s = scratch->solvers[size_t(wire.object)];
    const std::vector<std::array<RrVec3d, 4>> &liveRests =
        s.jointRests;
    std::vector<char> liveFlags;
    if (ws.hasLiveRest) {
        liveFlags.assign(s.jointRests.size(), 0);
        for (size_t k = 0;
             k < ws.restIsLive.size() && k < liveFlags.size(); ++k) {
            liveFlags[k] = ws.restIsLive[k];
        }
    }
    if ((scratch->ladderRecomputed && !ws.restSlots.empty()) ||
        ws.hasLiveRest) {
        if (!_RrRefreshSolverRests(program, step, size_t(wire.object),
                                   error)) {
            return false;
        }
    }
    RrPointFrameArray &aggregate =
        store.aggregates[size_t(wire.object)];
    aggregate.frames.clear();
    aggregate.rests.clear();
    const std::string type = program->TextOrEmpty(ws.type);
    const auto finAt = [&](uint32_t version, const RrPointFrame **out) {
        if (size_t(version) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        *out = &store.fin[size_t(version)];
        return true;
    };
    if (ws.degenerate) {
    } else if (type == "RigExecFkChain") {
        if (s.elements.size() != ws.controls.size() ||
            ws.controlReads.size() != ws.controls.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no solver control";
            }
            return false;
        }
        const bool jointBasis =
            s.jointRests.size() == ws.controls.size();
        aggregate.rests = s.controlRests;
        for (size_t k = 0; k < ws.controls.size(); ++k) {
            if (k >= s.controlRests.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no solver control";
                }
                return false;
            }
            s.elements[k].restPoints = s.controlRests[k];
            const RrPointFrame *pose = nullptr;
            if (!finAt(ws.controlReads[k], &pose)) {
                return false;
            }
            s.elements[k].posePoints = pose->points;
            const bool live = jointBasis && k < ws.restIsLive.size() &&
                              ws.restIsLive[k];
            s.elements[k].hasOutRest = live;
            if (live) {
                if (k >= s.jointRests.size() ||
                    k >= aggregate.rests.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no live rest";
                    }
                    return false;
                }
                s.elements[k].outRestPoints = s.jointRests[k];
                aggregate.rests[k] = s.jointRests[k];
            }
            s.elements[k].parentIndex =
                ws.parentRelative ? -1 : int(k) - 1;
        }
        aggregate.frames = _RrSolveFkChain(s.elements);
    } else if (type == "RigExecTwoBoneIk") {
        RrPoseTwoBoneIkParams params = s.ikParams;
        if (_RrLiveSolver(program, size_t(wire.object),
                          RrSolverBend) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverStretch) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverSoftness) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverUpperOffset) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverLowerOffset)) {
            params.preferredBendRadians =
                program->ReadSolver(size_t(wire.object),
                                    RrSolverBend).f64;
            params.stretch = double(
                program->ReadSolver(size_t(wire.object),
                                    RrSolverStretch).f32);
            params.softness = double(
                program->ReadSolver(size_t(wire.object),
                                    RrSolverSoftness).f32);
            params.upperLength =
                s.upperLengthBase +
                program->ReadSolver(size_t(wire.object),
                                    RrSolverUpperOffset).f64;
            params.lowerLength =
                s.lowerLengthBase +
                program->ReadSolver(size_t(wire.object),
                                    RrSolverLowerOffset).f64;
        }
        const RrPointFrame *root = nullptr;
        const RrPointFrame *end = nullptr;
        const RrPointFrame *pole = nullptr;
        if (!finAt(ws.rootRead, &root) ||
            !finAt(ws.endRead, &end) || !finAt(ws.poleRead, &pole)) {
            return false;
        }
        const std::array<RrPointFrame, 3> frames =
            _RrSolveTwoBoneIk(*root, *end, *pole, s.ikRests, params);
        aggregate.frames.assign(frames.begin(), frames.end());
        aggregate.rests.assign(s.ikRests.begin(), s.ikRests.end());
    } else if (type == "RigExecBlendPointFrames") {
        if ((ws.inA >= 0 &&
             size_t(ws.inA) >= store.aggregates.size()) ||
            (ws.inB >= 0 &&
             size_t(ws.inB) >= store.aggregates.size())) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no solver aggregate";
            }
            return false;
        }
        const RrPointFrameArray *a =
            ws.inA >= 0 ? &store.aggregates[size_t(ws.inA)] : nullptr;
        const RrPointFrameArray *b =
            ws.inB >= 0 ? &store.aggregates[size_t(ws.inB)] : nullptr;
        if (!a) {
            if (b) {
                aggregate = *b;
            }
        } else if (!b) {
            aggregate = *a;
        } else if (ws.blendRotationRejected) {
        } else {
            const size_t n = a->frames.size();
            const double w = std::min(
                std::max(double(program->ReadSolver(size_t(wire.object),
                                                    RrSolverBlendWeight)
                                    .f32),
                         0.0),
                1.0);
            if (n == b->frames.size() && a->rests.size() == n) {
                aggregate.frames.reserve(n);
                aggregate.rests.reserve(n);
                for (size_t k = 0; k < n; ++k) {
                    const bool live = k < liveFlags.size() &&
                                      liveFlags[k] &&
                                      k < liveRests.size();
                    aggregate.frames.push_back(_RrBlendFrames(
                        a->frames[k], b->frames[k], a->rests[k], w,
                        ws.scaleMode == 0,
                        live ? &liveRests[k] : nullptr));
                    aggregate.rests.push_back(
                        live ? liveRests[k] : a->rests[k]);
                }
            }
        }
    } else if (type == "RigExecTwistDistribution") {
        const RrPointFrame *start = nullptr;
        const RrPointFrame *end = nullptr;
        if (!finAt(ws.rootRead, &start) ||
            !finAt(ws.endRead, &end)) {
            return false;
        }
        aggregate = _RrSolveTwistDistribution(
            *start, *end, s.twistStartRest, s.twistEndRest,
            s.twistWeights,
            program->ReadSolver(size_t(wire.object),
                                RrSolverTwistTurns).f64,
            liveRests, liveFlags);
    } else if (type == "RigExecRibbon") {
        if (size_t(wire.object) >= store.ribbonPoints.size() ||
            size_t(wire.object) >= store.ribbonConstant.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no solver";
            }
            return false;
        }
        aggregate = _RrSampleRibbonFrames(
            ws.ribbonPointsVarying
                ? store.ribbonPoints[size_t(wire.object)]
                : store.ribbonConstant[size_t(wire.object)],
            s.ribbonRestPoints,
            program->ReadSolver(size_t(wire.object),
                                RrSolverRibbonSampleCount).i32,
            liveRests, liveFlags);
    } else if (type == "RigExecSplineIk") {
        _RrSplineIkParams params;
        params.preserveVolume = ws.splineParams.preserveVolume;
        params.midFollowWeight = ws.splineParams.midFollowWeight;
        params.roll = ws.splineParams.roll;
        params.twist = ws.splineParams.twist;
        params.minLengthRatio = ws.splineParams.minLengthRatio;
        params.aimRootTangent = ws.splineParams.aimRootTangent;
        if (ws.splineParamsVary ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverPreserveVolume) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverMidFollowWeight) ||
            _RrLiveSolver(program, size_t(wire.object), RrSolverRoll) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverTwist) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverMinLengthRatio)) {
            params.preserveVolume =
                program->ReadSolver(size_t(wire.object),
                                    RrSolverPreserveVolume).f64;
            params.midFollowWeight =
                program->ReadSolver(size_t(wire.object),
                                    RrSolverMidFollowWeight).f64;
            params.roll = RrDegreesToRadians(
                program->ReadSolver(size_t(wire.object),
                                    RrSolverRoll).f64);
            params.twist = RrDegreesToRadians(
                program->ReadSolver(size_t(wire.object),
                                    RrSolverTwist).f64);
            params.minLengthRatio =
                program->ReadSolver(size_t(wire.object),
                                    RrSolverMinLengthRatio).f64;
        }
        const RrPointFrame *root = nullptr;
        const RrPointFrame *mid = nullptr;
        const RrPointFrame *end = nullptr;
        if (!finAt(ws.rootRead, &root) ||
            !finAt(ws.midRead, &mid) || !finAt(ws.endRead, &end)) {
            return false;
        }
        _RrSplineIkControls controls;
        controls.root = *root;
        controls.mid = *mid;
        controls.end = *end;
        _RrSplineIkResult solved;
        _RrSolveSplineIk(s.splineRest, controls, params, &solved);
        if (solved.joints.size() == size_t(ws.splineCount)) {
            aggregate.frames.reserve(size_t(ws.splineCount));
            for (const auto &joint : solved.joints) {
                aggregate.frames.push_back(joint.frame);
            }
            aggregate.rests = s.splineJointRests;
        }
    }
    if (size_t(wire.object) >= store.solverOutFrames.size() ||
        size_t(wire.object) >= store.solverOutPresent.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    if (store.solverOutFrames[size_t(wire.object)].size() !=
            ws.outputs.size() ||
        store.solverOutPresent[size_t(wire.object)].size() !=
            ws.outputs.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    for (size_t k = 0; k < ws.outputs.size(); ++k) {
        const int32_t slot = ws.outputs[k].first;
        const int32_t element = ws.outputs[k].second;
        if (element < 0 ||
            size_t(element) >= aggregate.frames.size()) {
            store.solverOutPresent[size_t(wire.object)][k] = 0;
            (void)slot;
            continue;
        }
        store.solverOutFrames[size_t(wire.object)][k] =
            RrExtractElementFrame(&aggregate, size_t(element));
        store.solverOutPresent[size_t(wire.object)][k] = 1;
    }
    return true;
}

bool
_RrRunSolverCommitStep(RrProgram *program, size_t step,
                       std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->commits.size() ||
        size_t(wire.object) >= store.commits.size() ||
        size_t(wire.object) >= program->poses->walkSteps.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    const RigExecWireCommit &wireCommit =
        program->poses->commits[size_t(wire.object)];
    RrCommitScratch &commit = store.commits[size_t(wire.object)];
    std::fill(commit.present.begin(), commit.present.end(), 0);
    for (const int si :
         program->poses->walkSteps[size_t(wire.object)].batchSolvers) {
        if (si < 0 ||
            size_t(si) >= program->poses->solvers.size() ||
            size_t(si) >= store.solverOutFrames.size() ||
            size_t(si) >= store.solverOutPresent.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no solver";
            }
            return false;
        }
        const RigExecWireSolver &s =
            program->poses->solvers[size_t(si)];
        if (store.solverOutFrames[size_t(si)].size() !=
                s.outputs.size() ||
            store.solverOutPresent[size_t(si)].size() !=
                s.outputs.size() ||
            s.outPosition.size() != s.outputs.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no solver";
            }
            return false;
        }
        for (size_t k = 0; k < s.outputs.size(); ++k) {
            if (!store.solverOutPresent[size_t(si)][k] ||
                s.outPosition[k] < 0) {
                continue;
            }
            if (size_t(s.outPosition[k]) >= commit.frames.size() ||
                size_t(s.outPosition[k]) >= commit.present.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no candidate slot";
                }
                return false;
            }
            commit.frames[size_t(s.outPosition[k])] =
                store.solverOutFrames[size_t(si)][k];
            commit.present[size_t(s.outPosition[k])] = 1;
        }
    }
    commit.abandoned =
        std::find(commit.present.begin(), commit.present.end(), 1) ==
        commit.present.end();
    if (wireCommit.split) {
        return true;
    }
    if (!commit.abandoned) {
        if (!_RrComputeCommitDeltas(program, step, wireCommit, &commit,
                                    error)) {
            return false;
        }
        if (!_RrStageCommitPairs(program, step, wireCommit, &commit, 0,
                                 wireCommit.propagate.size(), error)) {
            return false;
        }
    }
    return _RrFinishCommit(program, step, size_t(wire.object), error);
}

// ---------------------------------------------------------------------------
// Point-frame SRT substrate (pointFrame.cpp, matrix3d.cpp): the SVD
// decomposition Blend, Twist and the FBX constraints solve through.
// ---------------------------------------------------------------------------

// GfMatrix3d::Orthonormalize (matrix3d.cpp): orthogonalize and
// normalize the row vectors. The frame path never reads the result.
void
_RrMat3Orthonormalize(RrMat3d *m)
{
    RrVec3d r0((*m)[0][0], (*m)[0][1], (*m)[0][2]);
    RrVec3d r1((*m)[1][0], (*m)[1][1], (*m)[1][2]);
    RrVec3d r2((*m)[2][0], (*m)[2][1], (*m)[2][2]);
    RrOrthogonalizeBasis(&r0, &r1, &r2, true);
    (*m)[0][0] = r0[0];
    (*m)[0][1] = r0[1];
    (*m)[0][2] = r0[2];
    (*m)[1][0] = r1[0];
    (*m)[1][1] = r1[1];
    (*m)[1][2] = r1[2];
    (*m)[2][0] = r2[0];
    (*m)[2][1] = r2[1];
    (*m)[2][2] = r2[2];
}

// GfMatrix3d::_SetRotateFromQuat (matrix3d.cpp), verbatim.
void
_RrMat3SetRotate(RrMat3d *m, const RrQuatd &rot)
{
    const double r = rot.GetReal();
    const RrVec3d i = rot.GetImaginary();
    (*m)[0][0] = 1.0 - 2.0 * (i[1] * i[1] + i[2] * i[2]);
    (*m)[0][1] = 2.0 * (i[0] * i[1] + i[2] * r);
    (*m)[0][2] = 2.0 * (i[2] * i[0] - i[1] * r);

    (*m)[1][0] = 2.0 * (i[0] * i[1] - i[2] * r);
    (*m)[1][1] = 1.0 - 2.0 * (i[2] * i[2] + i[0] * i[0]);
    (*m)[1][2] = 2.0 * (i[1] * i[2] + i[0] * r);

    (*m)[2][0] = 2.0 * (i[2] * i[0] + i[1] * r);
    (*m)[2][1] = 2.0 * (i[1] * i[2] - i[0] * r);
    (*m)[2][2] = 1.0 - 2.0 * (i[1] * i[1] + i[0] * i[0]);
}

// GfMatrix3d::ExtractRotationQuaternion (matrix3d.cpp, including the
// int-typed 4 in the else arm), followed by the GfRotation round trip
// ExtractRotation().GetQuat() performs.
RrQuatd
_RrMat3ExtractRotationQuat(const RrMat3d &m)
{
    int i;
    if (m[0][0] > m[1][1]) {
        i = (m[0][0] > m[2][2] ? 0 : 2);
    } else {
        i = (m[1][1] > m[2][2] ? 1 : 2);
    }

    RrVec3d im(0.0, 0.0, 0.0);
    double r = 0.0;

    if (m[0][0] + m[1][1] + m[2][2] > m[i][i]) {
        r = 0.5 * std::sqrt(m[0][0] + m[1][1] + m[2][2] + 1);
        im[0] = (m[1][2] - m[2][1]) / (4.0 * r);
        im[1] = (m[2][0] - m[0][2]) / (4.0 * r);
        im[2] = (m[0][1] - m[1][0]) / (4.0 * r);
    } else {
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        const double q =
            0.5 * std::sqrt(m[i][i] - m[j][j] - m[k][k] + 1);

        im[i] = q;
        im[j] = (m[i][j] + m[j][i]) / (4 * q);
        im[k] = (m[k][i] + m[i][k]) / (4 * q);
        r = (m[j][k] - m[k][j]) / (4 * q);
    }

    r = RrClamp(r, -1.0, 1.0);
    RrRotation rotation;
    rotation.SetQuat(RrQuatd(r, im));
    return rotation.GetQuat();
}

// RigExecTransformParams (pointFrame.h), field for field.
struct _RrTransformParams {
    RrVec3d translation{0, 0, 0};
    RrQuatd rotation{1, RrVec3d(0, 0, 0)};
    RrVec3d scale{1, 1, 1};
    RrVec3d shear{0, 0, 0};
    int reflectionAxis = 2;
};

// Jacobi eigen decomposition of a symmetric 3x3 matrix (pointFrame.cpp):
// eigenvalues descending, eigenvectors as the columns of V.
void
_RrSymmetricEigen3(const double m[3][3], double eval[3],
                   double evec[3][3])
{
    double a[3][3];
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a[i][j] = m[i][j];
        }
    }

    for (int sweep = 0; sweep < 64; ++sweep) {
        double off = std::abs(a[0][1]) + std::abs(a[0][2]) +
                     std::abs(a[1][2]);
        if (off < 1e-15) {
            break;
        }
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::abs(a[p][q]) < 1e-18) {
                    continue;
                }
                const double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) /
                    (std::abs(theta) + std::sqrt(theta * theta + 1));
                const double cth = 1.0 / std::sqrt(t * t + 1);
                const double sth = t * cth;
                for (int k = 0; k < 3; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = cth * akp - sth * akq;
                    a[k][q] = sth * akp + cth * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = cth * apk - sth * aqk;
                    a[q][k] = sth * apk + cth * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = cth * vkp - sth * vkq;
                    v[k][q] = sth * vkp + cth * vkq;
                }
            }
        }
    }

    int order[3] = {0, 1, 2};
    double d[3] = {a[0][0], a[1][1], a[2][2]};
    std::sort(order, order + 3,
              [&](int x, int y) { return d[x] > d[y]; });
    for (int i = 0; i < 3; ++i) {
        eval[i] = d[order[i]];
        for (int k = 0; k < 3; ++k) {
            evec[k][i] = v[k][order[i]];
        }
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(evec[k][i]) > std::abs(evec[maxK][i])) {
                maxK = k;
            }
        }
        if (evec[maxK][i] < 0) {
            for (int k = 0; k < 3; ++k) {
                evec[k][i] = -evec[k][i];
            }
        }
    }

    const double tol =
        1e-8 * std::max({std::abs(eval[0]), std::abs(eval[2]), 1.0});
    const bool eq01 = std::abs(eval[0] - eval[1]) <= tol;
    const bool eq12 = std::abs(eval[1] - eval[2]) <= tol;
    if (eq01 && eq12) {
        for (int i = 0; i < 3; ++i) {
            for (int k = 0; k < 3; ++k) {
                evec[k][i] = (k == i) ? 1.0 : 0.0;
            }
        }
    } else if (eq01 || eq12) {
        const int a = eq01 ? 0 : 1;
        const int other = eq01 ? 2 : 0;
        const double n[3] = {evec[0][other], evec[1][other],
                             evec[2][other]};
        int bestAxis = 0;
        double bestLen = -1.0;
        double b1[3] = {0, 0, 0};
        for (int axis = 0; axis < 3; ++axis) {
            double p[3];
            for (int k = 0; k < 3; ++k) {
                p[k] = ((k == axis) ? 1.0 : 0.0) - n[k] * n[axis];
            }
            const double len =
                std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            if (len > bestLen) {
                bestLen = len;
                bestAxis = axis;
                for (int k = 0; k < 3; ++k) {
                    b1[k] = p[k] / len;
                }
            }
        }
        (void)bestAxis;
        double b2[3] = {
            n[1] * b1[2] - n[2] * b1[1],
            n[2] * b1[0] - n[0] * b1[2],
            n[0] * b1[1] - n[1] * b1[0]};
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(b2[k]) > std::abs(b2[maxK])) {
                maxK = k;
            }
        }
        if (b2[maxK] < 0) {
            for (int k = 0; k < 3; ++k) {
                b2[k] = -b2[k];
            }
        }
        for (int k = 0; k < 3; ++k) {
            evec[k][a] = b1[k];
            evec[k][a + 1] = b2[k];
        }
    }
}

// RigExecPointsToParams: the affine map rest->pose decomposed into SRT
// via SVD. False for a singular reference or posed frame.
bool
_RrPointsToParams(const std::array<RrVec3d, 4> &restPoints,
                  const std::array<RrVec3d, 4> &posePoints,
                  int reflectionAxis, _RrTransformParams *params)
{
    RrMat4d m = _RrIdentity();
    if (!RrPointsToMatrix(restPoints, posePoints, &m)) {
        return false;
    }

    double L[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            L[i][j] = m[j][i];
        }
    }

    double ltl[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += L[k][i] * L[k][j];
            }
            ltl[i][j] = sum;
        }
    }
    double s2[3], V[3][3];
    _RrSymmetricEigen3(ltl, s2, V);

    double sigma[3];
    for (int i = 0; i < 3; ++i) {
        sigma[i] = std::sqrt(std::max(s2[i], 0.0));
    }
    const double detL =
        L[0][0] * (L[1][1] * L[2][2] - L[1][2] * L[2][1]) -
        L[0][1] * (L[1][0] * L[2][2] - L[1][2] * L[2][0]) +
        L[0][2] * (L[1][0] * L[2][1] - L[1][1] * L[2][0]);
    if (sigma[2] < 1e-14 * std::max(1.0, sigma[0])) {
        return false;
    }

    double U[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += L[i][k] * V[k][j];
            }
            U[i][j] = sum / sigma[j];
        }
    }

    const double delta = (detL >= 0) ? 1.0 : -1.0;

    int k = 2;
    if (delta < 0) {
        const int axis = reflectionAxis;
        double best = -1.0;
        for (int i = 0; i < 3; ++i) {
            const double align = std::abs(V[axis][i]);
            if (align > best) {
                best = align;
                k = i;
            }
        }
    }

    double R[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) {
                const double dkk = (kk == k) ? delta : 1.0;
                sum += U[i][kk] * dkk * V[j][kk];
            }
            R[i][j] = sum;
        }
    }

    double H[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) {
                sum += R[kk][i] * L[kk][j];
            }
            H[i][j] = sum;
        }
    }

    RrMat3d rowR(
        R[0][0], R[1][0], R[2][0],
        R[0][1], R[1][1], R[2][1],
        R[0][2], R[1][2], R[2][2]);
    _RrMat3Orthonormalize(&rowR);
    const RrQuatd q = _RrMat3ExtractRotationQuat(rowR);

    params->translation = RrVec3d(m[3][0], m[3][1], m[3][2]);
    params->rotation = q.GetNormalized();
    params->scale = RrVec3d(H[0][0], H[1][1], H[2][2]);
    params->shear = RrVec3d(
        H[0][1] / H[0][0],
        H[0][2] / H[0][0],
        H[1][2] / H[1][1]);
    params->reflectionAxis = reflectionAxis;
    return true;
}

// RigExecParamsToMatrix: L = R(q) H(s,h), t.
RrMat4d
_RrParamsToMatrix(const _RrTransformParams &params)
{
    RrMat3d rowR;
    _RrMat3SetRotate(&rowR, params.rotation);
    double R[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = rowR[j][i];
        }
    }

    const RrVec3d &sc = params.scale;
    const RrVec3d &sh = params.shear;
    double H[3][3] = {
        {sc[0], sh[0] * sc[0], sh[1] * sc[0]},
        {sh[0] * sc[0], sc[1], sh[2] * sc[1]},
        {sh[1] * sc[0], sh[2] * sc[1], sc[2]},
    };

    double L[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += R[i][k] * H[k][j];
            }
            L[i][j] = sum;
        }
    }

    return RrMat4d(
        L[0][0], L[1][0], L[2][0], 0.0,
        L[0][1], L[1][1], L[2][1], 0.0,
        L[0][2], L[1][2], L[2][2], 0.0,
        params.translation[0], params.translation[1],
        params.translation[2], 1.0);
}

// RigExecBlendEnvelope (envelope.h): the endpoint branches are the
// contract, not an optimization.
template <class T, class Weight>
T
_RrBlendEnvelope(const T &preceding, const T &full, Weight weight)
{
    if (weight <= Weight(0)) {
        return preceding;
    }
    if (weight >= Weight(1)) {
        return full;
    }
    return preceding + (full - preceding) * weight;
}

// RigExecBlendFrames (solvers.cpp).
RrPointFrame
_RrBlendFrames(const RrPointFrame &a, const RrPointFrame &b,
               const std::array<RrVec3d, 4> &restPoints,
               double weight, bool logScale,
               const std::array<RrVec3d, 4> *outRestPoints)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (!outRestPoints) {
        if (w <= 0.0) {
            return a;
        }
        if (w >= 1.0) {
            return b;
        }
    } else if (w <= 0.0 || w >= 1.0) {
        const RrPointFrame &pick = (w <= 0.0) ? a : b;
        RrMat4d end = _RrIdentity();
        if (!RrPointsToMatrix(restPoints, pick.points, &end)) {
            RrPointFrame held = pick;
            held.flags |= RrPointFrameDegenerate;
            return held;
        }
        return RrMatrixToPoints(*outRestPoints, end);
    }

    _RrTransformParams pa, pb;
    if (!_RrPointsToParams(restPoints, a.points, 2, &pa) ||
        !_RrPointsToParams(restPoints, b.points, 2, &pb)) {
        RrPointFrame held = (w < 0.5) ? a : b;
        held.flags |= RrPointFrameDegenerate;
        return held;
    }

    _RrTransformParams pr;
    pr.translation = pa.translation * (1 - w) + pb.translation * w;

    RrQuatd qa = pa.rotation, qb = pb.rotation;
    if (RrDot(qa.GetImaginary(), qb.GetImaginary()) +
            qa.GetReal() * qb.GetReal() <
        0) {
        qb = -qb;
    }
    pr.rotation = RrSlerp(w, qa, qb).GetNormalized();

    for (int i = 0; i < 3; ++i) {
        if (logScale && pa.scale[i] > 0 && pb.scale[i] > 0) {
            pr.scale[i] = std::exp(
                std::log(pa.scale[i]) * (1 - w) +
                std::log(pb.scale[i]) * w);
        } else {
            pr.scale[i] = pa.scale[i] * (1 - w) + pb.scale[i] * w;
        }
        pr.shear[i] = pa.shear[i] * (1 - w) + pb.shear[i] * w;
    }

    const RrMat4d m = _RrParamsToMatrix(pr);
    return RrMatrixToPoints(
        outRestPoints ? *outRestPoints : restPoints, m);
}

// Swing/twist decomposition of q about unit axis (solvers.cpp).
void
_RrSwingTwist(const RrQuatd &q, const RrVec3d &axis,
              RrQuatd *swing, RrQuatd *twist)
{
    const RrVec3d im = q.GetImaginary();
    const double proj = RrDot(im, axis);
    RrQuatd t(q.GetReal(), axis * proj);
    const double len = std::sqrt(
        t.GetReal() * t.GetReal() +
        RrDot(t.GetImaginary(), t.GetImaginary()));
    if (len < 1e-15) {
        *twist = RrQuatd::GetIdentity();
    } else {
        *twist = t * (1.0 / len);
    }
    *swing = q * twist->GetInverse();
}

// RigExecDistributeTwist (solvers.cpp).
std::vector<RrPointFrame>
_RrDistributeTwist(const RrPointFrame &start, const RrPointFrame &end,
                   const std::array<RrVec3d, 4> &startRest,
                   const std::array<RrVec3d, 4> &endRest,
                   const std::vector<double> &weights,
                   double twistTurns)
{
    std::vector<RrPointFrame> result;
    if (!std::isfinite(twistTurns)) {
        return result;
    }
    result.reserve(weights.size());

    _RrTransformParams ps, pe;
    const bool okS =
        _RrPointsToParams(startRest, start.points, 2, &ps);
    const bool okE =
        _RrPointsToParams(endRest, end.points, 2, &pe);
    if (!okS || !okE) {
        for (double w : weights) {
            RrPointFrame f = (w < 0.5) ? start : end;
            f.flags |= RrPointFrameDegenerate;
            result.push_back(f);
        }
        return result;
    }

    const RrVec3d aim =
        (start.points[1] - start.points[0]).GetNormalized();
    RrQuatd qs = ps.rotation, qe = pe.rotation;
    if (RrDot(qs.GetImaginary(), qe.GetImaginary()) +
            qs.GetReal() * qe.GetReal() <
        0) {
        qe = -qe;
    }
    const RrQuatd rel = qs.GetInverse() * qe;
    const RrQuatd qsInv = qs.GetInverse();
    const RrVec3d localAim = qsInv.Transform(aim);
    RrQuatd swing, twist;
    _RrSwingTwist(rel, localAim, &swing, &twist);

    double twistAngle = 2.0 * std::atan2(
        RrDot(twist.GetImaginary(), localAim), twist.GetReal());
    twistAngle += 2.0 * std::acos(-1.0) * twistTurns;
    if (!std::isfinite(twistAngle)) {
        return result;
    }

    for (size_t k = 0; k < weights.size(); ++k) {
        const double w = std::min(std::max(weights[k], 0.0), 1.0);

        const RrQuatd swingK =
            RrSlerp(w, RrQuatd::GetIdentity(), swing).GetNormalized();
        const double angK = twistAngle * w;
        const RrQuatd twistK(
            std::cos(angK / 2), localAim * std::sin(angK / 2));
        const RrQuatd qk = (qs * swingK * twistK).GetNormalized();

        _RrTransformParams pk;
        pk.translation =
            ps.translation * (1 - w) + pe.translation * w;
        pk.rotation = qk;
        pk.scale = ps.scale * (1 - w) + pe.scale * w;
        pk.shear = ps.shear * (1 - w) + pe.shear * w;

        const RrMat4d m = _RrParamsToMatrix(pk);
        RrPointFrame f = RrMatrixToPoints(startRest, m);
        const RrVec3d origin =
            start.points[0] * (1 - w) + end.points[0] * w;
        const RrVec3d shift = origin - f.points[0];
        for (auto &p : f.points) {
            p += shift;
        }
        result.push_back(f);
    }
    return result;
}

// ---------------------------------------------------------------------------
// Ribbon substrate (geometryKernels.cpp, solverKernels.cpp).
// ---------------------------------------------------------------------------

struct _RrCurveFrameSamples {
    std::vector<RrVec3f> positions;
    std::vector<RrVec3f> tangents;
    std::vector<RrVec3f> normals;
    std::vector<RrVec3f> binormals;
    std::vector<float> parameters;

    size_t GetSize() const { return positions.size(); }
};

RrVec3f
_RrBsplinePoint(const RrVec3f &p0, const RrVec3f &p1,
                const RrVec3f &p2, const RrVec3f &p3, float t)
{
    const float t2 = t * t, t3 = t2 * t;
    const float b0 = (1 - 3 * t + 3 * t2 - t3) / 6.0f;
    const float b1 = (4 - 6 * t2 + 3 * t3) / 6.0f;
    const float b2 = (1 + 3 * t + 3 * t2 - 3 * t3) / 6.0f;
    const float b3 = t3 / 6.0f;
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

// RigExecSampleCurveRMF: a cubic uniform B-spline at sampleCount
// arc-length parameters, rotation-minimizing frames by the
// double-reflection method.
_RrCurveFrameSamples
_RrSampleCurveRMF(const std::vector<RrVec3f> &controlPoints,
                  int sampleCount)
{
    _RrCurveFrameSamples samples;
    if (sampleCount < 2 || controlPoints.size() < 2) {
        return samples;
    }

    const int dense = std::max(sampleCount * 16, 64);
    std::vector<RrVec3f> densePoints;
    densePoints.reserve(size_t(dense) + 1);
    const int spans = int(controlPoints.size()) - 3;
    for (int i = 0; i <= dense; ++i) {
        const float u = float(i) / float(dense);
        if (spans >= 1) {
            const float s = u * spans;
            const int span = std::min(spans - 1, int(s));
            const float t = s - span;
            densePoints.push_back(_RrBsplinePoint(
                controlPoints[size_t(span)],
                controlPoints[size_t(span) + 1],
                controlPoints[size_t(span) + 2],
                controlPoints[size_t(span) + 3], t));
        } else {
            const float s = u * (controlPoints.size() - 1);
            const int seg = std::min(
                int(controlPoints.size()) - 2, int(s));
            densePoints.push_back(
                controlPoints[size_t(seg)] +
                (controlPoints[size_t(seg) + 1] -
                 controlPoints[size_t(seg)]) *
                    (s - seg));
        }
    }
    std::vector<float> arcLength(densePoints.size(), 0.0f);
    for (size_t i = 1; i < densePoints.size(); ++i) {
        arcLength[i] = arcLength[i - 1] +
                       (densePoints[i] - densePoints[i - 1]).GetLength();
    }
    const float total = arcLength.back();
    if (total <= 1e-12f) {
        return samples;
    }

    samples.positions.reserve(size_t(sampleCount));
    samples.parameters.reserve(size_t(sampleCount));
    size_t cursor = 0;
    for (int k = 0; k < sampleCount; ++k) {
        const float target = total * float(k) / float(sampleCount - 1);
        while (cursor + 1 < arcLength.size() &&
               arcLength[cursor + 1] < target) {
            ++cursor;
        }
        const float span = arcLength[cursor + 1] - arcLength[cursor];
        const float t = span > 1e-12f
            ? (target - arcLength[cursor]) / span
            : 0.0f;
        samples.positions.push_back(
            densePoints[cursor] +
            (densePoints[cursor + 1] - densePoints[cursor]) * t);
        samples.parameters.push_back(float(k) / float(sampleCount - 1));
    }

    samples.tangents.resize(size_t(sampleCount));
    for (int k = 0; k < sampleCount; ++k) {
        const RrVec3f &prev =
            samples.positions[size_t(std::max(0, k - 1))];
        const RrVec3f &next =
            samples.positions[size_t(std::min(sampleCount - 1, k + 1))];
        RrVec3f tangent = next - prev;
        const float len = tangent.GetLength();
        samples.tangents[size_t(k)] =
            len > 1e-12f ? tangent / len : RrVec3f(1, 0, 0);
    }
    samples.normals.resize(size_t(sampleCount));
    samples.binormals.resize(size_t(sampleCount));
    {
        const RrVec3f t0 = samples.tangents[0];
        RrVec3f candidate(1, 0, 0);
        float best = 2.0f;
        for (const RrVec3f axis :
             {RrVec3f(1, 0, 0), RrVec3f(0, 1, 0),
              RrVec3f(0, 0, 1)}) {
            const float align = std::abs(RrDot(axis, t0));
            if (align < best) {
                best = align;
                candidate = axis;
            }
        }
        RrVec3f n0 = candidate - t0 * RrDot(t0, candidate);
        n0.Normalize();
        samples.normals[0] = n0;
        samples.binormals[0] = RrCross(t0, n0);
    }
    for (int k = 0; k + 1 < sampleCount; ++k) {
        const RrVec3f v1 =
            samples.positions[size_t(k) + 1] - samples.positions[size_t(k)];
        const float c1 = RrDot(v1, v1);
        if (c1 <= 1e-20f) {
            samples.normals[size_t(k) + 1] = samples.normals[size_t(k)];
            samples.binormals[size_t(k) + 1] =
                samples.binormals[size_t(k)];
            continue;
        }
        const RrVec3f nL =
            samples.normals[size_t(k)] - v1 * (2.0f / c1) *
                RrDot(v1, samples.normals[size_t(k)]);
        const RrVec3f tL =
            samples.tangents[size_t(k)] - v1 * (2.0f / c1) *
                RrDot(v1, samples.tangents[size_t(k)]);
        const RrVec3f v2 = samples.tangents[size_t(k) + 1] - tL;
        const float c2 = RrDot(v2, v2);
        RrVec3f n = c2 > 1e-20f
            ? nL - v2 * (2.0f / c2) * RrDot(v2, nL)
            : nL;
        n -= samples.tangents[size_t(k) + 1] *
             RrDot(samples.tangents[size_t(k) + 1], n);
        const float len = n.GetLength();
        samples.normals[size_t(k) + 1] =
            len > 1e-12f ? n / len : samples.normals[size_t(k)];
        samples.binormals[size_t(k) + 1] = RrCross(
            samples.tangents[size_t(k) + 1],
            samples.normals[size_t(k) + 1]);
    }
    return samples;
}

RrVec3d
_RrWiden(const RrVec3f &v)
{
    return RrVec3d(double(v[0]), double(v[1]), double(v[2]));
}

// Re-bases one aggregate element onto a joint's rest reference
// (solverKernels.cpp).
bool
_RrRebaseElement(const std::array<RrVec3d, 4> &ownRest,
                 const std::array<RrVec3d, 4> &jointRest,
                 RrPointFrame *frame)
{
    RrMat4d map = _RrIdentity();
    if (!RrPointsToMatrix(ownRest, frame->points, &map)) {
        return false;
    }
    const uint32_t flags = frame->flags;
    *frame = RrMatrixToPoints(jointRest, map);
    frame->flags = flags;
    return true;
}

// RigExecSampleRibbonFrames (solverKernels.cpp).
RrPointFrameArray
_RrSampleRibbonFrames(
    const std::vector<RrVec3f> &posed,
    const std::vector<RrVec3f> &rest, int sampleCount,
    const std::vector<std::array<RrVec3d, 4>> &jointRests,
    const std::vector<char> &jointRestLive)
{
    RrPointFrameArray result;
    if (posed.empty() || rest.empty() || sampleCount < 2) {
        return result;
    }
    const _RrCurveFrameSamples posedSamples =
        _RrSampleCurveRMF(posed, sampleCount);
    const _RrCurveFrameSamples restSamples =
        _RrSampleCurveRMF(rest, sampleCount);
    if (posedSamples.GetSize() != size_t(sampleCount) ||
        restSamples.GetSize() != size_t(sampleCount)) {
        return result;
    }
    result.frames.reserve(size_t(sampleCount));
    result.rests.reserve(size_t(sampleCount));
    for (int k = 0; k < sampleCount; ++k) {
        RrPointFrame frame;
        frame.points = {
            _RrWiden(posedSamples.positions[size_t(k)]),
            _RrWiden(posedSamples.positions[size_t(k)] +
                     posedSamples.tangents[size_t(k)]),
            _RrWiden(posedSamples.positions[size_t(k)] +
                     posedSamples.normals[size_t(k)]),
            _RrWiden(posedSamples.positions[size_t(k)] +
                     posedSamples.binormals[size_t(k)])};
        frame.flags = RrPointFrameValid;
        std::array<RrVec3d, 4> restPoints = {
            _RrWiden(restSamples.positions[size_t(k)]),
            _RrWiden(restSamples.positions[size_t(k)] +
                     restSamples.tangents[size_t(k)]),
            _RrWiden(restSamples.positions[size_t(k)] +
                     restSamples.normals[size_t(k)]),
            _RrWiden(restSamples.positions[size_t(k)] +
                     restSamples.binormals[size_t(k)])};
        if (size_t(k) < jointRests.size() &&
            size_t(k) < jointRestLive.size() && jointRestLive[size_t(k)] &&
            _RrRebaseElement(restPoints, jointRests[size_t(k)], &frame)) {
            restPoints = jointRests[size_t(k)];
        }
        result.frames.push_back(frame);
        result.rests.push_back(restPoints);
    }
    return result;
}

// RigExecSolveTwistDistribution (solverKernels.cpp): every frame paired
// with the START landmarks, re-based where a lower step wrote.
RrPointFrameArray
_RrSolveTwistDistribution(
    const RrPointFrame &start, const RrPointFrame &end,
    const std::array<RrVec3d, 4> &startRest,
    const std::array<RrVec3d, 4> &endRest,
    const std::vector<double> &weights, double twistTurns,
    const std::vector<std::array<RrVec3d, 4>> &jointRests,
    const std::vector<char> &jointRestLive)
{
    RrPointFrameArray result;
    result.frames = _RrDistributeTwist(start, end, startRest, endRest,
                                       weights, twistTurns);
    result.rests.assign(result.frames.size(), startRest);
    for (size_t k = 0; k < result.frames.size(); ++k) {
        if (k < jointRests.size() && k < jointRestLive.size() &&
            jointRestLive[k] &&
            _RrRebaseElement(startRest, jointRests[k],
                             &result.frames[k])) {
            result.rests[k] = jointRests[k];
        }
    }
    return result;
}

// ---------------------------------------------------------------------------
// Spline-IK solve (splineIk.cpp).
// ---------------------------------------------------------------------------

bool
_RrSplineIsFinite(const RrVec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) &&
           std::isfinite(v[2]);
}

bool
_RrSplineIsFinite(const RrPointFrame &f)
{
    for (const RrVec3d &p : f.points) {
        if (!_RrSplineIsFinite(p)) {
            return false;
        }
    }
    return true;
}

// Rotates v by the minimal rotation taking unit `from` to unit `to`.
RrVec3d
_RrSplineRotateToward(const RrVec3d &from, const RrVec3d &to,
                      const RrVec3d &v)
{
    const RrVec3d axis = RrCross(from, to);
    const double s = axis.GetLength();
    const double c = RrDot(from, to);
    if (s <= _RrSplineIkEpsilon) {
        return v;
    }
    const RrVec3d k = axis / s;
    return v * c + RrCross(k, v) * s + k * (RrDot(k, v) * (1.0 - c));
}

RrVec3d
_RrSplineFallbackUp(const RrVec3d &x)
{
    int least = 0;
    double best = std::abs(x[0]);
    for (int i = 1; i < 3; ++i) {
        if (std::abs(x[i]) < best) {
            best = std::abs(x[i]);
            least = i;
        }
    }
    RrVec3d up(0.0);
    up[least] = 1.0;
    up -= x * RrDot(x, up);
    return up.GetNormalized();
}

RrPointFrame
_RrSplineDegenerateCopy(const RrPointFrame &frame)
{
    RrPointFrame copy = frame;
    copy.flags |= RrPointFrameDegenerate;
    return copy;
}

struct _RrSplineBasis {
    RrVec3d origin{0.0};
    RrVec3d x{1.0, 0.0, 0.0};
    RrVec3d y{0.0, 1.0, 0.0};
    RrVec3d z{0.0, 0.0, 1.0};
    double lx = 1.0, ly = 1.0, lz = 1.0;
    double handedness = 1.0;
    bool ok = false;
};

_RrSplineBasis
_RrSplineMakeBasis(const RrPointFrame &frame)
{
    _RrSplineBasis b;
    b.origin = frame.points[0];
    const RrVec3d ax = frame.points[1] - b.origin;
    const RrVec3d ay = frame.points[2] - b.origin;
    const RrVec3d az = frame.points[3] - b.origin;
    b.lx = ax.GetLength();
    b.ly = ay.GetLength();
    b.lz = az.GetLength();
    if (!_RrSplineIsFinite(frame) || b.lx <= _RrSplineIkEpsilon ||
        b.ly <= _RrSplineIkEpsilon) {
        return b;
    }
    b.x = ax / b.lx;
    RrVec3d up = ay - b.x * RrDot(b.x, ay);
    const double upLength = up.GetLength();
    if (upLength <= _RrSplineIkEpsilon * std::max(1.0, b.ly)) {
        return b;
    }
    b.y = up / upLength;
    const RrVec3d cross = RrCross(b.x, b.y);
    b.handedness = (RrDot(cross, az) < 0.0) ? -1.0 : 1.0;
    b.z = cross * b.handedness;
    if (b.lz <= _RrSplineIkEpsilon) {
        b.lz = 1.0;
    }
    b.ok = true;
    return b;
}

// RigExecSplineIkPoseCvs: the rest CVs carried by the control frames.
bool
_RrSplineIkPoseCvs(const RrPoseSplineIkRest &rest,
                   const _RrSplineIkControls &controls,
                   const _RrSplineIkParams &params,
                   std::array<RrVec3d, 4> *cvs)
{
    *cvs = rest.cvs;
    RrMat4d rootMap = _RrIdentity();
    RrMat4d endMap = _RrIdentity();
    if (!RrPointsToMatrix(rest.rootControl.points,
                          controls.root.points, &rootMap) ||
        !RrPointsToMatrix(rest.endControl.points, controls.end.points,
                          &endMap)) {
        return false;
    }
    (*cvs)[0] = rootMap.TransformAffine(rest.cvs[0]);
    (*cvs)[1] = rootMap.TransformAffine(rest.cvs[1]);
    (*cvs)[2] = endMap.TransformAffine(rest.cvs[2]);
    (*cvs)[3] = endMap.TransformAffine(rest.cvs[3]);

    const RrVec3d midRest = rest.midControl.points[0];
    const double w = params.midFollowWeight;
    const RrVec3d follow = rootMap.TransformAffine(midRest) * (1.0 - w) +
                           endMap.TransformAffine(midRest) * w;
    const RrVec3d offset = controls.mid.points[0] - follow;

    const RrVec3d restChord = rest.cvs[3] - rest.cvs[0];
    const double restChordLength = restChord.GetLength();
    RrVec3d rootAxis = rootMap.TransformDir(restChord);
    const bool haveAxis =
        restChordLength > _RrSplineIkEpsilon &&
        rootAxis.GetLength() > _RrSplineIkEpsilon;
    if (haveAxis) {
        rootAxis.Normalize();
    }

    if (params.minLengthRatio > 0.0 && haveAxis) {
        const double minAlong =
            params.minLengthRatio * restChordLength;
        const double along = RrDot((*cvs)[3] - (*cvs)[0], rootAxis);
        if (along < minAlong) {
            const RrVec3d lift = rootAxis * (minAlong - along);
            (*cvs)[2] += lift;
            (*cvs)[3] += lift;
        }
    }

    if (params.aimRootTangent && haveAxis) {
        const RrVec3d to = (*cvs)[3] - (*cvs)[0];
        if (to.GetLength() > _RrSplineIkEpsilon) {
            (*cvs)[1] = (*cvs)[0] + _RrSplineRotateToward(
                rootAxis, to.GetNormalized(),
                (*cvs)[1] - (*cvs)[0]);
        }
    }

    (*cvs)[1] += offset;
    (*cvs)[2] += offset;

    for (const RrVec3d &cv : *cvs) {
        if (!_RrSplineIsFinite(cv)) {
            *cvs = rest.cvs;
            return false;
        }
    }
    return true;
}

// RigExecSplineIkTwistAboutAxis: twist of the rest->pose rotation
// about the axis, radians in [-pi, pi].
double
_RrSplineIkTwistAboutAxis(const RrPointFrame &restFrame,
                          const RrPointFrame &posedFrame,
                          const RrVec3d &axis)
{
    const double axisLength = axis.GetLength();
    if (!_RrSplineIsFinite(axis) ||
        axisLength <= _RrSplineIkEpsilon ||
        !_RrSplineIsFinite(restFrame) ||
        !_RrSplineIsFinite(posedFrame)) {
        return 0.0;
    }
    RrMat4d map = _RrIdentity();
    if (!RrPointsToMatrix(restFrame.points, posedFrame.points, &map)) {
        return 0.0;
    }
    RrMat4d rotation = map;
    rotation.SetTranslateOnly(RrVec3d(0.0));
    rotation.Orthonormalize();
    if (rotation.GetDeterminant3() < 0.0) {
        rotation.SetRow(2, -rotation.GetRow(2));
    }
    RrQuatd q = rotation.ExtractRotationQuat();
    if (q.GetReal() < 0.0) {
        q = -q;
    }
    const double along = RrDot(q.GetImaginary(), axis / axisLength);
    return 2.0 * std::atan2(along, q.GetReal());
}

// RigExecSolveSplineIk: the chain laid out along the posed curve.
bool
_RrSolveSplineIk(const RrPoseSplineIkRest &rest,
                 const _RrSplineIkControls &controls,
                 const _RrSplineIkParams &params,
                 _RrSplineIkResult *result)
{
    *result = _RrSplineIkResult();
    result->cvs = rest.cvs;

    const size_t n = rest.joints.size();
    if (n == 0 || rest.segmentLengths.size() + 1 != n ||
        (!rest.volumeWeights.empty() && rest.volumeWeights.size() != n)) {
        return false;
    }

    const auto failWithRest = [&]() {
        result->joints.resize(n);
        for (size_t i = 0; i < n; ++i) {
            result->joints[i].frame =
                _RrSplineDegenerateCopy(rest.joints[i]);
        }
        return false;
    };

    bool finiteInputs = std::isfinite(params.preserveVolume) &&
                        std::isfinite(params.midFollowWeight) &&
                        std::isfinite(params.roll) &&
                        std::isfinite(params.twist) &&
                        std::isfinite(rest.restArcLength);
    for (const double s : rest.segmentLengths) {
        finiteInputs = finiteInputs && std::isfinite(s);
    }
    for (const double w : rest.volumeWeights) {
        finiteInputs = finiteInputs && std::isfinite(w);
    }
    if (!finiteInputs || rest.restArcLength <= _RrSplineIkEpsilon) {
        return failWithRest();
    }

    if (!_RrSplineIkPoseCvs(rest, controls, params, &result->cvs)) {
        return failWithRest();
    }
    const _RrSplineIkCurve curve(result->cvs);
    result->arcLength = curve.ArcLength();
    result->ratio = result->arcLength / rest.restArcLength;
    const bool curveOk = !curve.IsDegenerate();

    RrVec3d chainAxis = rest.cvs[3] - rest.cvs[0];
    if (chainAxis.GetLength() <= _RrSplineIkEpsilon) {
        chainAxis = rest.rootControl.points[1] -
                    rest.rootControl.points[0];
    }
    double rootTwist = 0.0, endTwist = 0.0;
    if (chainAxis.GetLength() > _RrSplineIkEpsilon) {
        chainAxis.Normalize();
        rootTwist = _RrSplineIkTwistAboutAxis(
            rest.rootControl, controls.root, chainAxis);
        endTwist = _RrSplineIkTwistAboutAxis(
            rest.endControl, controls.end, chainAxis);
    }
    result->roll = rootTwist + params.roll;
    result->twist = (endTwist - rootTwist) + params.twist;

    result->joints.resize(n);
    std::vector<RrVec3d> positions(n), tangents(n);
    double cumulative = 0.0;
    for (size_t i = 0; i < n; ++i) {
        if (i > 0) {
            cumulative += rest.segmentLengths[i - 1];
        }
        const double d = result->ratio * cumulative;
        result->joints[i].arcDistance = d;
        result->joints[i].arcParam =
            curveOk ? d / result->arcLength : 0.0;
        result->joints[i].twist =
            result->roll + result->twist * result->joints[i].arcParam;
        if (!curve.PointAtArcLength(d, &positions[i], &tangents[i])) {
            positions[i] = result->cvs[0];
            tangents[i] = RrVec3d(0.0);
        }
    }

    bool allOk = curveOk;
    for (size_t i = 0; i < n; ++i) {
        _RrSplineIkJoint &joint = result->joints[i];
        const _RrSplineBasis restBasis =
            _RrSplineMakeBasis(rest.joints[i]);
        if (!restBasis.ok) {
            joint.frame = _RrSplineDegenerateCopy(rest.joints[i]);
            allOk = false;
            continue;
        }

        RrVec3d aim = (i + 1 < n) ? positions[i + 1] - positions[i]
                                  : tangents[i];
        if (aim.GetLength() <= _RrSplineIkEpsilon) {
            aim = tangents[i];
        }
        bool oriented = true;
        if (aim.GetLength() <= _RrSplineIkEpsilon) {
            aim = restBasis.x;
            oriented = false;
        }
        const RrVec3d x = aim.GetNormalized();

        RrVec3d y = _RrSplineRotateToward(restBasis.x, x, restBasis.y);
        y -= x * RrDot(x, y);
        if (y.GetLength() <= _RrSplineIkEpsilon) {
            y = _RrSplineFallbackUp(x);
        } else {
            y.Normalize();
        }

        const double theta = joint.twist;
        const RrVec3d yTwisted =
            y * std::cos(theta) + RrCross(x, y) * std::sin(theta);
        const RrVec3d z = RrCross(x, yTwisted) * restBasis.handedness;

        const double w = rest.volumeWeights.empty()
            ? 0.0
            : rest.volumeWeights[i];
        const double s =
            1.0 - w * params.preserveVolume * (result->ratio - 1.0);
        joint.scale = RrVec3d(1.0, s, s);

        RrPointFrame &frame = joint.frame;
        frame.points[0] = positions[i];
        frame.points[1] = positions[i] + x * restBasis.lx;
        frame.points[2] = positions[i] + yTwisted * (restBasis.ly * s);
        frame.points[3] = positions[i] + z * (restBasis.lz * s);
        frame.flags = RrPointFrameValid;
        if (s != 1.0) {
            frame.flags |= RrPointFrameAffine;
        }
        if (s < 0.0) {
            frame.flags |= RrPointFrameReflected;
        }
        if (!oriented || !curveOk) {
            frame.flags |= RrPointFrameDegenerate;
            allOk = false;
        }
    }
    return allOk;
}

// ---------------------------------------------------------------------------
// FBX constraint kernels (solvers.cpp).
// ---------------------------------------------------------------------------

enum class _RrEulerOrder : uint8_t {
    XYZ = 0,
    XZY,
    YXZ,
    YZX,
    ZXY,
    ZYX,
};

struct _RrConstraintAxisMask {
    bool x = true;
    bool y = true;
    bool z = true;
};

struct _RrConstraintSource {
    RrPointFrame frame;
    double normalizedWeight = 0.0;
    RrVec3d translationOffset{0.0};
    RrVec3d rotationOffsetDegrees{0.0};
};

struct _RrPositionConstraintParams {
    double weight = 1.0;
    RrVec3d offset{0.0};
    _RrConstraintAxisMask affect;
};

struct _RrRotationConstraintParams {
    double weight = 1.0;
    RrVec3d offsetDegrees{0.0};
    _RrConstraintAxisMask affect;
    _RrEulerOrder rotationOrder = _RrEulerOrder::XYZ;
};

struct _RrScaleConstraintParams {
    double weight = 1.0;
    RrVec3d offset{1.0, 1.0, 1.0};
    _RrConstraintAxisMask affect;
};

struct _RrParentConstraintParams {
    double weight = 1.0;
    _RrConstraintAxisMask translationAxes;
    _RrConstraintAxisMask rotationAxes;
    _RrConstraintAxisMask scaleAxes;
    _RrEulerOrder rotationOrder = _RrEulerOrder::XYZ;
};

struct _RrAimConstraintParams {
    double weight = 1.0;
    RrVec3d localAimVector{1.0, 0.0, 0.0};
    RrVec3d localUpVector{0.0, 1.0, 0.0};
    RrVec3d rotationOffsetDegrees{0.0};
    _RrConstraintAxisMask affectRotation;
    _RrEulerOrder rotationOrder = _RrEulerOrder::XYZ;
    const RrVec3d *worldUpDirection = nullptr;
    bool preserveInputUp = false;
};

bool
_RrConstraintFrameUsable(const RrPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const RrVec3d &point : frame.points) {
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
            !std::isfinite(point[2])) {
            return false;
        }
    }
    return true;
}

bool
_RrConstraintVecFinite(const RrVec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) &&
           std::isfinite(v[2]);
}

RrPointFrame
_RrConstraintFailure(const RrPointFrame &input)
{
    RrPointFrame failed = input;
    failed.flags |= RrPointFrameDegenerate;
    return failed;
}

bool _RrHasFinitePoints(const RrPointFrame &frame);

bool
_RrDecomposeConstraintFrame(const RrPointFrame &frame,
                            _RrTransformParams *params)
{
    if (!_RrConstraintFrameUsable(frame) ||
        !_RrPointsToParams(RrIdentityLandmarks(), frame.points, 2,
                           params)) {
        return false;
    }
    return _RrConstraintVecFinite(params->translation) &&
           _RrConstraintVecFinite(params->scale) &&
           _RrConstraintVecFinite(params->shear) &&
           std::isfinite(params->rotation.GetReal()) &&
           _RrConstraintVecFinite(params->rotation.GetImaginary());
}

RrPointFrame
_RrFrameFromConstraintParams(const RrPointFrame &input,
                             const _RrTransformParams &params)
{
    RrPointFrame result = RrMatrixToPoints(
        RrIdentityLandmarks(), _RrParamsToMatrix(params));
    if (!_RrHasFinitePoints(result)) {
        return _RrConstraintFailure(input);
    }
    // Affine is descriptive rather than required for reconstruction, but an
    // input explicitly classified affine remains classified affine when its
    // shear is carried through the constraint.
    result.flags |= input.flags & RrPointFrameAffine;
    return result;
}

bool
_RrValidateGlobalWeight(double weight, double *clamped)
{
    if (!std::isfinite(weight)) {
        return false;
    }
    *clamped = std::min(std::max(weight, 0.0), 1.0);
    return true;
}

bool
_RrValidateSourceWeight(const _RrConstraintSource &source)
{
    return std::isfinite(source.normalizedWeight) &&
           source.normalizedWeight >= 0.0;
}

bool
_RrAffects(const _RrConstraintAxisMask &mask, int axis)
{
    return axis == 0 ? mask.x : axis == 1 ? mask.y : mask.z;
}

bool
_RrIsValidOrder(_RrEulerOrder order)
{
    const int raw = int(order);
    return raw >= 0 && raw <= 5;
}

std::array<int, 3>
_RrOrderIndices(_RrEulerOrder order)
{
    switch (order) {
    case _RrEulerOrder::XYZ:
        return {0, 1, 2};
    case _RrEulerOrder::XZY:
        return {0, 2, 1};
    case _RrEulerOrder::YXZ:
        return {1, 0, 2};
    case _RrEulerOrder::YZX:
        return {1, 2, 0};
    case _RrEulerOrder::ZXY:
        return {2, 0, 1};
    case _RrEulerOrder::ZYX:
        return {2, 1, 0};
    }
    return {0, 1, 2};
}

RrQuatd
_RrQuatFromEulerDegrees(const RrVec3d &degrees, _RrEulerOrder order)
{
    static const RrVec3d axes[3] = {
        RrVec3d(1, 0, 0), RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
    RrMat4d matrix = _RrIdentity();
    const std::array<int, 3> indices = _RrOrderIndices(order);
    for (const int axis : indices) {
        matrix = matrix *
                 RrMat4d(RrRotation(axes[axis], degrees[axis]),
                         RrVec3d(0.0));
    }
    return matrix.ExtractRotation().GetQuat().GetNormalized();
}

RrVec3d
_RrEulerDegreesFromQuat(const RrQuatd &rotation, _RrEulerOrder order)
{
    static const RrVec3d axes[3] = {
        RrVec3d(1, 0, 0), RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
    const std::array<int, 3> indices = _RrOrderIndices(order);
    // GfRotation::Decompose(a,b,c) describes row-matrix factors in the
    // reverse order Rc * Rb * Ra.  Pass the authored application sequence
    // reversed so returned components invert _QuatFromEulerDegrees exactly.
    const RrVec3d ordered = RrRotation(rotation).Decompose(
        axes[indices[2]], axes[indices[1]], axes[indices[0]]);
    RrVec3d result(0.0);
    result[indices[2]] = ordered[0];
    result[indices[1]] = ordered[1];
    result[indices[0]] = ordered[2];
    return result;
}

double
_RrShortestDegrees(double degrees)
{
    double wrapped = std::fmod(degrees + 180.0, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    wrapped -= 180.0;
    // Resolve the exact half-turn tie without depending on fmod's sign.
    return (wrapped == -180.0 && degrees > 0.0) ? 180.0 : wrapped;
}

RrVec3d
_RrApplyEulerDelta(const RrVec3d &inputEuler,
                   const RrVec3d &targetEuler,
                   const _RrConstraintAxisMask &affect, double weight)
{
    RrVec3d output = inputEuler;
    for (int axis = 0; axis < 3; ++axis) {
        if (_RrAffects(affect, axis)) {
            // The target Euler representation is the full-strength
            // candidate.  Select it directly at the endpoint: reconstructing
            // the equivalent input + shortestDelta representation can differ
            // bit-for-bit (for example, 190 versus -170 degrees).
            if (weight >= 1.0) {
                output[axis] = targetEuler[axis];
            } else {
                output[axis] += weight * _RrShortestDegrees(
                    targetEuler[axis] - inputEuler[axis]);
            }
        }
    }
    return output;
}

bool
_RrHasFinitePoints(const RrPointFrame &frame)
{
    for (const RrVec3d &point : frame.points) {
        if (!_RrConstraintVecFinite(point)) {
            return false;
        }
    }
    return true;
}

bool
_RrNormalizeDirection(RrVec3d *direction, double eps = 1e-12)
{
    if (!_RrConstraintVecFinite(*direction)) {
        return false;
    }
    const double largest = std::max(
        {std::abs((*direction)[0]), std::abs((*direction)[1]),
         std::abs((*direction)[2])});
    if (largest < eps) {
        return false;
    }
    *direction /= largest;
    const double length = direction->GetLength();
    if (!std::isfinite(length) || length < eps) {
        return false;
    }
    *direction /= length;
    return _RrConstraintVecFinite(*direction);
}

// GfRotation::RotateOntoProjected (rotation.cpp): the twist about
// `axisParam` taking v1's projection onto v2's.
RrRotation
_RrRotateOntoProjected(const RrVec3d &v1, const RrVec3d &v2,
                       const RrVec3d &axisParam)
{
    RrVec3d axis = axisParam.GetNormalized();

    RrVec3d v1Proj = v1 - RrDot(v1, axis) * axis;
    RrVec3d v2Proj = v2 - RrDot(v2, axis) * axis;
    v1Proj.Normalize();
    v2Proj.Normalize();
    RrVec3d crossAxis = RrCross(v1Proj, v2Proj);
    double sinTheta = RrDot(crossAxis, axis);
    double cosTheta = RrDot(v1Proj, v2Proj);
    double theta = 0;
    if (!(std::fabs(sinTheta) < 1e-6 && std::fabs(cosTheta) < 1e-6)) {
        theta = std::atan2(sinTheta, cosTheta);
    }

    const double toDeg = (180.0) / std::acos(-1.0);
    return RrRotation(axis, theta * toDeg);
}

RrPointFrame
_RrApplyPositionConstraint(
    const RrPointFrame &input,
    const std::vector<_RrConstraintSource> &sources,
    const _RrPositionConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_RrConstraintFrameUsable(input) ||
        !_RrConstraintVecFinite(params.offset)) {
        return _RrConstraintFailure(input);
    }

    RrVec3d target(0.0);
    double totalWeight = 0.0;
    for (const _RrConstraintSource &source : sources) {
        if (!_RrValidateSourceWeight(source)) {
            return _RrConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!_RrConstraintFrameUsable(source.frame)) {
            return _RrConstraintFailure(input);
        }
        target += source.frame.points[0] * source.normalizedWeight;
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input
                                  : _RrConstraintFailure(input);
    }
    target = target / totalWeight + params.offset;
    if (!_RrConstraintVecFinite(target)) {
        return _RrConstraintFailure(input);
    }

    RrVec3d constrainedOrigin = input.points[0];
    for (int axis = 0; axis < 3; ++axis) {
        if (_RrAffects(params.affect, axis)) {
            constrainedOrigin[axis] = _RrBlendEnvelope(
                input.points[0][axis], target[axis], globalWeight);
        }
    }
    if (!_RrConstraintVecFinite(constrainedOrigin)) {
        return _RrConstraintFailure(input);
    }
    RrPointFrame output = input;
    if (globalWeight >= 1.0) {
        for (size_t point = 1; point < output.points.size(); ++point) {
            output.points[point] =
                constrainedOrigin +
                (input.points[point] - input.points[0]);
        }
        output.points[0] = constrainedOrigin;
    } else {
        const RrVec3d translation =
            constrainedOrigin - input.points[0];
        for (RrVec3d &point : output.points) {
            point += translation;
        }
    }
    return output;
}

RrPointFrame
_RrApplyRotationConstraint(
    const RrPointFrame &input,
    const std::vector<_RrConstraintSource> &sources,
    const _RrRotationConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_RrIsValidOrder(params.rotationOrder) ||
        !_RrConstraintVecFinite(params.offsetDegrees)) {
        return _RrConstraintFailure(input);
    }

    _RrTransformParams inputParams;
    if (!_RrDecomposeConstraintFrame(input, &inputParams)) {
        return _RrConstraintFailure(input);
    }
    const RrVec3d inputEuler =
        _RrEulerDegreesFromQuat(inputParams.rotation,
                                params.rotationOrder);
    if (!_RrConstraintVecFinite(inputEuler)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d sourceAnchor(0.0), weightedDelta(0.0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const _RrConstraintSource &source : sources) {
        if (!_RrValidateSourceWeight(source)) {
            return _RrConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        _RrTransformParams sourceParams;
        if (!_RrDecomposeConstraintFrame(source.frame, &sourceParams)) {
            return _RrConstraintFailure(input);
        }
        const RrVec3d sourceEuler = _RrEulerDegreesFromQuat(
            sourceParams.rotation, params.rotationOrder);
        if (!_RrConstraintVecFinite(sourceEuler)) {
            return _RrConstraintFailure(input);
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedDelta[axis] += source.normalizedWeight *
                _RrShortestDegrees(sourceEuler[axis] -
                                   sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input
                                  : _RrConstraintFailure(input);
    }
    const RrVec3d targetEuler =
        sourceAnchor + weightedDelta / totalWeight + params.offsetDegrees;
    if (!_RrConstraintVecFinite(weightedDelta) ||
        !_RrConstraintVecFinite(targetEuler)) {
        return _RrConstraintFailure(input);
    }
    const RrVec3d outputEuler = _RrApplyEulerDelta(
        inputEuler, targetEuler, params.affect, globalWeight);
    inputParams.rotation =
        _RrQuatFromEulerDegrees(outputEuler, params.rotationOrder);
    return _RrFrameFromConstraintParams(input, inputParams);
}

RrPointFrame
_RrApplyScaleConstraint(
    const RrPointFrame &input,
    const std::vector<_RrConstraintSource> &sources,
    const _RrScaleConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_RrConstraintVecFinite(params.offset)) {
        return _RrConstraintFailure(input);
    }

    _RrTransformParams inputParams;
    if (!_RrDecomposeConstraintFrame(input, &inputParams)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d targetScale(0.0);
    double totalWeight = 0.0;
    for (const _RrConstraintSource &source : sources) {
        if (!_RrValidateSourceWeight(source)) {
            return _RrConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        _RrTransformParams sourceParams;
        if (!_RrDecomposeConstraintFrame(source.frame, &sourceParams)) {
            return _RrConstraintFailure(input);
        }
        targetScale += sourceParams.scale * source.normalizedWeight;
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input
                                  : _RrConstraintFailure(input);
    }
    targetScale = targetScale / totalWeight + params.offset;
    if (!_RrConstraintVecFinite(targetScale)) {
        return _RrConstraintFailure(input);
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (_RrAffects(params.affect, axis)) {
            inputParams.scale[axis] = _RrBlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    return _RrFrameFromConstraintParams(input, inputParams);
}

RrPointFrame
_RrApplyParentConstraint(
    const RrPointFrame &input,
    const std::vector<_RrConstraintSource> &sources,
    const _RrParentConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!_RrIsValidOrder(params.rotationOrder)) {
        return _RrConstraintFailure(input);
    }

    _RrTransformParams inputParams;
    if (!_RrDecomposeConstraintFrame(input, &inputParams)) {
        return _RrConstraintFailure(input);
    }
    const RrVec3d inputEuler =
        _RrEulerDegreesFromQuat(inputParams.rotation,
                                params.rotationOrder);
    if (!_RrConstraintVecFinite(inputEuler)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d targetTranslation(0.0), targetScale(0.0);
    RrVec3d sourceAnchor(0.0), weightedRotationDelta(0.0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const _RrConstraintSource &source : sources) {
        if (!_RrValidateSourceWeight(source)) {
            return _RrConstraintFailure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!_RrConstraintVecFinite(source.translationOffset) ||
            !_RrConstraintVecFinite(source.rotationOffsetDegrees)) {
            return _RrConstraintFailure(input);
        }
        if (!_RrConstraintFrameUsable(source.frame)) {
            return _RrConstraintFailure(input);
        }
        RrMat4d sourceMatrix = _RrIdentity();
        if (!RrPointsToMatrix(RrIdentityLandmarks(), source.frame.points,
                              &sourceMatrix)) {
            return _RrConstraintFailure(input);
        }

        _RrTransformParams offsetParams;
        offsetParams.translation = source.translationOffset;
        offsetParams.rotation = _RrQuatFromEulerDegrees(
            source.rotationOffsetDegrees, params.rotationOrder);
        const RrMat4d targetMatrix =
            _RrParamsToMatrix(offsetParams) * sourceMatrix;
        const RrPointFrame targetFrame = RrMatrixToPoints(
            RrIdentityLandmarks(), targetMatrix);
        _RrTransformParams targetParams;
        if (!_RrDecomposeConstraintFrame(targetFrame, &targetParams)) {
            return _RrConstraintFailure(input);
        }
        targetTranslation +=
            targetParams.translation * source.normalizedWeight;
        targetScale += targetParams.scale * source.normalizedWeight;

        const RrVec3d sourceEuler = _RrEulerDegreesFromQuat(
            targetParams.rotation, params.rotationOrder);
        if (!_RrConstraintVecFinite(sourceEuler)) {
            return _RrConstraintFailure(input);
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedRotationDelta[axis] += source.normalizedWeight *
                _RrShortestDegrees(sourceEuler[axis] -
                                   sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input
                                  : _RrConstraintFailure(input);
    }
    targetTranslation /= totalWeight;
    targetScale /= totalWeight;
    const RrVec3d targetEuler =
        sourceAnchor + weightedRotationDelta / totalWeight;
    if (!_RrConstraintVecFinite(targetTranslation) ||
        !_RrConstraintVecFinite(targetScale) ||
        !_RrConstraintVecFinite(weightedRotationDelta) ||
        !_RrConstraintVecFinite(targetEuler)) {
        return _RrConstraintFailure(input);
    }

    for (int axis = 0; axis < 3; ++axis) {
        if (_RrAffects(params.translationAxes, axis)) {
            inputParams.translation[axis] = _RrBlendEnvelope(
                inputParams.translation[axis], targetTranslation[axis],
                globalWeight);
        }
        if (_RrAffects(params.scaleAxes, axis)) {
            inputParams.scale[axis] = _RrBlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    const RrVec3d outputEuler = _RrApplyEulerDelta(
        inputEuler, targetEuler, params.rotationAxes, globalWeight);
    inputParams.rotation =
        _RrQuatFromEulerDegrees(outputEuler, params.rotationOrder);
    return _RrFrameFromConstraintParams(input, inputParams);
}

RrPointFrame
_RrApplyAimConstraint(const RrPointFrame &input,
                      const RrVec3d &targetPoint,
                      const _RrAimConstraintParams &params)
{
    double globalWeight = 0.0;
    if (!_RrValidateGlobalWeight(params.weight, &globalWeight)) {
        return _RrConstraintFailure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    const bool needsRollCorrection =
        params.worldUpDirection != nullptr || params.preserveInputUp;
    if (!_RrIsValidOrder(params.rotationOrder) ||
        !_RrConstraintVecFinite(targetPoint) ||
        !_RrConstraintVecFinite(params.localAimVector) ||
        (needsRollCorrection &&
         !_RrConstraintVecFinite(params.localUpVector)) ||
        !_RrConstraintVecFinite(params.rotationOffsetDegrees) ||
        (params.worldUpDirection &&
         !_RrConstraintVecFinite(*params.worldUpDirection))) {
        return _RrConstraintFailure(input);
    }

    _RrTransformParams inputParams;
    if (!_RrDecomposeConstraintFrame(input, &inputParams)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d localAim = params.localAimVector;
    if (!_RrNormalizeDirection(&localAim)) {
        return _RrConstraintFailure(input);
    }
    RrVec3d targetAim = targetPoint - inputParams.translation;
    if (!_RrNormalizeDirection(&targetAim)) {
        return _RrConstraintFailure(input);
    }

    const RrQuatd inputRotation = inputParams.rotation.GetNormalized();
    RrVec3d currentAim = inputRotation.Transform(localAim);
    if (!_RrNormalizeDirection(&currentAim)) {
        return _RrConstraintFailure(input);
    }
    const RrRotation swing(currentAim, targetAim);
    RrRotation twist(RrVec3d(1, 0, 0), 0.0);
    if (needsRollCorrection) {
        RrVec3d localUp = params.localUpVector;
        if (!_RrNormalizeDirection(&localUp)) {
            return _RrConstraintFailure(input);
        }
        localUp -= localAim * RrDot(localAim, localUp);
        if (!_RrNormalizeDirection(&localUp)) {
            return _RrConstraintFailure(input);
        }

        RrVec3d currentUp = inputRotation.Transform(localUp);
        if (!_RrNormalizeDirection(&currentUp)) {
            return _RrConstraintFailure(input);
        }
        RrVec3d swungUp = swing.TransformDir(currentUp);
        swungUp -= targetAim * RrDot(targetAim, swungUp);
        if (!_RrNormalizeDirection(&swungUp)) {
            return _RrConstraintFailure(input);
        }

        RrVec3d desiredUp;
        if (params.worldUpDirection) {
            desiredUp = *params.worldUpDirection;
            if (!_RrNormalizeDirection(&desiredUp)) {
                return _RrConstraintFailure(input);
            }
            desiredUp -= targetAim * RrDot(targetAim, desiredUp);
            if (!_RrNormalizeDirection(&desiredUp)) {
                return _RrConstraintFailure(input);
            }
        } else {
            desiredUp =
                currentUp - targetAim * RrDot(targetAim, currentUp);
            if (!_RrNormalizeDirection(&desiredUp)) {
                desiredUp = swungUp;
            }
        }
        twist = _RrRotateOntoProjected(swungUp, desiredUp, targetAim);
    }

    RrMat4d aimedMatrix = _RrIdentity();
    aimedMatrix.SetRotateOnly(inputRotation);
    RrMat4d swingMatrix = _RrIdentity(), twistMatrix = _RrIdentity();
    swingMatrix.SetRotate(swing);
    twistMatrix.SetRotate(twist);
    aimedMatrix = aimedMatrix * swingMatrix * twistMatrix;
    const RrQuatd aimedRotation =
        aimedMatrix.ExtractRotation().GetQuat().GetNormalized();

    const RrVec3d inputEuler =
        _RrEulerDegreesFromQuat(inputRotation, params.rotationOrder);
    const RrVec3d targetEuler =
        _RrEulerDegreesFromQuat(aimedRotation, params.rotationOrder) +
        params.rotationOffsetDegrees;
    if (!_RrConstraintVecFinite(inputEuler) ||
        !_RrConstraintVecFinite(targetEuler)) {
        return _RrConstraintFailure(input);
    }
    const RrVec3d outputEuler = _RrApplyEulerDelta(
        inputEuler, targetEuler, params.affectRotation, globalWeight);
    inputParams.rotation =
        _RrQuatFromEulerDegrees(outputEuler, params.rotationOrder);
    return _RrFrameFromConstraintParams(input, inputParams);
}

RrPointFrame
_RrApplyAimLandmarks(const RrPointFrame &input,
                     const RrVec3d &targetOrigin, double weight,
                     int aimLandmarkIndex)
{
    if (!std::isfinite(weight)) {
        RrPointFrame flagged = input;
        flagged.flags |= RrPointFrameDegenerate;
        return flagged;
    }
    weight = std::min(std::max(weight, 0.0), 1.0);

    const int aimIdx = std::min(std::max(aimLandmarkIndex, 1), 3);
    const int upIdx = (aimIdx % 3) + 1;
    const int sideIdx = (upIdx % 3) + 1;

    const RrVec3d origin = input.points[0];
    const RrVec3d currentAim = input.points[size_t(aimIdx)] - origin;
    const double aimLen = currentAim.GetLength();
    const RrVec3d toTarget = targetOrigin - origin;
    if (aimLen < 1e-12 || toTarget.GetLength() < 1e-12 ||
        weight <= 0.0) {
        return input;
    }
    const RrVec3d a0 = currentAim / aimLen;
    const RrVec3d a1 = toTarget.GetNormalized();

    const RrVec3d inUp = input.points[size_t(upIdx)] - origin;
    const RrVec3d inSide = input.points[size_t(sideIdx)] - origin;
    const double handedness =
        RrDot(RrCross(currentAim, inUp), inSide) < 0 ? -1.0 : 1.0;

    const RrRotation full(a0, a1);
    const RrRotation partial(full.GetAxis(),
                             full.GetAngle() * weight);
    const RrVec3d newAim = weight >= 1.0
        ? a1
        : partial.TransformDir(a0).GetNormalized();

    const double upLen = inUp.GetLength();
    const double sideLen = inSide.GetLength();
    RrVec3d up = inUp;
    up -= newAim * RrDot(newAim, up);
    if (up.GetLength() < 1e-12) {
        up = weight >= 1.0 ? full.TransformDir(inUp)
                           : partial.TransformDir(inUp);
        up -= newAim * RrDot(newAim, up);
    }
    if (up.GetLength() < 1e-12) {
        RrPointFrame flagged = input;
        flagged.flags |= RrPointFrameDegenerate;
        return flagged;
    }
    up.Normalize();
    const RrVec3d side =
        handedness * RrCross(newAim, up).GetNormalized();

    RrPointFrame out = input;
    out.points[size_t(aimIdx)] = origin + newAim * aimLen;
    out.points[size_t(upIdx)] = origin + up * upLen;
    out.points[size_t(sideIdx)] = origin + side * sideLen;
    return out;
}

// ---------------------------------------------------------------------------
// Single-chain IK (singleChainIk.cpp).
// ---------------------------------------------------------------------------

enum : int {
    _RrFabrikIterations = 128,
};

enum class _RrSingleChainIkMode : uint8_t {
    RotatePlane = 0,
    SingleChain = 1,
};

struct _RrSingleChainIkParams {
    _RrSingleChainIkMode mode = _RrSingleChainIkMode::RotatePlane;
    RrVec3d pole{0.0, 1.0, 0.0};
    double twistDegrees = 0.0;
    double weight = 1.0;
};

bool
_RrIkVecFinite(const RrVec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) &&
           std::isfinite(v[2]);
}

bool
_RrIkFrameFinite(const RrPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const RrVec3d &point : frame.points) {
        if (!_RrIkVecFinite(point)) {
            return false;
        }
    }
    return true;
}

std::vector<RrPointFrame>
_RrIkDegenerate(const std::vector<RrPointFrame> &frames)
{
    std::vector<RrPointFrame> failed = frames;
    for (RrPointFrame &frame : failed) {
        frame.flags |= RrPointFrameDegenerate;
    }
    return failed;
}

RrVec3d
_RrIkProjectPerpendicular(const RrVec3d &v, const RrVec3d &axis)
{
    return v - axis * RrDot(v, axis);
}

RrVec3d
_RrIkWorldAxisLeastParallel(const RrVec3d &axis)
{
    const RrVec3d world[3] = {
        RrVec3d(1.0, 0.0, 0.0),
        RrVec3d(0.0, 1.0, 0.0),
        RrVec3d(0.0, 0.0, 1.0),
    };
    int best = 0;
    double bestAlignment = std::abs(RrDot(world[0], axis));
    for (int i = 1; i < 3; ++i) {
        const double alignment = std::abs(RrDot(world[i], axis));
        if (alignment < bestAlignment) {
            best = i;
            bestAlignment = alignment;
        }
    }
    return world[best];
}

RrVec3d
_RrIkStablePerpendicular(const RrVec3d &axis, const RrVec3d &first,
                          const RrVec3d &second, double epsilon)
{
    auto normalized = [](const RrVec3d &candidate) {
        const double length = candidate.GetLength();
        return std::isfinite(length) && length > 0.0
            ? candidate / length
            : RrVec3d(0.0);
    };
    RrVec3d result = _RrIkProjectPerpendicular(normalized(first), axis);
    if (result.GetLength() <= epsilon) {
        result = _RrIkProjectPerpendicular(normalized(second), axis);
    }
    if (result.GetLength() <= epsilon) {
        result = _RrIkProjectPerpendicular(
            _RrIkWorldAxisLeastParallel(axis), axis);
    }
    return result.GetNormalized();
}

RrVec3d
_RrIkRotate(const RrVec3d &v, const RrVec3d &unitAxis, double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return v * c + RrCross(unitAxis, v) * s +
           unitAxis * RrDot(unitAxis, v) * (1.0 - c);
}

RrVec3d
_RrIkRotateToward(const RrVec3d &v, const RrVec3d &fromAxis,
                  const RrVec3d &toAxis, double weight,
                  const RrVec3d &fallback, double epsilon)
{
    const RrVec3d from = fromAxis.GetNormalized();
    const RrVec3d to = toAxis.GetNormalized();
    const double dot =
        std::min(std::max(RrDot(from, to), -1.0), 1.0);
    RrVec3d rotationAxis = RrCross(from, to);
    const double sinAngle = rotationAxis.GetLength();
    double angle = 0.0;
    if (sinAngle > epsilon) {
        rotationAxis /= sinAngle;
        angle = std::atan2(sinAngle, dot);
    } else if (dot < 0.0) {
        rotationAxis = _RrIkStablePerpendicular(
            from, fallback, RrVec3d(0.0), epsilon);
        angle = _RrPi;
    } else {
        return v;
    }
    return _RrIkRotate(v, rotationAxis, angle * weight);
}

RrVec3d
_RrIkBlendDirection(const RrVec3d &from, const RrVec3d &to,
                    double weight, const RrVec3d &fallback,
                    double epsilon)
{
    RrVec3d result =
        _RrIkRotateToward(from, from, to, weight, fallback, epsilon);
    const double length = result.GetLength();
    if (length <= epsilon || !_RrIkVecFinite(result)) {
        return from.GetNormalized();
    }
    return result / length;
}

struct _RrIkFrameBasis {
    RrVec3d x;
    RrVec3d up;
    double xLength = 0.0;
    double yLength = 0.0;
    double zLength = 0.0;
    double handedness = 1.0;
};

bool
_RrIkExtractBasis(const RrPointFrame &frame, double epsilon,
                  _RrIkFrameBasis *basis)
{
    const RrVec3d x = frame.points[1] - frame.points[0];
    const RrVec3d y = frame.points[2] - frame.points[0];
    const RrVec3d z = frame.points[3] - frame.points[0];
    const double xLength = x.GetLength();
    const double yLength = y.GetLength();
    const double zLength = z.GetLength();
    const double frameScale = std::max({xLength, yLength, zLength});
    if (!std::isfinite(frameScale) || frameScale <= 0.0 ||
        xLength <= frameScale * epsilon ||
        yLength <= frameScale * epsilon ||
        zLength <= frameScale * epsilon) {
        return false;
    }

    const RrVec3d unitX = x / xLength;
    RrVec3d up = _RrIkProjectPerpendicular(y / yLength, unitX);
    if (up.GetLength() <= epsilon) {
        return false;
    }
    up.Normalize();

    const double determinant = RrDot(
        RrCross(x / xLength, y / yLength), z / zLength);
    if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-10) {
        return false;
    }

    basis->x = unitX;
    basis->up = up;
    basis->xLength = xLength;
    basis->yLength = yLength;
    basis->zLength = zLength;
    basis->handedness = determinant < 0.0 ? -1.0 : 1.0;
    return true;
}

RrPointFrame
_RrIkFrameFromBasis(const _RrIkFrameBasis &basis, const RrVec3d &origin,
                    const RrVec3d &unitX, const RrVec3d &unitUp)
{
    const RrVec3d side =
        basis.handedness * RrCross(unitX, unitUp).GetNormalized();
    RrPointFrame frame;
    frame.points[0] = origin;
    frame.points[1] = origin + unitX * basis.xLength;
    frame.points[2] = origin + unitUp * basis.yLength;
    frame.points[3] = origin + side * basis.zLength;
    frame.flags = RrPointFrameValid;
    if (basis.handedness < 0.0) {
        frame.flags |= RrPointFrameReflected;
    }
    return frame;
}

_RrIkFrameBasis
_RrIkAimedBasis(const _RrIkFrameBasis &basis, const RrVec3d &unitAim,
                double epsilon)
{
    RrVec3d up = _RrIkRotateToward(basis.up, basis.x, unitAim, 1.0,
                                   basis.up, epsilon);
    up = _RrIkProjectPerpendicular(up, unitAim);
    if (up.GetLength() <= epsilon) {
        up = _RrIkStablePerpendicular(unitAim, basis.up, basis.x,
                                       epsilon);
    } else {
        up.Normalize();
    }
    _RrIkFrameBasis aimed = basis;
    aimed.x = unitAim;
    aimed.up = up;
    return aimed;
}

_RrIkFrameBasis
_RrIkTransportedBasis(const _RrIkFrameBasis &basis,
                      const RrVec3d &fromDirection,
                      const RrVec3d &toDirection, double epsilon)
{
    _RrIkFrameBasis transported = basis;
    transported.x = _RrIkRotateToward(basis.x, fromDirection,
                                      toDirection, 1.0, basis.up,
                                      epsilon);
    transported.x.Normalize();
    transported.up = _RrIkRotateToward(basis.up, fromDirection,
                                       toDirection, 1.0, basis.up,
                                       epsilon);
    transported.up = _RrIkProjectPerpendicular(transported.up,
                                                transported.x);
    if (transported.up.GetLength() <= epsilon) {
        transported.up = _RrIkStablePerpendicular(
            transported.x, basis.up, basis.x, epsilon);
    } else {
        transported.up.Normalize();
    }
    return transported;
}

double
_RrIkSignedAngle(const RrVec3d &from, const RrVec3d &to,
                 const RrVec3d &unitAxis)
{
    return std::atan2(
        RrDot(unitAxis, RrCross(from, to)),
        std::min(std::max(RrDot(from, to), -1.0), 1.0));
}

RrPointFrame
_RrIkBlendEndFrame(const _RrIkFrameBasis &input,
                   const _RrIkFrameBasis &effector,
                   const RrVec3d &origin, double weight,
                   double epsilon)
{
    const RrVec3d x = _RrIkBlendDirection(
        input.x, effector.x, weight, input.up, epsilon);

    RrVec3d carriedAtTarget = _RrIkRotateToward(
        input.up, input.x, effector.x, 1.0, input.up, epsilon);
    carriedAtTarget =
        _RrIkProjectPerpendicular(carriedAtTarget, effector.x);
    if (carriedAtTarget.GetLength() <= epsilon) {
        carriedAtTarget = _RrIkStablePerpendicular(
            effector.x, input.up, effector.up, epsilon);
    } else {
        carriedAtTarget.Normalize();
    }
    const double roll =
        _RrIkSignedAngle(carriedAtTarget, effector.up, effector.x);

    RrVec3d up = _RrIkRotateToward(input.up, input.x, effector.x,
                                   weight, input.up, epsilon);
    up = _RrIkProjectPerpendicular(up, x);
    if (up.GetLength() <= epsilon) {
        up = _RrIkStablePerpendicular(x, input.up, effector.up,
                                       epsilon);
    } else {
        up.Normalize();
    }
    up = _RrIkRotate(up, x, roll * weight).GetNormalized();
    return _RrIkFrameFromBasis(input, origin, x, up);
}

RrVec3d
_RrIkDirectionOrFallback(const RrVec3d &direction,
                         const RrVec3d &fallback,
                         double positionEpsilon, double angularEpsilon)
{
    const double directionLength = direction.GetLength();
    if (directionLength > positionEpsilon) {
        return direction / directionLength;
    }
    const double fallbackLength = fallback.GetLength();
    if (fallbackLength > angularEpsilon) {
        return fallback / fallbackLength;
    }
    return RrVec3d(1.0, 0.0, 0.0);
}

std::vector<RrVec3d>
_RrIkSolvePositions(
    const std::vector<RrVec3d> &original,
    const std::vector<double> &lengths,
    const std::vector<_RrIkFrameBasis> &bases,
    const _RrIkFrameBasis &effectorBasis, const RrVec3d &goal,
    const _RrSingleChainIkParams &params, double positionEpsilon,
    double angularEpsilon)
{
    const size_t count = original.size();
    const RrVec3d root = original.front();
    const double totalLength = [&]() {
        double total = 0.0;
        for (double length : lengths)
            total += length;
        return total;
    }();

    RrVec3d rootToGoal = goal - root;
    const double goalDistance = rootToGoal.GetLength();
    RrVec3d solveAxis;
    if (goalDistance > positionEpsilon) {
        solveAxis = rootToGoal / goalDistance;
    } else {
        solveAxis = original.back() - root;
        if (solveAxis.GetLength() <= positionEpsilon) {
            solveAxis = original[1] - root;
        }
        solveAxis = _RrIkDirectionOrFallback(
            solveAxis, bases.front().x, positionEpsilon, angularEpsilon);
    }

    if (goalDistance >= totalLength - positionEpsilon &&
        goalDistance > positionEpsilon) {
        std::vector<RrVec3d> extended(count);
        extended[0] = root;
        for (size_t i = 0; i < lengths.size(); ++i) {
            extended[i + 1] = extended[i] + solveAxis * lengths[i];
        }
        return extended;
    }

    std::vector<RrVec3d> positions = original;
    RrVec3d planeUp;
    RrVec3d planeNormal;
    if (params.mode == _RrSingleChainIkMode::RotatePlane) {
        RrVec3d poleOffset = params.pole - root;
        if (poleOffset.GetLength() <= positionEpsilon) {
            poleOffset = RrVec3d(0.0);
        }
        planeUp = _RrIkStablePerpendicular(
            solveAxis, poleOffset, bases.front().up, angularEpsilon);
        planeUp = _RrIkRotate(
            planeUp, solveAxis, params.twistDegrees * (_RrPi / 180.0));
        planeUp.Normalize();
    } else {
        planeUp = _RrIkStablePerpendicular(
            solveAxis, effectorBasis.up, effectorBasis.x,
            angularEpsilon);
    }
    planeNormal = RrCross(solveAxis, planeUp).GetNormalized();

    for (size_t i = 1; i + 1 < count; ++i) {
        const RrVec3d relative = positions[i] - root;
        const double along = RrDot(relative, solveAxis);
        const double towardPlane = std::abs(RrDot(relative, planeUp));
        positions[i] =
            root + solveAxis * along + planeUp * towardPlane;
    }

    double maxOffAxis = 0.0;
    for (size_t i = 1; i + 1 < count; ++i) {
        maxOffAxis = std::max(
            maxOffAxis,
            _RrIkProjectPerpendicular(positions[i] - root, solveAxis)
                .GetLength());
    }
    if (count > 2 && maxOffAxis <= positionEpsilon * 8.0 &&
        goalDistance < totalLength - positionEpsilon) {
        const RrVec3d bend = planeUp;
        double distance = 0.0;
        const double amplitude = std::max(
            totalLength * 0.05, positionEpsilon * 16.0);
        for (size_t i = 1; i + 1 < count; ++i) {
            distance += lengths[i - 1];
            positions[i] += bend *
                (amplitude *
                 std::sin(_RrPi * distance / totalLength));
        }
    }

    std::vector<RrVec3d> fallback(lengths.size());
    for (size_t i = 0; i < lengths.size(); ++i) {
        RrVec3d direction = positions[i + 1] - positions[i];
        direction -= planeNormal * RrDot(direction, planeNormal);
        fallback[i] = _RrIkDirectionOrFallback(
            direction, solveAxis, positionEpsilon, angularEpsilon);
    }

    std::vector<RrVec3d> best = positions;
    double bestError = std::numeric_limits<double>::infinity();
    const double tolerance = std::max(
        positionEpsilon * 8.0, totalLength * 1e-12);
    for (int iteration = 0; iteration < _RrFabrikIterations;
         ++iteration) {
        positions.back() = goal;

        for (size_t i = count - 1; i-- > 0;) {
            const RrVec3d direction = _RrIkDirectionOrFallback(
                positions[i] - positions[i + 1], -fallback[i],
                positionEpsilon, angularEpsilon);
            positions[i] = positions[i + 1] + direction * lengths[i];
        }

        positions[0] = root;
        for (size_t i = 0; i < lengths.size(); ++i) {
            const RrVec3d direction = _RrIkDirectionOrFallback(
                positions[i + 1] - positions[i], fallback[i],
                positionEpsilon, angularEpsilon);
            positions[i + 1] = positions[i] + direction * lengths[i];
            fallback[i] = direction;
        }

        const double error = (positions.back() - goal).GetLength();
        if (error < bestError) {
            bestError = error;
            best = positions;
        }
        if (error <= tolerance) {
            break;
        }
    }
    return best;
}

std::vector<RrPointFrame>
_RrSolveSingleChainIk(const std::vector<RrPointFrame> &currentFrames,
                      const RrPointFrame &effectorFrame,
                      const _RrSingleChainIkParams &params)
{
    if (!std::isfinite(params.weight)) {
        return _RrIkDegenerate(currentFrames);
    }
    const double weight =
        std::min(std::max(params.weight, 0.0), 1.0);
    if (weight <= 0.0) {
        return currentFrames;
    }
    if (currentFrames.size() < 2) {
        return _RrIkDegenerate(currentFrames);
    }
    if (params.mode != _RrSingleChainIkMode::RotatePlane &&
        params.mode != _RrSingleChainIkMode::SingleChain) {
        return _RrIkDegenerate(currentFrames);
    }
    if (!_RrIkVecFinite(effectorFrame.points[0])) {
        return _RrIkDegenerate(currentFrames);
    }
    if (params.mode == _RrSingleChainIkMode::RotatePlane &&
        (!_RrIkVecFinite(params.pole) ||
         !std::isfinite(params.twistDegrees))) {
        return _RrIkDegenerate(currentFrames);
    }
    for (const RrPointFrame &frame : currentFrames) {
        if (!_RrIkFrameFinite(frame)) {
            return _RrIkDegenerate(currentFrames);
        }
    }

    std::vector<RrVec3d> original(currentFrames.size());
    std::vector<double> lengths(currentFrames.size() - 1);
    double segmentScale = 0.0;
    for (size_t i = 0; i < currentFrames.size(); ++i) {
        original[i] = currentFrames[i].points[0];
        if (i + 1 < currentFrames.size()) {
            lengths[i] =
                (currentFrames[i + 1].points[0] - original[i])
                    .GetLength();
            segmentScale = std::max(segmentScale, lengths[i]);
        }
    }
    if (!std::isfinite(segmentScale) || segmentScale <= 0.0) {
        return _RrIkDegenerate(currentFrames);
    }
    const double positionEpsilon = segmentScale * 1e-10;
    constexpr double angularEpsilon = 1e-12;

    std::vector<_RrIkFrameBasis> bases(currentFrames.size());
    for (size_t i = 0; i < currentFrames.size(); ++i) {
        if (!_RrIkExtractBasis(currentFrames[i], angularEpsilon,
                               &bases[i])) {
            return _RrIkDegenerate(currentFrames);
        }
    }
    _RrIkFrameBasis effectorBasis = bases.back();
    if (params.mode == _RrSingleChainIkMode::SingleChain) {
        if (!_RrIkFrameFinite(effectorFrame) ||
            !_RrIkExtractBasis(effectorFrame, angularEpsilon,
                               &effectorBasis)) {
            return _RrIkDegenerate(currentFrames);
        }
    }
    for (double length : lengths) {
        if (!std::isfinite(length) || length <= positionEpsilon) {
            return _RrIkDegenerate(currentFrames);
        }
    }

    const std::vector<RrVec3d> solved = _RrIkSolvePositions(
        original, lengths, bases, effectorBasis,
        effectorFrame.points[0], params, positionEpsilon,
        angularEpsilon);
    if (solved.size() != currentFrames.size()) {
        return _RrIkDegenerate(currentFrames);
    }

    std::vector<_RrIkFrameBasis> solvedBases = bases;
    for (size_t i = 0; i + 1 < solvedBases.size(); ++i) {
        const RrVec3d solvedSegment = solved[i + 1] - solved[i];
        const RrVec3d aim = solvedSegment / solvedSegment.GetLength();
        solvedBases[i] = _RrIkAimedBasis(bases[i], aim, angularEpsilon);
    }
    if (params.mode == _RrSingleChainIkMode::SingleChain) {
        solvedBases.back() = effectorBasis;
    } else {
        const RrVec3d currentTerminalSegment =
            original.back() - original[original.size() - 2];
        const RrVec3d currentTerminal =
            currentTerminalSegment / currentTerminalSegment.GetLength();
        const RrVec3d solvedTerminalSegment =
            solved.back() - solved[solved.size() - 2];
        const RrVec3d solvedTerminal =
            solvedTerminalSegment / solvedTerminalSegment.GetLength();
        solvedBases.back() = _RrIkTransportedBasis(
            bases.back(), currentTerminal, solvedTerminal,
            angularEpsilon);
    }

    std::vector<RrPointFrame> result(currentFrames.size());
    if (weight >= 1.0) {
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] = _RrIkFrameFromBasis(
                bases[i], solved[i], solvedBases[i].x, solvedBases[i].up);
        }
    } else {
        std::vector<RrVec3d> blended(currentFrames.size());
        blended[0] = original[0];
        for (size_t i = 0; i < lengths.size(); ++i) {
            const RrVec3d currentDirection =
                (original[i + 1] - original[i]) / lengths[i];
            const RrVec3d solvedSegment = solved[i + 1] - solved[i];
            const RrVec3d solvedDirection =
                solvedSegment / solvedSegment.GetLength();
            const RrVec3d direction = _RrIkBlendDirection(
                currentDirection, solvedDirection, weight, bases[i].up,
                angularEpsilon);
            blended[i + 1] = blended[i] + direction * lengths[i];
        }
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] = _RrIkBlendEndFrame(
                bases[i], solvedBases[i], blended[i], weight,
                angularEpsilon);
        }
    }

    for (const RrPointFrame &frame : result) {
        if (!_RrIkFrameFinite(frame)) {
            return _RrIkDegenerate(currentFrames);
        }
    }
    return result;
}

// RigExecPrepareRestDerivedIkChain (rigEvaluator.cpp): rest segment
// lengths laid along the current chain's directions.
bool
_RrPrepareRestDerivedIkChain(
    const std::vector<RrPointFrame> &current,
    const std::vector<RrPointFrame> &rest,
    std::vector<RrPointFrame> *prepared)
{
    if (current.size() != rest.size() || current.empty()) {
        return false;
    }
    bool usableRestLayout = true;
    for (size_t i = 1; i < rest.size(); ++i) {
        const double segmentLength =
            (rest[i].points[0] - rest[i - 1].points[0]).GetLength();
        if (!std::isfinite(segmentLength) || segmentLength <= 0.0) {
            usableRestLayout = false;
            break;
        }
    }
    const std::vector<RrPointFrame> &lengthReference =
        usableRestLayout ? rest : current;
    prepared->clear();
    prepared->reserve(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
        if (!_RrConstraintFrameUsable(current[i]) ||
            !_RrConstraintFrameUsable(rest[i])) {
            return false;
        }
        RrVec3d origin = current[i].points[0];
        if (i > 0) {
            RrMat4d parentRest = _RrIdentity();
            RrMat4d parentPrepared = _RrIdentity();
            if (!RrPointsToMatrix(RrIdentityLandmarks(),
                                  lengthReference[i - 1].points,
                                  &parentRest) ||
                !RrPointsToMatrix(RrIdentityLandmarks(),
                                  prepared->back().points,
                                  &parentPrepared)) {
                return false;
            }
            origin = parentPrepared.TransformAffine(
                parentRest.GetInverse().TransformAffine(
                    lengthReference[i].points[0]));
        }

        RrPointFrame frame = current[i];
        frame.points[0] = origin;
        for (size_t axis = 1; axis < frame.points.size(); ++axis) {
            RrVec3d direction =
                current[i].points[axis] - current[i].points[0];
            const double directionLength = direction.GetLength();
            const double length =
                (lengthReference[i].points[axis] -
                 lengthReference[i].points[0]).GetLength();
            if (!std::isfinite(length) || length <= 0.0 ||
                !std::isfinite(directionLength) ||
                directionLength <= 0.0) {
                return false;
            }
            direction /= directionLength;
            frame.points[axis] = origin + direction * length;
        }
        if (!_RrConstraintFrameUsable(frame)) {
            return false;
        }
        prepared->push_back(frame);
    }
    return true;
}

// ---------------------------------------------------------------------------
// The Constraint step (bakedPose.cpp).
// ---------------------------------------------------------------------------

// RigExecWeightPacket::ResolveAll (types.cpp), over an RrWeightPacket.
bool
_RrResolveWeightPacketAll(const RrProgram *program,
                          const RrWeightPacket &packet, size_t count,
                          std::vector<float> *resolved)
{
    if (!resolved || !packet.valid) {
        return false;
    }
    if (packet.rangePolicy != 0 &&
        !program->TokenEquals(packet.rangePolicy, "strict") &&
        !program->TokenEquals(packet.rangePolicy, "clamp")) {
        return false;
    }
    const bool isConstant =
        program->TokenEquals(packet.representation, "constant");
    const bool isDense =
        program->TokenEquals(packet.representation, "dense");
    const bool isSparse =
        program->TokenEquals(packet.representation, "sparse");
    if (isConstant) {
        if (!packet.values.empty() || !packet.indices.empty()) {
            return false;
        }
    } else if (isDense) {
        if (!packet.indices.empty() || packet.values.size() != count) {
            return false;
        }
    } else if (isSparse) {
        if (packet.indices.size() != packet.values.size()) {
            return false;
        }
        for (size_t i = 0; i < packet.indices.size(); ++i) {
            if (packet.indices[i] < 0 ||
                size_t(packet.indices[i]) >= count ||
                (i > 0 &&
                 packet.indices[i] <= packet.indices[i - 1])) {
                return false;
            }
        }
    } else {
        return false;
    }

    const auto usable = [](float v) {
        return std::isfinite(v) && v >= 0.0f && v <= 1.0f;
    };

    if (isConstant) {
        if (!usable(packet.defaultWeight)) {
            return false;
        }
        resolved->assign(count, packet.defaultWeight);
        return true;
    }

    if (isDense) {
        for (size_t i = 0; i < count; ++i) {
            if (!usable(packet.values[i])) {
                return false;
            }
        }
        resolved->assign(packet.values.begin(),
                         packet.values.begin() + count);
        return true;
    }

    if (packet.indices.size() < count && !usable(packet.defaultWeight)) {
        return false;
    }
    for (const float value : packet.values) {
        if (!usable(value)) {
            return false;
        }
    }
    std::vector<float> valuesOut(count, packet.defaultWeight);
    for (size_t i = 0; i < packet.indices.size(); ++i) {
        valuesOut[size_t(packet.indices[i])] = packet.values[i];
    }
    resolved->swap(valuesOut);
    return true;
}

// Whether the wire weight object names a type the oracle understands
// (_IsWeightObjectType: static, dynamic, curvenet, combine, and the
// three volumetric kinds).
bool
_RrIsWeightObjectType(const RrProgram *program, uint32_t type)
{
    return program->TokenEquals(type, "RigExecStaticWeight") ||
           program->TokenEquals(type, "RigExecDynamicWeight") ||
           program->TokenEquals(type, "RigExecCurvenetWeight") ||
           program->TokenEquals(type, "RigExecCombineWeight") ||
           program->TokenEquals(type, "RigExecSphereWeight") ||
           program->TokenEquals(type, "RigExecPlaneWeight") ||
           program->TokenEquals(type, "RigExecCurveWeight");
}

// RigExecApplyRevisedAncestorDelta (rigEvaluator.cpp): the native
// source rides the delta of the deepest revised ancestor above it.
// Provider paths are never the root, where slash-counting and
// SdfPath's element count agree everywhere else.
bool
_RrApplyRevisedAncestorDelta(
    const RrProgram *program, const std::string &xformPath,
    const std::vector<RigExecWireAncestorRead> &ancestors,
    RrPointFrame *frame)
{
    const RrStore &store = program->store;
    const auto elementCount = [](const std::string &path) {
        return size_t(std::count(path.begin(), path.end(), '/'));
    };
    std::string closest;
    RrPointFrame closestBase, closestCurrent;
    for (const RigExecWireAncestorRead &a : ancestors) {
        if (a.slot < 0 ||
            size_t(a.slot) >= program->slotMeta->paths.size() ||
            size_t(a.base) >= store.base.size() ||
            size_t(a.fin) >= store.fin.size()) {
            return false;
        }
        const std::string provider =
            program->TextOrEmpty(program->slotMeta->paths[size_t(a.slot)]);
        const RrPointFrame &base = store.base[size_t(a.base)];
        const RrPointFrame &current = store.fin[size_t(a.fin)];
        if (provider == xformPath ||
            xformPath.size() <= provider.size() ||
            xformPath.compare(0, provider.size(), provider) != 0 ||
            xformPath[provider.size()] != '/' ||
            current.points == base.points) {
            continue;
        }
        if (closest.empty() ||
            elementCount(provider) > elementCount(closest)) {
            closest = provider;
            closestBase = base;
            closestCurrent = current;
        }
    }
    if (!closest.empty()) {
        RrMat4d delta = _RrIdentity();
        if (!RrPointsToMatrix(closestBase.points, closestCurrent.points,
                              &delta)) {
            return false;
        }
        *frame = RrMatrixToPoints(frame->points, delta);
    }
    return frame->IsValid();
}

bool
_RrRunConstraintStep(RrProgram *program, size_t step,
                     std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->commits.size() ||
        size_t(wire.object) >= store.commits.size() ||
        size_t(wire.object) >= program->poses->walkSteps.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    const RigExecWireCommit &wireCommit =
        program->poses->commits[size_t(wire.object)];
    RrCommitScratch &commit = store.commits[size_t(wire.object)];
    const int walkIndex =
        program->poses->walkSteps[size_t(wire.object)].index;
    if (walkIndex < 0 ||
        size_t(walkIndex) >= program->poses->constraints.size() ||
        size_t(walkIndex) >= scratch->weightScratch.size() ||
        size_t(walkIndex) >= scratch->weightError.size() ||
        size_t(walkIndex) >= scratch->constraintWeights.size() ||
        size_t(walkIndex) >= scratch->constraintHaveWeight.size()) {
        if (error) {
            *error = _RrStepHead(program, step) +
                     " names no constraint";
        }
        return false;
    }
    const size_t ci = size_t(walkIndex);
    const RigExecWireConstraint &c = program->poses->constraints[ci];
    const std::string cpath = program->TextOrEmpty(c.path);
    RrStepOutput &output = store.stepOutputs[step];
    commit.abandoned = true;
    std::fill(commit.present.begin(), commit.present.end(), 0);

    const auto finish = [&]() {
        if (wireCommit.split) {
            return true;
        }
        if (!commit.abandoned) {
            if (!_RrComputeCommitDeltas(program, step, wireCommit,
                                        &commit, error) ||
                !_RrStageCommitPairs(program, step, wireCommit, &commit,
                                     0, wireCommit.propagate.size(),
                                     error)) {
                return false;
            }
        }
        return _RrFinishCommit(program, step, size_t(wire.object),
                               error);
    };

    const int deltaBase = c.deltaBase;
    if (size_t(wire.object) >= scratch->recordAfter.size() ||
        size_t(wire.object) >= scratch->recordEveryTarget.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    scratch->recordAfter[size_t(wire.object)] = 1;
    scratch->recordEveryTarget[size_t(wire.object)] = 1;
    if (deltaBase >= 0) {
        if (size_t(deltaBase) >= scratch->deltaPresent.size() ||
            size_t(deltaBase) >= store.deltaPresent.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no delta path";
            }
            return false;
        }
        scratch->deltaPresent[size_t(deltaBase)] = 0;
        // Published for the geometry fold (the store pair mirrors the
        // baked B.deltaValues/deltaPresent the fold consumes).
        store.deltaPresent[size_t(deltaBase)] = 0;
    }
    if (c.target < 0) {
        return finish();
    }
    if (!program->ReadConstraint(ci, RrConstraintEnabled).boolean) {
        return finish();
    }
    // The envelope, in the dynamic path's THREE exclusive arms. The
    // oracle call resolves into this constraint's own scratch; the
    // runtime answers it from the weight packets the weights family
    // resolved, falling back to the captured envelope when the
    // weights family is masked out of the run.
    double weight = 1.0;
    if (c.weightObject != 0 && c.pointsTarget == 0) {
        std::vector<float> &envelope = scratch->weightScratch[ci];
        std::string &envelopeError = scratch->weightError[ci];
        envelope.clear();
        envelopeError.clear();
        envelope.assign(1, 1.0f);
        bool resolved = false;
        const auto found = scratch->weightIndex.find(c.weightObject);
        if (found != scratch->weightIndex.end() &&
            found->second < store.weightPackets.size() &&
            !store.weightPackets.empty()) {
            envelope.clear();
            resolved = _RrResolveWeightPacketAll(
                program, store.weightPackets[found->second], 1,
                &envelope);
        }
        if (!resolved && scratch->constraintHaveWeight[ci]) {
            envelope.assign(
                1, scratch->constraintWeights[ci]);
            resolved = true;
        }
        if (!resolved || envelope.size() != 1) {
            const std::string wpath =
                program->TextOrEmpty(c.weightObject);
            if (found == scratch->weightIndex.end()) {
                envelopeError = "missing weight object " + wpath;
            } else {
                const uint32_t type =
                    program->geometry
                        ->weightObjects[found->second]
                        .type;
                if (!_RrIsWeightObjectType(program, type)) {
                    envelopeError = "unknown weight object type " +
                                    program->TextOrEmpty(type) + " on " +
                                    wpath;
                } else {
                    envelopeError =
                        "could not resolve weights on " + wpath;
                }
            }
            output.diagnostics.push_back(
                cpath + ": " + envelopeError +
                "; constraint passed through");
            return finish();
        }
        weight = envelope[0];
    } else if (c.weightObject == 0) {
        weight = double(program->ReadConstraint(ci,
                                                RrConstraintDefaultWeight)
                            .f32);
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
            output.diagnostics.push_back(
                cpath +
                " has inputs:defaultWeight outside finite [0, 1]; "
                "constraint passed through");
            return finish();
        }
    }
    if (weight <= 0.0 && (c.pointsTarget == 0 || c.weightObject == 0)) {
        if (deltaBase >= 0) {
            if (size_t(deltaBase) >= scratch->deltaValues.size() ||
                size_t(deltaBase) >= store.deltaValues.size() ||
                size_t(deltaBase) >= store.deltaPresent.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no delta path";
                }
                return false;
            }
            scratch->deltaValues[size_t(deltaBase)] = _RrIdentity();
            scratch->deltaPresent[size_t(deltaBase)] = 1;
            store.deltaValues[size_t(deltaBase)] = _RrIdentity();
            store.deltaPresent[size_t(deltaBase)] = 1;
        }
        return finish();
    }

    const auto resolveSource =
        [&](int slot, uint32_t finRead, int native,
            const std::vector<RigExecWireAncestorRead> &ancestors,
            RrPointFrame *out) {
            if (slot >= 0) {
                if (size_t(finRead) >= store.fin.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no fin version";
                    }
                    return false;
                }
                *out = store.fin[size_t(finRead)];
                return out->IsValid();
            }
            if (native < 0 ||
                size_t(native) >= store.nativeFrameOk.size() ||
                size_t(native) >= store.nativeFrames.size() ||
                size_t(native) >= program->poses->nativeSources.size() ||
                !store.nativeFrameOk[size_t(native)]) {
                return false;
            }
            *out = store.nativeFrames[size_t(native)];
            if (!out->IsValid()) {
                return false;
            }
            return _RrApplyRevisedAncestorDelta(
                program,
                program->TextOrEmpty(
                    program->poses->nativeSources[size_t(native)].path),
                ancestors, out);
        };

    if (size_t(c.arrays) >= store.arrays.size()) {
        if (error) {
            *error = _RrStepHead(program, step) +
                     " names no constraint arrays";
        }
        return false;
    }
    const RrConstraintArraysLive &arrays = store.arrays[size_t(c.arrays)];

    if (c.singleChainIk) {
        if (wireCommit.targetReads.size() != c.targetSlots.size() ||
            commit.ikChain.size() != c.targetSlots.size() ||
            commit.ikRest.size() != c.targetSlots.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no chain target";
            }
            return false;
        }
        bool inputsValid = true;
        for (size_t k = 0; k < c.targetSlots.size(); ++k) {
            if (size_t(wireCommit.targetReads[k]) >= store.fin.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no fin version";
                }
                return false;
            }
            commit.ikChain[k] =
                store.fin[size_t(wireCommit.targetReads[k])];
        }
        RrPointFrame effector;
        if (!resolveSource(c.effector, wireCommit.effectorRead,
                           c.effectorNative,
                           wireCommit.effectorAncestors, &effector)) {
            output.diagnostics.push_back(
                cpath + " could not resolve its effector; constraint "
                "passed through");
            inputsValid = false;
        }
        _RrSingleChainIkParams params;
        params.mode = _RrSingleChainIkMode(c.ikMode);
        params.weight = weight;
        if (c.ikMode == 0) {
            params.pole = program->ReadConstraint(ci,
                                                   RrConstraintPoleVector)
                              .vec;
            params.twistDegrees =
                program->ReadConstraint(ci,
                                        RrConstraintTwistDegrees)
                    .f64;
        }
        if (inputsValid && c.ikMode == 0 && c.poleModeObject) {
            if (c.poleObjects.empty()) {
                output.diagnostics.push_back(
                    cpath + " uses object pole mode with no pole-vector "
                    "objects; constraint passed through");
                inputsValid = false;
            }
            if (inputsValid && !arrays.poleOk) {
                inputsValid = false;
            }
            RrVec3d polePoint(0.0);
            double total = 0;
            for (size_t k = 0;
                 inputsValid && k < c.poleObjects.size(); ++k) {
                if (k >= wireCommit.poleReads.size() ||
                    k >= wireCommit.poleAncestors.size() ||
                    k >= c.poleObjectNatives.size() ||
                    k >= arrays.poleWeights.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no pole object";
                    }
                    return false;
                }
                RrPointFrame poleFrame;
                if (!resolveSource(
                        c.poleObjects[k], wireCommit.poleReads[k],
                        c.poleObjectNatives[k],
                        wireCommit.poleAncestors[k], &poleFrame) ||
                    !std::isfinite(arrays.poleWeights[k]) ||
                    arrays.poleWeights[k] < 0) {
                    output.diagnostics.push_back(
                        cpath + " has an invalid pole-vector source or "
                        "weight; constraint passed through");
                    inputsValid = false;
                    break;
                }
                polePoint +=
                    poleFrame.points[0] * arrays.poleWeights[k];
                total += arrays.poleWeights[k];
            }
            if (inputsValid && total <= 0) {
                output.diagnostics.push_back(
                    cpath + " has zero total pole-vector weight; "
                    "constraint passed through");
                inputsValid = false;
            }
            if (inputsValid) {
                params.pole = polePoint / total;
            }
        }
        const std::vector<RrPointFrame> *solveChain = &commit.ikChain;
        if (inputsValid && !c.useAnimatedTs) {
            for (size_t k = 0; k < c.targetSlots.size(); ++k) {
                if (c.targetSlots[k] < 0 ||
                    size_t(c.targetSlots[k]) >=
                        scratch->restFrames.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no rest slot";
                    }
                    return false;
                }
                commit.ikRest[k] =
                    (k < c.ikRestLive.size() && c.ikRestLive[k] &&
                     k < commit.ikChain.size())
                        ? commit.ikChain[k]
                        : scratch->restFrames[size_t(c.targetSlots[k])];
            }
            if (!_RrPrepareRestDerivedIkChain(commit.ikChain,
                                              commit.ikRest,
                                              &commit.ikPrepared)) {
                output.diagnostics.push_back(
                    cpath + " could not prepare rest-derived IK inputs; "
                    "constraint passed through");
                inputsValid = false;
            } else {
                solveChain = &commit.ikPrepared;
            }
        }
        commit.ikSolved.clear();
        if (inputsValid) {
            commit.ikSolved =
                _RrSolveSingleChainIk(*solveChain, effector, params);
        }
        if (inputsValid &&
            (commit.ikSolved.size() != c.targetSlots.size() ||
             std::any_of(commit.ikSolved.begin(), commit.ikSolved.end(),
                         [](const RrPointFrame &frame) {
                             return !RrFrameUsable(frame);
                         }))) {
            output.diagnostics.push_back(
                cpath + " failed to solve its joint chain; constraint "
                "passed through atomically");
            inputsValid = false;
        }
        if (inputsValid) {
            for (size_t k = 0; k < c.targetSlots.size(); ++k) {
                const auto found = std::lower_bound(
                    wireCommit.slots.begin(), wireCommit.slots.end(),
                    c.targetSlots[k]);
                if (found == wireCommit.slots.end() ||
                    *found != c.targetSlots[k]) {
                    continue;
                }
                const size_t pos = size_t(found - wireCommit.slots.begin());
                if (pos >= commit.frames.size() ||
                    pos >= commit.present.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no candidate slot";
                    }
                    return false;
                }
                commit.frames[pos] = commit.ikSolved[k];
                commit.present[pos] = 1;
            }
            commit.abandoned = false;
        }
        return finish();
    }

    scratch->recordEveryTarget[size_t(wire.object)] = 0;

    bool sourcesReady = arrays.ok;
    for (size_t k = 0; sourcesReady && k < c.sources.size(); ++k) {
        if (k >= wireCommit.sourceReads.size() ||
            k >= wireCommit.sourceAncestors.size() ||
            k >= c.sourceNatives.size() ||
            k >= c.sourcePaths.size() ||
            k >= commit.sources.size() ||
            k >= arrays.weights.size() ||
            k >= arrays.translationOffsets.size() ||
            k >= arrays.rotationOffsets.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no constraint source";
            }
            return false;
        }
        RrPointFrame frame;
        if (!resolveSource(c.sources[k], wireCommit.sourceReads[k],
                           c.sourceNatives[k],
                           wireCommit.sourceAncestors[k], &frame)) {
            output.diagnostics.push_back(
                cpath + " could not resolve source " +
                program->TextOrEmpty(c.sourcePaths[k]));
            sourcesReady = false;
            break;
        }
        commit.sources[k].frame = frame;
        commit.sources[k].normalizedWeight = arrays.weights[k];
        commit.sources[k].translationOffset =
            arrays.translationOffsets[k];
        commit.sources[k].rotationOffsetDegrees =
            arrays.rotationOffsets[k];
    }
    if (!sourcesReady) {
        output.diagnostics.push_back(
            cpath + " has unusable constraint inputs; constraint passed "
            "through");
        return finish();
    }
    scratch->recordAfter[size_t(wire.object)] = deltaBase < 0 ? 1 : 0;
    const double solveWeight = deltaBase < 0 ? weight : 1.0;
    if (size_t(wireCommit.targetRead) >= store.fin.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no fin version";
        }
        return false;
    }
    const std::vector<RrConstraintSource> &scratchSources =
        commit.sources;
    std::vector<_RrConstraintSource> sources;
    sources.reserve(scratchSources.size());
    for (const RrConstraintSource &s : scratchSources) {
        _RrConstraintSource out;
        out.frame = s.frame;
        out.normalizedWeight = s.normalizedWeight;
        out.translationOffset = s.translationOffset;
        out.rotationOffsetDegrees = s.rotationOffsetDegrees;
        sources.push_back(out);
    }
    const RrPointFrame input =
        store.fin[size_t(wireCommit.targetRead)];
    RrPointFrame candidate = input;
    bool candidateReady = true;
    _RrConstraintAxisMask affect;
    affect.x = program->ReadConstraint(ci, RrConstraintAffectX).boolean;
    affect.y = program->ReadConstraint(ci, RrConstraintAffectY).boolean;
    affect.z = program->ReadConstraint(ci, RrConstraintAffectZ).boolean;
    const std::string ctype = program->TextOrEmpty(c.type);
    const _RrEulerOrder order = _RrEulerOrder(c.order);
    if (ctype == "RigExecPositionConstraint") {
        _RrPositionConstraintParams params;
        params.offset =
            program->ReadConstraint(ci, RrConstraintOffset).vec;
        params.affect = affect;
        params.weight = solveWeight;
        candidate = _RrApplyPositionConstraint(input, sources, params);
    } else if (ctype == "RigExecRotationConstraint") {
        _RrRotationConstraintParams params;
        params.offsetDegrees =
            program->ReadConstraint(ci, RrConstraintOffset).vec;
        params.affect = affect;
        params.rotationOrder = order;
        params.weight = solveWeight;
        candidate = _RrApplyRotationConstraint(input, sources, params);
    } else if (ctype == "RigExecScaleConstraint") {
        _RrScaleConstraintParams params;
        params.offset =
            program->ReadConstraint(ci, RrConstraintOffset).vec;
        params.affect = affect;
        params.weight = solveWeight;
        candidate = _RrApplyScaleConstraint(input, sources, params);
    } else if (ctype == "RigExecParentConstraint") {
        _RrParentConstraintParams params;
        params.translationAxes.x =
            program->ReadConstraint(ci, RrConstraintTX).boolean;
        params.translationAxes.y =
            program->ReadConstraint(ci, RrConstraintTY).boolean;
        params.translationAxes.z =
            program->ReadConstraint(ci, RrConstraintTZ).boolean;
        params.rotationAxes.x =
            program->ReadConstraint(ci, RrConstraintRX).boolean;
        params.rotationAxes.y =
            program->ReadConstraint(ci, RrConstraintRY).boolean;
        params.rotationAxes.z =
            program->ReadConstraint(ci, RrConstraintRZ).boolean;
        params.scaleAxes.x =
            program->ReadConstraint(ci, RrConstraintSX).boolean;
        params.scaleAxes.y =
            program->ReadConstraint(ci, RrConstraintSY).boolean;
        params.scaleAxes.z =
            program->ReadConstraint(ci, RrConstraintSZ).boolean;
        params.rotationOrder = order;
        params.weight = solveWeight;
        candidate = _RrApplyParentConstraint(input, sources, params);
    } else {
        RrVec3d target(0.0);
        double total = 0;
        for (const _RrConstraintSource &source : sources) {
            if (!std::isfinite(source.normalizedWeight) ||
                source.normalizedWeight < 0) {
                output.diagnostics.push_back(
                    cpath + " has an invalid source weight; constraint "
                    "passed through");
                candidateReady = false;
                break;
            }
            target += source.frame.points[0] * source.normalizedWeight;
            total += source.normalizedWeight;
        }
        if (candidateReady && total > 0) {
            target /= total;
            _RrAimConstraintParams params;
            params.localAimVector =
                c.aimVectorAuthored
                    ? program->ReadConstraint(ci,
                                              RrConstraintAimVector).vec
                    : RrVec3d(c.aimAxisFallback[0], c.aimAxisFallback[1],
                              c.aimAxisFallback[2]);
            params.localUpVector =
                program->ReadConstraint(ci, RrConstraintUpVector).vec;
            params.rotationOffsetDegrees =
                program->ReadConstraint(ci,
                                        RrConstraintRotationOffset).vec;
            params.affectRotation = affect;
            params.rotationOrder = order;
            params.weight = solveWeight;
            params.preserveInputUp = c.preserveInputUp;
            const RrVec3d authoredWorldUp =
                program->ReadConstraint(ci,
                                        RrConstraintWorldUpVector).vec;
            const RrVec3d sceneUp(c.sceneUp[0], c.sceneUp[1],
                                  c.sceneUp[2]);
            RrVec3d worldUpStorage(0.0);
            bool haveWorldUp = false;
            const std::string worldUpType =
                program->TextOrEmpty(c.worldUpType);
            if (worldUpType == "sceneUp") {
                worldUpStorage = sceneUp;
                haveWorldUp = true;
            } else if (worldUpType == "vector") {
                worldUpStorage = authoredWorldUp;
                haveWorldUp = true;
            } else if (worldUpType == "objectUp") {
                RrPointFrame upObject;
                if (!c.worldUpObjectNamed) {
                    worldUpStorage = -input.points[0];
                    haveWorldUp = true;
                } else if (!resolveSource(c.worldUpObject,
                                          wireCommit.worldUpRead,
                                          c.worldUpNative,
                                          wireCommit.worldUpAncestors,
                                          &upObject)) {
                    output.diagnostics.push_back(
                        cpath + " could not resolve its world-up object; "
                        "constraint passed through");
                    candidateReady = false;
                } else {
                    worldUpStorage =
                        upObject.points[0] - input.points[0];
                    haveWorldUp = true;
                }
            } else if (worldUpType == "objectRotationUp") {
                if (!c.worldUpObjectNamed) {
                    worldUpStorage = authoredWorldUp;
                    haveWorldUp = true;
                } else {
                    RrPointFrame upObject;
                    if (!resolveSource(c.worldUpObject,
                                       wireCommit.worldUpRead,
                                       c.worldUpNative,
                                       wireCommit.worldUpAncestors,
                                       &upObject)) {
                        output.diagnostics.push_back(
                            cpath +
                            " could not resolve its world-up object; "
                            "constraint passed through");
                        candidateReady = false;
                    }
                    RrMat4d up = _RrIdentity();
                    if (candidateReady &&
                        !RrPointsToMatrix(RrIdentityLandmarks(),
                                          upObject.points, &up)) {
                        output.diagnostics.push_back(
                            cpath + " has a degenerate world-up object");
                        candidateReady = false;
                    } else if (candidateReady) {
                        worldUpStorage = up.ExtractRotation().TransformDir(
                            authoredWorldUp);
                        haveWorldUp = true;
                    }
                }
            }
            if (haveWorldUp) {
                params.worldUpDirection = &worldUpStorage;
            }
            if (candidateReady) {
                candidate =
                    _RrApplyAimConstraint(input, target, params);
            }
        }
    }
    if (candidateReady && deltaBase >= 0) {
        if (size_t(deltaBase) >= scratch->deltaValues.size() ||
            size_t(deltaBase) >= scratch->deltaPresent.size() ||
            size_t(deltaBase) >= store.deltaValues.size() ||
            size_t(deltaBase) >= store.deltaPresent.size() ||
            size_t(deltaBase) >= store.deltaBaseMatrix.size() ||
            size_t(deltaBase) >= store.deltaBaseOk.size() ||
            size_t(deltaBase) >=
                program->geometry->deltaBasePaths.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no delta path";
            }
            return false;
        }
        const RrMat4d &baseMatrix =
            store.deltaBaseMatrix[size_t(deltaBase)];
        RrMat4d solvedMatrix = _RrIdentity();
        if (RrFrameUsable(candidate) &&
            store.deltaBaseOk[size_t(deltaBase)] &&
            std::isfinite(baseMatrix.GetDeterminant()) &&
            baseMatrix.GetDeterminant() != 0.0 &&
            RrPointsToMatrix(RrIdentityLandmarks(), candidate.points,
                             &solvedMatrix)) {
            scratch->deltaValues[size_t(deltaBase)] =
                solvedMatrix * baseMatrix.GetInverse();
            scratch->deltaPresent[size_t(deltaBase)] = 1;
            store.deltaValues[size_t(deltaBase)] =
                solvedMatrix * baseMatrix.GetInverse();
            store.deltaPresent[size_t(deltaBase)] = 1;
        } else {
            output.diagnostics.push_back(
                cpath + " could not measure its delta against " +
                program->TextOrEmpty(
                    program->geometry->deltaBasePaths[size_t(deltaBase)]) +
                "; constraint passed through");
        }
    } else if (candidateReady) {
        if (!RrFrameUsable(candidate)) {
            if (c.target < 0 ||
                size_t(c.target) >= program->slotMeta->paths.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no target slot";
                }
                return false;
            }
            output.diagnostics.push_back(
                cpath + " produced an invalid or degenerate frame for " +
                program->TextOrEmpty(
                    program->slotMeta->paths[size_t(c.target)]) +
                "; constraint passed through");
        } else {
            if (commit.frames.empty() || commit.present.empty()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no candidate slot";
                }
                return false;
            }
            commit.frames[0] = candidate;
            commit.present[0] = 1;
            commit.abandoned = false;
        }
    }
    return finish();
}

}  // namespace _RrPoseSteps
}  // namespace rigExec
