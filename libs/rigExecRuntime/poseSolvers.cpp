// Runtime aggregate solvers: FK, two-bone IK, spline IK, ribbon, and twist.

#include "poseInternal.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

// Solver kernels (rigExecMath/solvers.cpp, splineIk.cpp): Gf -> Rr, the
// arithmetic untouched.

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

// RigExecTwoBoneIkLengths: the chain's bone lengths measured in \p space,
// each authored offset riding its own bone's factor.
void
_RrTwoBoneIkLengths(const std::array<std::array<RrVec3d, 4>, 3> &restPoints,
                    const RrMat4d &space, double upperOffset,
                    double lowerOffset, double *upperLength,
                    double *lowerLength)
{
    const double rawUpper =
        (restPoints[1][0] - restPoints[0][0]).GetLength();
    const double rawLower =
        (restPoints[2][0] - restPoints[1][0]).GetLength();
    const RrVec3d s0 = space.Transform(restPoints[0][0]);
    const RrVec3d s1 = space.Transform(restPoints[1][0]);
    const RrVec3d s2 = space.Transform(restPoints[2][0]);
    const double spacedUpper = (s1 - s0).GetLength();
    const double spacedLower = (s2 - s1).GetLength();
    if (upperLength) {
        const double f = rawUpper > 1e-12 ? spacedUpper / rawUpper : 1.0;
        *upperLength = spacedUpper + upperOffset * f;
    }
    if (lowerLength) {
        const double f = rawLower > 1e-12 ? spacedLower / rawLower : 1.0;
        *lowerLength = spacedLower + lowerOffset * f;
    }
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

    double reach = dist;
    double s1 = l1, s2 = l2;
    if (params.softDistancePolicy) {
        // RigExecSolveTwoBoneIk's limb model, through the shared kernel.
        rigExec::RigExecLimbSegmentLengths(
            l1, l2, dist, (pole - root).GetLength(),
            (goal - pole).GetLength(), params.limb, &s1, &s2);
        s1 = std::max(s1, 1e-9);
        s2 = std::max(s2, 1e-9);
        reach = std::min(dist, s1 + s2);
    } else {
    const double soft = std::max(params.softness, 0.0) * chain;
    if (soft > 1e-12 && dist > chain - soft) {
        reach = chain - soft * std::exp(-(dist - (chain - soft)) / soft);
    } else if (dist > chain) {
        reach = chain;
    }

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
    if (params.twistRadians != 0.0 && std::isfinite(params.twistRadians)) {
        bendUp = _RrRotate(bendUp, aim, params.twistRadians);
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
    if (params.scaleSegments) {
        rigExec::RigExecScaleFrameAlong(out[0].points.data(), upperDir,
                                        s1 / l1);
        rigExec::RigExecScaleFrameAlong(out[1].points.data(), lowerDir,
                                        (endPos - mid).GetLength() / l2);
    }

    // The space carries into the root and mid frames' axis handles, which
    // is the scale a child that is not solver-posed inherits. The effector
    // frame copies a control, which already carries its scale.
    if (params.space != RrMat4d(1.0)) {
        for (size_t f = 0; f < 2; ++f) {
            const RrVec3d origin = out[f].points[0];
            for (size_t a = 1; a < 4; ++a) {
                const RrVec3d handle = out[f].points[a] - origin;
                const double len = handle.GetLength();
                if (len < 1e-12) {
                    continue;
                }
                const RrVec3d dir = handle / len;
                const double factor =
                    params.space.TransformDir(dir).GetLength();
                out[f].points[a] = origin + dir * (len * factor);
            }
        }
    }

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
        if (wire.start >= 0) {
            if (size_t(wire.start) >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no rest slot";
                }
                return false;
            }
            s.startRest = scratch->restPts[size_t(wire.start)];
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
        // The constant arm's lengths, measured in the folded spaceMatrix as
        // the baked rest refresh measures them.
        _RrTwoBoneIkLengths(s.ikRests, RrWireInputConstant(wire.ikSpace).matrix,
                            wire.upperOffset.f64, wire.lowerOffset.f64,
                            &s.ikParams.upperLength, &s.ikParams.lowerLength);
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

// Ribbon substrate (geometryKernels.cpp, solverKernels.cpp).

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

// Spline-IK solve (splineIk.cpp).

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

} // namespace

namespace runtimePoseDetail {

bool _RrRunSolveStep(RrProgram *program, size_t step,
                     std::string *error);

bool _RrRunSolverCommitStep(RrProgram *program, size_t step,
                            std::string *error);

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
        const size_t base = ws.start >= 0 ? 1 : 0;
        if (s.elements.size() != ws.controls.size() + base ||
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
        if (base) {
            s.elements[0].restPoints = s.startRest;
            const RrPointFrame *startPose = nullptr;
            if (!finAt(ws.startRead, &startPose)) {
                return false;
            }
            s.elements[0].posePoints = startPose->points;
            s.elements[0].hasOutRest = false;
            s.elements[0].parentIndex = -1;
        }
        for (size_t k = 0; k < ws.controls.size(); ++k) {
            if (k >= s.controlRests.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no solver control";
                }
                return false;
            }
            const size_t e = k + base;
            s.elements[e].restPoints = s.controlRests[k];
            const RrPointFrame *pose = nullptr;
            if (!finAt(ws.controlReads[k], &pose)) {
                return false;
            }
            s.elements[e].posePoints = pose->points;
            const bool live = jointBasis && k < ws.restIsLive.size() &&
                              ws.restIsLive[k];
            s.elements[e].hasOutRest = live;
            if (live) {
                if (k >= s.jointRests.size() ||
                    k >= aggregate.rests.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no live rest";
                    }
                    return false;
                }
                s.elements[e].outRestPoints = s.jointRests[k];
                aggregate.rests[k] = s.jointRests[k];
            }
            s.elements[e].parentIndex =
                ws.parentRelative ? int(base) - 1 : int(k) + int(base) - 1;
        }
        aggregate.frames = _RrSolveFkChain(s.elements);
        const int32_t limb =
            program->limbBySolver.empty()
                ? -1 : program->limbBySolver[size_t(wire.object)];
        if (limb >= 0 &&
            (program->poses->limbSolvers[size_t(limb)].flags & 2) != 0) {
            // RigExecScaleFkSegments: each element scales along the bone
            // to the next by posed over rest length.
            const size_t n = s.elements.size();
            std::vector<RrVec3d> dirs(n, RrVec3d(0.0));
            std::vector<double> factors(n, 1.0);
            for (size_t i = 0; i + 1 < n && i + 1 < aggregate.frames.size();
                 ++i) {
                const auto &ri = s.elements[i].hasOutRest
                                     ? s.elements[i].outRestPoints
                                     : s.elements[i].restPoints;
                const auto &rc = s.elements[i + 1].hasOutRest
                                     ? s.elements[i + 1].outRestPoints
                                     : s.elements[i + 1].restPoints;
                const double rest = rigExec::RigExecBoneLengthUnderFrame(
                    ri.data(), aggregate.frames[i].points.data(),
                    RrVec3d(rc[0] - ri[0]));
                const RrVec3d bone = aggregate.frames[i + 1].points[0] -
                                     aggregate.frames[i].points[0];
                const double posed = bone.GetLength();
                if (rest > 1e-12 && posed > 1e-12) {
                    dirs[i] = bone;
                    factors[i] = posed / rest;
                }
            }
            for (size_t i = 0; i < n && i < aggregate.frames.size(); ++i) {
                rigExec::RigExecScaleFrameAlong(
                    aggregate.frames[i].points.data(), dirs[i], factors[i]);
            }
        }
        if (base && !aggregate.frames.empty()) {
            aggregate.frames.erase(aggregate.frames.begin());
        }
    } else if (type == "RigExecTwoBoneIk") {
        RrPoseTwoBoneIkParams params = s.ikParams;
        const size_t solver = size_t(wire.object);
        // rigExec:spaceMatrix, composed after rigExec:space's rest -> pose
        // map when a space is named (spaceSlot, not spaceRead: an unnamed
        // space reads no frame).
        RrMat4d ikSpace = program->ReadSolver(solver, RrSolverIkSpace).matrix;
        bool spaceMoved = false;
        if (ws.spaceSlot >= 0) {
            const RrPointFrame *spaceFrame = nullptr;
            if (!finAt(ws.spaceRead, &spaceFrame)) {
                return false;
            }
            RrMat4d delta(1.0);
            if (RrPointsToMatrix(s.spaceRest, *spaceFrame, &delta)) {
                ikSpace = delta * ikSpace;
                spaceMoved = true;
            }
        }
        if (_RrLiveSolver(program, solver, RrSolverBend) ||
            _RrLiveSolver(program, solver, RrSolverStretch) ||
            _RrLiveSolver(program, solver, RrSolverSoftness) ||
            _RrLiveSolver(program, solver, RrSolverUpperOffset) ||
            _RrLiveSolver(program, solver, RrSolverLowerOffset) ||
            _RrLiveSolver(program, solver, RrSolverIkSpace) || spaceMoved) {
            params.preferredBendRadians =
                program->ReadSolver(solver, RrSolverBend).f64;
            params.stretch =
                double(program->ReadSolver(solver, RrSolverStretch).f32);
            params.softness =
                double(program->ReadSolver(solver, RrSolverSoftness).f32);
            params.space = ikSpace;
            _RrTwoBoneIkLengths(
                s.ikRests, ikSpace,
                program->ReadSolver(solver, RrSolverUpperOffset).f64,
                program->ReadSolver(solver, RrSolverLowerOffset).f64,
                &params.upperLength, &params.lowerLength);
        }
        const RrPointFrame *root = nullptr;
        const RrPointFrame *end = nullptr;
        const RrPointFrame *pole = nullptr;
        if (!finAt(ws.rootRead, &root) ||
            !finAt(ws.endRead, &end) || !finAt(ws.poleRead, &pole)) {
            return false;
        }
        const int32_t limb =
            program->limbBySolver.empty()
                ? -1 : program->limbBySolver[size_t(wire.object)];
        if (limb >= 0) {
            const RigExecWireLimbSolver &l =
                program->poses->limbSolvers[size_t(limb)];
            const auto value = [&](size_t which) {
                const RrInputValue v = program->ReadLimb(size_t(limb), which);
                return v.tag == RigExecWireInput::Tag::Double ? v.f64
                                                              : double(v.f32);
            };
            params.softDistancePolicy = (l.flags & 1) != 0;
            params.scaleSegments = (l.flags & 2) != 0;
            params.limb.stretch = params.stretch;
            params.limb.pin = value(0);
            params.limb.upperScale = value(1);
            params.limb.lowerScale = value(2);
            params.limb.softDistance = value(3);
            params.limb.scaleCalibration = l.scaleCalibration;
            params.twistRadians = value(4) * _RrPi / 180.0;
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
        // The rest rebuilt in rigExec:space once the space has moved
        // (bakedPose.cpp): the bind-time rest is measured at identity and
        // the placement ratio is arcLength / restArcLength, so a scaled
        // space would otherwise read as stretch. s.splineRest holds the
        // rest frames and the root, mid and end rests it was made from.
        const RrPoseSplineIkRest *restForSolve = &s.splineRest;
        RrPoseSplineIkRest spacedRest;
        if (ws.spaceSlot >= 0) {
            const RrPointFrame *spaceFrame = nullptr;
            if (!finAt(ws.spaceRead, &spaceFrame)) {
                return false;
            }
            RrMat4d delta(1.0);
            if (RrPointsToMatrix(s.spaceRest, *spaceFrame, &delta) &&
                delta != RrMat4d(1.0)) {
                std::vector<RrPointFrame> spaced;
                spaced.reserve(s.splineRest.joints.size());
                for (const RrPointFrame &f : s.splineRest.joints) {
                    spaced.push_back(_RrTransformFrame(f, delta));
                }
                spacedRest = _RrSplineIkMakeRest(
                    spaced, _RrTransformFrame(s.splineRest.rootControl, delta),
                    _RrTransformFrame(s.splineRest.midControl, delta),
                    _RrTransformFrame(s.splineRest.endControl, delta),
                    ws.splineRestWeights, ws.splineRestMode);
                restForSolve = &spacedRest;
            }
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
        _RrSolveSplineIk(*restForSolve, controls, params, &solved);
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

} // namespace runtimePoseDetail

} // namespace rigExec
