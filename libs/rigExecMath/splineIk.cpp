//
// RigExec spline-IK spine kernel.
//
#include "splineIk.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/quatd.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rigExec {
namespace {

constexpr double kEps = RigExecSplineIkEpsilon;

// Table resolution: sub-intervals per Bezier span. The speed of a
// quadratic Bezier is sqrt of a quadratic polynomial in u, smooth unless the
// curve has a cusp, so 8-point Gauss-Legendre on 1/32 of a span integrates
// it to near machine precision.
constexpr int kSubdivisionsPerSpan = 32;
constexpr int kSpans = 2;

// 8-point Gauss-Legendre nodes and weights on [-1, 1].
constexpr double kGaussNodes[8] = {
    -0.9602898564975363, -0.7966664774136267, -0.5255324099163290,
    -0.1834346424956498,  0.1834346424956498,  0.5255324099163290,
     0.7966664774136267,  0.9602898564975363,
};
constexpr double kGaussWeights[8] = {
    0.1012285362903763, 0.2223810344533745, 0.3137066458778873,
    0.3626837833783620, 0.3626837833783620, 0.3137066458778873,
    0.2223810344533745, 0.1012285362903763,
};

bool
_IsFinite(const GfVec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
_IsFinite(const RigExecPointFrame &f)
{
    for (const GfVec3d &p : f.points) {
        if (!_IsFinite(p)) {
            return false;
        }
    }
    return true;
}

// Quadratic Bezier control points of span s (0 or 1), see the header.
void
_SpanBezier(const std::array<GfVec3d, 4> &cvs, int span, GfVec3d q[3])
{
    const GfVec3d mid = (cvs[1] + cvs[2]) * 0.5;
    if (span == 0) {
        q[0] = cvs[0];
        q[1] = cvs[1];
        q[2] = mid;
    } else {
        q[0] = mid;
        q[1] = cvs[2];
        q[2] = cvs[3];
    }
}

// The orthonormal basis of a frame with its rest handle lengths and
// handedness. ok is false when the X or Y handle is collapsed or the two
// are parallel (no up direction can be recovered).
struct _Basis {
    GfVec3d origin{0.0};
    GfVec3d x{1.0, 0.0, 0.0};
    GfVec3d y{0.0, 1.0, 0.0};
    GfVec3d z{0.0, 0.0, 1.0};
    double lx = 1.0, ly = 1.0, lz = 1.0;
    double handedness = 1.0;
    bool ok = false;
};

_Basis
_MakeBasis(const RigExecPointFrame &frame)
{
    _Basis b;
    b.origin = frame.Origin();
    const GfVec3d ax = frame.X() - b.origin;
    const GfVec3d ay = frame.Y() - b.origin;
    const GfVec3d az = frame.Z() - b.origin;
    b.lx = ax.GetLength();
    b.ly = ay.GetLength();
    b.lz = az.GetLength();
    if (!_IsFinite(frame) || b.lx <= kEps || b.ly <= kEps) {
        return b;
    }
    b.x = ax / b.lx;
    GfVec3d up = ay - b.x * GfDot(b.x, ay);
    const double upLength = up.GetLength();
    if (upLength <= kEps * std::max(1.0, b.ly)) {
        return b;
    }
    b.y = up / upLength;
    const GfVec3d cross = GfCross(b.x, b.y);
    b.handedness = (GfDot(cross, az) < 0.0) ? -1.0 : 1.0;
    b.z = cross * b.handedness;
    if (b.lz <= kEps) {
        b.lz = 1.0;  // a collapsed Z handle: rebuild it at unit length
    }
    b.ok = true;
    return b;
}

// Rotates v by the minimal rotation taking unit vector `from` to unit
// vector `to` (Rodrigues with sin/cos supplied by the cross/dot products).
// Antiparallel input has no unique minimal rotation; v is returned
// unchanged, which is the rotation by pi about v itself when v is
// perpendicular to `from` (the only way it is used here).
GfVec3d
_RotateToward(const GfVec3d &from, const GfVec3d &to, const GfVec3d &v)
{
    const GfVec3d axis = GfCross(from, to);
    const double s = axis.GetLength();
    const double c = GfDot(from, to);
    if (s <= kEps) {
        return v;
    }
    const GfVec3d k = axis / s;
    return v * c + GfCross(k, v) * s + k * (GfDot(k, v) * (1.0 - c));
}

// The world axis least parallel to x, projected perpendicular to it and
// normalised. Used only when the transported up vector collapses.
GfVec3d
_FallbackUp(const GfVec3d &x)
{
    int least = 0;
    double best = std::abs(x[0]);
    for (int i = 1; i < 3; ++i) {
        if (std::abs(x[i]) < best) {
            best = std::abs(x[i]);
            least = i;
        }
    }
    GfVec3d up(0.0);
    up[least] = 1.0;
    up -= x * GfDot(x, up);
    return up.GetNormalized();
}

RigExecPointFrame
_DegenerateCopy(const RigExecPointFrame &frame)
{
    RigExecPointFrame copy = frame;
    copy.flags |= RigExecPointFrameDegenerate;
    return copy;
}

}  // namespace

// ---------------------------------------------------------------------------
// RigExecSplineIkCurve
// ---------------------------------------------------------------------------

RigExecSplineIkCurve::RigExecSplineIkCurve()
    : RigExecSplineIkCurve(std::array<GfVec3d, 4>{
          GfVec3d(0.0), GfVec3d(0.0), GfVec3d(0.0), GfVec3d(0.0)})
{
}

RigExecSplineIkCurve::RigExecSplineIkCurve(const std::array<GfVec3d, 4> &cvs)
    : _cvs(cvs)
{
    const int cells = kSpans * kSubdivisionsPerSpan;
    _u.resize(cells + 1);
    _cumulative.resize(cells + 1);
    _u[0] = 0.0;
    _cumulative[0] = 0.0;
    for (int i = 0; i < cells; ++i) {
        const double a = static_cast<double>(i) / kSubdivisionsPerSpan;
        const double b = static_cast<double>(i + 1) / kSubdivisionsPerSpan;
        const double half = 0.5 * (b - a);
        const double mid = 0.5 * (a + b);
        double sum = 0.0;
        for (int g = 0; g < 8; ++g) {
            GfVec3d d;
            Evaluate(mid + half * kGaussNodes[g], nullptr, &d);
            sum += kGaussWeights[g] * d.GetLength();
        }
        _u[i + 1] = b;
        _cumulative[i + 1] = _cumulative[i] + sum * half;
    }
    _length = _cumulative[cells];
    if (!std::isfinite(_length)) {
        _length = 0.0;
    }
}

void
RigExecSplineIkCurve::Evaluate(
    double u, GfVec3d *position, GfVec3d *derivative) const
{
    u = std::clamp(u, 0.0, static_cast<double>(kSpans));
    const int span = (u < 1.0) ? 0 : 1;
    const double t = u - span;
    GfVec3d q[3];
    _SpanBezier(_cvs, span, q);
    const double s = 1.0 - t;
    if (position) {
        *position = q[0] * (s * s) + q[1] * (2.0 * s * t) + q[2] * (t * t);
    }
    if (derivative) {
        // dB/dt = 2[(1-t)(q1-q0) + t(q2-q1)], and du = dt within a span.
        *derivative = ((q[1] - q[0]) * s + (q[2] - q[1]) * t) * 2.0;
    }
}

double
RigExecSplineIkCurve::ParamAtArcLength(double distance) const
{
    if (IsDegenerate()) {
        return 0.0;
    }
    distance = std::clamp(distance, 0.0, _length);
    // Locate the table cell: first cumulative value >= distance.
    const auto it = std::lower_bound(
        _cumulative.begin(), _cumulative.end(), distance);
    size_t hi = static_cast<size_t>(it - _cumulative.begin());
    if (hi == 0) {
        return _u[0];
    }
    if (hi >= _cumulative.size()) {
        hi = _cumulative.size() - 1;
    }
    const size_t lo = hi - 1;
    const double a = _u[lo], b = _u[hi];
    const double cellLength = _cumulative[hi] - _cumulative[lo];
    if (cellLength <= kEps * std::max(1.0, _length)) {
        return a;  // a collapsed cell: any parameter in it is the same point
    }
    const double target = distance - _cumulative[lo];

    // Partial arc length from a to u by the same quadrature as the table,
    // so the cell edges are consistent.
    auto partial = [&](double u) {
        const double half = 0.5 * (u - a);
        const double mid = 0.5 * (u + a);
        double sum = 0.0;
        for (int g = 0; g < 8; ++g) {
            GfVec3d d;
            Evaluate(mid + half * kGaussNodes[g], nullptr, &d);
            sum += kGaussWeights[g] * d.GetLength();
        }
        return sum * half;
    };

    // Safeguarded Newton: bracket [bl, bh] always contains the root.
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
        GfVec3d d;
        Evaluate(u, nullptr, &d);
        const double speed = d.GetLength();
        double next = (speed > kEps) ? u - g / speed : 0.5 * (bl + bh);
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

bool
RigExecSplineIkCurve::PointAtArcLength(
    double distance, GfVec3d *position, GfVec3d *tangent) const
{
    if (IsDegenerate()) {
        if (position) {
            *position = _cvs[0];
        }
        if (tangent) {
            *tangent = GfVec3d(0.0);
        }
        return false;
    }
    const double clamped = std::clamp(distance, 0.0, _length);
    const double u = ParamAtArcLength(clamped);
    GfVec3d p, d;
    Evaluate(u, &p, &d);
    GfVec3d dir = d;
    double speed = dir.GetLength();
    if (speed <= kEps) {
        // Zero derivative (coincident CVs at this end). Use the chord to
        // the nearest distinct curve point in the direction of travel.
        const double step = 1.0 / kSubdivisionsPerSpan;
        for (int i = 1; i <= kSpans * kSubdivisionsPerSpan && speed <= kEps;
             ++i) {
            GfVec3d ahead, behind;
            Evaluate(u + i * step, &ahead, nullptr);
            Evaluate(u - i * step, &behind, nullptr);
            dir = ahead - p;
            speed = dir.GetLength();
            if (speed <= kEps) {
                dir = p - behind;
                speed = dir.GetLength();
            }
        }
    }
    if (speed <= kEps) {
        if (position) {
            *position = p;
        }
        if (tangent) {
            *tangent = GfVec3d(0.0);
        }
        return false;
    }
    dir /= speed;
    if (distance != clamped) {
        p += dir * (distance - clamped);  // straight extrapolation, no clamp
    }
    if (position) {
        *position = p;
    }
    if (tangent) {
        *tangent = dir;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Rest construction and CV posing
// ---------------------------------------------------------------------------

RigExecSplineIkRest
RigExecSplineIkMakeRest(
    const std::vector<RigExecPointFrame> &joints,
    const RigExecPointFrame &rootControl,
    const RigExecPointFrame &midControl,
    const RigExecPointFrame &endControl,
    const std::vector<double> &volumeWeights,
    RigExecSplineIkRestLength restLength)
{
    RigExecSplineIkRest rest;
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
        return joints[std::min(i, n - 1)].Origin();
    };
    rest.cvs = {origin(0), origin(1), origin(n >= 2 ? n - 2 : 0),
                origin(n - 1)};

    double chainLength = 0.0;
    rest.segmentLengths.resize(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
        rest.segmentLengths[i] = (origin(i + 1) - origin(i)).GetLength();
        chainLength += rest.segmentLengths[i];
    }
    rest.restArcLength = (restLength == RigExecSplineIkRestLength::Chain)
        ? chainLength
        : RigExecSplineIkCurve(rest.cvs).ArcLength();
    return rest;
}

bool
RigExecSplineIkPoseCvs(
    const RigExecSplineIkRest &rest,
    const RigExecSplineIkControls &controls,
    const RigExecSplineIkParams &params,
    std::array<GfVec3d, 4> *cvs)
{
    *cvs = rest.cvs;
    GfMatrix4d rootMap, endMap;
    if (!RigExecPointsToMatrix(rest.rootControl.points, controls.root.points,
                               &rootMap) ||
        !RigExecPointsToMatrix(rest.endControl.points, controls.end.points,
                               &endMap)) {
        return false;
    }
    // cv0, cv1 ride the root control; cv2, cv3 ride the end control.
    (*cvs)[0] = rootMap.TransformAffine(rest.cvs[0]);
    (*cvs)[1] = rootMap.TransformAffine(rest.cvs[1]);
    (*cvs)[2] = endMap.TransformAffine(rest.cvs[2]);
    (*cvs)[3] = endMap.TransformAffine(rest.cvs[3]);

    // The mid control's follow point: its rest origin carried by both
    // parents and blended (parentConstraint with maintainOffset). Taken
    // from the UNCLAMPED maps, so a follow helper in the rig that reads
    // the same controls stays exact whatever the floor and the root aim
    // below do to the CVs.
    const GfVec3d midRest = rest.midControl.Origin();
    const double w = params.midFollowWeight;
    const GfVec3d follow = rootMap.TransformAffine(midRest) * (1.0 - w) +
                           endMap.TransformAffine(midRest) * w;
    const GfVec3d offset = controls.mid.Origin() - follow;

    // The root's posed chain axis: the rest chord carried by the root map.
    const GfVec3d restChord = rest.cvs[3] - rest.cvs[0];
    const double restChordLength = restChord.GetLength();
    GfVec3d rootAxis = rootMap.TransformDir(restChord);
    const bool haveAxis =
        restChordLength > kEps && rootAxis.GetLength() > kEps;
    if (haveAxis) {
        rootAxis.Normalize();
    }

    // Length floor: hold the end CVs at least minLengthRatio of the rest
    // chord ahead of cv0 along the root's posed chain axis. Measured
    // against the root's axis rather than the current chord so that an
    // end control driven onto (or past) the root has a defined "forward"
    // and the chain lifts instead of flipping. cv2 rides with cv3 so the
    // end tangent keeps its direction.
    if (params.minLengthRatio > 0.0 && haveAxis) {
        const double minAlong = params.minLengthRatio * restChordLength;
        const double along = GfDot((*cvs)[3] - (*cvs)[0], rootAxis);
        if (along < minAlong) {
            const GfVec3d lift = rootAxis * (minAlong - along);
            (*cvs)[2] += lift;
            (*cvs)[3] += lift;
        }
    }

    // Root tangent aim: turn cv1 about cv0 from the root's axis onto the
    // (floored) chord direction. Antiparallel or degenerate chords leave
    // cv1 riding the root rigidly.
    if (params.aimRootTangent && haveAxis) {
        const GfVec3d to = (*cvs)[3] - (*cvs)[0];
        if (to.GetLength() > kEps) {
            (*cvs)[1] = (*cvs)[0] + _RotateToward(rootAxis, to.GetNormalized(),
                                                  (*cvs)[1] - (*cvs)[0]);
        }
    }

    // The animator's local bend of the mid control, on top of everything.
    (*cvs)[1] += offset;
    (*cvs)[2] += offset;

    for (const GfVec3d &cv : *cvs) {
        if (!_IsFinite(cv)) {
            *cvs = rest.cvs;
            return false;
        }
    }
    return true;
}

double
RigExecSplineIkTwistAboutAxis(
    const RigExecPointFrame &restFrame,
    const RigExecPointFrame &posedFrame,
    const GfVec3d &axis)
{
    const double axisLength = axis.GetLength();
    if (!_IsFinite(axis) || axisLength <= kEps || !_IsFinite(restFrame) ||
        !_IsFinite(posedFrame)) {
        return 0.0;
    }
    GfMatrix4d map;
    if (!RigExecPointsToMatrix(restFrame.points, posedFrame.points, &map)) {
        return 0.0;
    }
    // Strip translation and scale; keep the proper rotation. Row-vector
    // layout: rows are the images of the basis vectors.
    GfMatrix4d rotation = map;
    rotation.SetTranslateOnly(GfVec3d(0.0));
    rotation.Orthonormalize();
    if (rotation.GetDeterminant3() < 0.0) {
        rotation.SetRow(2, -rotation.GetRow(2));  // drop the reflection
    }
    GfQuatd q = rotation.ExtractRotationQuat();
    if (q.GetReal() < 0.0) {
        q = -q;  // double cover: keep the twist in [-pi, pi]
    }
    const double along = GfDot(q.GetImaginary(), axis / axisLength);
    return 2.0 * std::atan2(along, q.GetReal());
}

// ---------------------------------------------------------------------------
// The solve
// ---------------------------------------------------------------------------

bool
RigExecSolveSplineIk(
    const RigExecSplineIkRest &rest,
    const RigExecSplineIkControls &controls,
    const RigExecSplineIkParams &params,
    RigExecSplineIkResult *result)
{
    *result = RigExecSplineIkResult();
    result->cvs = rest.cvs;

    const size_t n = rest.joints.size();
    if (n == 0 || rest.segmentLengths.size() + 1 != n ||
        (!rest.volumeWeights.empty() && rest.volumeWeights.size() != n)) {
        return false;
    }

    // Everything-degenerate fallback: rest frames, flagged.
    const auto failWithRest = [&]() {
        result->joints.resize(n);
        for (size_t i = 0; i < n; ++i) {
            result->joints[i].frame = _DegenerateCopy(rest.joints[i]);
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
    if (!finiteInputs || rest.restArcLength <= kEps) {
        return failWithRest();
    }

    // 1. Pose the CVs and build the curve.
    if (!RigExecSplineIkPoseCvs(rest, controls, params, &result->cvs)) {
        return failWithRest();
    }
    const RigExecSplineIkCurve curve(result->cvs);
    result->arcLength = curve.ArcLength();
    result->ratio = result->arcLength / rest.restArcLength;
    const bool curveOk = !curve.IsDegenerate();

    // 2. Roll and twist about the rest chain axis.
    GfVec3d chainAxis = rest.cvs[3] - rest.cvs[0];
    if (chainAxis.GetLength() <= kEps) {
        chainAxis = rest.rootControl.X() - rest.rootControl.Origin();
    }
    double rootTwist = 0.0, endTwist = 0.0;
    if (chainAxis.GetLength() > kEps) {
        chainAxis.Normalize();
        rootTwist = RigExecSplineIkTwistAboutAxis(
            rest.rootControl, controls.root, chainAxis);
        endTwist = RigExecSplineIkTwistAboutAxis(
            rest.endControl, controls.end, chainAxis);
    }
    result->roll = rootTwist + params.roll;
    result->twist = (endTwist - rootTwist) + params.twist;

    // 3. Lay the joints out along the curve by arc length.
    result->joints.resize(n);
    std::vector<GfVec3d> positions(n), tangents(n);
    double cumulative = 0.0;
    for (size_t i = 0; i < n; ++i) {
        if (i > 0) {
            cumulative += rest.segmentLengths[i - 1];
        }
        const double d = result->ratio * cumulative;
        result->joints[i].arcDistance = d;
        result->joints[i].arcParam = curveOk ? d / result->arcLength : 0.0;
        result->joints[i].twist =
            result->roll + result->twist * result->joints[i].arcParam;
        if (!curve.PointAtArcLength(d, &positions[i], &tangents[i])) {
            positions[i] = result->cvs[0];
            tangents[i] = GfVec3d(0.0);
        }
    }

    // 4. Orient, twist, and scale each joint.
    bool allOk = curveOk;
    for (size_t i = 0; i < n; ++i) {
        RigExecSplineIkJoint &joint = result->joints[i];
        const _Basis restBasis = _MakeBasis(rest.joints[i]);
        if (!restBasis.ok) {
            joint.frame = _DegenerateCopy(rest.joints[i]);
            allOk = false;
            continue;
        }

        // Aim at the next joint; the last joint follows the tangent. Fall
        // back to the tangent, then to the rest aim, when the aim vanishes.
        GfVec3d aim = (i + 1 < n) ? positions[i + 1] - positions[i]
                                  : tangents[i];
        if (aim.GetLength() <= kEps) {
            aim = tangents[i];
        }
        bool oriented = true;
        if (aim.GetLength() <= kEps) {
            aim = restBasis.x;
            oriented = false;
        }
        const GfVec3d x = aim.GetNormalized();

        // Rotation-minimising transport of the rest up vector.
        GfVec3d y = _RotateToward(restBasis.x, x, restBasis.y);
        y -= x * GfDot(x, y);
        if (y.GetLength() <= kEps) {
            y = _FallbackUp(x);
        } else {
            y.Normalize();
        }

        // Twist about the aim axis: y lies in the plane perpendicular to x,
        // so Rodrigues reduces to a rotation in the (y, x cross y) plane.
        const double theta = joint.twist;
        const GfVec3d yTwisted =
            y * std::cos(theta) + GfCross(x, y) * std::sin(theta);
        const GfVec3d z = GfCross(x, yTwisted) * restBasis.handedness;

        // Volume: s_y = s_z = 1 - w_i * preserveVolume * (ratio - 1).
        const double w = rest.volumeWeights.empty() ? 0.0
                                                    : rest.volumeWeights[i];
        const double s =
            1.0 - w * params.preserveVolume * (result->ratio - 1.0);
        joint.scale = GfVec3d(1.0, s, s);

        RigExecPointFrame &frame = joint.frame;
        frame.points[0] = positions[i];
        frame.points[1] = positions[i] + x * restBasis.lx;
        frame.points[2] = positions[i] + yTwisted * (restBasis.ly * s);
        frame.points[3] = positions[i] + z * (restBasis.lz * s);
        frame.flags = RigExecPointFrameValid;
        if (s != 1.0) {
            frame.flags |= RigExecPointFrameAffine;
        }
        if (s < 0.0) {
            frame.flags |= RigExecPointFrameReflected;
        }
        if (!oriented || !curveOk) {
            frame.flags |= RigExecPointFrameDegenerate;
            allOk = false;
        }
    }
    return allOk;
}

}  // namespace rigExec
