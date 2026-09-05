//
// Curvenet representation and scaled frames (2022 paper §3).
//
// Matrices here are in MATH (column-vector) convention: column k of a frame
// is its k-th axis, and M * v applies it, which is what GfMatrix3d's
// matrix*vector operator already does. USD's row-vector convention only
// appears where these meet point arrays, and the transpose is written there.
//
#include "curvenet.h"
#include "pxr/base/gf/vec4d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>

namespace rigExec {

namespace {

constexpr double kEps = 1e-12;

GfVec3d _ToD(const GfVec3f &v) { return GfVec3d(v[0], v[1], v[2]); }

double _SafeNormalize(GfVec3d *v)
{
    const double n = v->GetLength();
    if (n <= kEps) {
        return 0.0;
    }
    *v /= n;
    return n;
}

/// Any unit vector perpendicular to \p v.
GfVec3d _AnyPerpendicular(const GfVec3d &v)
{
    const GfVec3d axis = (std::abs(v[0]) < 0.9) ? GfVec3d(1.0, 0.0, 0.0)
                                                : GfVec3d(0.0, 1.0, 0.0);
    GfVec3d p = GfCross(v, axis);
    if (_SafeNormalize(&p) == 0.0) {
        return GfVec3d(0.0, 0.0, 1.0);
    }
    return p;
}

/// Removes any component of \p n along \p t and renormalizes. Returns false
/// when nothing usable is left.
bool _Orthogonalize(GfVec3d *n, const GfVec3d &t)
{
    *n -= t * GfDot(*n, t);
    return _SafeNormalize(n) > 0.0;
}

GfMatrix3d _Outer(const GfVec3d &a, const GfVec3d &b)
{
    GfMatrix3d m;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            m[i][j] = a[i] * b[j];
        }
    }
    return m;
}

/// Unit eigenvector of the smallest eigenvalue of a symmetric PSD 3x3.
///
/// Inverse power iteration on (trace*I - M), whose dominant eigenvector is
/// M's least one. A handful of iterations is ample: this only has to pick a
/// plane to sort a fan in, and the fan is frozen afterwards.
GfVec3d _SmallestEigenvector(const GfMatrix3d &m)
{
    const double trace = m[0][0] + m[1][1] + m[2][2];
    if (trace <= kEps) {
        return GfVec3d(0.0);
    }
    GfMatrix3d shifted = GfMatrix3d(1.0) * trace - m;
    GfVec3d v(0.31622776601683794, 0.5477225575051661, 0.7745966692414834);
    for (int i = 0; i < 48; ++i) {
        GfVec3d next = shifted * v;
        if (_SafeNormalize(&next) == 0.0) {
            break;
        }
        v = next;
    }
    return v;
}

/// Rotation of \p angle radians about unit \p axis (Rodrigues).
GfMatrix3d _AxisAngle(const GfVec3d &axis, double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    GfMatrix3d k(0.0, -axis[2], axis[1],
                 axis[2], 0.0, -axis[0],
                 -axis[1], axis[0], 0.0);
    GfMatrix3d out(1.0);
    out = out + k * s + (k * k) * (1.0 - c);
    return out;
}

// ---------------------------------------------------------------------------
// Spline evaluation
// ---------------------------------------------------------------------------

GfVec3d _EvalBezier(const GfVec3d p[4], double t)
{
    const double u = 1.0 - t;
    return p[0] * (u * u * u) + p[1] * (3.0 * u * u * t) +
           p[2] * (3.0 * u * t * t) + p[3] * (t * t * t);
}

/// Centripetal Catmull-Rom over p0..p3, evaluated on the p1 -> p2 span.
/// alpha = 0.5 is the centripetal parameterization the 2026 talk calls for;
/// it never cusps or self-intersects, and every control point lies exactly on
/// the curve, which is why that talk uses it for parametrization work.
GfVec3d _EvalCatmullRom(const GfVec3d p[4], double t)
{
    auto knot = [](double ti, const GfVec3d &a, const GfVec3d &b) {
        const double d = (b - a).GetLength();
        return ti + std::pow(std::max(d, kEps), 0.5);
    };
    const double t0 = 0.0;
    const double t1 = knot(t0, p[0], p[1]);
    const double t2 = knot(t1, p[1], p[2]);
    const double t3 = knot(t2, p[2], p[3]);
    if (t2 - t1 <= kEps) {
        return p[1];
    }
    const double tt = t1 + t * (t2 - t1);

    auto lerpKnot = [&](const GfVec3d &a, const GfVec3d &b, double ta,
                        double tb) {
        if (tb - ta <= kEps) {
            return a;
        }
        const double w = (tb - tt) / (tb - ta);
        return a * w + b * (1.0 - w);
    };
    const GfVec3d a1 = lerpKnot(p[0], p[1], t0, t1);
    const GfVec3d a2 = lerpKnot(p[1], p[2], t1, t2);
    const GfVec3d a3 = lerpKnot(p[2], p[3], t2, t3);
    const GfVec3d b1 = lerpKnot(a1, a2, t0, t2);
    const GfVec3d b2 = lerpKnot(a2, a3, t1, t3);
    return lerpKnot(b1, b2, t1, t2);
}

GfVec3d _EvalSpline(RigExecCurvenetBasis basis, const GfVec3d p[4], double t)
{
    return (basis == RigExecCurvenetBasis::Bezier) ? _EvalBezier(p, t)
                                                   : _EvalCatmullRom(p, t);
}

/// Gathers a spline's four control points, reversing when the spline is
/// traversed backwards. Reversal of both bases is exact: a cubic Bezier and
/// a Catmull-Rom span are both symmetric under reversing their control tuple.
void _GatherSpline(const RigExecCurvenetTopology &topology,
                   const std::vector<GfVec3f> &points, size_t spline,
                   bool reversed, GfVec3d out[4])
{
    for (int i = 0; i < 4; ++i) {
        const int src = reversed ? 3 - i : i;
        out[i] = _ToD(points[topology.splineIndices[4 * spline + src]]);
    }
}

/// Even arc-length samples of one spline span, inclusive of both ends.
/// Uniform refinement in parametric space first, then resampling by arc
/// length, exactly as §3 describes.
GfVec4d _SplineWeights(RigExecCurvenetBasis basis, const GfVec3d p[4], double t)
{
    if (basis == RigExecCurvenetBasis::Bezier) {
        const double u = 1.0 - t;
        return GfVec4d(u*u*u, 3*u*u*t, 3*u*t*t, t*t*t);
    }
    double knots[4] = {0, 0, 0, 0};
    for (int i = 1; i < 4; ++i)
        knots[i] = knots[i-1] + std::sqrt(std::max((p[i]-p[i-1]).GetLength(), kEps));
    const double at = knots[1] + t * (knots[2] - knots[1]);
    auto mix = [&](const GfVec4d &a, const GfVec4d &b, int lo, int hi) {
        const double w = (at-knots[lo])/(knots[hi]-knots[lo]);
        return a*(1-w) + b*w;
    };
    const GfVec4d a = mix(GfVec4d(1,0,0,0), GfVec4d(0,1,0,0), 0,1);
    const GfVec4d b = mix(GfVec4d(0,1,0,0), GfVec4d(0,0,1,0), 1,2);
    const GfVec4d c = mix(GfVec4d(0,0,1,0), GfVec4d(0,0,0,1), 2,3);
    return mix(mix(a,b,0,2), mix(b,c,1,3), 1,2);
}

void _SampleSplineSpan(RigExecCurvenetBasis basis, const GfVec3d p[4],
                       int subdivisions, std::vector<GfVec3d> *out,
                       std::vector<GfVec4d> *stencils)
{
    const int refine = std::max(8 * subdivisions, 32);
    std::vector<GfVec3d> dense(refine + 1);
    std::vector<GfVec4d> coefficients(refine + 1);
    std::vector<double> arc(refine + 1, 0.0);
    for (int i = 0; i <= refine; ++i) {
        dense[i] = _EvalSpline(basis, p, double(i) / double(refine));
        coefficients[i] = _SplineWeights(basis, p, double(i) / double(refine));
        if (i > 0) {
            arc[i] = arc[i - 1] + (dense[i] - dense[i - 1]).GetLength();
        }
    }
    const double total = arc[refine];
    out->clear();
    stencils->clear();
    out->reserve(subdivisions + 1);
    if (total <= kEps) {
        // Degenerate span: every sample coincides. Emitted rather than
        // dropped so sample counts stay in correspondence across poses; the
        // frame stage rejects the resulting zero-length segments by name.
        for (int j = 0; j <= subdivisions; ++j) {
            out->push_back(dense[0]);
            stencils->push_back(coefficients[0]);
        }
        return;
    }
    int cursor = 0;
    for (int j = 0; j <= subdivisions; ++j) {
        const double target = total * double(j) / double(subdivisions);
        while (cursor + 1 < refine && arc[cursor + 1] < target) {
            ++cursor;
        }
        const double span = arc[cursor + 1] - arc[cursor];
        const double w = (span <= kEps) ? 0.0 : (target - arc[cursor]) / span;
        out->push_back(dense[cursor] * (1.0 - w) + dense[cursor + 1] * w);
        stencils->push_back(coefficients[cursor]*(1-w) + coefficients[cursor+1]*w);
    }
}

}  // namespace

GfMatrix3d RigExecSmallestRotation(const GfVec3d &from, const GfVec3d &to)
{
    GfVec3d a = from, b = to;
    if (_SafeNormalize(&a) == 0.0 || _SafeNormalize(&b) == 0.0) {
        return GfMatrix3d(1.0);
    }
    const double c = GfClamp(GfDot(a, b), -1.0, 1.0);
    if (c > 1.0 - 1e-15) {
        return GfMatrix3d(1.0);
    }
    if (c < -1.0 + 1e-12) {
        // Antipodal: the rotation is a half turn about any perpendicular.
        // Deterministic choice so a rest/posed pair cannot disagree.
        return _AxisAngle(_AnyPerpendicular(a), M_PI);
    }
    GfVec3d axis = GfCross(a, b);
    _SafeNormalize(&axis);
    return _AxisAngle(axis, std::acos(c));
}

// ---------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------

bool RigExecBuildCurvenetTopology(
    const std::vector<int> &splineIndices, size_t pointCount,
    RigExecCurvenetBasis basis, const std::vector<GfVec3f> &neutralPoints,
    const std::function<GfVec3d(const GfVec3d &)> *normalAt,
    RigExecCurvenetTopology *topology, std::string *error)
{
    auto fail = [&](const std::string &message) {
        if (error) {
            *error = message;
        }
        return false;
    };

    if (splineIndices.size() % 4 != 0) {
        return fail("rigExec:splineIndices length " +
                    std::to_string(splineIndices.size()) +
                    " is not a multiple of four (one cubic spline is four "
                    "control point indices)");
    }
    for (size_t i = 0; i < splineIndices.size(); ++i) {
        if (splineIndices[i] < 0 ||
            size_t(splineIndices[i]) >= pointCount) {
            return fail("rigExec:splineIndices[" + std::to_string(i) +
                        "] = " + std::to_string(splineIndices[i]) +
                        " is out of range for " + std::to_string(pointCount) +
                        " control points");
        }
    }
    if (neutralPoints.size() != pointCount) {
        return fail("neutral point count " +
                    std::to_string(neutralPoints.size()) +
                    " disagrees with the declared pool size " +
                    std::to_string(pointCount));
    }

    *topology = RigExecCurvenetTopology();
    topology->basis = basis;
    topology->pointCount = int(pointCount);
    topology->splineIndices = splineIndices;

    const size_t splineCount = splineIndices.size() / 4;

    // Endpoint valence, and the incidence list that drives the chain walk.
    // (spline, atStart) pairs per knot.
    std::vector<std::vector<std::pair<int, bool>>> incident(pointCount);
    std::vector<int> valence(pointCount, 0);
    std::vector<bool> isHandle(pointCount, false);
    for (size_t s = 0; s < splineCount; ++s) {
        const int a = topology->GetSplineStartKnot(s);
        const int b = topology->GetSplineEndKnot(s);
        incident[a].push_back({int(s), true});
        incident[b].push_back({int(s), false});
        ++valence[a];
        ++valence[b];
        isHandle[splineIndices[4 * s + (basis == RigExecCurvenetBasis::Bezier ? 1 : 0)]] = true;
        isHandle[splineIndices[4 * s + (basis == RigExecCurvenetBasis::Bezier ? 2 : 3)]] = true;
    }

    topology->knotValence.assign(pointCount, 0);
    topology->knotKinds.assign(pointCount, RigExecCurvenetKnotKind::Unused);
    for (size_t k = 0; k < pointCount; ++k) {
        topology->knotValence[k] = valence[k];
        if (valence[k] >= 3) {
            topology->knotKinds[k] = RigExecCurvenetKnotKind::Intersection;
        } else if (valence[k] == 2) {
            topology->knotKinds[k] = RigExecCurvenetKnotKind::Interior;
        } else if (valence[k] == 1) {
            topology->knotKinds[k] = RigExecCurvenetKnotKind::Anchor;
        } else if (isHandle[k]) {
            topology->knotKinds[k] = RigExecCurvenetKnotKind::Handle;
        }
    }

    auto isLabelled = [&](int knot) {
        return topology->knotKinds[knot] ==
                   RigExecCurvenetKnotKind::Intersection ||
               topology->knotKinds[knot] == RigExecCurvenetKnotKind::Anchor;
    };
    auto otherEnd = [&](int spline, int knot) {
        const int a = topology->GetSplineStartKnot(spline);
        const int b = topology->GetSplineEndKnot(spline);
        return (knot == a) ? b : a;
    };

    // Chains between labelled knots. A spline is consumed by exactly one
    // curve; a knot of valence two is walked through rather than stopped at.
    std::vector<bool> consumed(splineCount, false);
    for (size_t k = 0; k < pointCount; ++k) {
        if (!isLabelled(int(k))) {
            continue;
        }
        for (const auto &start : incident[k]) {
            if (consumed[start.first]) {
                continue;
            }
            RigExecCurvenetCurve curve;
            curve.startKnot = int(k);
            curve.startIsIntersection =
                topology->knotKinds[k] == RigExecCurvenetKnotKind::Intersection;

            int currentKnot = int(k);
            int currentSpline = start.first;
            for (;;) {
                consumed[currentSpline] = true;
                const bool reversed =
                    topology->GetSplineStartKnot(currentSpline) != currentKnot;
                curve.splines.push_back(currentSpline);
                curve.reversed.push_back(reversed);
                const int next = otherEnd(currentSpline, currentKnot);
                currentKnot = next;
                if (isLabelled(next)) {
                    break;
                }
                // Valence two: continue through with the other spline.
                int following = -1;
                for (const auto &inc : incident[next]) {
                    if (inc.first != currentSpline && !consumed[inc.first]) {
                        following = inc.first;
                        break;
                    }
                }
                if (following < 0) {
                    break;  // closed back on itself
                }
                currentSpline = following;
            }
            curve.endKnot = currentKnot;
            curve.endIsIntersection =
                topology->knotKinds[currentKnot] ==
                RigExecCurvenetKnotKind::Intersection;
            topology->curves.push_back(std::move(curve));
        }
    }

    // Whatever is left is a cycle of valence-two knots: an isolated closed
    // curve (§3 "we also group any remaining spline connected solely by
    // unlabelled endpoints").
    for (size_t s = 0; s < splineCount; ++s) {
        if (consumed[s]) {
            continue;
        }
        RigExecCurvenetCurve curve;
        curve.closed = true;
        const int origin = topology->GetSplineStartKnot(s);
        curve.startKnot = origin;
        int currentKnot = origin;
        int currentSpline = int(s);
        for (;;) {
            consumed[currentSpline] = true;
            const bool reversed =
                topology->GetSplineStartKnot(currentSpline) != currentKnot;
            curve.splines.push_back(currentSpline);
            curve.reversed.push_back(reversed);
            const int next = otherEnd(currentSpline, currentKnot);
            currentKnot = next;
            if (next == origin) {
                break;
            }
            int following = -1;
            for (const auto &inc : incident[next]) {
                if (inc.first != currentSpline && !consumed[inc.first]) {
                    following = inc.first;
                    break;
                }
            }
            if (following < 0) {
                break;
            }
            currentSpline = following;
        }
        curve.endKnot = currentKnot;
        topology->curves.push_back(std::move(curve));
    }

    // Intersections and their counter-clockwise fans.
    topology->intersectionOfKnot.assign(pointCount, -1);
    for (size_t c = 0; c < topology->curves.size(); ++c) {
        const RigExecCurvenetCurve &curve = topology->curves[c];
        for (int end = 0; end < 2; ++end) {
            const bool atEnd = (end == 1);
            const bool isIntersection =
                atEnd ? curve.endIsIntersection : curve.startIsIntersection;
            if (!isIntersection) {
                continue;
            }
            const int knot = atEnd ? curve.endKnot : curve.startKnot;
            int slot = topology->intersectionOfKnot[knot];
            if (slot < 0) {
                slot = int(topology->intersections.size());
                topology->intersectionOfKnot[knot] = slot;
                RigExecCurvenetIntersection created;
                created.knot = knot;
                topology->intersections.push_back(created);
            }
            RigExecCurvenetSpoke spoke;
            spoke.curve = int(c);
            spoke.atCurveEnd = atEnd;
            topology->intersections[slot].spokes.push_back(spoke);
        }
    }

    RigExecOrientCurvenetIntersections(
        neutralPoints,
        normalAt ? *normalAt : std::function<GfVec3d(const GfVec3d &)>(),
        topology);
    return true;
}

void RigExecOrientCurvenetIntersections(
    const std::vector<GfVec3f> &neutralPoints,
    const std::function<GfVec3d(const GfVec3d &)> &normalAt,
    RigExecCurvenetTopology *topology)
{
    // Sort each fan counter-clockwise about the surface normal at the
    // intersection, using neutral-pose emanating directions. The ORDER is
    // frozen here and reused in every pose: it is layout, not geometry.
    for (RigExecCurvenetIntersection &intersection : topology->intersections) {
        const GfVec3d origin = _ToD(neutralPoints[intersection.knot]);
        std::vector<GfVec3d> directions;
        directions.reserve(intersection.spokes.size());
        for (const RigExecCurvenetSpoke &spoke : intersection.spokes) {
            const RigExecCurvenetCurve &curve = topology->curves[spoke.curve];
            // The emanating direction is toward the first control point that
            // is not coincident with the knot, along the traversal that
            // leaves this intersection.
            const size_t which =
                spoke.atCurveEnd ? curve.splines.size() - 1 : 0;
            const bool reversed = spoke.atCurveEnd
                                      ? !curve.reversed[which]
                                      : curve.reversed[which];
            GfVec3d cp[4];
            _GatherSpline(*topology, neutralPoints, curve.splines[which],
                          reversed, cp);
            GfVec3d dir(0.0);
            for (int i = 1; i < 4; ++i) {
                dir = cp[i] - cp[0];
                if (dir.GetLength() > kEps) {
                    break;
                }
            }
            _SafeNormalize(&dir);
            directions.push_back(dir);
        }

        GfVec3d normal(0.0);
        if (normalAt) {
            normal = normalAt(origin);
        }
        if (_SafeNormalize(&normal) == 0.0) {
            // No surface to consult: take the fan's own best-fit plane as the
            // least-variance direction of its spokes.
            //
            // NOT the sum of consecutive cross products. The spokes arrive in
            // whatever order the curves were built, and for the commonest
            // intersection there is -- four spokes in two antiparallel pairs,
            // which is what a grid-like net produces everywhere -- that sum
            // cancels to exactly zero, leaving an arbitrary axis to sort a
            // fan about. This is stable for any arrangement.
            GfMatrix3d covariance(0.0);
            for (const GfVec3d &d : directions) {
                covariance += _Outer(d, d);
            }
            normal = _SmallestEigenvector(covariance);
            if (_SafeNormalize(&normal) == 0.0) {
                normal = _AnyPerpendicular(
                    directions.empty() ? GfVec3d(0, 0, 1) : directions[0]);
            }
        }
        intersection.referenceNormal = normal;

        GfVec3d basisX = directions.empty() ? _AnyPerpendicular(normal)
                                            : directions[0];
        if (!_Orthogonalize(&basisX, normal)) {
            basisX = _AnyPerpendicular(normal);
        }
        const GfVec3d basisY = GfCross(normal, basisX);

        std::vector<size_t> order(intersection.spokes.size());
        for (size_t i = 0; i < order.size(); ++i) {
            order[i] = i;
        }
        std::vector<double> angle(order.size(), 0.0);
        for (size_t i = 0; i < order.size(); ++i) {
            const GfVec3d &d = directions[i];
            angle[i] = std::atan2(GfDot(d, basisY), GfDot(d, basisX));
        }
        std::stable_sort(order.begin(), order.end(),
                         [&](size_t a, size_t b) {
                             return angle[a] < angle[b];
                         });
        std::vector<RigExecCurvenetSpoke> sorted;
        sorted.reserve(order.size());
        for (size_t i : order) {
            sorted.push_back(intersection.spokes[i]);
        }
        intersection.spokes = std::move(sorted);
    }
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

std::vector<int> RigExecPlanCurvenetSamples(
    const RigExecCurvenetTopology &topology,
    const std::vector<GfVec3f> &neutralPoints, double meshMeanEdgeLength,
    int samplesPerSpline)
{
    const size_t splineCount = topology.GetSplineCount();
    std::vector<int> counts(splineCount, 1);
    const double perSpline = std::max(1, samplesPerSpline);
    const double edge = (meshMeanEdgeLength > kEps) ? meshMeanEdgeLength : 0.0;
    for (size_t s = 0; s < splineCount; ++s) {
        double polygon = 0.0;
        for (int i = 0; i < 3; ++i) {
            polygon += (_ToD(neutralPoints[topology.splineIndices[4 * s + i + 1]]) -
                        _ToD(neutralPoints[topology.splineIndices[4 * s + i]]))
                           .GetLength();
        }
        // With no mesh to scale against, the user's value is the count.
        const double ratio = (edge > 0.0) ? polygon / edge : 1.0;
        const long n = std::lround(perSpline * ratio);
        counts[s] = int(std::max<long>(1, std::min<long>(n, 4096)));
    }
    return counts;
}

RigExecCurvenetSampling RigExecSampleCurvenet(
    const RigExecCurvenetTopology &topology,
    const std::vector<GfVec3f> &points,
    const std::vector<int> &samplesPerSpline)
{
    RigExecCurvenetSampling sampling;
    sampling.curveBegin.reserve(topology.curves.size() + 1);
    sampling.curveBegin.push_back(0);

    std::vector<GfVec3d> span;
    std::vector<GfVec4d> stencils;
    for (const RigExecCurvenetCurve &curve : topology.curves) {
        for (size_t i = 0; i < curve.splines.size(); ++i) {
            const size_t spline = curve.splines[i];
            GfVec3d cp[4];
            _GatherSpline(topology, points, spline, curve.reversed[i], cp);
            const int subdivisions =
                (spline < samplesPerSpline.size())
                    ? std::max(1, samplesPerSpline[spline])
                    : 1;
            _SampleSplineSpan(topology.basis, cp, subdivisions, &span, &stencils);

            // The first sample of every span but the first is the previous
            // span's last: drop it so shared knots appear once.
            const size_t first = (i == 0) ? 0 : 1;
            const int startKnot =
                curve.reversed[i] ? topology.GetSplineEndKnot(spline)
                                  : topology.GetSplineStartKnot(spline);
            const int endKnot =
                curve.reversed[i] ? topology.GetSplineStartKnot(spline)
                                  : topology.GetSplineEndKnot(spline);
            for (size_t j = first; j < span.size(); ++j) {
                sampling.positions.push_back(span[j]);
                std::array<int,4> indices;
                std::array<double,4> weights;
                for (int k = 0; k < 4; ++k) {
                    indices[k] = topology.splineIndices[4*spline + (curve.reversed[i] ? 3-k : k)];
                    weights[k] = stencils[j][k];
                }
                sampling.stencilIndices.push_back(indices);
                sampling.stencilWeights.push_back(weights);
                int knot = -1;
                if (j == 0) {
                    knot = startKnot;
                } else if (j + 1 == span.size()) {
                    knot = endKnot;
                }
                sampling.knotOfSample.push_back(knot);
            }
        }
        if (curve.closed && sampling.positions.size() >
                                size_t(sampling.curveBegin.back())) {
            // A closed curve's last sample is its first: segments wrap
            // instead, so the duplicate is removed.
            sampling.positions.pop_back();
            sampling.knotOfSample.pop_back();
            sampling.stencilIndices.pop_back();
            sampling.stencilWeights.pop_back();
        }
        sampling.curveBegin.push_back(int(sampling.positions.size()));
    }
    return sampling;
}

// ---------------------------------------------------------------------------
// Scaled frames
// ---------------------------------------------------------------------------

GfMatrix3d RigExecCurvenetFrames::GetScaledFrame(size_t segment,
                                                 bool left) const
{
    const GfVec3d &t = tangent[segment];
    const GfVec3d &n = left ? normalLeft[segment] : normalRight[segment];
    const double l = length[segment];
    const double w = left ? widthLeft[segment] : widthRight[segment];
    const double h = std::sqrt(std::max(l * w, 0.0));
    const GfVec3d b = GfCross(n, t);
    GfMatrix3d m;
    for (int i = 0; i < 3; ++i) {
        m[i][0] = t[i] * l;
        m[i][1] = b[i] * w;
        m[i][2] = n[i] * h;
    }
    return m;
}

RigExecCurvenetFrames RigExecComputeCurvenetFrames(
    const RigExecCurvenetTopology &topology,
    const RigExecCurvenetSampling &sampling)
{
    RigExecCurvenetFrames frames;
    const size_t curveCount = topology.curves.size();
    frames.segmentBegin.assign(curveCount + 1, 0);
    frames.curveFramed.assign(curveCount, false);

    // Segment layout and tangents.
    for (size_t c = 0; c < curveCount; ++c) {
        const int n = sampling.GetCurveSampleCount(c);
        const bool closed = topology.curves[c].closed;
        const int segments = closed ? std::max(n, 0) : std::max(n - 1, 0);
        frames.segmentBegin[c + 1] = frames.segmentBegin[c] + segments;
    }
    const size_t segmentCount = size_t(frames.segmentBegin.back());
    frames.tangent.assign(segmentCount, GfVec3d(0.0));
    frames.length.assign(segmentCount, 0.0);
    frames.normalLeft.assign(segmentCount, GfVec3d(0.0));
    frames.normalRight.assign(segmentCount, GfVec3d(0.0));
    frames.widthLeft.assign(segmentCount, 0.0);
    frames.widthRight.assign(segmentCount, 0.0);

    for (size_t c = 0; c < curveCount; ++c) {
        const int base = sampling.curveBegin[c];
        const int n = sampling.GetCurveSampleCount(c);
        const bool closed = topology.curves[c].closed;
        const int segments = frames.segmentBegin[c + 1] - frames.segmentBegin[c];
        for (int i = 0; i < segments; ++i) {
            const GfVec3d &a = sampling.positions[base + i];
            const GfVec3d &b = sampling.positions[base + ((i + 1) % n)];
            GfVec3d d = b - a;
            const double len = d.GetLength();
            frames.length[frames.segmentBegin[c] + i] = len;
            if (len > kEps) {
                d /= len;
                frames.tangent[frames.segmentBegin[c] + i] = d;
            }
        }
    }

    // Corner normals and widths at every intersection (§3).
    // Per (intersection, spoke): the emanating segment's left/right normal
    // and width, in EMANATING orientation.
    struct _SpokeFrame {
        GfVec3d normalLeft{0.0, 0.0, 0.0};
        GfVec3d normalRight{0.0, 0.0, 0.0};
        double widthLeft = 0.0;
        double widthRight = 0.0;
        bool valid = false;
    };
    std::vector<std::vector<_SpokeFrame>> spokeFrames(
        topology.intersections.size());

    for (size_t x = 0; x < topology.intersections.size(); ++x) {
        const RigExecCurvenetIntersection &intersection =
            topology.intersections[x];
        const size_t k = intersection.spokes.size();
        spokeFrames[x].assign(k, _SpokeFrame());
        if (k == 0) {
            continue;
        }

        // Emanating tangent and length per spoke.
        std::vector<GfVec3d> t(k);
        std::vector<double> l(k, 0.0);
        std::vector<int> segmentOf(k, -1);
        for (size_t i = 0; i < k; ++i) {
            const RigExecCurvenetSpoke &spoke = intersection.spokes[i];
            const int begin = frames.segmentBegin[spoke.curve];
            const int end = frames.segmentBegin[spoke.curve + 1];
            if (end <= begin) {
                continue;
            }
            const int segment = spoke.atCurveEnd ? end - 1 : begin;
            segmentOf[i] = segment;
            l[i] = frames.length[segment];
            t[i] = spoke.atCurveEnd ? -frames.tangent[segment]
                                    : frames.tangent[segment];
        }

        // Corner vectors between consecutive spokes.
        std::vector<GfVec3d> corner(k, GfVec3d(0.0));
        std::vector<double> cornerLen(k, 0.0);
        for (size_t i = 0; i < k; ++i) {
            corner[i] = GfCross(t[i], t[(i + 1) % k]);
            cornerLen[i] = corner[i].GetLength();
        }
        std::vector<GfVec3d> m(k, GfVec3d(0.0));
        for (size_t i = 0; i < k; ++i) {
            if (cornerLen[i] > 1e-9) {
                m[i] = corner[i] / cornerLen[i];
                continue;
            }
            // Parallel tangents (a T-junction's straight-through pair):
            // borrow the neighbouring corners.
            GfVec3d blended = corner[(i + 1) % k] + corner[(i + k - 1) % k];
            if (_SafeNormalize(&blended) > 0.0) {
                m[i] = blended;
            } else {
                m[i] = intersection.referenceNormal;
            }
        }
        // Keep every corner normal on the same side of the surface: a fan
        // sorted counter-clockwise produces corners along +normal, and a
        // reflected one would silently flip a frame's handedness.
        //
        // Measured against the fan's OWN corners in THIS pose, not against
        // the reference normal frozen at bind time. That reference cannot
        // move with the rig, so a corner normal lying near its equator flips
        // under a modest rotation -- and a deformation gradient that flips
        // sign with the pose is not a deformation gradient. The fan sum is
        // equivariant, so the same corner is chosen in every pose; the frozen
        // reference is only the fallback for a fan that cancels.
        GfVec3d fanNormal(0.0);
        for (size_t i = 0; i < k; ++i) {
            fanNormal += m[i];
        }
        if (_SafeNormalize(&fanNormal) == 0.0) {
            fanNormal = intersection.referenceNormal;
        }
        for (size_t i = 0; i < k; ++i) {
            if (GfDot(m[i], fanNormal) < 0.0) {
                m[i] = -m[i];
            }
        }

        for (size_t i = 0; i < k; ++i) {
            const size_t prev = (i + k - 1) % k;
            const size_t next = (i + 1) % k;
            _SpokeFrame &out = spokeFrames[x][i];
            out.normalLeft = m[i];
            out.normalRight = m[prev];
            out.widthLeft = l[i] + cornerLen[i] * (l[next] - l[i]);
            out.widthRight = l[i] + cornerLen[prev] * (l[prev] - l[i]);
            // A width is a scale: it must stay positive. The formula can go
            // non-positive only for a near-straight corner between segments
            // of very different length, where the segment's own length is
            // the meaningful fallback.
            if (!(out.widthLeft > kEps)) {
                out.widthLeft = std::max(l[i], kEps);
            }
            if (!(out.widthRight > kEps)) {
                out.widthRight = std::max(l[i], kEps);
            }
            out.valid = (segmentOf[i] >= 0);
        }
    }

    // Propagate along each curve, per side.
    for (size_t c = 0; c < curveCount; ++c) {
        const RigExecCurvenetCurve &curve = topology.curves[c];
        const int begin = frames.segmentBegin[c];
        const int end = frames.segmentBegin[c + 1];
        const int count = end - begin;
        if (count <= 0 || curve.IsIsolated()) {
            continue;  // isolated: rotation-only gradient, no frame needed
        }

        // Boundary values at whichever ends are intersections, expressed in
        // CURVE-traversal orientation. Leaving an intersection at the curve's
        // end means travelling against the curve, so left and right swap.
        auto boundary = [&](bool atCurveEnd, GfVec3d *nLeft, GfVec3d *nRight,
                            double *wLeft, double *wRight) -> bool {
            const int knot = atCurveEnd ? curve.endKnot : curve.startKnot;
            const int slot = topology.intersectionOfKnot[knot];
            if (slot < 0) {
                return false;
            }
            const auto &spokes = topology.intersections[slot].spokes;
            for (size_t i = 0; i < spokes.size(); ++i) {
                if (spokes[i].curve != int(c) ||
                    spokes[i].atCurveEnd != atCurveEnd) {
                    continue;
                }
                const _SpokeFrame &sf = spokeFrames[slot][i];
                if (!sf.valid) {
                    return false;
                }
                if (atCurveEnd) {
                    *nLeft = sf.normalRight;
                    *nRight = sf.normalLeft;
                    *wLeft = sf.widthRight;
                    *wRight = sf.widthLeft;
                } else {
                    *nLeft = sf.normalLeft;
                    *nRight = sf.normalRight;
                    *wLeft = sf.widthLeft;
                    *wRight = sf.widthRight;
                }
                return true;
            }
            return false;
        };

        GfVec3d startLeft, startRight, endLeft, endRight;
        double startWLeft = 0, startWRight = 0, endWLeft = 0, endWRight = 0;
        const bool haveStart =
            curve.startIsIntersection &&
            boundary(false, &startLeft, &startRight, &startWLeft, &startWRight);
        const bool haveEnd =
            curve.endIsIntersection &&
            boundary(true, &endLeft, &endRight, &endWLeft, &endWRight);
        if (!haveStart && !haveEnd) {
            continue;
        }

        // Normalized arc length per segment (§3): alpha_i uses the arc length
        // BEFORE segment i over the curve's total.
        std::vector<double> alpha(count, 0.0);
        double total = 0.0;
        for (int i = 0; i < count; ++i) {
            alpha[i] = total;
            total += frames.length[begin + i];
        }
        if (total > kEps) {
            for (int i = 0; i < count; ++i) {
                alpha[i] /= total;
            }
        }

        for (int side = 0; side < 2; ++side) {
            const bool left = (side == 0);
            std::vector<GfVec3d> &normals =
                left ? frames.normalLeft : frames.normalRight;
            std::vector<double> &widths =
                left ? frames.widthLeft : frames.widthRight;

            // Walk from whichever end has an intersection. When only the far
            // end does, walk backwards; the arithmetic is identical with the
            // segment order reversed.
            const bool forward = haveStart;
            GfVec3d seed = forward ? (left ? startLeft : startRight)
                                   : (left ? endLeft : endRight);
            const double seedWidth = forward
                                         ? (left ? startWLeft : startWRight)
                                         : (left ? endWLeft : endWRight);

            std::vector<GfVec3d> transported(count, GfVec3d(0.0));
            const int firstIndex = forward ? 0 : count - 1;
            const int step = forward ? 1 : -1;

            GfVec3d n = seed;
            if (!_Orthogonalize(&n, frames.tangent[begin + firstIndex])) {
                n = _AnyPerpendicular(frames.tangent[begin + firstIndex]);
            }
            transported[firstIndex] = n;
            for (int i = firstIndex + step; i >= 0 && i < count; i += step) {
                const GfVec3d &prevT = frames.tangent[begin + i - step];
                const GfVec3d &curT = frames.tangent[begin + i];
                const GfMatrix3d r = RigExecSmallestRotation(prevT, curT);
                GfVec3d next = r * transported[i - step];
                if (!_Orthogonalize(&next, curT)) {
                    next = _AnyPerpendicular(curT);
                }
                transported[i] = next;
            }

            // Torsion, when the far end also prescribes a normal.
            double torsion = 0.0;
            const bool bothEnds = haveStart && haveEnd;
            if (bothEnds) {
                const int lastIndex = forward ? count - 1 : 0;
                GfVec3d target = forward ? (left ? endLeft : endRight)
                                         : (left ? startLeft : startRight);
                const GfVec3d &tk = frames.tangent[begin + lastIndex];
                if (_Orthogonalize(&target, tk)) {
                    const GfVec3d &arrived = transported[lastIndex];
                    // atan2, not the paper's one-argument atan: the sign of
                    // the adjacent term matters once the disagreement passes
                    // 90 degrees, and Fig. 12's 180-degree twist is exactly
                    // that case.
                    torsion = std::atan2(GfDot(arrived, GfCross(target, tk)),
                                         GfDot(arrived, target));
                    // The rotation carries `arrived` onto `target`, so it
                    // turns by -torsion about the tangent.
                    torsion = -torsion;
                }
            }

            for (int i = 0; i < count; ++i) {
                // alpha runs along the CURVE; when walking backwards the
                // blend parameter is measured from the seed end.
                const double a = forward ? alpha[i] : (1.0 - alpha[i]);
                GfVec3d n_i = transported[i];
                if (bothEnds && std::abs(torsion) > 0.0) {
                    n_i = _AxisAngle(frames.tangent[begin + i], a * torsion) *
                          n_i;
                    if (!_Orthogonalize(&n_i, frames.tangent[begin + i])) {
                        n_i = transported[i];
                    }
                }
                normals[begin + i] = n_i;

                if (bothEnds) {
                    const double w0 = forward
                                          ? (left ? startWLeft : startWRight)
                                          : (left ? endWLeft : endWRight);
                    const double w1 = forward
                                          ? (left ? endWLeft : endWRight)
                                          : (left ? startWLeft : startWRight);
                    widths[begin + i] = (1.0 - a) * w0 + a * w1;
                } else {
                    // One end at an anchor: uniform width along the side.
                    widths[begin + i] = seedWidth;
                }
            }
        }
        frames.curveFramed[c] = true;
    }

    return frames;
}

bool RigExecCurvenetFramesAreValid(const RigExecCurvenetFrames &frames,
                                   std::string *error)
{
    for (size_t c = 0; c + 1 < frames.segmentBegin.size(); ++c) {
        for (int i = frames.segmentBegin[c]; i < frames.segmentBegin[c + 1];
             ++i) {
            if (frames.length[i] <= kEps) {
                if (error) {
                    char buffer[192];
                    std::snprintf(
                        buffer, sizeof(buffer),
                        "curve %zu segment %d has zero length: two curvenet "
                        "samples coincide, which leaves the segment tangent "
                        "undefined",
                        c, i - frames.segmentBegin[c]);
                    *error = buffer;
                }
                return false;
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Deformation gradients
// ---------------------------------------------------------------------------

RigExecCurvenetGradients RigExecComputeCurvenetGradients(
    const RigExecCurvenetFrames &restFrames,
    const RigExecCurvenetFrames &posedFrames)
{
    RigExecCurvenetGradients gradients;
    const size_t count =
        std::min(restFrames.GetSegmentCount(), posedFrames.GetSegmentCount());
    gradients.left.assign(count, GfMatrix3d(1.0));
    gradients.right.assign(count, GfMatrix3d(1.0));

    for (size_t c = 0; c + 1 < posedFrames.segmentBegin.size(); ++c) {
        const bool framed =
            c < posedFrames.curveFramed.size() && posedFrames.curveFramed[c] &&
            c < restFrames.curveFramed.size() && restFrames.curveFramed[c];
        const int begin = posedFrames.segmentBegin[c];
        const int end = posedFrames.segmentBegin[c + 1];
        for (int i = begin; i < end && size_t(i) < count; ++i) {
            if (!framed) {
                // §3: isolated curves take the smallest rotation from rest
                // tangent to posed tangent, times the length ratio. Both
                // sides get the same matrix -- with no net there is no hinge.
                const double restLen = restFrames.length[i];
                const double scale =
                    (restLen > kEps) ? posedFrames.length[i] / restLen : 1.0;
                const GfMatrix3d rotation = RigExecSmallestRotation(
                    restFrames.tangent[i], posedFrames.tangent[i]);
                gradients.left[i] = rotation * scale;
                gradients.right[i] = gradients.left[i];
                continue;
            }
            for (int side = 0; side < 2; ++side) {
                const bool left = (side == 0);
                const GfMatrix3d posed = posedFrames.GetScaledFrame(i, left);
                const GfMatrix3d rest = restFrames.GetScaledFrame(i, left);
                // rest is orthonormal-times-diagonal, so its inverse is
                // exactly S^-1 B^T; GetInverse is used for clarity and is
                // well conditioned because every scale is positive.
                const GfMatrix3d f = posed * rest.GetInverse();
                (left ? gradients.left : gradients.right)[i] = f;
            }
        }
    }
    return gradients;
}

RigExecCurvenetSampleGradients RigExecRemapGradientsToSamples(
    const RigExecCurvenetTopology &topology,
    const RigExecCurvenetSampling &sampling,
    const RigExecCurvenetFrames &frames,
    const RigExecCurvenetGradients &gradients)
{
    RigExecCurvenetSampleGradients out;
    const size_t sampleCount = sampling.GetSampleCount();
    out.left.assign(sampleCount, GfMatrix3d(1.0));
    out.right.assign(sampleCount, GfMatrix3d(1.0));

    for (size_t c = 0; c < topology.curves.size(); ++c) {
        const int base = sampling.curveBegin[c];
        const int n = sampling.GetCurveSampleCount(c);
        const int segBase = frames.segmentBegin[c];
        const int segCount = frames.segmentBegin[c + 1] - segBase;
        if (n <= 0 || segCount <= 0) {
            continue;
        }
        const bool closed = topology.curves[c].closed;
        for (int i = 0; i < n; ++i) {
            // Segments incident to sample i within this curve.
            int before = i - 1;
            int after = i;
            if (closed) {
                before = (i - 1 + segCount) % segCount;
                after = i % segCount;
            } else {
                if (after >= segCount) {
                    after = -1;
                }
            }
            auto pick = [&](const std::vector<GfMatrix3d> &source) {
                if (before >= 0 && after >= 0) {
                    return (source[segBase + before] + source[segBase + after]) *
                           0.5;
                }
                if (before >= 0) {
                    return source[segBase + before];
                }
                if (after >= 0) {
                    return source[segBase + after];
                }
                return GfMatrix3d(1.0);
            };
            out.left[base + i] = pick(gradients.left);
            out.right[base + i] = pick(gradients.right);
        }
    }
    return out;
}

}  // namespace rigExec
