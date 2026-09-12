//
// RigExec spline-IK spine kernel tests: the degree-2 curve against a de
// Boor reference and a closed-form arc length, rest reproduction on the
// real biped spine and neck (with the interior rest residual measured and
// printed, not asserted away), rigid-motion invariance, arc-length stretch,
// the exact squash formula, constant roll and a linear twist gradient, the
// mid control bending only the interior, and the degenerate inputs.
//
#include "rigExecMath/splineIk.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/matrix4d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace rigExec;

static int failures = 0;
static constexpr double kPi = 3.141592653589793238462643383279502884;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

static bool
Near(double a, double b, double tol = 1e-9)
{
    return std::abs(a - b) <= tol;
}

static bool
Near(const GfVec3d &a, const GfVec3d &b, double tol = 1e-9)
{
    return (a - b).GetLength() <= tol;
}

static GfVec3d
UnitX(const RigExecPointFrame &f)
{
    return (f.X() - f.Origin()).GetNormalized();
}

static GfVec3d
UnitY(const RigExecPointFrame &f)
{
    return (f.Y() - f.Origin()).GetNormalized();
}

// Frame with unit handles, X along xDirection, Y the projected up.
static RigExecPointFrame
MakeFrame(
    const GfVec3d &origin,
    const GfVec3d &xDirection = GfVec3d(1.0, 0.0, 0.0),
    const GfVec3d &upCandidate = GfVec3d(0.0, 1.0, 0.0))
{
    const GfVec3d x = xDirection.GetNormalized();
    GfVec3d up = upCandidate - x * GfDot(x, upCandidate);
    up.Normalize();
    RigExecPointFrame f;
    f.points = {origin, origin + x, origin + up, origin + GfCross(x, up)};
    f.flags = RigExecPointFrameValid;
    return f;
}

// Rigidly transforms every landmark of a frame: rotate about a pivot, then
// translate.
static RigExecPointFrame
Moved(
    const RigExecPointFrame &f, const GfVec3d &axis, double degrees,
    const GfVec3d &pivot, const GfVec3d &translation = GfVec3d(0.0))
{
    const GfRotation r(axis, degrees);
    RigExecPointFrame out = f;
    for (GfVec3d &p : out.points) {
        p = r.TransformDir(p - pivot) + pivot + translation;
    }
    return out;
}

// Chain rest frames: X aims at the next joint (the last joint repeats the
// previous aim), Y is the projected `up`.
static std::vector<RigExecPointFrame>
ChainFrames(const std::vector<GfVec3d> &origins, const GfVec3d &up)
{
    std::vector<RigExecPointFrame> frames;
    const size_t n = origins.size();
    for (size_t i = 0; i < n; ++i) {
        GfVec3d aim;
        if (i + 1 < n) {
            aim = origins[i + 1] - origins[i];
        } else if (n >= 2) {
            aim = origins[i] - origins[i - 1];
        } else {
            aim = GfVec3d(0.0, 1.0, 0.0);
        }
        frames.push_back(MakeFrame(origins[i], aim, up));
    }
    return frames;
}

// Reference de Boor evaluation of the degree-2 B-spline on the full knot
// vector [0,0,0,1,2,2,2], independent of the kernel's Bezier split.
static GfVec3d
DeBoor(const std::array<GfVec3d, 4> &cvs, double u)
{
    const double knots[7] = {0, 0, 0, 1, 2, 2, 2};
    const int p = 2;
    const int k = (u < 1.0) ? 2 : 3;  // knot span index with knots[k] <= u
    GfVec3d d[3];
    for (int j = 0; j <= p; ++j) {
        d[j] = cvs[j + k - p];
    }
    for (int r = 1; r <= p; ++r) {
        for (int j = p; j >= r; --j) {
            const int i = j + k - p;
            const double den = knots[i + p - r + 1] - knots[i];
            const double alpha = (den == 0.0) ? 0.0 : (u - knots[i]) / den;
            d[j] = d[j - 1] * (1.0 - alpha) + d[j] * alpha;
        }
    }
    return d[p];
}

// The real biped rest data (world cm) from joint_positions.data.
static const std::array<GfVec3d, 4> kSpineCvs = {
    GfVec3d(0.0, 94.6565, -3.4684), GfVec3d(0.0, 102.3655, -2.4877),
    GfVec3d(0.0, 128.2558, -3.3208), GfVec3d(0.0, 129.0082, -3.3208)};
static const std::vector<double> kSpineSegments = {
    7.771, 6.494, 6.492, 6.499, 6.496, 0.752};
static const std::vector<double> kSpineWeights = {
    0.1429, 0.2857, 0.4286, 0.5, 0.3571, 0.2143, 0.0714};

static const std::array<GfVec3d, 4> kNeckCvs = {
    GfVec3d(0.0, 145.0937, -6.9883), GfVec3d(0.0, 149.6725, -6.5980),
    GfVec3d(0.0, 157.3636, -5.9426), GfVec3d(0.0, 159.4253, -5.6901)};
static const std::vector<double> kNeckWeights = {0.16, 0.32, 0.4, 0.24, 0.08};

// Places `segments.size() - 1` interior points between a and b on the
// circular arc whose chord is |b - a| and whose consecutive chord lengths
// are `segments` (so the polyline is longer than the chord and bows toward
// `bulge`). Only the CV joints of the real chains are recorded in
// joint_positions.data; the interior rest origins are synthesised this
// way, which is enough for the rest-residual measurement.
static std::vector<GfVec3d>
ArcInterior(
    const GfVec3d &a, const GfVec3d &b, const std::vector<double> &segments,
    const GfVec3d &bulge)
{
    const double chord = (b - a).GetLength();
    const auto chordOf = [&](double radius) {
        double phi = 0.0;
        for (const double s : segments) {
            phi += 2.0 * std::asin(std::min(1.0, s / (2.0 * radius)));
        }
        return 2.0 * radius * std::sin(0.5 * phi);
    };
    // Bracket the radius from the half-circle limit upward; chordOf is
    // increasing in radius on that range.
    double lo = 0.0;
    for (const double s : segments) {
        lo = std::max(lo, 0.5 * s);
    }
    while (chordOf(lo) > chord) {
        lo *= 0.5;  // not reachable for sane data, keeps the loop finite
        if (lo < 1e-9) break;
    }
    double hi = 1e7;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (chordOf(mid) < chord) lo = mid; else hi = mid;
    }
    const double radius = 0.5 * (lo + hi);

    const GfVec3d e1 = (b - a).GetNormalized();
    GfVec3d e2 = bulge - e1 * GfDot(e1, bulge);
    e2.Normalize();
    const double sagitta = radius - std::sqrt(
        std::max(0.0, radius * radius - 0.25 * chord * chord));
    const GfVec3d centre = (a + b) * 0.5 + e2 * (sagitta - radius);
    const GfVec3d ra = (a - centre).GetNormalized();
    const GfVec3d rb = GfCross(GfCross(e1, e2).GetNormalized(), ra)
                           .GetNormalized();  // perpendicular, toward b
    const GfVec3d rbSigned = (GfDot(rb, e1) >= 0.0) ? rb : -rb;

    std::vector<GfVec3d> interior;
    double psi = 0.0;
    for (size_t i = 0; i + 1 < segments.size(); ++i) {
        psi += 2.0 * std::asin(segments[i] / (2.0 * radius));
        interior.push_back(
            centre + (ra * std::cos(psi) + rbSigned * std::sin(psi)) * radius);
    }
    return interior;
}

// Real-data rest chain: CV joints exact, interior joints synthesised.
static std::vector<GfVec3d>
RealChainOrigins(
    const std::array<GfVec3d, 4> &cvs, const std::vector<double> &segments)
{
    std::vector<GfVec3d> origins;
    origins.push_back(cvs[0]);
    origins.push_back(cvs[1]);
    // segments[1 .. n-3] lie between joint 1 (cvs[1]) and joint n-2 (cvs[2]).
    std::vector<double> inner(segments.begin() + 1, segments.end() - 1);
    const std::vector<GfVec3d> interior =
        ArcInterior(cvs[1], cvs[2], inner, GfVec3d(0.0, 0.0, -1.0));
    origins.insert(origins.end(), interior.begin(), interior.end());
    origins.push_back(cvs[2]);
    origins.push_back(cvs[3]);
    return origins;
}

struct RealRig {
    RigExecSplineIkRest rest;
    RigExecSplineIkControls controls;  // at rest
    std::vector<GfVec3d> origins;
};

static RealRig
MakeRealRig(
    const std::array<GfVec3d, 4> &cvs, const std::vector<double> &segments,
    const std::vector<double> &weights, RigExecSplineIkRestLength restLength)
{
    RealRig rig;
    rig.origins = RealChainOrigins(cvs, segments);
    const std::vector<RigExecPointFrame> joints =
        ChainFrames(rig.origins, GfVec3d(0.0, 0.0, -1.0));
    const RigExecPointFrame root = MakeFrame(cvs[0], GfVec3d(0, 1, 0),
                                             GfVec3d(0, 0, -1));
    const RigExecPointFrame mid = MakeFrame((cvs[1] + cvs[2]) * 0.5,
                                            GfVec3d(0, 1, 0), GfVec3d(0, 0, -1));
    const RigExecPointFrame end = MakeFrame(cvs[3], GfVec3d(0, 1, 0),
                                            GfVec3d(0, 0, -1));
    rig.rest = RigExecSplineIkMakeRest(joints, root, mid, end, weights,
                                       restLength);
    // The recorded rest spacing is ground truth; the synthesised interior
    // reproduces it to ~1e-9 but use the recorded numbers regardless.
    rig.rest.segmentLengths = segments;
    if (restLength == RigExecSplineIkRestLength::Chain) {
        rig.rest.restArcLength = 0.0;
        for (const double s : segments) rig.rest.restArcLength += s;
    }
    rig.controls = {root, mid, end};
    return rig;
}

// A synthetic straight chain along +Y: joints at y = 0..6, unit segments,
// CVs (0,0,0) (0,1,0) (0,5,0) (0,6,0), curve length 6.
struct StraightRig {
    RigExecSplineIkRest rest;
    RigExecSplineIkControls controls;
    std::vector<RigExecPointFrame> joints;
};

static StraightRig
MakeStraightRig(RigExecSplineIkRestLength restLength =
                    RigExecSplineIkRestLength::Chain,
                const std::vector<double> &weights = {})
{
    StraightRig rig;
    std::vector<GfVec3d> origins;
    for (int i = 0; i <= 6; ++i) origins.push_back(GfVec3d(0.0, i, 0.0));
    rig.joints = ChainFrames(origins, GfVec3d(0, 0, 1));
    const RigExecPointFrame root = MakeFrame(origins[0], GfVec3d(0, 1, 0),
                                             GfVec3d(0, 0, 1));
    const RigExecPointFrame mid = MakeFrame(origins[3], GfVec3d(0, 1, 0),
                                            GfVec3d(0, 0, 1));
    const RigExecPointFrame end = MakeFrame(origins[6], GfVec3d(0, 1, 0),
                                            GfVec3d(0, 0, 1));
    rig.rest = RigExecSplineIkMakeRest(rig.joints, root, mid, end, weights,
                                       restLength);
    rig.controls = {root, mid, end};
    return rig;
}

// ---------------------------------------------------------------------------

static void
TestCurveAgainstDeBoor()
{
    const std::array<GfVec3d, 4> cvs = {
        GfVec3d(0, 0, 0), GfVec3d(1, 2, 0), GfVec3d(3, 2, 1), GfVec3d(4, 0, 2)};
    const RigExecSplineIkCurve curve(cvs);
    GfVec3d p;
    curve.Evaluate(0.0, &p, nullptr);
    CHECK(Near(p, cvs[0], 1e-15));
    curve.Evaluate(2.0, &p, nullptr);
    CHECK(Near(p, cvs[3], 1e-15));
    curve.Evaluate(1.0, &p, nullptr);
    CHECK(Near(p, (cvs[1] + cvs[2]) * 0.5, 1e-15));
    for (int i = 0; i <= 200; ++i) {
        const double u = 2.0 * i / 200.0;
        GfVec3d d;
        curve.Evaluate(u, &p, &d);
        CHECK(Near(p, DeBoor(cvs, u), 1e-13));
        // Derivative against a central difference of the reference.
        const double h = 1e-6;
        if (u > h && u < 2.0 - h && std::abs(u - 1.0) > h) {
            const GfVec3d fd = (DeBoor(cvs, u + h) - DeBoor(cvs, u - h)) /
                               (2.0 * h);
            CHECK(Near(d, fd, 1e-8));
        }
    }
    // Real spine curve, against the same reference.
    const RigExecSplineIkCurve spine(kSpineCvs);
    for (int i = 0; i <= 50; ++i) {
        const double u = 2.0 * i / 50.0;
        spine.Evaluate(u, &p, nullptr);
        CHECK(Near(p, DeBoor(kSpineCvs, u), 1e-11));
    }
}

static void
TestArcLength()
{
    // Colinear CVs: the curve is the segment, length = chord.
    {
        const std::array<GfVec3d, 4> cvs = {
            GfVec3d(0, 0, 0), GfVec3d(1, 0, 0), GfVec3d(5, 0, 0),
            GfVec3d(6, 0, 0)};
        const RigExecSplineIkCurve curve(cvs);
        CHECK(Near(curve.ArcLength(), 6.0, 1e-13));
        // Arc-length parametrisation of a straight, non-uniformly
        // parametrised curve hits the expected points.
        for (int i = 0; i <= 12; ++i) {
            GfVec3d p, t;
            CHECK(curve.PointAtArcLength(0.5 * i, &p, &t));
            CHECK(Near(p, GfVec3d(0.5 * i, 0, 0), 1e-12));
            CHECK(Near(t, GfVec3d(1, 0, 0), 1e-12));
        }
        // Extrapolation past either end is straight.
        GfVec3d p, t;
        CHECK(curve.PointAtArcLength(7.5, &p, &t));
        CHECK(Near(p, GfVec3d(7.5, 0, 0), 1e-12));
        CHECK(curve.PointAtArcLength(-1.0, &p, &t));
        CHECK(Near(p, GfVec3d(-1.0, 0, 0), 1e-12));
    }
    // Closed form: span 0 is the parabola (t, t^2) for t in [0,1], length
    // sqrt(5)/2 + asinh(2)/4; span 1 is the segment (1,1)->(2,3), sqrt(5).
    {
        const std::array<GfVec3d, 4> cvs = {
            GfVec3d(0, 0, 0), GfVec3d(0.5, 0, 0), GfVec3d(1.5, 2, 0),
            GfVec3d(2, 3, 0)};
        const RigExecSplineIkCurve curve(cvs);
        const double parabola = std::sqrt(5.0) / 2.0 + std::asinh(2.0) / 4.0;
        CHECK(Near(curve.ArcLength(), parabola + std::sqrt(5.0), 1e-12));
        // Inverse arc length lands on the parabola at the right x.
        GfVec3d p;
        curve.Evaluate(curve.ParamAtArcLength(parabola), &p, nullptr);
        CHECK(Near(p, GfVec3d(1, 1, 0), 1e-11));
        // Round trip param -> distance -> param through the table.
        for (int i = 1; i < 40; ++i) {
            const double u = 2.0 * i / 40.0;
            GfVec3d q;
            curve.Evaluate(u, &q, nullptr);
            // Distance to q along the curve: integrate by fine sampling.
            double dist = 0.0;
            GfVec3d prev;
            curve.Evaluate(0.0, &prev, nullptr);
            const int steps = 20000;
            for (int k = 1; k <= steps; ++k) {
                GfVec3d cur;
                curve.Evaluate(u * k / steps, &cur, nullptr);
                dist += (cur - prev).GetLength();
                prev = cur;
            }
            CHECK(Near(curve.ParamAtArcLength(dist), u, 1e-7));
        }
    }
    // The real spine and neck: the rest curve versus the chain length.
    {
        const double spineCurve = RigExecSplineIkCurve(kSpineCvs).ArcLength();
        const double neckCurve = RigExecSplineIkCurve(kNeckCvs).ArcLength();
        std::printf("  spine rest curve length %.4f cm (chain 34.504)\n",
                    spineCurve);
        std::printf("  neck  rest curve length %.4f cm (chain 14.392)\n",
                    neckCurve);
        CHECK(Near(spineCurve, 34.3857, 5e-4));
        CHECK(Near(neckCurve, 14.3909, 5e-4));
    }
}

static void
TestTwistExtraction()
{
    const RigExecPointFrame rest = MakeFrame(GfVec3d(1, 2, 3), GfVec3d(0, 1, 0),
                                             GfVec3d(0, 0, 1));
    const GfVec3d axis(0, 1, 0);
    // Pure twist about the axis reads back exactly, with sign.
    for (const double deg : {0.0, 12.5, -40.0, 90.0, 179.0, -179.0}) {
        const RigExecPointFrame posed = Moved(rest, axis, deg, rest.Origin());
        CHECK(Near(RigExecSplineIkTwistAboutAxis(rest, posed, axis),
                   deg * kPi / 180.0, 1e-12));
    }
    // A swing about a perpendicular axis carries no twist.
    {
        const RigExecPointFrame posed = Moved(rest, GfVec3d(1, 0, 0), 70.0,
                                              rest.Origin(), GfVec3d(5, 5, 5));
        CHECK(Near(RigExecSplineIkTwistAboutAxis(rest, posed, axis), 0.0, 1e-12));
    }
    // Twist then swing: the twist survives the swing.
    {
        RigExecPointFrame posed = Moved(rest, axis, 30.0, rest.Origin());
        posed = Moved(posed, GfVec3d(1, 0, 0), 60.0, rest.Origin());
        CHECK(Near(RigExecSplineIkTwistAboutAxis(rest, posed, axis),
                   30.0 * kPi / 180.0, 1e-12));
    }
    // Uniform scale on the control does not change the twist.
    {
        RigExecPointFrame posed = Moved(rest, axis, 45.0, rest.Origin());
        for (int i = 1; i < 4; ++i) {
            posed.points[i] = posed.Origin() + (posed.points[i] - posed.Origin()) * 3.0;
        }
        CHECK(Near(RigExecSplineIkTwistAboutAxis(rest, posed, axis),
                   45.0 * kPi / 180.0, 1e-12));
    }
    // Degenerate axis and singular rest frame read as zero twist.
    {
        const RigExecPointFrame posed = Moved(rest, axis, 45.0, rest.Origin());
        CHECK(RigExecSplineIkTwistAboutAxis(rest, posed, GfVec3d(0.0)) == 0.0);
        RigExecPointFrame flat = rest;
        flat.points[3] = flat.points[0];
        CHECK(RigExecSplineIkTwistAboutAxis(flat, posed, axis) == 0.0);
    }
}

// Rest reproduction on the real data: endpoints exact, ratio and scale
// exactly one with the curve rest length, and the interior residual
// measured.
static void
TestRestReproduction(
    const char *name, const std::array<GfVec3d, 4> &cvs,
    const std::vector<double> &segments, const std::vector<double> &weights)
{
    const size_t n = segments.size() + 1;
    double chain = 0.0;
    for (const double s : segments) chain += s;

    // (a) restArcLength = rest curve length: ratio == 1, scale == 1.
    {
        const RealRig rig = MakeRealRig(cvs, segments, weights,
                                        RigExecSplineIkRestLength::Curve);
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, rig.controls, {}, &r));
        CHECK(r.joints.size() == n);
        CHECK(Near(r.ratio, 1.0, 1e-12));
        CHECK(Near(r.roll, 0.0, 1e-12));
        CHECK(Near(r.twist, 0.0, 1e-12));
        for (size_t i = 0; i < 4; ++i) CHECK(Near(r.cvs[i], cvs[i], 1e-12));
        CHECK(Near(r.joints[0].frame.Origin(), cvs[0], 1e-12));
        std::printf("  %s rest residual (restArcLength = curve %.4f, ratio %.6f):\n",
                    name, rig.rest.restArcLength, r.ratio);
        double worst = 0.0;
        for (size_t i = 0; i < n; ++i) {
            CHECK(Near(r.joints[i].scale, GfVec3d(1.0), 1e-12));
            CHECK(r.joints[i].frame.IsValid());
            CHECK(!r.joints[i].frame.IsDegenerate());
            const double residual =
                (r.joints[i].frame.Origin() - rig.origins[i]).GetLength();
            worst = std::max(worst, residual);
            const bool cvJoint = (i <= 1 || i + 2 >= n);
            std::printf("    joint %zu: %.4f cm%s\n", i, residual,
                        cvJoint ? "" : "  (interior, synthesised rest origin)");
        }
        // Not asserted tight: the degree-2 curve does not interpolate its
        // interior CVs. A gross bound catches a broken layout.
        CHECK(worst < 1.0);
        // The last joint overshoots the curve end by chain - curve, along
        // the end tangent.
        CHECK(Near(r.joints[n - 1].arcDistance, chain, 1e-12));
        CHECK(Near((r.joints[n - 1].frame.Origin() - cvs[3]).GetLength(),
                   chain - rig.rest.restArcLength, 1e-9));
    }
    // (b) restArcLength = chain length: the chain spans the curve exactly
    // and the last joint reproduces its rest origin.
    {
        const RealRig rig = MakeRealRig(cvs, segments, weights,
                                        RigExecSplineIkRestLength::Chain);
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, rig.controls, {}, &r));
        CHECK(Near(rig.rest.restArcLength, chain, 1e-12));
        CHECK(Near(r.ratio, r.arcLength / chain, 1e-15));
        CHECK(Near(r.joints[0].frame.Origin(), cvs[0], 1e-12));
        CHECK(Near(r.joints[n - 1].frame.Origin(), cvs[3], 1e-9));
        CHECK(Near(r.joints[n - 1].arcDistance, r.arcLength, 1e-12));
        CHECK(Near(r.joints[n - 1].arcParam, 1.0, 1e-12));
        std::printf("  %s rest residual (restArcLength = chain %.4f, ratio %.6f):\n",
                    name, chain, r.ratio);
        for (size_t i = 0; i < n; ++i) {
            const double residual =
                (r.joints[i].frame.Origin() - rig.origins[i]).GetLength();
            std::printf("    joint %zu: %.4f cm  scale %.6f\n", i, residual,
                        r.joints[i].scale[1]);
            // Exact squash formula at this (tiny) rest ratio.
            CHECK(Near(r.joints[i].scale[1],
                       1.0 - weights[i] * (r.ratio - 1.0), 1e-12));
        }
    }
}

static void
TestRigidMotionInvariance()
{
    // Real spine, translated: origins shift, ratio and spacing unchanged.
    {
        const RealRig rig = MakeRealRig(kSpineCvs, kSpineSegments,
                                        kSpineWeights,
                                        RigExecSplineIkRestLength::Curve);
        RigExecSplineIkResult atRest;
        CHECK(RigExecSolveSplineIk(rig.rest, rig.controls, {}, &atRest));
        const GfVec3d t(3.0, -2.0, 5.0);
        RigExecSplineIkControls moved = rig.controls;
        for (RigExecPointFrame *f : {&moved.root, &moved.mid, &moved.end}) {
            for (GfVec3d &p : f->points) p += t;
        }
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, moved, {}, &r));
        CHECK(Near(r.ratio, 1.0, 1e-12));
        CHECK(Near(r.arcLength, atRest.arcLength, 1e-11));
        for (size_t i = 0; i < r.joints.size(); ++i) {
            for (int k = 0; k < 4; ++k) {
                CHECK(Near(r.joints[i].frame.points[k],
                           atRest.joints[i].frame.points[k] + t, 1e-9));
            }
            if (i > 0) {
                const double spacing = (r.joints[i].frame.Origin() -
                                        r.joints[i - 1].frame.Origin()).GetLength();
                const double restSpacing = (atRest.joints[i].frame.Origin() -
                                            atRest.joints[i - 1].frame.Origin()).GetLength();
                CHECK(Near(spacing, restSpacing, 1e-9));
            }
        }
    }
    // Straight chain, arbitrary rigid motion of all three controls: every
    // joint frame is the rigidly moved rest frame, exactly.
    {
        const StraightRig rig = MakeStraightRig();
        const GfVec3d axis = GfVec3d(1.0, 2.0, -0.5).GetNormalized();
        const GfVec3d pivot(4.0, -1.0, 2.0);
        const GfVec3d t(-7.0, 3.0, 11.0);
        const double degrees = 137.0;
        RigExecSplineIkControls moved;
        moved.root = Moved(rig.controls.root, axis, degrees, pivot, t);
        moved.mid = Moved(rig.controls.mid, axis, degrees, pivot, t);
        moved.end = Moved(rig.controls.end, axis, degrees, pivot, t);
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, moved, {}, &r));
        CHECK(Near(r.ratio, 1.0, 1e-12));
        CHECK(Near(r.twist, 0.0, 1e-12));
        for (size_t i = 0; i < r.joints.size(); ++i) {
            const RigExecPointFrame expected =
                Moved(rig.joints[i], axis, degrees, pivot, t);
            for (int k = 0; k < 4; ++k) {
                CHECK(Near(r.joints[i].frame.points[k], expected.points[k], 1e-9));
            }
            CHECK(Near(r.joints[i].scale, GfVec3d(1.0), 1e-12));
        }
    }
}

static void
TestStretch()
{
    // Straight chain, end control pulled +3 along the chain: ratio 1.5,
    // joints at 1.5 spacing, chain spans the whole curve.
    const StraightRig rig = MakeStraightRig();
    RigExecSplineIkControls c = rig.controls;
    for (GfVec3d &p : c.end.points) p += GfVec3d(0.0, 3.0, 0.0);
    // Keep the mid control on its follow point so it adds no offset.
    for (GfVec3d &p : c.mid.points) p += GfVec3d(0.0, 1.5, 0.0);
    RigExecSplineIkResult r;
    CHECK(RigExecSolveSplineIk(rig.rest, c, {}, &r));
    CHECK(Near(r.arcLength, 9.0, 1e-12));
    CHECK(Near(r.ratio, 1.5, 1e-12));
    for (size_t i = 0; i < 7; ++i) {
        CHECK(Near(r.joints[i].frame.Origin(), GfVec3d(0.0, 1.5 * i, 0.0), 1e-11));
        CHECK(Near(r.joints[i].arcDistance, 1.5 * i, 1e-12));
        CHECK(Near(r.joints[i].arcParam, i / 6.0, 1e-12));
        CHECK(Near(UnitX(r.joints[i].frame), GfVec3d(0, 1, 0), 1e-12));
        CHECK(Near(UnitY(r.joints[i].frame), GfVec3d(0, 0, 1), 1e-12));
        // Handles: X stays unit, no weights so Y/Z stay unit too.
        CHECK(Near((r.joints[i].frame.X() - r.joints[i].frame.Origin()).GetLength(), 1.0, 1e-12));
        CHECK(Near((r.joints[i].frame.Y() - r.joints[i].frame.Origin()).GetLength(), 1.0, 1e-12));
    }
    CHECK(Near(r.joints[6].arcDistance, r.arcLength, 1e-12));

    // Pulling the end control off-axis bends the curve; the chain still
    // spans it and the spacing along the curve is proportional.
    RigExecSplineIkControls bent = rig.controls;
    for (GfVec3d &p : bent.end.points) p += GfVec3d(4.0, 2.0, 0.0);
    for (GfVec3d &p : bent.mid.points) p += GfVec3d(2.0, 1.0, 0.0);
    RigExecSplineIkResult b;
    CHECK(RigExecSolveSplineIk(rig.rest, bent, {}, &b));
    CHECK(b.ratio > 1.0);
    CHECK(Near(b.ratio, b.arcLength / 6.0, 1e-12));
    for (size_t i = 0; i < 7; ++i) {
        CHECK(Near(b.joints[i].arcDistance, b.ratio * i, 1e-12));
    }
    CHECK(Near(b.joints[6].frame.Origin(), b.cvs[3], 1e-9));
    // Interior joints aim at their successors.
    for (size_t i = 0; i + 1 < 7; ++i) {
        const GfVec3d chord = (b.joints[i + 1].frame.Origin() -
                               b.joints[i].frame.Origin()).GetNormalized();
        CHECK(Near(UnitX(b.joints[i].frame), chord, 1e-12));
    }
}

static void
TestSquash()
{
    // Exact formula against hand-computed values.
    const std::vector<double> weights = {0.0, 0.1429, 0.5, 1.0, 0.25, 0.3571, 0.0714};
    const StraightRig rig = MakeStraightRig(RigExecSplineIkRestLength::Chain, weights);
    const auto solveWithEndAt = [&](double y, double preserveVolume,
                                    RigExecSplineIkResult *r) {
        RigExecSplineIkControls c = rig.controls;
        for (GfVec3d &p : c.end.points) p += GfVec3d(0.0, y - 6.0, 0.0);
        for (GfVec3d &p : c.mid.points) p += GfVec3d(0.0, (y - 6.0) * 0.5, 0.0);
        RigExecSplineIkParams params;
        params.preserveVolume = preserveVolume;
        CHECK(RigExecSolveSplineIk(rig.rest, c, params, r));
    };
    RigExecSplineIkResult r;
    // ratio 1.5, preserveVolume 1: s = 1 - w * 0.5.
    solveWithEndAt(9.0, 1.0, &r);
    CHECK(Near(r.ratio, 1.5, 1e-12));
    CHECK(Near(r.joints[0].scale, GfVec3d(1.0, 1.0, 1.0), 1e-12));
    CHECK(Near(r.joints[2].scale, GfVec3d(1.0, 0.75, 0.75), 1e-12));
    CHECK(Near(r.joints[3].scale, GfVec3d(1.0, 0.5, 0.5), 1e-12));
    CHECK(Near(r.joints[1].scale, GfVec3d(1.0, 1.0 - 0.1429 * 0.5, 1.0 - 0.1429 * 0.5), 1e-12));
    // The Y/Z handles carry the scale, X does not.
    CHECK(Near((r.joints[3].frame.Y() - r.joints[3].frame.Origin()).GetLength(), 0.5, 1e-12));
    CHECK(Near((r.joints[3].frame.Z() - r.joints[3].frame.Origin()).GetLength(), 0.5, 1e-12));
    CHECK(Near((r.joints[3].frame.X() - r.joints[3].frame.Origin()).GetLength(), 1.0, 1e-12));
    CHECK(r.joints[3].frame.flags & RigExecPointFrameAffine);
    CHECK(!(r.joints[0].frame.flags & RigExecPointFrameAffine));
    // preserveVolume 0.4: s = 1 - w * 0.4 * 0.5.
    solveWithEndAt(9.0, 0.4, &r);
    CHECK(Near(r.joints[3].scale[1], 0.8, 1e-12));
    CHECK(Near(r.joints[2].scale[2], 0.9, 1e-12));
    // preserveVolume 0: no thinning at all.
    solveWithEndAt(9.0, 0.0, &r);
    for (const RigExecSplineIkJoint &j : r.joints) {
        CHECK(Near(j.scale, GfVec3d(1.0), 1e-12));
    }
    // ratio 0.8 (squash): s = 1 + w * 0.2, thickening.
    solveWithEndAt(4.8, 1.0, &r);
    CHECK(Near(r.ratio, 0.8, 1e-12));
    CHECK(Near(r.joints[3].scale, GfVec3d(1.0, 1.2, 1.2), 1e-12));
    CHECK(Near(r.joints[2].scale[1], 1.1, 1e-12));
    CHECK(Near(r.joints[4].scale[1], 1.05, 1e-12));
    CHECK(Near((r.joints[3].frame.Y() - r.joints[3].frame.Origin()).GetLength(), 1.2, 1e-12));
}

static void
TestTwist()
{
    const StraightRig rig = MakeStraightRig();
    const GfVec3d axis(0.0, 1.0, 0.0);  // the chain axis
    RigExecSplineIkResult plain;
    CHECK(RigExecSolveSplineIk(rig.rest, rig.controls, {}, &plain));

    // Signed rotation of the posed up about the aim relative to `plain`.
    const auto upRotation = [&](const RigExecSplineIkResult &r, size_t i) {
        const GfVec3d x = UnitX(r.joints[i].frame);
        const GfVec3d y0 = UnitY(plain.joints[i].frame);
        const GfVec3d y1 = UnitY(r.joints[i].frame);
        return std::atan2(GfDot(GfCross(y0, y1), x), GfDot(y0, y1));
    };

    // Root twist alone: every joint rolls by the same angle, and the
    // frames are the rest frames rotated about the chain axis.
    {
        const double deg = 35.0;
        RigExecSplineIkControls c = rig.controls;
        c.root = Moved(c.root, axis, deg, c.root.Origin());
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, c, {}, &r));
        CHECK(Near(r.roll, deg * kPi / 180.0, 1e-12));
        CHECK(Near(r.twist, -deg * kPi / 180.0, 1e-12));  // end - root
        CHECK(Near(r.ratio, 1.0, 1e-12));
        for (size_t i = 0; i < 7; ++i) {
            // roll + twist * t_i = roll * (1 - t_i): the end control did
            // not twist, so the gradient runs back to zero at the tip.
            const double expected = deg * kPi / 180.0 * (1.0 - i / 6.0);
            CHECK(Near(r.joints[i].twist, expected, 1e-12));
            CHECK(Near(upRotation(r, i), expected, 1e-12));
        }
    }
    // Root AND end twisted by the same angle: constant roll everywhere.
    {
        const double deg = 35.0;
        RigExecSplineIkControls c = rig.controls;
        c.root = Moved(c.root, axis, deg, c.root.Origin());
        c.end = Moved(c.end, axis, deg, c.end.Origin());
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, c, {}, &r));
        CHECK(Near(r.roll, deg * kPi / 180.0, 1e-12));
        CHECK(Near(r.twist, 0.0, 1e-12));
        for (size_t i = 0; i < 7; ++i) {
            CHECK(Near(r.joints[i].twist, deg * kPi / 180.0, 1e-12));
            CHECK(Near(upRotation(r, i), deg * kPi / 180.0, 1e-12));
            const RigExecPointFrame expected =
                Moved(rig.joints[i], axis, deg, GfVec3d(0.0));
            for (int k = 0; k < 4; ++k) {
                CHECK(Near(r.joints[i].frame.points[k], expected.points[k], 1e-11));
            }
        }
    }
    // End twist alone: a gradient exactly linear in t_i across the chain.
    {
        const double deg = -80.0;
        RigExecSplineIkControls c = rig.controls;
        c.end = Moved(c.end, axis, deg, c.end.Origin());
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, c, {}, &r));
        CHECK(Near(r.roll, 0.0, 1e-12));
        CHECK(Near(r.twist, deg * kPi / 180.0, 1e-12));
        const double slope = deg * kPi / 180.0;  // per unit t
        for (size_t i = 0; i < 7; ++i) {
            const double t = r.joints[i].arcParam;
            CHECK(Near(t, i / 6.0, 1e-12));
            CHECK(Near(r.joints[i].twist, slope * t, 1e-12));
            CHECK(Near(upRotation(r, i), slope * t, 1e-12));
            if (i > 0) {
                // Linearity: equal increments between equally spaced joints.
                CHECK(Near(upRotation(r, i) - upRotation(r, i - 1),
                           slope / 6.0, 1e-12));
            }
        }
    }
    // Twist with a swing on the end control: the swing carries no twist
    // and the bend does not disturb the gradient's endpoints.
    {
        RigExecSplineIkControls c = rig.controls;
        c.end = Moved(c.end, axis, 50.0, c.end.Origin());
        c.end = Moved(c.end, GfVec3d(1, 0, 0), 40.0, c.end.Origin());
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, c, {}, &r));
        CHECK(Near(r.roll, 0.0, 1e-12));
        CHECK(Near(r.twist, 50.0 * kPi / 180.0, 1e-12));
    }
    // The additive parameters stack on the control contribution.
    {
        RigExecSplineIkControls c = rig.controls;
        c.root = Moved(c.root, axis, 10.0, c.root.Origin());
        RigExecSplineIkParams params;
        params.roll = 0.25;
        params.twist = -0.5;
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, c, params, &r));
        CHECK(Near(r.roll, 10.0 * kPi / 180.0 + 0.25, 1e-12));
        CHECK(Near(r.twist, -10.0 * kPi / 180.0 - 0.5, 1e-12));
        CHECK(Near(r.joints[6].twist, r.roll + r.twist, 1e-12));
    }
}

static void
TestMidControl()
{
    const StraightRig rig = MakeStraightRig();
    RigExecSplineIkResult plain;
    CHECK(RigExecSolveSplineIk(rig.rest, rig.controls, {}, &plain));

    RigExecSplineIkControls c = rig.controls;
    for (GfVec3d &p : c.mid.points) p += GfVec3d(1.5, 0.0, 0.0);
    RigExecSplineIkResult r;
    CHECK(RigExecSolveSplineIk(rig.rest, c, {}, &r));
    // cv1 and cv2 took the offset, cv0 and cv3 did not.
    CHECK(Near(r.cvs[0], GfVec3d(0, 0, 0), 1e-12));
    CHECK(Near(r.cvs[1], GfVec3d(1.5, 1, 0), 1e-12));
    CHECK(Near(r.cvs[2], GfVec3d(1.5, 5, 0), 1e-12));
    CHECK(Near(r.cvs[3], GfVec3d(0, 6, 0), 1e-12));
    // The curve got longer; the endpoints stayed put; the interior moved
    // toward the offset, most in the middle.
    CHECK(r.ratio > 1.0);
    CHECK(Near(r.joints[0].frame.Origin(), GfVec3d(0, 0, 0), 1e-12));
    CHECK(Near(r.joints[6].frame.Origin(), GfVec3d(0, 6, 0), 1e-9));
    for (size_t i = 1; i < 6; ++i) {
        CHECK(r.joints[i].frame.Origin()[0] > 0.1);
        CHECK(r.joints[i].frame.Origin()[0] <= 1.5);
    }
    CHECK(r.joints[3].frame.Origin()[0] > r.joints[1].frame.Origin()[0]);
    CHECK(r.joints[3].frame.Origin()[0] > r.joints[5].frame.Origin()[0]);
    // No twist from a translation.
    CHECK(Near(r.roll, 0.0, 1e-12));
    CHECK(Near(r.twist, 0.0, 1e-12));
    // The up vectors stay in the bend plane's normal direction: the bend
    // is in the XY plane and the rest up is +Z, so up remains +Z exactly.
    for (size_t i = 0; i < 7; ++i) {
        CHECK(Near(UnitY(r.joints[i].frame), GfVec3d(0, 0, 1), 1e-12));
    }

    // The follow weight moves the follow point: with the end control
    // pulled up, a mid control that does not move registers an offset
    // equal to minus the follow displacement.
    {
        RigExecSplineIkControls pulled = rig.controls;
        for (GfVec3d &p : pulled.end.points) p += GfVec3d(0.0, 2.0, 0.0);
        std::array<GfVec3d, 4> cvs;
        RigExecSplineIkParams params;
        params.midFollowWeight = 0.5;
        CHECK(RigExecSplineIkPoseCvs(rig.rest, pulled, params, &cvs));
        CHECK(Near(cvs[1], GfVec3d(0, 1 - 1, 0), 1e-12));  // 1 - 0.5 * 2
        CHECK(Near(cvs[2], GfVec3d(0, 7 - 1, 0), 1e-12));
        params.midFollowWeight = 0.0;  // follows the root only: no offset
        CHECK(RigExecSplineIkPoseCvs(rig.rest, pulled, params, &cvs));
        CHECK(Near(cvs[1], GfVec3d(0, 1, 0), 1e-12));
        CHECK(Near(cvs[2], GfVec3d(0, 7, 0), 1e-12));
        params.midFollowWeight = 1.0;  // follows the end only: full offset
        CHECK(RigExecSplineIkPoseCvs(rig.rest, pulled, params, &cvs));
        CHECK(Near(cvs[1], GfVec3d(0, -1, 0), 1e-12));
        CHECK(Near(cvs[2], GfVec3d(0, 5, 0), 1e-12));
    }
}

static bool
AllFinite(const RigExecSplineIkResult &r)
{
    for (const RigExecSplineIkJoint &j : r.joints) {
        for (const GfVec3d &p : j.frame.points) {
            for (int k = 0; k < 3; ++k) {
                if (!std::isfinite(p[k])) return false;
            }
        }
        for (int k = 0; k < 3; ++k) {
            if (!std::isfinite(j.scale[k])) return false;
        }
    }
    return std::isfinite(r.arcLength) && std::isfinite(r.ratio) &&
           std::isfinite(r.roll) && std::isfinite(r.twist);
}

static void
TestDegenerates()
{
    // Coincident controls (all three posed identically): the carried CVs
    // still spread, so this is a legitimate solve.
    {
        const StraightRig rig = MakeStraightRig();
        RigExecSplineIkControls c;
        c.root = c.mid = c.end = rig.controls.root;
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, c, {}, &r));
        CHECK(AllFinite(r));
        CHECK(r.joints.size() == 7);
        // With every control at the root's rest pose: cv0, cv1 unchanged,
        // cv2, cv3 shifted down by 6, mid offset shifts cv1, cv2 by 3
        // more toward the root. Whatever the shape, the joints span it.
        CHECK(Near(r.joints[6].arcDistance, r.arcLength, 1e-12));
    }
    // Coincident CVs (a rest chain collapsed to a point): degenerate
    // curve, joints at the point with rest orientation, flagged.
    {
        std::vector<RigExecPointFrame> joints(4, MakeFrame(GfVec3d(1, 1, 1)));
        const RigExecPointFrame ctrl = MakeFrame(GfVec3d(1, 1, 1));
        RigExecSplineIkRest rest = RigExecSplineIkMakeRest(joints, ctrl, ctrl, ctrl);
        CHECK(rest.restArcLength == 0.0);
        rest.restArcLength = 1.0;  // otherwise the zero-rest-length path
        RigExecSplineIkResult r;
        CHECK(!RigExecSolveSplineIk(rest, {ctrl, ctrl, ctrl}, {}, &r));
        CHECK(AllFinite(r));
        CHECK(r.joints.size() == 4);
        CHECK(Near(r.ratio, 0.0, 1e-12));
        for (const RigExecSplineIkJoint &j : r.joints) {
            CHECK(j.frame.IsDegenerate());
            CHECK(Near(j.frame.Origin(), GfVec3d(1, 1, 1), 1e-12));
            CHECK(Near(UnitX(j.frame), GfVec3d(1, 0, 0), 1e-12));
        }
    }
    // Zero rest arc length: rest frames back, flagged, finite.
    {
        StraightRig rig = MakeStraightRig();
        rig.rest.restArcLength = 0.0;
        RigExecSplineIkResult r;
        CHECK(!RigExecSolveSplineIk(rig.rest, rig.controls, {}, &r));
        CHECK(AllFinite(r));
        CHECK(r.joints.size() == 7);
        for (size_t i = 0; i < 7; ++i) {
            CHECK(r.joints[i].frame.IsDegenerate());
            CHECK(r.joints[i].frame.points == rig.joints[i].points);
        }
    }
    // Single-joint chain: sits at the curve start, aimed along the start
    // tangent, and follows the root control.
    {
        const RigExecPointFrame joint = MakeFrame(GfVec3d(0, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1));
        const RigExecPointFrame ctrl = MakeFrame(GfVec3d(0, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1));
        RigExecSplineIkRest rest = RigExecSplineIkMakeRest({joint}, ctrl, ctrl, ctrl);
        CHECK(rest.segmentLengths.empty());
        CHECK(rest.restArcLength == 0.0);  // every CV coincides at rest
        // Give it a curve by moving the end control: the single joint
        // then lays at the start and aims along it.
        rest.restArcLength = 1.0;
        RigExecSplineIkControls c{ctrl, ctrl, ctrl};
        for (GfVec3d &p : c.end.points) p += GfVec3d(0, 4, 0);
        for (GfVec3d &p : c.mid.points) p += GfVec3d(0, 2, 0);
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rest, c, {}, &r));
        CHECK(r.joints.size() == 1);
        CHECK(!r.joints[0].frame.IsDegenerate());
        CHECK(Near(r.joints[0].frame.Origin(), GfVec3d(0, 0, 0), 1e-12));
        CHECK(Near(UnitX(r.joints[0].frame), GfVec3d(0, 1, 0), 1e-12));
        CHECK(Near(r.joints[0].arcParam, 0.0, 1e-12));
        CHECK(Near(r.ratio, 4.0, 1e-12));
    }
    // Empty chain and mismatched sizes: no joints, false.
    {
        RigExecSplineIkRest rest;
        RigExecSplineIkResult r;
        CHECK(!RigExecSolveSplineIk(rest, {}, {}, &r));
        CHECK(r.joints.empty());
        StraightRig rig = MakeStraightRig();
        rig.rest.volumeWeights = {1.0, 2.0};  // wrong size
        CHECK(!RigExecSolveSplineIk(rig.rest, rig.controls, {}, &r));
        CHECK(r.joints.empty());
    }
    // A singular rest control frame: rest frames back, flagged.
    {
        StraightRig rig = MakeStraightRig();
        rig.rest.rootControl.points[1] = rig.rest.rootControl.points[0];
        RigExecSplineIkResult r;
        CHECK(!RigExecSolveSplineIk(rig.rest, rig.controls, {}, &r));
        CHECK(AllFinite(r));
        for (const RigExecSplineIkJoint &j : r.joints) {
            CHECK(j.frame.IsDegenerate());
        }
    }
    // One collapsed rest joint frame: that joint alone is flagged; the
    // others solve normally.
    {
        StraightRig rig = MakeStraightRig();
        rig.rest.joints[3].points[2] = rig.rest.joints[3].points[0];
        RigExecSplineIkResult r;
        CHECK(!RigExecSolveSplineIk(rig.rest, rig.controls, {}, &r));
        CHECK(AllFinite(r));
        CHECK(r.joints[3].frame.IsDegenerate());
        for (size_t i = 0; i < 7; ++i) {
            if (i == 3) continue;
            CHECK(!r.joints[i].frame.IsDegenerate());
            CHECK(Near(r.joints[i].frame.Origin(), GfVec3d(0.0, i, 0.0), 1e-12));
        }
    }
    // A joint flipped to aim opposite its rest (end control folded back
    // over the root): the transport is deterministic and finite.
    {
        const StraightRig rig = MakeStraightRig();
        RigExecSplineIkControls c = rig.controls;
        c.end = Moved(c.end, GfVec3d(0, 0, 1), 180.0, GfVec3d(0, 0, 0));
        c.mid = Moved(c.mid, GfVec3d(0, 0, 1), 180.0, GfVec3d(0, 0, 0));
        RigExecSplineIkResult r;
        RigExecSolveSplineIk(rig.rest, c, {}, &r);
        CHECK(AllFinite(r));
    }
}

static void
TestMinLengthFloor()
{
    // Straight chain along +Y, chord 6. The end control is driven 4.5
    // back toward the root (chord 1.5 = 0.25 of rest); the mid control is
    // placed on its own follow point so it contributes nothing.
    const StraightRig rig = MakeStraightRig();
    const auto pushed = [&](double back) {
        RigExecSplineIkControls c = rig.controls;
        for (GfVec3d &p : c.end.points) p += GfVec3d(0.0, -back, 0.0);
        for (GfVec3d &p : c.mid.points) p += GfVec3d(0.0, -0.5 * back, 0.0);
        return c;
    };
    // Off (the default): the chain crumples onto the folded curve.
    {
        RigExecSplineIkResult r;
        RigExecSolveSplineIk(rig.rest, pushed(4.5), {}, &r);
        CHECK(Near(r.cvs[3], GfVec3d(0.0, 1.5, 0.0), 1e-12));
        CHECK(Near(r.cvs[2], GfVec3d(0.0, 0.5, 0.0), 1e-12));
        CHECK(r.ratio < 0.5);
        const double span = r.joints.back().frame.Origin()[1] -
                            r.joints.front().frame.Origin()[1];
        CHECK(span < 3.0);
    }
    // Floor 0.5: cv3 held 3.0 ahead of cv0 along the root's chain axis,
    // cv2 lifted with it, the curve straight, the joints laid at exactly
    // half spacing, ordered and all aiming forward.
    {
        RigExecSplineIkParams params;
        params.minLengthRatio = 0.5;
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, pushed(4.5), params, &r));
        CHECK(Near(r.cvs[3], GfVec3d(0.0, 3.0, 0.0), 1e-12));
        CHECK(Near(r.cvs[2], GfVec3d(0.0, 2.0, 0.0), 1e-12));
        CHECK(Near(r.cvs[1], GfVec3d(0.0, 1.0, 0.0), 1e-12));
        CHECK(Near(r.ratio, 0.5, 1e-12));
        CHECK(Near(r.twist, 0.0, 1e-12));
        for (size_t i = 0; i < r.joints.size(); ++i) {
            CHECK(Near(r.joints[i].frame.Origin(),
                       GfVec3d(0.0, 0.5 * i, 0.0), 1e-9));
            CHECK(GfDot(UnitX(r.joints[i].frame), GfVec3d(0, 1, 0)) > 0.99);
        }
    }
    // Past the root (end 7 back, cv3 below cv0): still held at 3.0.
    {
        RigExecSplineIkParams params;
        params.minLengthRatio = 0.5;
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, pushed(7.0), params, &r));
        CHECK(Near(r.cvs[3], GfVec3d(0.0, 3.0, 0.0), 1e-12));
        CHECK(Near(r.cvs[2], GfVec3d(0.0, 2.0, 0.0), 1e-12));
        CHECK(Near(r.ratio, 0.5, 1e-12));
    }
    // Inert whenever the chord is above the floor: a stretch and a mild
    // squash solve bit-for-bit as without it.
    for (double back : {-1.5, 2.0}) {
        RigExecSplineIkParams params;
        params.minLengthRatio = 0.5;
        RigExecSplineIkResult with, without;
        CHECK(RigExecSolveSplineIk(rig.rest, pushed(back), params, &with));
        CHECK(RigExecSolveSplineIk(rig.rest, pushed(back), {}, &without));
        for (int k = 0; k < 4; ++k) CHECK(with.cvs[k] == without.cvs[k]);
        CHECK(with.ratio == without.ratio);
        for (size_t i = 0; i < with.joints.size(); ++i) {
            for (int k = 0; k < 4; ++k) {
                CHECK(with.joints[i].frame.points[k] ==
                      without.joints[i].frame.points[k]);
            }
        }
    }
    // The axis is the ROOT's posed chain axis: turn every control 90
    // degrees about Z (chain now along -X) and push the end 4.5 back along
    // it; the floor lifts along -X, not world Y.
    {
        const GfVec3d z(0, 0, 1);
        RigExecSplineIkControls c;
        c.root = Moved(rig.controls.root, z, 90.0, GfVec3d(0.0));
        c.mid = Moved(rig.controls.mid, z, 90.0, GfVec3d(0.0),
                      GfVec3d(2.25, 0.0, 0.0));
        c.end = Moved(rig.controls.end, z, 90.0, GfVec3d(0.0),
                      GfVec3d(4.5, 0.0, 0.0));
        RigExecSplineIkParams params;
        params.minLengthRatio = 0.5;
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig.rest, c, params, &r));
        CHECK(Near(r.cvs[3], GfVec3d(-3.0, 0.0, 0.0), 1e-12));
        CHECK(Near(r.cvs[2], GfVec3d(-2.0, 0.0, 0.0), 1e-12));
        CHECK(Near(r.ratio, 0.5, 1e-12));
        for (size_t i = 0; i < r.joints.size(); ++i) {
            CHECK(Near(r.joints[i].frame.Origin(),
                       GfVec3d(-0.5 * i, 0.0, 0.0), 1e-9));
        }
    }
    // Real spine, chest driven straight down onto hip_swivel with the
    // shipped floor of 0.5: the chain keeps half its chord along the
    // root's axis, stays ordered along it and every bone aims forward.
    {
        const RealRig rig2 = MakeRealRig(kSpineCvs, kSpineSegments,
                                         kSpineWeights,
                                         RigExecSplineIkRestLength::Curve);
        const GfVec3d chord = kSpineCvs[3] - kSpineCvs[0];
        const GfVec3d axis = chord.GetNormalized();
        RigExecSplineIkControls c = rig2.controls;
        for (GfVec3d &p : c.end.points) p -= chord;
        for (GfVec3d &p : c.mid.points) p -= chord * 0.5;
        RigExecSplineIkParams params;
        params.minLengthRatio = 0.5;
        RigExecSplineIkResult r;
        CHECK(RigExecSolveSplineIk(rig2.rest, c, params, &r));
        const double along = GfDot(r.cvs[3] - r.cvs[0], axis);
        CHECK(Near(along, 0.5 * chord.GetLength(), 1e-9));
        double previous = -1.0;
        for (const RigExecSplineIkJoint &j : r.joints) {
            const double a = GfDot(j.frame.Origin() - r.cvs[0], axis);
            CHECK(a > previous);
            previous = a;
            CHECK(GfDot(UnitX(j.frame), axis) > 0.0);
        }
        CHECK(r.ratio >= 0.5 - 1e-9);
    }
}

int
main()
{
    std::printf("testRigExecSplineIk\n");
    TestCurveAgainstDeBoor();
    TestArcLength();
    TestTwistExtraction();
    TestRestReproduction("spine", kSpineCvs, kSpineSegments, kSpineWeights);
    {
        // Neck segments: the CV joints fix neck_0->neck_1 and neck_3->skull;
        // the recorded chain length 14.392 fixes the interior two.
        const double first = (kNeckCvs[1] - kNeckCvs[0]).GetLength();
        const double last = (kNeckCvs[3] - kNeckCvs[2]).GetLength();
        const double inner = 14.392 - first - last;
        const std::vector<double> neckSegments = {first, inner * 0.5, inner * 0.5, last};
        TestRestReproduction("neck", kNeckCvs, neckSegments, kNeckWeights);
    }
    TestRigidMotionInvariance();
    TestStretch();
    TestSquash();
    TestTwist();
    TestMidControl();
    TestMinLengthFloor();
    TestDegenerates();
    if (failures == 0) {
        std::printf("testRigExecSplineIk: PASS\n");
        return 0;
    }
    std::printf("testRigExecSplineIk: %d FAILURE(S)\n", failures);
    return 1;
}
