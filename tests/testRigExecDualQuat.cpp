//
// RigExec dual-quaternion tests: matrix round trips (including 180 degree
// rotations), single-influence exactness, exact translation blending,
// length preservation versus linear matrix blending, shortest-arc sign
// correction, unit-norm invariants, degenerate weights, and the scale/shear
// drop policy. Also prints a timing comparison of the direct point
// transform against the matrix path (informational, not asserted).
//
#include "rigExecMath/dualQuat.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/rotation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
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
Near(double a, double b, double tol = 1e-12)
{
    return std::abs(a - b) <= tol;
}

static bool
Near(const GfVec3d &a, const GfVec3d &b, double tol = 1e-12)
{
    return (a - b).GetLength() <= tol;
}

static bool
NearMatrix(const GfMatrix4d &a, const GfMatrix4d &b, double tol = 1e-12)
{
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!Near(a[i][j], b[i][j], tol)) {
                return false;
            }
        }
    }
    return true;
}

static bool
IsIdentity(const RigExecDualQuat &dq)
{
    return dq.real == GfQuatd(1.0) && dq.dual == GfQuatd(0.0);
}

// Row-vector rigid matrix: rotate by degrees about axis, then translate.
static GfMatrix4d
Rigid(const GfVec3d &axis, double degrees, const GfVec3d &translation)
{
    GfMatrix4d m;
    m.SetRotate(GfRotation(axis, degrees));
    m.SetTranslateOnly(translation);
    return m;
}

static GfVec3d
Apply(const GfMatrix4d &m, const GfVec3d &p)
{
    return m.TransformAffine(p);  // p * M, row-vector convention
}

// The whole module hinges on p * M == q.Transform(p) for the quaternion
// ExtractRotationQuat returns. Pin that with a case whose answer is known
// without any library: +90 degrees about Z takes +X to +Y.
static void
TestRowVectorConvention()
{
    const GfMatrix4d m = Rigid(GfVec3d(0, 0, 1), 90.0, GfVec3d(0));
    CHECK(Near(Apply(m, GfVec3d(1, 0, 0)), GfVec3d(0, 1, 0)));
    const GfQuatd q = m.ExtractRotationQuat();
    CHECK(Near(q.Transform(GfVec3d(1, 0, 0)), GfVec3d(0, 1, 0)));

    // And the translation lives in row 3 and is applied after rotation.
    const GfMatrix4d mt = Rigid(GfVec3d(0, 0, 1), 90.0, GfVec3d(5, 6, 7));
    CHECK(Near(Apply(mt, GfVec3d(1, 0, 0)), GfVec3d(5, 7, 7)));
    CHECK(Near(mt.ExtractTranslation(), GfVec3d(5, 6, 7)));

    // The dual quaternion reproduces the same mapping directly.
    const RigExecDualQuat dq = RigExecDualQuatFromMatrix(mt);
    CHECK(Near(RigExecDualQuatTransformPoint(dq, GfVec3d(1, 0, 0)),
               GfVec3d(5, 7, 7)));
    CHECK(Near(RigExecDualQuatTranslation(dq), GfVec3d(5, 6, 7)));
}

static void
TestMatrixRoundTrip()
{
    const GfVec3d axes[] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1),
        GfVec3d(1, 1, 0).GetNormalized(), GfVec3d(-1, 2, 3).GetNormalized(),
        GfVec3d(0.3, -0.9, 0.2).GetNormalized(),
    };
    const double angles[] = {
        0.0, 1e-7, 13.5, 45.0, 90.0, 135.0, 179.999, 180.0, -180.0,
        222.0, -90.0, 359.0,
    };
    const GfVec3d translations[] = {
        GfVec3d(0), GfVec3d(1, 2, 3), GfVec3d(-1000, 0.001, 42),
    };
    int cases = 0;
    for (const GfVec3d &axis : axes) {
        for (const double angle : angles) {
            for (const GfVec3d &t : translations) {
                const GfMatrix4d m = Rigid(axis, angle, t);
                bool rigid = false;
                const RigExecDualQuat dq = RigExecDualQuatFromMatrix(m, &rigid);
                CHECK(rigid);
                CHECK(RigExecDualQuatIsUnit(dq, 1e-13));
                const GfMatrix4d back = RigExecDualQuatToMatrix(dq);
                // Tight relative tolerance: the -1000 translation column
                // carries magnitude 1e3 so allow 1e3 ulps-ish there.
                CHECK(NearMatrix(m, back, 1e-12 * std::max(1.0, t.GetLength())));
                // The direct point transform agrees with the matrix path.
                const GfVec3d p(0.7, -1.3, 2.9);
                CHECK(Near(RigExecDualQuatTransformPoint(dq, p),
                           Apply(m, p), 1e-11 * std::max(1.0, t.GetLength())));
                ++cases;
            }
        }
    }
    CHECK(cases == 6 * 12 * 3);

    // Composition round trip: a non-axis-aligned product of rotations.
    GfMatrix4d m = Rigid(GfVec3d(1, 0, 0), 30.0, GfVec3d(1, 0, 0)) *
                   Rigid(GfVec3d(0, 1, 0), -70.0, GfVec3d(0, 2, 0)) *
                   Rigid(GfVec3d(0, 0, 1), 200.0, GfVec3d(0, 0, 3));
    const RigExecDualQuat dq = RigExecDualQuatFromMatrix(m);
    CHECK(NearMatrix(m, RigExecDualQuatToMatrix(dq), 1e-12));
}

static void
TestSingleInfluenceExact()
{
    const GfMatrix4d m = Rigid(
        GfVec3d(2, -1, 0.5).GetNormalized(), 123.0, GfVec3d(4, -5, 6));
    const RigExecDualQuat joint = RigExecDualQuatFromMatrix(m);

    // Weight 1 on one joint: bit-identical to the input, since the
    // normalisation of an already unit input is a multiply by exactly 1.0
    // followed by a projection along a vanishing component.
    const double one = 1.0;
    RigExecDualQuat blended;
    CHECK(RigExecBlendDualQuats(&joint, &one, 1, &blended));
    CHECK(Near(blended.real.GetReal(), joint.real.GetReal(), 1e-15));
    CHECK(Near(blended.real.GetImaginary(), joint.real.GetImaginary(), 1e-15));
    CHECK(Near(blended.dual.GetReal(), joint.dual.GetReal(), 1e-15));
    CHECK(Near(blended.dual.GetImaginary(), joint.dual.GetImaginary(), 1e-15));
    CHECK(NearMatrix(RigExecDualQuatToMatrix(blended), m, 1e-13));

    // Any single non-zero weight, including negative and huge, is the same
    // transform: the normalisation absorbs it.
    for (const double w : {0.25, 7.0, -1.0, 1e-6, 1e6}) {
        CHECK(RigExecBlendDualQuats(&joint, &w, 1, &blended));
        CHECK(NearMatrix(RigExecDualQuatToMatrix(blended), m, 1e-12));
    }

    // Zero-weighted companions do not perturb the single influence.
    const RigExecDualQuat pair[2] = {
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(0, 1, 0), 170.0, GfVec3d(9, 9, 9))),
        joint};
    const double weights[2] = {0.0, 1.0};
    CHECK(RigExecBlendDualQuats(pair, weights, 2, &blended));
    CHECK(NearMatrix(RigExecDualQuatToMatrix(blended), m, 1e-13));
}

static void
TestPureTranslationBlendIsExact()
{
    const GfVec3d ta(1, 2, 3);
    const GfVec3d tb(-5, 10, 0.25);
    const RigExecDualQuat dqs[2] = {
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(1, 0, 0), 0.0, ta)),
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(1, 0, 0), 0.0, tb)),
    };
    const double half[2] = {0.5, 0.5};
    RigExecDualQuat blended;
    CHECK(RigExecBlendDualQuats(dqs, half, 2, &blended));
    // Translation-only blending is linear, hence exact: (ta + tb) / 2.
    const GfVec3d midpoint = (ta + tb) * 0.5;
    CHECK(Near(RigExecDualQuatTranslation(blended), midpoint, 1e-15));
    CHECK(blended.real == GfQuatd(1.0));
    const GfVec3d p(0.1, 0.2, 0.3);
    CHECK(Near(RigExecDualQuatTransformPoint(blended, p), p + midpoint, 1e-15));

    // Unnormalised weights give the weighted mean, still exactly.
    const double lopsided[2] = {3.0, 1.0};
    CHECK(RigExecBlendDualQuats(dqs, lopsided, 2, &blended));
    CHECK(Near(RigExecDualQuatTranslation(blended),
               (ta * 3.0 + tb) * 0.25, 1e-15));
}

// The reason DQS exists: linear blending of two joint matrices with a large
// relative rotation collapses the skinned geometry toward the rotation axis
// (candy wrapper); the dual-quaternion blend stays rigid.
static void
TestLengthPreservationVersusLinearBlend()
{
    const GfMatrix4d ma = Rigid(GfVec3d(0, 0, 1), 0.0, GfVec3d(0));
    const GfMatrix4d mb = Rigid(GfVec3d(0, 0, 1), 150.0, GfVec3d(0));
    const RigExecDualQuat dqs[2] = {
        RigExecDualQuatFromMatrix(ma), RigExecDualQuatFromMatrix(mb)};
    const double half[2] = {0.5, 0.5};
    RigExecDualQuat blended;
    CHECK(RigExecBlendDualQuats(dqs, half, 2, &blended));
    CHECK(RigExecDualQuatIsUnit(blended, 1e-14));

    const GfVec3d p(1.0, 0.0, 0.0);

    // Linear matrix blending, computed inline: 0.5 * (p * Ma) + 0.5 * (p * Mb).
    const GfVec3d linear = Apply(ma, p) * 0.5 + Apply(mb, p) * 0.5;
    const double linearLength = linear.GetLength();
    // |p| cos(75 deg) = 0.2588...: the point has collapsed toward the axis.
    CHECK(Near(linearLength, std::cos(75.0 * kPi / 180.0), 1e-12));
    CHECK(linearLength < 0.3);

    // DQS: rigid, so |p'| == |p| == 1 exactly (to rounding).
    const GfVec3d dqsPoint = RigExecDualQuatTransformPoint(blended, p);
    CHECK(Near(dqsPoint.GetLength(), 1.0, 1e-14));
    // And it is the 75 degree rotation of p, i.e. the screw midpoint.
    CHECK(Near(dqsPoint, GfVec3d(std::cos(75.0 * kPi / 180.0),
                                 std::sin(75.0 * kPi / 180.0), 0.0), 1e-13));
    CHECK(std::abs(dqsPoint.GetLength() - 1.0) <
          std::abs(linearLength - 1.0));

    // With translations on both joints the blended motion is still rigid:
    // the distance between two co-skinned points is preserved by DQS and
    // shortened by the linear blend.
    const GfMatrix4d mat = Rigid(GfVec3d(0, 0, 1), 0.0, GfVec3d(1, 2, 3));
    const GfMatrix4d mbt = Rigid(GfVec3d(0, 0, 1), 150.0, GfVec3d(-2, 0, 5));
    const RigExecDualQuat dqt[2] = {
        RigExecDualQuatFromMatrix(mat), RigExecDualQuatFromMatrix(mbt)};
    const double w[2] = {0.6, 0.4};
    CHECK(RigExecBlendDualQuats(dqt, w, 2, &blended));
    CHECK(RigExecDualQuatIsUnit(blended, 1e-14));
    const GfVec3d q0(0.3, 0.1, -0.4);
    const GfVec3d q1(1.3, -0.7, 0.2);
    const double restDistance = (q1 - q0).GetLength();
    const double dqsDistance =
        (RigExecDualQuatTransformPoint(blended, q1) -
         RigExecDualQuatTransformPoint(blended, q0)).GetLength();
    const GfVec3d l0 = Apply(mat, q0) * w[0] + Apply(mbt, q0) * w[1];
    const GfVec3d l1 = Apply(mat, q1) * w[0] + Apply(mbt, q1) * w[1];
    const double linearDistance = (l1 - l0).GetLength();
    CHECK(Near(dqsDistance, restDistance, 1e-13));
    CHECK(linearDistance < 0.7 * restDistance);
    CHECK(std::abs(dqsDistance - restDistance) <
          std::abs(linearDistance - restDistance));
    // The blended matrix is orthonormal too.
    bool rigid = false;
    RigExecDualQuatFromMatrix(RigExecDualQuatToMatrix(blended), &rigid);
    CHECK(rigid);
}

static void
TestShortestArcSignCorrection()
{
    // Antipodal representation of the same motion: -dq encodes exactly the
    // transform dq does. Blending them 0.5/0.5 without sign correction
    // would sum to zero; with it, the result is dq itself.
    const GfMatrix4d m = Rigid(
        GfVec3d(1, 1, 1).GetNormalized(), 100.0, GfVec3d(1, -2, 3));
    const RigExecDualQuat dq = RigExecDualQuatFromMatrix(m);
    const RigExecDualQuat neg(-dq.real, -dq.dual);
    CHECK(NearMatrix(RigExecDualQuatToMatrix(neg), m, 1e-13));
    const RigExecDualQuat pair[2] = {dq, neg};
    const double half[2] = {0.5, 0.5};
    RigExecDualQuat blended;
    CHECK(RigExecBlendDualQuats(pair, half, 2, &blended));
    CHECK(NearMatrix(RigExecDualQuatToMatrix(blended), m, 1e-13));

    // Reference order matters: the reference is the first non-zero weight,
    // so the result takes the sign of whichever input comes first, but the
    // motion is the same either way.
    const RigExecDualQuat pairReversed[2] = {neg, dq};
    RigExecDualQuat blendedReversed;
    CHECK(RigExecBlendDualQuats(pairReversed, half, 2, &blendedReversed));
    CHECK(NearMatrix(RigExecDualQuatToMatrix(blendedReversed), m, 1e-13));
    CHECK(GfDot(blended.real, blendedReversed.real) < 0.0);

    // Rotations of +170 and -170 degrees about Z, built from explicit
    // quaternions (cos 85, 0, 0, +-sin 85) so the sign choice is ours and
    // not whatever matrix extraction happens to return. Their dot product
    // is cos 170 deg < 0, so the naive average (2 cos 85, 0, 0, 0) is the
    // 0 degree rotation, 180 degrees away from the right answer. The
    // shortest arc between them passes through 180 degrees: +X maps to -X.
    const double c85 = std::cos(85.0 * kPi / 180.0);
    const double s85 = std::sin(85.0 * kPi / 180.0);
    const RigExecDualQuat wrap[2] = {
        RigExecDualQuatFromRotationTranslation(
            GfQuatd(c85, GfVec3d(0, 0, s85)), GfVec3d(0)),
        RigExecDualQuatFromRotationTranslation(
            GfQuatd(c85, GfVec3d(0, 0, -s85)), GfVec3d(0)),
    };
    CHECK(GfDot(wrap[0].real, wrap[1].real) < 0.0);
    CHECK(Near(RigExecDualQuatTransformPoint(wrap[0], GfVec3d(1, 0, 0)),
               Apply(Rigid(GfVec3d(0, 0, 1), 170.0, GfVec3d(0)),
                     GfVec3d(1, 0, 0)), 1e-13));
    CHECK(Near(RigExecDualQuatTransformPoint(wrap[1], GfVec3d(1, 0, 0)),
               Apply(Rigid(GfVec3d(0, 0, 1), -170.0, GfVec3d(0)),
                     GfVec3d(1, 0, 0)), 1e-13));
    CHECK(RigExecBlendDualQuats(wrap, half, 2, &blended));
    CHECK(Near(RigExecDualQuatTransformPoint(blended, GfVec3d(1, 0, 0)),
               GfVec3d(-1, 0, 0), 1e-13));
    // The naive (uncorrected) sum really is the wrong answer: compute it
    // inline so the test documents what the sign correction prevents.
    RigExecDualQuat naive(wrap[0].real * 0.5 + wrap[1].real * 0.5,
                          wrap[0].dual * 0.5 + wrap[1].dual * 0.5);
    CHECK(RigExecDualQuatNormalize(&naive));
    CHECK(Near(RigExecDualQuatTransformPoint(naive, GfVec3d(1, 0, 0)),
               GfVec3d(1, 0, 0), 1e-13));

    // Three influences where only the middle one needs flipping.
    const RigExecDualQuat three[3] = {
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(0, 1, 0), 10.0, GfVec3d(1, 0, 0))),
        RigExecDualQuat(-wrap[0].real, -wrap[0].dual),
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(0, 1, 0), 20.0, GfVec3d(0, 1, 0))),
    };
    const RigExecDualQuat threeUnflipped[3] = {three[0], wrap[0], three[2]};
    const double thirds[3] = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
    RigExecDualQuat a;
    RigExecDualQuat b;
    CHECK(RigExecBlendDualQuats(three, thirds, 3, &a));
    CHECK(RigExecBlendDualQuats(threeUnflipped, thirds, 3, &b));
    CHECK(NearMatrix(RigExecDualQuatToMatrix(a), RigExecDualQuatToMatrix(b), 1e-13));
}

static void
TestUnitNormInvariant()
{
    std::vector<RigExecDualQuat> dqs;
    std::vector<double> weights;
    const GfVec3d axes[] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1),
        GfVec3d(1, -1, 1).GetNormalized(),
    };
    for (int i = 0; i < 8; ++i) {
        dqs.push_back(RigExecDualQuatFromMatrix(Rigid(
            axes[i % 4], 37.0 * i - 100.0, GfVec3d(i, -2.0 * i, 0.5 * i))));
        weights.push_back(0.1 * (i + 1));  // sums to 3.6, deliberately not 1
    }
    RigExecDualQuat blended;
    CHECK(RigExecBlendDualQuats(dqs.data(), weights.data(), dqs.size(), &blended));
    CHECK(RigExecDualQuatIsUnit(blended, 1e-14));
    CHECK(Near(blended.real.GetLength(), 1.0, 1e-15));
    CHECK(Near(GfDot(blended.real, blended.dual), 0.0, 1e-15));
    bool rigid = false;
    const GfMatrix4d m = RigExecDualQuatToMatrix(blended);
    RigExecDualQuatFromMatrix(m, &rigid);
    CHECK(rigid);
    // Direct transform and matrix transform agree on the blended motion.
    const GfVec3d p(3, -4, 5);
    CHECK(Near(RigExecDualQuatTransformPoint(blended, p), Apply(m, p), 1e-12));

    // Normalise on a deliberately scaled, non-orthogonal dual pair.
    RigExecDualQuat scaled(blended.real * 3.0,
                           blended.dual * 3.0 + blended.real * 0.7);
    CHECK(!RigExecDualQuatIsUnit(scaled));
    CHECK(RigExecDualQuatNormalize(&scaled));
    CHECK(RigExecDualQuatIsUnit(scaled, 1e-14));
    // The stray component along the real part never touched the motion.
    CHECK(NearMatrix(RigExecDualQuatToMatrix(scaled), m, 1e-13));
}

static void
TestDegenerateWeights()
{
    const RigExecDualQuat dqs[3] = {
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(1, 0, 0), 40.0, GfVec3d(1, 0, 0))),
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(0, 1, 0), -80.0, GfVec3d(0, 2, 0))),
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(0, 0, 1), 120.0, GfVec3d(0, 0, 3))),
    };
    RigExecDualQuat out(dqs[2].real, dqs[2].dual);  // pre-dirty the output

    // Empty.
    CHECK(!RigExecBlendDualQuats(dqs, nullptr, 0, &out));
    CHECK(IsIdentity(out));
    CHECK(!RigExecBlendDualQuats(nullptr, nullptr, 0, &out));
    CHECK(IsIdentity(out));

    // All weights zero.
    const double zeros[3] = {0.0, 0.0, 0.0};
    out = dqs[1];
    CHECK(!RigExecBlendDualQuats(dqs, zeros, 3, &out));
    CHECK(IsIdentity(out));

    // Weights summing to zero on identical inputs: exact cancellation.
    const RigExecDualQuat same[2] = {dqs[0], dqs[0]};
    const double cancel[2] = {1.0, -1.0};
    out = dqs[1];
    CHECK(!RigExecBlendDualQuats(same, cancel, 2, &out));
    CHECK(IsIdentity(out));

    // Weights summing to zero on different inputs is NOT degenerate: the
    // sign correction keeps the accumulation constructive and the result
    // is a well-defined unit motion. Document that by asserting it.
    out = RigExecDualQuat();
    CHECK(RigExecBlendDualQuats(dqs, cancel, 2, &out));
    CHECK(RigExecDualQuatIsUnit(out, 1e-14));

    // Near-zero-norm accumulation: weights that nearly cancel.
    const double nearCancel[2] = {1.0, -(1.0 - 1e-12)};
    out = dqs[1];
    CHECK(!RigExecBlendDualQuats(same, nearCancel, 2, &out));
    CHECK(IsIdentity(out));
    // ... but a clearly resolvable near-cancellation still normalises to
    // the same single motion, because the scale is absorbed.
    const double resolvable[2] = {1.0, -0.5};
    CHECK(RigExecBlendDualQuats(same, resolvable, 2, &out));
    CHECK(NearMatrix(RigExecDualQuatToMatrix(out),
                     RigExecDualQuatToMatrix(dqs[0]), 1e-12));

    // Non-finite weight fails atomically.
    const double nan[3] = {0.5, std::numeric_limits<double>::quiet_NaN(), 0.5};
    out = dqs[1];
    CHECK(!RigExecBlendDualQuats(dqs, nan, 3, &out));
    CHECK(IsIdentity(out));
    const double inf[3] = {0.5, 0.5, std::numeric_limits<double>::infinity()};
    CHECK(!RigExecBlendDualQuats(dqs, inf, 3, &out));
    CHECK(IsIdentity(out));

    // Normalising a zero or non-finite dual quaternion resets to identity.
    RigExecDualQuat zero(GfQuatd(0.0), GfQuatd(0.0));
    CHECK(!RigExecDualQuatNormalize(&zero));
    CHECK(IsIdentity(zero));
    RigExecDualQuat bad(
        GfQuatd(1.0), GfQuatd(std::numeric_limits<double>::quiet_NaN()));
    CHECK(!RigExecDualQuatNormalize(&bad));
    CHECK(IsIdentity(bad));
}

static void
TestIndexedBlend()
{
    const RigExecDualQuat palette[3] = {
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(1, 0, 0), 40.0, GfVec3d(1, 0, 0))),
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(0, 1, 0), -80.0, GfVec3d(0, 2, 0))),
        RigExecDualQuatFromMatrix(Rigid(GfVec3d(0, 0, 1), 120.0, GfVec3d(0, 0, 3))),
    };
    // Same influences through both entry points give the same motion.
    const int indices[2] = {2, 0};
    const float fweights[2] = {0.25f, 0.75f};
    const RigExecDualQuat gathered[2] = {palette[2], palette[0]};
    const double dweights[2] = {0.25, 0.75};
    RigExecDualQuat a;
    RigExecDualQuat b;
    CHECK(RigExecBlendDualQuats(palette, 3, indices, fweights, 2, &a));
    CHECK(RigExecBlendDualQuats(gathered, dweights, 2, &b));
    CHECK(NearMatrix(RigExecDualQuatToMatrix(a), RigExecDualQuatToMatrix(b), 1e-14));

    // Out-of-range index with non-zero weight: degenerate.
    const int badIndices[2] = {0, 3};
    CHECK(!RigExecBlendDualQuats(palette, 3, badIndices, fweights, 2, &a));
    CHECK(IsIdentity(a));
    const int negIndices[2] = {-1, 1};
    CHECK(!RigExecBlendDualQuats(palette, 3, negIndices, fweights, 2, &a));
    CHECK(IsIdentity(a));
    // Out-of-range index with zero weight (padding) is ignored.
    const float padded[2] = {1.0f, 0.0f};
    CHECK(RigExecBlendDualQuats(palette, 3, badIndices, padded, 2, &a));
    CHECK(NearMatrix(RigExecDualQuatToMatrix(a),
                     RigExecDualQuatToMatrix(palette[0]), 1e-13));
}

static void
TestScaleShearPolicy()
{
    // Uniform scale: rotation and translation survive, scale is dropped.
    GfMatrix4d scaled = Rigid(GfVec3d(0, 1, 0), 60.0, GfVec3d(0));
    scaled = GfMatrix4d().SetScale(GfVec3d(2.0, 2.0, 2.0)) * scaled;
    scaled.SetTranslateOnly(GfVec3d(1, 2, 3));
    bool rigid = true;
    const RigExecDualQuat dq = RigExecDualQuatFromMatrix(scaled, &rigid);
    CHECK(!rigid);
    CHECK(RigExecDualQuatIsUnit(dq, 1e-13));
    CHECK(NearMatrix(RigExecDualQuatToMatrix(dq),
                     Rigid(GfVec3d(0, 1, 0), 60.0, GfVec3d(1, 2, 3)), 1e-12));

    // Non-uniform scale plus shear: still the polar rotation.
    GfMatrix4d sheared = Rigid(GfVec3d(1, 0, 0), -35.0, GfVec3d(0));
    GfMatrix4d shear(1.0);
    shear[0][1] = 0.4;  // row-vector shear: x picks up 0.4 of y
    sheared = shear * GfMatrix4d().SetScale(GfVec3d(1.5, 0.5, 3.0)) * sheared;
    sheared.SetTranslateOnly(GfVec3d(-4, 0, 9));
    rigid = true;
    const RigExecDualQuat dq2 = RigExecDualQuatFromMatrix(sheared, &rigid);
    CHECK(!rigid);
    CHECK(RigExecDualQuatIsUnit(dq2, 1e-12));
    CHECK(Near(RigExecDualQuatTranslation(dq2), GfVec3d(-4, 0, 9)));
    const GfMatrix4d rigidPart = RigExecDualQuatToMatrix(dq2);
    bool rigidBack = false;
    RigExecDualQuatFromMatrix(rigidPart, &rigidBack);
    CHECK(rigidBack);
    // Polar factor property: R is the rotation nearest to the linear part
    // L, so R^T L is symmetric (that is the definition of the polar
    // stretch). Check symmetry of R^T L in the row convention: S = L * R^T.
    GfMatrix4d rt = rigidPart.GetTranspose();
    rt.SetTranslateOnly(GfVec3d(0));
    GfMatrix4d l = sheared;
    l.SetTranslateOnly(GfVec3d(0));
    const GfMatrix4d s = l * rt;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK(Near(s[i][j], s[j][i], 1e-10));
        }
    }

    // Reflection: dropped, a proper rotation comes back.
    GfMatrix4d mirrored = Rigid(GfVec3d(0, 0, 1), 30.0, GfVec3d(0));
    mirrored = GfMatrix4d().SetScale(GfVec3d(1.0, -1.0, 1.0)) * mirrored;
    rigid = true;
    const RigExecDualQuat dq3 = RigExecDualQuatFromMatrix(mirrored, &rigid);
    CHECK(!rigid);
    CHECK(RigExecDualQuatIsUnit(dq3, 1e-13));
    rigidBack = false;
    RigExecDualQuatFromMatrix(RigExecDualQuatToMatrix(dq3), &rigidBack);
    CHECK(rigidBack);

    // Singular: identity rotation, translation kept.
    GfMatrix4d singular(0.0);
    singular[3][3] = 1.0;
    singular.SetTranslateOnly(GfVec3d(7, 8, 9));
    rigid = true;
    const RigExecDualQuat dq4 = RigExecDualQuatFromMatrix(singular, &rigid);
    CHECK(!rigid);
    CHECK(dq4.real == GfQuatd(1.0));
    CHECK(Near(RigExecDualQuatTranslation(dq4), GfVec3d(7, 8, 9)));
}

// Informational: is the direct point transform cheaper than going through
// a matrix? Measures the per-point skinning pattern (one blended dual
// quaternion per point) and a cluster pattern (many points per dual
// quaternion). Not asserted; the numbers go in the header's cost note.
static void
MeasureTransformCost()
{
    using Clock = std::chrono::steady_clock;
    const size_t n = 200000;
    std::vector<RigExecDualQuat> dqs(n);
    std::vector<GfVec3d> points(n);
    for (size_t i = 0; i < n; ++i) {
        const double a = 0.001 * static_cast<double>(i);
        dqs[i] = RigExecDualQuatFromRotationTranslation(
            GfQuatd(std::cos(a), GfVec3d(std::sin(a), 0, 0)),
            GfVec3d(a, -a, 2 * a));
        points[i] = GfVec3d(std::sin(a), std::cos(a), a);
    }

    auto bench = [&](const char *label, auto body) {
        GfVec3d sink(0);
        // Warm up once, then time the best of three.
        body(sink);
        double best = std::numeric_limits<double>::infinity();
        for (int rep = 0; rep < 3; ++rep) {
            const auto t0 = Clock::now();
            body(sink);
            const auto t1 = Clock::now();
            best = std::min(best, std::chrono::duration<double, std::nano>(
                                      t1 - t0).count());
        }
        std::printf("  %-44s %7.1f ns/point (sink %.3g)\n",
                    label, best / static_cast<double>(n), sink[0]);
    };

    std::printf("testRigExecDualQuat: transform cost, %zu points\n", n);
    bench("one DQ per point: ToMatrix + TransformAffine", [&](GfVec3d &sink) {
        for (size_t i = 0; i < n; ++i) {
            sink += RigExecDualQuatToMatrix(dqs[i]).TransformAffine(points[i]);
        }
    });
    bench("one DQ per point: TransformPoint", [&](GfVec3d &sink) {
        for (size_t i = 0; i < n; ++i) {
            sink += RigExecDualQuatTransformPoint(dqs[i], points[i]);
        }
    });
    const size_t cluster = 8;
    bench("8 points per DQ: ToMatrix + TransformAffine", [&](GfVec3d &sink) {
        for (size_t i = 0; i + cluster <= n; i += cluster) {
            const GfMatrix4d m = RigExecDualQuatToMatrix(dqs[i]);
            for (size_t k = 0; k < cluster; ++k) {
                sink += m.TransformAffine(points[i + k]);
            }
        }
    });
    bench("8 points per DQ: TransformPoint", [&](GfVec3d &sink) {
        for (size_t i = 0; i + cluster <= n; i += cluster) {
            for (size_t k = 0; k < cluster; ++k) {
                sink += RigExecDualQuatTransformPoint(dqs[i], points[i + k]);
            }
        }
    });
    bench("8 points per DQ: RotateVector + hoisted t", [&](GfVec3d &sink) {
        for (size_t i = 0; i + cluster <= n; i += cluster) {
            const GfVec3d t = RigExecDualQuatTranslation(dqs[i]);
            for (size_t k = 0; k < cluster; ++k) {
                sink += RigExecDualQuatRotateVector(dqs[i], points[i + k]) + t;
            }
        }
    });
}

static void
TestRotateVector()
{
    const GfMatrix4d m = Rigid(
        GfVec3d(1, 2, -3).GetNormalized(), 77.0, GfVec3d(10, 20, 30));
    const RigExecDualQuat dq = RigExecDualQuatFromMatrix(m);
    const GfVec3d v(0.5, -0.25, 2.0);
    // Rotation only: matches TransformDir (no translation) ...
    CHECK(Near(RigExecDualQuatRotateVector(dq, v), m.TransformDir(v), 1e-13));
    // ... and rotation plus hoisted translation matches the full transform.
    CHECK(Near(RigExecDualQuatRotateVector(dq, v) + RigExecDualQuatTranslation(dq),
               RigExecDualQuatTransformPoint(dq, v), 1e-13));
    CHECK(Near(RigExecDualQuatRotateVector(dq, v).GetLength(), v.GetLength(),
               1e-14));
}

// ---------------------------------------------------------------------------
// Scale-aware path
// ---------------------------------------------------------------------------

static bool
SameBits(double a, double b)
{
    return std::memcmp(&a, &b, sizeof(double)) == 0;
}

static bool
SameBits(const GfVec3d &a, const GfVec3d &b)
{
    return SameBits(a[0], b[0]) && SameBits(a[1], b[1]) && SameBits(a[2], b[2]);
}

static bool
SameBits(const GfQuatd &a, const GfQuatd &b)
{
    return SameBits(a.GetReal(), b.GetReal()) &&
           SameBits(a.GetImaginary(), b.GetImaginary());
}

static bool
SameBits(const RigExecDualQuat &a, const RigExecDualQuat &b)
{
    return SameBits(a.real, b.real) && SameBits(a.dual, b.dual);
}

static bool
NearMatrix3(const GfMatrix3d &a, const GfMatrix3d &b, double tol = 1e-12)
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!Near(a[i][j], b[i][j], tol)) {
                return false;
            }
        }
    }
    return true;
}

// Row-vector [S | 0] * [R | t]: stretch in the pre-rotation frame, then the
// rigid motion. This is the transform a scaled joint produces.
static GfMatrix4d
Scaled(const GfMatrix3d &stretch, const GfVec3d &axis, double degrees,
       const GfVec3d &translation)
{
    return GfMatrix4d(stretch, GfVec3d(0)) * Rigid(axis, degrees, translation);
}

static GfMatrix3d
Diag(double x, double y, double z)
{
    return GfMatrix3d(GfVec3d(x, y, z));
}

static void
TestScaledUniformScale()
{
    // A single scaled influence reproduces its transform exactly: the
    // decomposition of S * R with S = 2I is unique, so the round trip is
    // tight, and weight 1 through the blend is that same transform.
    const GfMatrix4d m = Scaled(
        Diag(2, 2, 2), GfVec3d(1, 2, 3).GetNormalized(), 70.0, GfVec3d(1, -1, 4));
    const RigExecScaledDualQuat single = RigExecScaledDualQuatFromMatrix(m);
    CHECK(!single.isRigid);
    CHECK(NearMatrix3(single.stretch, Diag(2, 2, 2), 1e-13));
    CHECK(RigExecDualQuatIsUnit(single.rigid, 1e-13));
    CHECK(NearMatrix(RigExecScaledDualQuatToMatrix(single), m, 1e-12));
    const double one = 1.0;
    RigExecScaledDualQuat blended;
    CHECK(RigExecBlendScaledDualQuats(&single, &one, 1, &blended));
    CHECK(NearMatrix(RigExecScaledDualQuatToMatrix(blended), m, 1e-12));
    const GfVec3d p(0.3, -0.6, 0.9);
    CHECK(Near(RigExecScaledDualQuatTransformPoint(blended, p), Apply(m, p), 1e-12));

    // Two influences with the same uniform scale s and a large relative
    // rotation blend to scale s exactly: |p'| == s * |p| on the unit
    // sphere, with the rotation the DQS midpoint (75 degrees about Z).
    const double s = 2.0;
    const RigExecScaledDualQuat pair[2] = {
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(s, s, s), GfVec3d(0, 0, 1), 0.0, GfVec3d(0))),
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(s, s, s), GfVec3d(0, 0, 1), 150.0, GfVec3d(0))),
    };
    const double half[2] = {0.5, 0.5};
    CHECK(RigExecBlendScaledDualQuats(pair, half, 2, &blended));
    CHECK(!blended.isRigid);
    CHECK(NearMatrix3(blended.stretch, Diag(s, s, s), 1e-13));
    const GfVec3d x(1, 0, 0);
    const GfVec3d out = RigExecScaledDualQuatTransformPoint(blended, x);
    CHECK(Near(out.GetLength(), s, 1e-13));
    CHECK(Near(out, GfVec3d(s * std::cos(75.0 * kPi / 180.0),
                            s * std::sin(75.0 * kPi / 180.0), 0.0), 1e-13));
}

static void
TestScaledNonUniformScale()
{
    // A: diag(2, 1, 0.5) at 0 degrees. B: diag(1, 3, 1) at 90 degrees
    // about Z. Hand values at 0.5/0.5: S = diag(1.5, 2, 0.75) and the
    // rotation is the DQ blend of 0 and 90 degrees, i.e. 45 degrees (equal
    // weights make nlerp and slerp agree about one axis).
    const RigExecScaledDualQuat ab[2] = {
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(2, 1, 0.5), GfVec3d(0, 0, 1), 0.0, GfVec3d(0))),
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(1, 3, 1), GfVec3d(0, 0, 1), 90.0, GfVec3d(0))),
    };
    CHECK(NearMatrix3(ab[0].stretch, Diag(2, 1, 0.5), 1e-13));
    CHECK(NearMatrix3(ab[1].stretch, Diag(1, 3, 1), 1e-13));
    const double half[2] = {0.5, 0.5};
    RigExecScaledDualQuat blended;
    CHECK(RigExecBlendScaledDualQuats(ab, half, 2, &blended));
    CHECK(NearMatrix3(blended.stretch, Diag(1.5, 2, 0.75), 1e-13));
    const double c45 = std::cos(45.0 * kPi / 180.0);
    const double s45 = std::sin(45.0 * kPi / 180.0);
    // (1, 0, 0) -> stretched to (1.5, 0, 0) -> rotated 45 degrees.
    CHECK(Near(RigExecScaledDualQuatTransformPoint(blended, GfVec3d(1, 0, 0)),
               GfVec3d(1.5 * c45, 1.5 * s45, 0.0), 1e-13));
    // (0, 1, 0) -> (0, 2, 0) -> (-2 sin45, 2 cos45, 0).
    CHECK(Near(RigExecScaledDualQuatTransformPoint(blended, GfVec3d(0, 1, 0)),
               GfVec3d(-2.0 * s45, 2.0 * c45, 0.0), 1e-13));
    // (0, 0, 1) -> (0, 0, 0.75), unrotated by a Z rotation.
    CHECK(Near(RigExecScaledDualQuatTransformPoint(blended, GfVec3d(0, 0, 1)),
               GfVec3d(0, 0, 0.75), 1e-13));
    // Matrix form agrees with the direct form.
    const GfMatrix4d bm = RigExecScaledDualQuatToMatrix(blended);
    const GfVec3d p(0.2, -0.7, 1.1);
    CHECK(Near(RigExecScaledDualQuatTransformPoint(blended, p), Apply(bm, p), 1e-13));

    // Unequal weights 0.25/0.75, plus translations: S = diag(1.25, 2.5,
    // 0.875); the rotation is the normalised weighted quaternion sum about
    // Z, angle 2 * atan2(0.75 sin45, 0.25 + 0.75 cos45); translation is the
    // same DQ blend the rigid path produces for those two rigid parts.
    const GfVec3d ta(1, 0, 0);
    const GfVec3d tb(0, 2, 0);
    const RigExecScaledDualQuat abt[2] = {
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(2, 1, 0.5), GfVec3d(0, 0, 1), 0.0, ta)),
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(1, 3, 1), GfVec3d(0, 0, 1), 90.0, tb)),
    };
    const double w[2] = {0.25, 0.75};
    CHECK(RigExecBlendScaledDualQuats(abt, w, 2, &blended));
    CHECK(NearMatrix3(blended.stretch, Diag(1.25, 2.5, 0.875), 1e-13));
    const double theta = 2.0 * std::atan2(0.75 * s45, 0.25 + 0.75 * c45);
    const RigExecDualQuat rigidParts[2] = {abt[0].rigid, abt[1].rigid};
    RigExecDualQuat rigidBlend;
    CHECK(RigExecBlendDualQuats(rigidParts, w, 2, &rigidBlend));
    CHECK(SameBits(blended.rigid, rigidBlend));
    const GfVec3d t = RigExecDualQuatTranslation(rigidBlend);
    CHECK(Near(RigExecScaledDualQuatTransformPoint(blended, GfVec3d(1, 0, 0)),
               GfVec3d(1.25 * std::cos(theta), 1.25 * std::sin(theta), 0.0) + t,
               1e-13));
}

static void
TestScaledShearSurvives()
{
    GfMatrix3d shear(1.0);
    shear[0][1] = 0.4;  // row-vector shear: x picks up 0.4 of y
    const GfMatrix3d stretch = Diag(1.5, 0.5, 3.0) * shear;
    const GfMatrix4d m = Scaled(stretch, GfVec3d(1, 0, 0), -35.0, GfVec3d(-4, 0, 9));

    // The rigid-only path drops the shear (its documented policy) ...
    bool rigid = true;
    const RigExecDualQuat dropped = RigExecDualQuatFromMatrix(m, &rigid);
    CHECK(!rigid);
    CHECK(!NearMatrix(RigExecDualQuatToMatrix(dropped), m, 1e-6));

    // ... and the scale-aware path keeps it: the round trip is tight even
    // though the input stretch was not symmetric (polar decomposition
    // re-expresses it as symmetric stretch times a slightly different
    // rotation; the product is the same matrix).
    const RigExecScaledDualQuat kept = RigExecScaledDualQuatFromMatrix(m);
    CHECK(!kept.isRigid);
    CHECK(NearMatrix(RigExecScaledDualQuatToMatrix(kept), m, 1e-12));
    CHECK(NearMatrix3(kept.stretch, kept.stretch.GetTranspose(), 1e-12));
    CHECK(std::abs(kept.stretch[0][1]) > 0.05);  // shear is really there
    const GfVec3d p(0.5, 1.5, -2.0);
    CHECK(Near(RigExecScaledDualQuatTransformPoint(kept, p), Apply(m, p), 1e-12));

    // Blended with a rigid partner at 0.5/0.5: S = (S_shear + I) / 2, and
    // the partner's isRigid does not make the blend rigid.
    const RigExecScaledDualQuat pair[2] = {
        kept,
        RigExecScaledDualQuatFromMatrix(Rigid(GfVec3d(0, 1, 0), 20.0, GfVec3d(1, 1, 1))),
    };
    CHECK(pair[1].isRigid);
    const double half[2] = {0.5, 0.5};
    RigExecScaledDualQuat blended;
    CHECK(RigExecBlendScaledDualQuats(pair, half, 2, &blended));
    CHECK(!blended.isRigid);
    CHECK(NearMatrix3(blended.stretch,
                      (kept.stretch + GfMatrix3d(1.0)) * 0.5, 1e-13));
}

static void
TestScaledVolumePreservationStillHolds()
{
    // The pure-rotation 0 / 150 degree case through the scale-aware path:
    // both inputs are rigid, so the result is rigid, |p'| == 1, and the
    // linear matrix blend still collapses to cos 75 degrees.
    const GfMatrix4d ma = Rigid(GfVec3d(0, 0, 1), 0.0, GfVec3d(0));
    const GfMatrix4d mb = Rigid(GfVec3d(0, 0, 1), 150.0, GfVec3d(0));
    const RigExecScaledDualQuat pair[2] = {
        RigExecScaledDualQuatFromMatrix(ma), RigExecScaledDualQuatFromMatrix(mb)};
    const double half[2] = {0.5, 0.5};
    RigExecScaledDualQuat blended;
    CHECK(RigExecBlendScaledDualQuats(pair, half, 2, &blended));
    CHECK(blended.isRigid);
    const GfVec3d p(1, 0, 0);
    const GfVec3d out = RigExecScaledDualQuatTransformPoint(blended, p);
    CHECK(Near(out.GetLength(), 1.0, 1e-14));
    const GfVec3d linear = Apply(ma, p) * 0.5 + Apply(mb, p) * 0.5;
    CHECK(linear.GetLength() < 0.3);
    CHECK(std::abs(out.GetLength() - 1.0) < std::abs(linear.GetLength() - 1.0));
}

static void
TestScaledWithLargeRelativeRotation()
{
    // A: diag(2, 1, 1) at 0 degrees; B: diag(1, 2, 1) at 150 degrees about
    // Z. Hand values at 0.5/0.5: S = diag(1.5, 1.5, 1), R = 75 degrees.
    // (1, 0, 0) -> (1.5, 0, 0) -> 1.5 (cos75, sin75, 0): length exactly 1.5,
    // the mean scale, with the rotation part length-preserving. The naive
    // full-matrix linear blend collapses instead.
    const GfMatrix4d ma = Scaled(Diag(2, 1, 1), GfVec3d(0, 0, 1), 0.0, GfVec3d(0));
    const GfMatrix4d mb = Scaled(Diag(1, 2, 1), GfVec3d(0, 0, 1), 150.0, GfVec3d(0));
    const RigExecScaledDualQuat pair[2] = {
        RigExecScaledDualQuatFromMatrix(ma), RigExecScaledDualQuatFromMatrix(mb)};
    const double half[2] = {0.5, 0.5};
    RigExecScaledDualQuat blended;
    CHECK(RigExecBlendScaledDualQuats(pair, half, 2, &blended));
    CHECK(NearMatrix3(blended.stretch, Diag(1.5, 1.5, 1), 1e-13));
    const GfVec3d p(1, 0, 0);
    const GfVec3d out = RigExecScaledDualQuatTransformPoint(blended, p);
    const double c75 = std::cos(75.0 * kPi / 180.0);
    const double s75 = std::sin(75.0 * kPi / 180.0);
    CHECK(Near(out, GfVec3d(1.5 * c75, 1.5 * s75, 0.0), 1e-13));
    CHECK(Near(out.GetLength(), 1.5, 1e-13));

    // Naive linear blend of the full matrices, inline:
    // 0.5 * (2, 0, 0) + 0.5 * (cos150, sin150, 0) = (1 + 0.5 cos150, 0.5 sin150, 0).
    const GfVec3d linear = Apply(ma, p) * 0.5 + Apply(mb, p) * 0.5;
    CHECK(Near(linear, GfVec3d(1.0 + 0.5 * std::cos(150.0 * kPi / 180.0),
                               0.5 * std::sin(150.0 * kPi / 180.0), 0.0), 1e-13));
    CHECK(linear.GetLength() < 0.7);
    CHECK(std::abs(out.GetLength() - 1.5) < std::abs(linear.GetLength() - 1.5));

    // And the other naive mistake, blending the stretch in the
    // post-rotation frame. At 0.5/0.5 the mean stretch diag(1.5, 1.5, 1)
    // is isotropic in the XY plane and commutes with a Z rotation, so use
    // 0.75/0.25 instead: S = diag(1.75, 1.25, 1) is anisotropic in the
    // rotation plane and S * R != R * S. Verify our composition is the
    // pre-rotation one, S_blend * R_blend in row form, not R * S.
    const double lopsided[2] = {0.75, 0.25};
    CHECK(RigExecBlendScaledDualQuats(pair, lopsided, 2, &blended));
    CHECK(NearMatrix3(blended.stretch, Diag(1.75, 1.25, 1), 1e-13));
    const GfMatrix4d bm = RigExecScaledDualQuatToMatrix(blended);
    const GfMatrix4d rigidM = RigExecDualQuatToMatrix(blended.rigid);
    const GfMatrix4d s4(blended.stretch, GfVec3d(0));
    CHECK(NearMatrix(bm, s4 * rigidM, 1e-13));
    CHECK(!NearMatrix(bm, rigidM * s4, 1e-6));
    // Hand value for the pre-rotation order at (1, 0, 0): stretched to
    // (1.75, 0, 0), then rotated by the DQ-blended angle
    // 2 atan2(0.25 sin75, 0.75 + 0.25 cos75).
    const double phi = 2.0 * std::atan2(0.25 * s75, 0.75 + 0.25 * c75);
    CHECK(Near(RigExecScaledDualQuatTransformPoint(blended, p),
               GfVec3d(1.75 * std::cos(phi), 1.75 * std::sin(phi), 0.0), 1e-13));
}

static void
TestScaledPathMatchesRigidPathBitForBit()
{
    // Rigid inputs through the scale-aware path: identical decomposition
    // branch, identical blend code, and the transform short-circuits the
    // identity stretch, so every intermediate is bit-for-bit the same as
    // the rigid-only path. Quantified: zero differing bits over the grid.
    const GfVec3d axes[] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1),
        GfVec3d(-1, 2, 3).GetNormalized(),
    };
    const double angles[] = {0.0, 13.5, 90.0, 135.0, 180.0, -170.0, 222.0};
    const GfVec3d translations[] = {GfVec3d(0), GfVec3d(-1000, 0.001, 42)};
    std::vector<RigExecDualQuat> rigid;
    std::vector<RigExecScaledDualQuat> scaled;
    for (const GfVec3d &axis : axes) {
        for (const double angle : angles) {
            for (const GfVec3d &t : translations) {
                const GfMatrix4d m = Rigid(axis, angle, t);
                rigid.push_back(RigExecDualQuatFromMatrix(m));
                scaled.push_back(RigExecScaledDualQuatFromMatrix(m));
                CHECK(scaled.back().isRigid);
                CHECK(SameBits(rigid.back(), scaled.back().rigid));
            }
        }
    }
    CHECK(rigid.size() == 4 * 7 * 2);

    int differingBlends = 0;
    int differingPoints = 0;
    const GfVec3d p(0.7, -1.3, 2.9);
    for (size_t i = 0; i + 3 < rigid.size(); i += 3) {
        const double w[4] = {0.4, 0.3, 0.2, 0.1};
        RigExecDualQuat a;
        RigExecScaledDualQuat b;
        CHECK(RigExecBlendDualQuats(&rigid[i], w, 4, &a));
        CHECK(RigExecBlendScaledDualQuats(&scaled[i], w, 4, &b));
        CHECK(b.isRigid);
        if (!SameBits(a, b.rigid)) {
            ++differingBlends;
        }
        if (!SameBits(RigExecDualQuatTransformPoint(a, p),
                      RigExecScaledDualQuatTransformPoint(b, p))) {
            ++differingPoints;
        }
    }
    CHECK(differingBlends == 0);
    CHECK(differingPoints == 0);
    std::printf("testRigExecDualQuat: rigid vs scale-aware path: "
                "%d/%zu blends and %d/%zu points differ in any bit\n",
                differingBlends, rigid.size() / 3, differingPoints,
                rigid.size() / 3);
}

static void
TestScaledDegenerates()
{
    const RigExecScaledDualQuat sdqs[2] = {
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(2, 1, 1), GfVec3d(1, 0, 0), 40.0, GfVec3d(1, 0, 0))),
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(1, 3, 1), GfVec3d(0, 1, 0), -80.0, GfVec3d(0, 2, 0))),
    };
    RigExecScaledDualQuat out = sdqs[1];
    const double zeros[2] = {0.0, 0.0};
    CHECK(!RigExecBlendScaledDualQuats(sdqs, zeros, 2, &out));
    CHECK(out.isRigid && IsIdentity(out.rigid) &&
          NearMatrix3(out.stretch, GfMatrix3d(1.0), 0.0));
    CHECK(!RigExecBlendScaledDualQuats(sdqs, nullptr, 0, &out));
    CHECK(out.isRigid && IsIdentity(out.rigid));

    // Weights that cancel: legal for the rigid blend on different inputs,
    // but the stretch mean is undefined, so the scale-aware blend fails.
    const double cancel[2] = {1.0, -1.0};
    RigExecDualQuat rigidOnly;
    const RigExecDualQuat rigidParts[2] = {sdqs[0].rigid, sdqs[1].rigid};
    CHECK(RigExecBlendDualQuats(rigidParts, cancel, 2, &rigidOnly));
    out = sdqs[1];
    CHECK(!RigExecBlendScaledDualQuats(sdqs, cancel, 2, &out));
    CHECK(out.isRigid && IsIdentity(out.rigid));

    // Non-finite weight fails atomically.
    const double nan[2] = {0.5, std::numeric_limits<double>::quiet_NaN()};
    out = sdqs[1];
    CHECK(!RigExecBlendScaledDualQuats(sdqs, nan, 2, &out));
    CHECK(out.isRigid && IsIdentity(out.rigid));

    // Singular influence: rotation identity, raw linear part as stretch,
    // so alone it still reproduces its transform exactly.
    GfMatrix4d flat = Scaled(Diag(1, 0, 1), GfVec3d(0, 0, 1), 30.0, GfVec3d(7, 8, 9));
    const RigExecScaledDualQuat single = RigExecScaledDualQuatFromMatrix(flat);
    CHECK(!single.isRigid);
    CHECK(single.rigid.real == GfQuatd(1.0));
    CHECK(NearMatrix(RigExecScaledDualQuatToMatrix(single), flat, 1e-13));

    // Reflection: negative scale on the pinned axis survives as a stretch
    // that is symmetric but not positive-definite; the transform is exact.
    const GfMatrix4d mirrored = Scaled(Diag(1, -1, 1), GfVec3d(0, 0, 1), 30.0, GfVec3d(0));
    const RigExecScaledDualQuat refl = RigExecScaledDualQuatFromMatrix(mirrored);
    CHECK(!refl.isRigid);
    CHECK(NearMatrix(RigExecScaledDualQuatToMatrix(refl), mirrored, 1e-12));
    const GfVec3d p(0.3, 0.4, 0.5);
    CHECK(Near(RigExecScaledDualQuatTransformPoint(refl, p), Apply(mirrored, p), 1e-12));
}

// ---------------------------------------------------------------------------
// Palette-indexed scale-aware blend and the skinning kernel over
// RigExecSkinLayout (RigExecApplyDualQuatSkin)
// ---------------------------------------------------------------------------

static bool
SameBits(const GfMatrix3d &a, const GfMatrix3d &b)
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!SameBits(a[i][j], b[i][j])) {
                return false;
            }
        }
    }
    return true;
}

static void
TestPaletteBlendMatchesArrayBlend()
{
    // The same influences through the array form and the palette form:
    // bit-identical rigid part and stretch. A palette index outside the
    // table is ignored at weight zero and degenerate otherwise.
    const std::vector<RigExecScaledDualQuat> palette = {
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(2, 1, 1), GfVec3d(1, 0, 0), 40.0, GfVec3d(1, 0, 0))),
        RigExecScaledDualQuatFromMatrix(Rigid(GfVec3d(0, 1, 0), -80.0, GfVec3d(0, 2, 0))),
        RigExecScaledDualQuatFromMatrix(Scaled(Diag(1, 3, 1), GfVec3d(0, 0, 1), 120.0, GfVec3d(0, 0, 3))),
        RigExecScaledDualQuatFromMatrix(Rigid(GfVec3d(0, 0, 1), 170.0, GfVec3d(9, 9, 9))),
    };
    const int indices[4] = {3, 1, 0, 2};
    const double weights[4] = {0.4, 0.3, 0.2, 0.1};
    RigExecScaledDualQuat gathered[4];
    for (size_t i = 0; i < 4; ++i) {
        gathered[i] = palette[indices[i]];
    }
    RigExecScaledDualQuat fromArray, fromPalette;
    CHECK(RigExecBlendScaledDualQuats(gathered, weights, 4, &fromArray));
    CHECK(RigExecBlendScaledDualQuats(
        palette.data(), palette.size(), indices, weights, 4, &fromPalette));
    CHECK(SameBits(fromArray.rigid, fromPalette.rigid));
    CHECK(SameBits(fromArray.stretch, fromPalette.stretch));
    CHECK(fromArray.isRigid == fromPalette.isRigid && !fromPalette.isRigid);

    const int badIgnored[4] = {3, 1, 0, 7};
    const double zeroLast[4] = {0.4, 0.3, 0.3, 0.0};
    const double zeroLastGathered[3] = {0.4, 0.3, 0.3};
    RigExecScaledDualQuat a, b;
    CHECK(RigExecBlendScaledDualQuats(gathered, zeroLastGathered, 3, &a));
    CHECK(RigExecBlendScaledDualQuats(
        palette.data(), palette.size(), badIgnored, zeroLast, 4, &b));
    CHECK(SameBits(a.rigid, b.rigid) && SameBits(a.stretch, b.stretch));
    const double badWeighted[4] = {0.4, 0.3, 0.2, 0.1};
    b = fromPalette;
    CHECK(!RigExecBlendScaledDualQuats(
        palette.data(), palette.size(), badIgnored, badWeighted, 4, &b));
    CHECK(b.isRigid && IsIdentity(b.rigid));
}

static RigExecSkinLayout
Layout(const std::vector<GfMatrix4d> &transforms,
       const std::vector<int> &indices, const std::vector<float> &weights,
       size_t elementSize)
{
    RigExecSkinLayout layout;
    layout.transforms = transforms.data();
    layout.transformCount = transforms.size();
    layout.indices = indices.data();
    layout.weights = weights.data();
    layout.indexCount = indices.size();
    layout.elementSize = elementSize;
    layout.pointCount = indices.size() / elementSize;
    return layout;
}

static bool
SameBits(const GfVec3f &a, const GfVec3f &b)
{
    return std::memcmp(&a, &b, sizeof(GfVec3f)) == 0;
}

static void
TestSkinKernelRules()
{
    // The kernel's documented rules, checked at the kernel rather than
    // through a stage (testRigExecArm covers the stage).
    // The last two are +/- theta about X with cos(theta / 2) = 0.75, for
    // the over-drive case at the end.
    const double theta = 2.0 * std::acos(0.75) * 180.0 / kPi;
    const std::vector<GfMatrix4d> transforms = {
        Rigid(GfVec3d(0, 0, 1), 90.0, GfVec3d(0)),
        Rigid(GfVec3d(0, 0, 1), 0.0, GfVec3d(0, 10, 0)),
        Rigid(GfVec3d(0, 0, 1), 0.0, GfVec3d(4, 0, 0)),
        Rigid(GfVec3d(1, 0, 0), theta, GfVec3d(0)),
        Rigid(GfVec3d(1, 0, 0), -theta, GfVec3d(0)),
    };

    // Shortfall: a lone 90 degree influence at weight 0.5 rotates the point
    // 45 degrees (on the arc, length 1) because the complement enters as
    // the identity; all-zero weights leave the point bit-identical.
    {
        const std::vector<int> indices = {0, 0};
        const std::vector<float> weights = {0.5f, 0.0f};
        const RigExecSkinLayout layout = Layout(transforms, indices, weights, 1);
        const GfVec3f in[2] = {GfVec3f(1, 0, 0), GfVec3f(1, 0, 0)};
        GfVec3f out[2];
        CHECK(RigExecApplyDualQuatSkin(in, out, layout));
        const double c45 = std::cos(45.0 * kPi / 180.0);
        CHECK(Near(GfVec3d(out[0]), GfVec3d(c45, c45, 0), 1e-6));
        CHECK(SameBits(out[1], in[1]));
        // The single-point reference form agrees with the array form.
        const std::vector<RigExecScaledDualQuat> palette =
            RigExecSkinDualQuatPalette(layout);
        CHECK(palette.size() == transforms.size() + 1);
        CHECK(palette.back().isRigid && IsIdentity(palette.back().rigid));
        GfVec3d single;
        CHECK(RigExecApplyDualQuatSkin(
            GfVec3d(in[0]), palette.data(), palette.size(), layout, 0, &single));
        CHECK(SameBits(GfVec3f(single), out[0]));
    }

    // Translation-only shortfall agrees with the linear kernel to float
    // precision: 0.25 / 0.25 on two translations keeps half the rest point.
    {
        const std::vector<int> indices = {1, 2};
        const std::vector<float> weights = {0.25f, 0.25f};
        const RigExecSkinLayout layout = Layout(transforms, indices, weights, 2);
        const GfVec3f in[1] = {GfVec3f(2, 0, 0)};
        GfVec3f dq[1], lbs[1];
        CHECK(RigExecApplyDualQuatSkin(in, dq, layout));
        RigExecApplyLinearBlendSkin(in, lbs, layout);
        CHECK(Near(GfVec3d(dq[0]), GfVec3d(3, 2.5, 0), 1e-6));
        CHECK(Near(GfVec3d(dq[0]), GfVec3d(lbs[0]), 1e-6));
    }

    // Pivot choice: with two influences the largest-weight-first gather is
    // bit-neutral (the sum is commutative), whichever slot is authored
    // first.
    {
        const std::vector<int> ab = {0, 1};
        const std::vector<int> ba = {1, 0};
        const std::vector<float> wab = {0.3f, 0.7f};
        const std::vector<float> wba = {0.7f, 0.3f};
        const GfVec3f in[1] = {GfVec3f(1, 0.5f, -2)};
        GfVec3f outAb[1], outBa[1];
        CHECK(RigExecApplyDualQuatSkin(in, outAb, Layout(transforms, ab, wab, 2)));
        CHECK(RigExecApplyDualQuatSkin(in, outBa, Layout(transforms, ba, wba, 2)));
        CHECK(SameBits(outAb[0], outBa[0]));
    }

    // In-place application (in == out) is what the mover does.
    {
        const std::vector<int> indices = {0, 1};
        const std::vector<float> weights = {0.5f, 0.5f};
        const RigExecSkinLayout layout = Layout(transforms, indices, weights, 2);
        GfVec3f separate[1], inPlace[1] = {GfVec3f(1, 2, 3)};
        const GfVec3f in[1] = {inPlace[0]};
        CHECK(RigExecApplyDualQuatSkin(in, separate, layout));
        CHECK(RigExecApplyDualQuatSkin(inPlace, inPlace, layout));
        CHECK(SameBits(separate[0], inPlace[0]));
    }

    // The one degenerate reachable through the layout: an over-driven sum
    // whose identity complement cancels the rotation. +theta and -theta
    // about X with cos(theta / 2) = 0.75, weight 2 each: their real parts
    // dot to 0.75^2 - 0.4375 = 0.125 > 0, so neither is flipped, the
    // axis parts cancel, the scalar parts sum to 2 * 0.75 * 2 = 3 along
    // the identity, and the complement 1 - 4 = -3 removes exactly that.
    // The kernel reports failure rather than skinning with an undefined
    // rotation. (With weights summing to at most one this cannot happen:
    // every sign-corrected term is constructive against the pivot.)
    {
        const std::vector<int> indices = {3, 4};
        const std::vector<float> weights = {2.0f, 2.0f};
        const RigExecSkinLayout layout = Layout(transforms, indices, weights, 2);
        const GfVec3f in[1] = {GfVec3f(0, 1, 0)};
        GfVec3f out[1];
        CHECK(!RigExecApplyDualQuatSkin(in, out, layout));
    }
}

// Informational: what the scale-aware path costs on the skinning hot
// path. The polar decomposition is per influence per evaluation, hoisted
// out of the point loop, so its cost is reported per influence for rigid
// input (the cheap orthonormality test) and scaled input (the SVD); the
// kernel cost is reported per point next to the linear kernels on the
// same layout, at the biped's shape (26,276 points, 137 influences).
static void
MeasureSkinningCost()
{
    using Clock = std::chrono::steady_clock;
    auto bestOf = [](int reps, auto body) {
        body();
        double best = std::numeric_limits<double>::infinity();
        for (int rep = 0; rep < reps; ++rep) {
            const auto t0 = Clock::now();
            body();
            const auto t1 = Clock::now();
            best = std::min(best, std::chrono::duration<double, std::nano>(
                                      t1 - t0).count());
        }
        return best;
    };

    const size_t influenceCount = 137;
    std::vector<GfMatrix4d> rigidJoints, scaledJoints;
    for (size_t j = 0; j < influenceCount; ++j) {
        const double a = 0.37 * static_cast<double>(j);
        const GfVec3d axis =
            GfVec3d(std::sin(a), std::cos(1.3 * a), 0.5).GetNormalized();
        const GfVec3d t(a, -0.5 * a, 2.0 * std::sin(a));
        rigidJoints.push_back(Rigid(axis, 30.0 + 2.0 * a, t));
        scaledJoints.push_back(Scaled(
            Diag(1.0, 1.0 + 0.1 * std::sin(a), 1.0 + 0.1 * std::sin(a)),
            axis, 30.0 + 2.0 * a, t));
    }
    std::printf("testRigExecDualQuat: scale-aware decomposition cost, "
                "%zu influences\n", influenceCount);
    bool sink = false;
    const int decompReps = 20;
    const double rigidNs = bestOf(decompReps, [&]() {
        for (const GfMatrix4d &m : rigidJoints) {
            sink ^= RigExecScaledDualQuatFromMatrix(m).isRigid;
        }
    });
    const double scaledNs = bestOf(decompReps, [&]() {
        for (const GfMatrix4d &m : scaledJoints) {
            sink ^= RigExecScaledDualQuatFromMatrix(m).isRigid;
        }
    });
    std::printf("  %-44s %8.1f ns/influence (%.1f us per evaluation)\n",
                "rigid input (orthonormality test only)",
                rigidNs / influenceCount, rigidNs / 1000.0);
    std::printf("  %-44s %8.1f ns/influence (%.1f us per evaluation)\n",
                "scaled input (polar decomposition)",
                scaledNs / influenceCount, scaledNs / 1000.0);

    const size_t pointCount = 26276;
    const size_t elementSize = 4;
    std::vector<int> indices(pointCount * elementSize);
    std::vector<float> weights(pointCount * elementSize);
    std::vector<GfVec3f> points(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        float total = 0.0f;
        for (size_t k = 0; k < elementSize; ++k) {
            indices[i * elementSize + k] =
                static_cast<int>((i * 7 + k * 31) % influenceCount);
            const float w = static_cast<float>(1 + ((i + k) % 5));
            weights[i * elementSize + k] = w;
            total += w;
        }
        for (size_t k = 0; k < elementSize; ++k) {
            weights[i * elementSize + k] /= total;
        }
        const double a = 0.001 * static_cast<double>(i);
        points[i] = GfVec3f(float(std::sin(a)), float(std::cos(a)), float(a));
    }
    std::vector<GfVec3f> out(pointCount);
    const int kernelReps = 5;
    auto report = [&](const char *label, double ns) {
        std::printf("  %-44s %7.1f ns/point (%.2f ms per evaluation)\n",
                    label, ns / static_cast<double>(pointCount), ns / 1e6);
    };
    std::printf("testRigExecDualQuat: skinning kernel cost, %zu points x %zu "
                "influences\n", pointCount, elementSize);
    for (const bool scaled : {false, true}) {
        const RigExecSkinLayout layout = Layout(
            scaled ? scaledJoints : rigidJoints, indices, weights, elementSize);
        report(scaled ? "classicLinear scalar, scaled joints"
                      : "classicLinear scalar, rigid joints",
               bestOf(kernelReps, [&]() {
                   RigExecApplyLinearBlendSkin(points.data(), out.data(), layout);
               }));
        report(scaled ? "classicLinear SSE2, scaled joints"
                      : "classicLinear SSE2, rigid joints",
               bestOf(kernelReps, [&]() {
                   RigExecApplyLinearBlendSkinSimd(
                       points.data(), out.data(), layout);
               }));
        report(scaled ? "dualQuaternion scalar, scaled joints"
                      : "dualQuaternion scalar, rigid joints",
               bestOf(kernelReps, [&]() {
                   sink ^= RigExecApplyDualQuatSkin(
                       points.data(), out.data(), layout);
               }));
    }
    std::printf("  (sink %d)\n", sink ? 1 : 0);
}

int
main()
{
    TestRowVectorConvention();
    TestMatrixRoundTrip();
    TestSingleInfluenceExact();
    TestPureTranslationBlendIsExact();
    TestLengthPreservationVersusLinearBlend();
    TestShortestArcSignCorrection();
    TestUnitNormInvariant();
    TestDegenerateWeights();
    TestIndexedBlend();
    TestScaleShearPolicy();
    TestRotateVector();
    TestScaledUniformScale();
    TestScaledNonUniformScale();
    TestScaledShearSurvives();
    TestScaledVolumePreservationStillHolds();
    TestScaledWithLargeRelativeRotation();
    TestScaledPathMatchesRigidPathBitForBit();
    TestScaledDegenerates();
    TestPaletteBlendMatchesArrayBlend();
    TestSkinKernelRules();
    MeasureTransformCost();
    MeasureSkinningCost();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecDualQuat: all tests passed\n");
    return 0;
}
