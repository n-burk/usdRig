// RigExec math conformance tests (spec §5, §14.3 exit criteria):
// Points -> Matrix -> Points round trips including reflection and shear;
// reconstruction policies; IK reach/stretch; blend endpoints; twist
// distribution; weighted matrix movement.
#include "rigExecMath/avarScale.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/surfaceProjectorKernel.h"
#include "rigExecMath/spatialAccel.h"

#include "pxr/base/gf/rotation.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <cstring>
#include <cstdio>
#include <limits>
#include <vector>

using namespace rigExec;

static int failures = 0;

// std::acos(-1) rather than M_PI: the latter is not a standard C++ macro
// (it needs _USE_MATH_DEFINES on MSVC and a non-strict mode on glibc).
static const double kPi = std::acos(-1);

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

static bool
Near(const GfVec3d &a, const GfVec3d &b, double tol = 1e-10)
{
    return (a - b).GetLength() <= tol;
}

static bool
SameBits(double a, double b)
{
    return std::memcmp(&a, &b, sizeof(double)) == 0;
}

static bool
SameBits(float a, float b)
{
    return std::memcmp(&a, &b, sizeof(float)) == 0;
}

static bool
SameVec3fBits(const GfVec3f &a, const GfVec3f &b)
{
    return SameBits(a[0], b[0]) && SameBits(a[1], b[1]) &&
           SameBits(a[2], b[2]);
}

static bool
SameVec3fArrayBits(const std::vector<GfVec3f> &a,
                   const std::vector<GfVec3f> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (!SameVec3fBits(a[i], b[i])) {
            return false;
        }
    }
    return true;
}

static bool
SameMatrix4dBits(const GfMatrix4d &a, const GfMatrix4d &b)
{
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!SameBits(a[i][j], b[i][j])) {
                return false;
            }
        }
    }
    return true;
}

static bool
NearMatrix4d(const GfMatrix4d &a, const GfMatrix4d &b, double tol)
{
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (std::abs(a[i][j] - b[i][j]) > tol) {
                return false;
            }
        }
    }
    return true;
}

static bool
SameFrameBits(const RigExecPointFrame &a, const RigExecPointFrame &b)
{
    if (a.flags != b.flags) {
        return false;
    }
    for (size_t point = 0; point < a.points.size(); ++point) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!SameBits(a.points[point][axis], b.points[point][axis])) {
                return false;
            }
        }
    }
    return true;
}

static const std::array<GfVec3d, 4> kUnitRest = {
    GfVec3d(0, 0, 0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};

static std::array<int, 3>
EulerOrderIndices(RigExecEulerOrder order)
{
    switch (order) {
    case RigExecEulerOrder::XYZ: return {0, 1, 2};
    case RigExecEulerOrder::XZY: return {0, 2, 1};
    case RigExecEulerOrder::YXZ: return {1, 0, 2};
    case RigExecEulerOrder::YZX: return {1, 2, 0};
    case RigExecEulerOrder::ZXY: return {2, 0, 1};
    case RigExecEulerOrder::ZYX: return {2, 1, 0};
    }
    return {0, 1, 2};
}

static GfMatrix4d
EulerMatrix(const GfVec3d &degrees, RigExecEulerOrder order)
{
    static const GfVec3d axes[3] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    GfMatrix4d result(1.0);
    for (const int axis : EulerOrderIndices(order)) {
        result = result * GfMatrix4d(
            GfRotation(axes[axis], degrees[axis]), GfVec3d(0));
    }
    return result;
}

static RigExecPointFrame
ConstraintFrame(
    const GfVec3d &translation = GfVec3d(0),
    const GfVec3d &rotationDegrees = GfVec3d(0),
    const GfVec3d &scale = GfVec3d(1),
    const GfVec3d &shear = GfVec3d(0),
    RigExecEulerOrder order = RigExecEulerOrder::XYZ)
{
    RigExecTransformParams params;
    params.translation = translation;
    params.rotation = EulerMatrix(rotationDegrees, order)
                          .ExtractRotation().GetQuat().GetNormalized();
    params.scale = scale;
    params.shear = shear;
    return RigExecMatrixToPoints(kUnitRest, RigExecParamsToMatrix(params));
}

static bool
ConstraintParams(
    const RigExecPointFrame &frame, RigExecTransformParams *params)
{
    return RigExecPointsToParams(
        kUnitRest, frame.points, RigExecAxis::Z, params);
}

static bool
SameLinearPart(
    const RigExecPointFrame &a, const RigExecPointFrame &b,
    double tolerance = 1e-10)
{
    for (int i = 1; i < 4; ++i) {
        if (!Near(a.points[i] - a.Origin(),
                  b.points[i] - b.Origin(), tolerance)) {
            return false;
        }
    }
    return true;
}

static bool
AllFinite(const RigExecPointFrame &frame)
{
    for (const GfVec3d &point : frame.points) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(point[axis])) {
                return false;
            }
        }
    }
    return true;
}

// Spec §4.5 shoulder rest landmarks (bone length 4).
static const std::array<GfVec3d, 4> kShoulderRest = {
    GfVec3d(0, 10, 0), GfVec3d(4, 10, 0), GfVec3d(0, 11, 0), GfVec3d(0, 10, 1)};

static void
TestPointsToMatrixRoundTrip()
{
    // Generic affine map with rotation, nonuniform scale, shear, and
    // translation: transform the rest landmarks, then verify the derived
    // matrix reproduces them (conformance by transforming points, spec §5.1).
    GfMatrix4d shear(1.0);
    shear[1][0] = 0.35;  // row-vector storage: y contributes to x
    GfMatrix4d m = GfMatrix4d(1.0).SetScale(GfVec3d(2, 3, 0.5)) * shear *
                   GfMatrix4d(GfRotation(GfVec3d(1, 2, 3), 41.0),
                              GfVec3d(5, -2, 7));

    std::array<GfVec3d, 4> pose;
    for (size_t i = 0; i < 4; ++i) {
        pose[i] = m.TransformAffine(kShoulderRest[i]);
    }

    GfMatrix4d derived;
    CHECK(RigExecPointsToMatrix(kShoulderRest, pose, &derived));
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(derived.TransformAffine(kShoulderRest[i]), pose[i], 1e-9));
    }

    // Matrix -> points -> matrix.
    const RigExecPointFrame frame = RigExecMatrixToPoints(kShoulderRest, m);
    GfMatrix4d again;
    CHECK(RigExecPointsToMatrix(kShoulderRest, frame, &again));
    // Compare by action on probe points.
    const GfVec3d probes[3] = {
        GfVec3d(1, 2, 3), GfVec3d(-4, 0.5, 2), GfVec3d(10, 10, 10)};
    for (const GfVec3d &p : probes) {
        CHECK(Near(again.TransformAffine(p), m.TransformAffine(p), 1e-8));
    }
}

static void
TestReflectionRoundTrip()
{
    // Reflection (negative determinant) must round trip and be flagged.
    GfMatrix4d m(1.0);
    m.SetScale(GfVec3d(1, 1, -1));
    const RigExecPointFrame frame = RigExecMatrixToPoints(kUnitRest, m);
    CHECK(frame.flags & RigExecPointFrameReflected);

    GfMatrix4d derived;
    CHECK(RigExecPointsToMatrix(kUnitRest, frame, &derived));
    CHECK(Near(derived.TransformAffine(GfVec3d(1, 2, 3)), GfVec3d(1, 2, -3)));

    RigExecTransformParams params;
    CHECK(RigExecPointsToParams(kUnitRest, frame.points, RigExecAxis::Z,
                                &params));
    // Reflection pinned to Z: exactly one negative scale on Z.
    CHECK(params.scale[2] < 0);
    CHECK(params.scale[0] > 0 && params.scale[1] > 0);
    const GfMatrix4d rebuilt = RigExecParamsToMatrix(params);
    CHECK(Near(rebuilt.TransformAffine(GfVec3d(1, 2, 3)), GfVec3d(1, 2, -3),
               1e-8));
}

static void
TestSingularReference()
{
    // Collapsed reference frame: no derivable map.
    std::array<GfVec3d, 4> collapsed = {
        GfVec3d(0, 0, 0), GfVec3d(0, 0, 0), GfVec3d(0, 0, 0), GfVec3d(0, 0, 0)};
    GfMatrix4d m;
    CHECK(!RigExecPointsToMatrix(collapsed, kUnitRest, &m));
}

static void
TestOrthogonalReconstruction()
{
    RigExecFrameReconstructionArgs args;
    args.policy = RigExecFramePolicy::Orthogonal;

    // Pose: aim rotated into +Y with doubled aim length and skewed up.
    std::array<GfVec3d, 4> pose = {
        GfVec3d(0, 0, 0), GfVec3d(0, 2, 0), GfVec3d(-0.9, 0.1, 0),
        GfVec3d(0, 0, 1)};
    const RigExecPointFrame f =
        RigExecReconstructFrame(kUnitRest, pose, args);
    CHECK(f.IsValid());
    CHECK(!f.IsDegenerate());
    // Aim preserved with sx = 2.
    CHECK(Near(f.X() - f.Origin(), GfVec3d(0, 2, 0), 1e-9));
    // Up orthogonalized against aim.
    CHECK(std::abs(GfDot((f.Y() - f.Origin()).GetNormalized(),
                         (f.X() - f.Origin()).GetNormalized())) < 1e-9);
    // Right-handed completion.
    const GfVec3d ex = (f.X() - f.Origin()).GetNormalized();
    const GfVec3d ey = (f.Y() - f.Origin()).GetNormalized();
    const GfVec3d ez = (f.Z() - f.Origin()).GetNormalized();
    CHECK(Near(GfCross(ex, ey), ez, 1e-9));

    // Rigid forces unit handles.
    args.policy = RigExecFramePolicy::Rigid;
    const RigExecPointFrame r =
        RigExecReconstructFrame(kUnitRest, pose, args);
    CHECK(std::abs((r.X() - r.Origin()).GetLength() - 1.0) < 1e-9);
    CHECK(std::abs((r.Y() - r.Origin()).GetLength() - 1.0) < 1e-9);

    // Twist rotates transverse axes about aim.
    args.policy = RigExecFramePolicy::Rigid;
    args.twist = kPi / 2;
    const RigExecPointFrame t =
        RigExecReconstructFrame(kUnitRest, kUnitRest, args);
    CHECK(Near(t.Y() - t.Origin(), GfVec3d(0, 0, 1), 1e-9));
    CHECK(Near(t.Z() - t.Origin(), GfVec3d(0, -1, 0), 1e-9));
}

static void
TestDegenerateFallbacks()
{
    RigExecFrameReconstructionArgs args;

    // Coincident aim landmark: degenerate, points preserved (no silent
    // identity).
    std::array<GfVec3d, 4> collapsedAim = {
        GfVec3d(1, 2, 3), GfVec3d(1, 2, 3), GfVec3d(1, 3, 3), GfVec3d(1, 2, 4)};
    const RigExecPointFrame f =
        RigExecReconstructFrame(kUnitRest, collapsedAim, args);
    CHECK(f.IsDegenerate());
    CHECK(Near(f.Origin(), GfVec3d(1, 2, 3)));

    // Up parallel to aim: twist underdetermined; deterministic fallback
    // still produces an orthonormal frame, flagged degenerate.
    std::array<GfVec3d, 4> upOnAim = {
        GfVec3d(0, 0, 0), GfVec3d(2, 0, 0), GfVec3d(1, 0, 0), GfVec3d(0, 0, 1)};
    const RigExecPointFrame g =
        RigExecReconstructFrame(kUnitRest, upOnAim, args);
    CHECK(g.IsDegenerate());
    const GfVec3d gex = (g.X() - g.Origin()).GetNormalized();
    const GfVec3d gey = (g.Y() - g.Origin()).GetNormalized();
    CHECK(std::abs(GfDot(gex, gey)) < 1e-9);

    // Same input twice gives identical results (determinism).
    const RigExecPointFrame g2 =
        RigExecReconstructFrame(kUnitRest, upOnAim, args);
    CHECK(g.points == g2.points && g.flags == g2.flags);
}

static void
TestFkChain()
{
    // Three-bone chain along +X, each bone length 4 (spec §4.5 layout).
    std::vector<RigExecFkChainElement> chain(3);
    chain[0].restPoints = kShoulderRest;
    chain[0].posePoints = kShoulderRest;
    chain[0].parentIndex = -1;
    chain[1].restPoints = {GfVec3d(4, 10, 0), GfVec3d(8, 10, 0),
                           GfVec3d(4, 11, 0), GfVec3d(4, 10, 1)};
    chain[1].posePoints = chain[1].restPoints;
    chain[1].parentIndex = 0;
    chain[2].restPoints = {GfVec3d(8, 10, 0), GfVec3d(10, 10, 0),
                           GfVec3d(8, 11, 0), GfVec3d(8, 10, 1)};
    chain[2].posePoints = chain[2].restPoints;
    chain[2].parentIndex = 1;

    // Rest pose in, rest pose out.
    auto frames = RigExecSolveFkChain(chain);
    CHECK(frames.size() == 3);
    CHECK(Near(frames[2].Origin(), GfVec3d(8, 10, 0)));

    // Rotate the root 90 degrees about Z at its origin: descendants follow.
    const GfMatrix4d rot =
        GfMatrix4d(1.0).SetTranslate(GfVec3d(0, -10, 0)) *
        GfMatrix4d(GfRotation(GfVec3d(0, 0, 1), 90.0), GfVec3d(0, 10, 0));
    for (size_t i = 0; i < 4; ++i) {
        chain[0].posePoints[i] = rot.TransformAffine(kShoulderRest[i]);
    }
    frames = RigExecSolveFkChain(chain);
    // Elbow rest origin (4,10,0) rotates to (0,14,0).
    CHECK(Near(frames[1].Origin(), GfVec3d(0, 14, 0), 1e-9));
    // Wrist rest origin (8,10,0) rotates to (0,18,0).
    CHECK(Near(frames[2].Origin(), GfVec3d(0, 18, 0), 1e-9));
}

static void
TestTwoBoneIk()
{
    RigExecTwoBoneIkParams params;
    params.upperLength = 4;
    params.lowerLength = 4;
    params.stretch = 0;
    params.softness = 0;

    RigExecPointFrame root;
    root.points = kShoulderRest;
    RigExecPointFrame pole;
    pole.points = {GfVec3d(4, 10, -4), GfVec3d(5, 10, -4), GfVec3d(4, 11, -4),
                   GfVec3d(4, 10, -3)};

    std::array<std::array<GfVec3d, 4>, 3> rests = {
        kShoulderRest,
        std::array<GfVec3d, 4>{GfVec3d(4, 10, 0), GfVec3d(8, 10, 0),
                               GfVec3d(4, 11, 0), GfVec3d(4, 10, 1)},
        std::array<GfVec3d, 4>{GfVec3d(8, 10, 0), GfVec3d(10, 10, 0),
                               GfVec3d(8, 11, 0), GfVec3d(8, 10, 1)}};

    // Reachable goal at distance 6: bone lengths must be preserved.
    RigExecPointFrame effector;
    effector.points = {GfVec3d(6, 10, 0), GfVec3d(8, 10, 0), GfVec3d(6, 11, 0),
                       GfVec3d(6, 10, 1)};
    auto frames =
        RigExecSolveTwoBoneIk(root, effector, pole, rests, params);
    CHECK(Near(frames[0].Origin(), GfVec3d(0, 10, 0)));
    CHECK(Near(frames[2].Origin(), GfVec3d(6, 10, 0), 1e-9));
    const double upper = (frames[1].Origin() - frames[0].Origin()).GetLength();
    const double lower = (frames[2].Origin() - frames[1].Origin()).GetLength();
    CHECK(std::abs(upper - 4) < 1e-9);
    CHECK(std::abs(lower - 4) < 1e-9);
    // Elbow bends toward the pole (negative Z side).
    CHECK(frames[1].Origin()[2] < -1e-6);

    // Unreachable goal, no stretch: clamps to full extension.
    effector.points[0] = GfVec3d(12, 10, 0);
    frames = RigExecSolveTwoBoneIk(root, effector, pole, rests, params);
    CHECK(Near(frames[2].Origin(), GfVec3d(8, 10, 0), 1e-9));

    // Full stretch reaches the goal with uniform segments.
    params.stretch = 1.0;
    frames = RigExecSolveTwoBoneIk(root, effector, pole, rests, params);
    CHECK(Near(frames[2].Origin(), GfVec3d(12, 10, 0), 1e-9));
    const double su = (frames[1].Origin() - frames[0].Origin()).GetLength();
    const double sl = (frames[2].Origin() - frames[1].Origin()).GetLength();
    CHECK(std::abs(su - sl) < 1e-9);  // uniformSegments
}

static void
TestBlendFrames()
{
    RigExecPointFrame a;
    a.points = kUnitRest;
    GfMatrix4d m(GfRotation(GfVec3d(0, 0, 1), 90.0), GfVec3d(2, 0, 0));
    const RigExecPointFrame b = RigExecMatrixToPoints(kUnitRest, m);

    // Endpoints are exact.
    CHECK(RigExecBlendFrames(a, b, kUnitRest, 0.0).points == a.points);
    CHECK(RigExecBlendFrames(a, b, kUnitRest, 1.0).points == b.points);

    // Midpoint: translation lerps, rotation is 45 degrees.
    const RigExecPointFrame mid = RigExecBlendFrames(a, b, kUnitRest, 0.5);
    CHECK(Near(mid.Origin(), GfVec3d(1, 0, 0), 1e-9));
    const GfVec3d ex = (mid.X() - mid.Origin()).GetNormalized();
    CHECK(Near(ex, GfVec3d(std::sqrt(0.5), std::sqrt(0.5), 0), 1e-9));

    // Log scale blend: scale 1 and 4 blend to 2 at the midpoint.
    GfMatrix4d ms(1.0);
    ms.SetScale(GfVec3d(4, 4, 4));
    const RigExecPointFrame c = RigExecMatrixToPoints(kUnitRest, ms);
    const RigExecPointFrame midS = RigExecBlendFrames(a, c, kUnitRest, 0.5);
    CHECK(std::abs((midS.X() - midS.Origin()).GetLength() - 2.0) < 1e-9);
}

static void
TestTwistDistribution()
{
    RigExecPointFrame start;
    start.points = kUnitRest;
    // End: same position, 90-degree twist about the +X aim axis.
    GfMatrix4d m(GfRotation(GfVec3d(1, 0, 0), 90.0), GfVec3d(4, 0, 0));
    const RigExecPointFrame end = RigExecMatrixToPoints(kUnitRest, m);
    std::array<GfVec3d, 4> endRest = kUnitRest;

    const std::vector<double> weights = {0, 0.5, 1};
    auto frames =
        RigExecDistributeTwist(start, end, kUnitRest, endRest, weights);
    CHECK(frames.size() == 3);
    CHECK(Near(frames[0].Y() - frames[0].Origin(), GfVec3d(0, 1, 0), 1e-9));
    // Half twist: up rotated 45 degrees about X.
    const GfVec3d halfUp =
        (frames[1].Y() - frames[1].Origin()).GetNormalized();
    CHECK(Near(halfUp, GfVec3d(0, std::sqrt(0.5), std::sqrt(0.5)), 1e-8));
    // Origins lerp along the segment.
    CHECK(Near(frames[1].Origin(), GfVec3d(2, 0, 0), 1e-9));
    // Full twist matches the end frame orientation.
    const GfVec3d endUp = (frames[2].Y() - frames[2].Origin()).GetNormalized();
    CHECK(Near(endUp, GfVec3d(0, 0, 1), 1e-8));
}

static void
TestParamsRoundTrip()
{
    // Matrix -> points -> params -> matrix must round trip for rotated
    // nonuniform scale and every shear component (spec §5.4; regression
    // for the upper-triangular stretch-rebuild defect).
    GfMatrix4d shearXY(1.0), shearXZ(1.0), shearYZ(1.0);
    shearXY[1][0] = 0.4;
    shearXZ[2][0] = -0.3;
    shearYZ[2][1] = 0.25;
    const GfMatrix4d cases[] = {
        GfMatrix4d(1.0).SetScale(GfVec3d(2, 3, 0.5)) * shearXY *
            GfMatrix4d(GfRotation(GfVec3d(1, 2, 3), 41.0), GfVec3d(5, -2, 7)),
        GfMatrix4d(1.0).SetScale(GfVec3d(1.5, 0.7, 2.2)) * shearXZ * shearYZ *
            GfMatrix4d(GfRotation(GfVec3d(-1, 0.5, 2), 117.0),
                       GfVec3d(-3, 4, 1)),
        // Reflection combined with shear.
        GfMatrix4d(1.0).SetScale(GfVec3d(2, -1.5, 1)) * shearXY *
            GfMatrix4d(GfRotation(GfVec3d(0, 1, 0), 63.0), GfVec3d(0, 2, 0)),
    };
    const GfVec3d probes[3] = {
        GfVec3d(1, 2, 3), GfVec3d(-4, 0.5, 2), GfVec3d(0.3, -7, 1.1)};
    for (const GfMatrix4d &m : cases) {
        const RigExecPointFrame frame = RigExecMatrixToPoints(kUnitRest, m);
        RigExecTransformParams params;
        CHECK(RigExecPointsToParams(kUnitRest, frame.points, RigExecAxis::Z,
                                    &params));
        const GfMatrix4d rebuilt = RigExecParamsToMatrix(params);
        for (const GfVec3d &p : probes) {
            CHECK(Near(rebuilt.TransformAffine(p), m.TransformAffine(p),
                       1e-8));
        }
    }

    // Reflections pinned independently to X, Y, and Z: the pinned axis
    // carries the single negative scale.
    for (int axis = 0; axis < 3; ++axis) {
        GfVec3d scale(1, 1, 1);
        scale[axis] = -1;
        GfMatrix4d m(1.0);
        m.SetScale(scale);
        const RigExecPointFrame frame = RigExecMatrixToPoints(kUnitRest, m);
        RigExecTransformParams params;
        CHECK(RigExecPointsToParams(kUnitRest, frame.points,
                                    static_cast<RigExecAxis>(axis), &params));
        CHECK(params.scale[axis] < 0);
        for (int other = 0; other < 3; ++other) {
            if (other != axis) {
                CHECK(params.scale[other] > 0);
            }
        }
        const GfMatrix4d rebuilt = RigExecParamsToMatrix(params);
        for (const GfVec3d &p : probes) {
            CHECK(Near(rebuilt.TransformAffine(p), m.TransformAffine(p),
                       1e-8));
        }
    }
}

static void
TestTwistSideScaleInteraction()
{
    // Side scale and handedness are measured on the untwisted basis
    // (spec §5.2): a mirrored side landmark stays mirrored under twist.
    RigExecFrameReconstructionArgs args;
    args.policy = RigExecFramePolicy::Orthogonal;
    args.twist = kPi / 2;

    std::array<GfVec3d, 4> mirrored = {
        GfVec3d(0, 0, 0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
        GfVec3d(0, 0, -1)};
    const RigExecPointFrame f =
        RigExecReconstructFrame(kUnitRest, mirrored, args);
    CHECK(f.flags & RigExecPointFrameReflected);
    // Untwisted side sign is -1; the twisted side axis rot90(z about x)
    // is -y, so the mirrored side landmark lands at +y.
    CHECK(Near(f.Z() - f.Origin(), GfVec3d(0, 1, 0), 1e-9));
    // Up landmark rotates with twist: rot90(y about x) = +z.
    CHECK(Near(f.Y() - f.Origin(), GfVec3d(0, 0, 1), 1e-9));

    // Rigid and axial preserve handedness (magnitudes only).
    args.twist = 0;
    args.policy = RigExecFramePolicy::Rigid;
    const RigExecPointFrame r =
        RigExecReconstructFrame(kUnitRest, mirrored, args);
    CHECK(r.flags & RigExecPointFrameReflected);
    CHECK(Near(r.Z() - r.Origin(), GfVec3d(0, 0, -1), 1e-9));

    // Zero side determinant follows the authored policy sign.
    std::array<GfVec3d, 4> planarSide = {
        GfVec3d(0, 0, 0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
        GfVec3d(0.5, 0.5, 0)};
    args.policy = RigExecFramePolicy::Orthogonal;
    args.zeroSideSign = -1.0;
    const RigExecPointFrame z =
        RigExecReconstructFrame(kUnitRest, planarSide, args);
    CHECK(z.flags & RigExecPointFrameReflected);
    CHECK(Near(z.Z() - z.Origin(), GfVec3d(0, 0, -1), 1e-9));
}

static void
TestAimUpAxisSelection()
{
    // aimAxis = y, upAxis = z: the Y landmark carries aim scale and the
    // X landmark carries side/handedness.
    RigExecFrameReconstructionArgs args;
    args.aimAxis = RigExecAxis::Y;
    args.upAxis = RigExecAxis::Z;

    std::array<GfVec3d, 4> pose = kUnitRest;
    pose[2] = GfVec3d(0, 2, 0);  // doubled aim (Y landmark)
    const RigExecPointFrame f = RigExecReconstructFrame(kUnitRest, pose, args);
    CHECK(!f.IsDegenerate());
    CHECK(Near(f.Y() - f.Origin(), GfVec3d(0, 2, 0), 1e-9));
    CHECK(Near(f.Z() - f.Origin(), GfVec3d(0, 0, 1), 1e-9));
    // Side (X landmark) completes the right-handed frame: y x z = x.
    CHECK(Near(f.X() - f.Origin(), GfVec3d(1, 0, 0), 1e-9));
    CHECK(!(f.flags & RigExecPointFrameReflected));
}

static void
TestIkDegeneracies()
{
    RigExecTwoBoneIkParams params;
    params.upperLength = 4;
    params.lowerLength = 4;

    auto allFinite = [](const std::array<RigExecPointFrame, 3> &frames) {
        for (const RigExecPointFrame &f : frames) {
            for (const GfVec3d &p : f.points) {
                for (int i = 0; i < 3; ++i) {
                    if (!std::isfinite(p[i])) return false;
                }
            }
        }
        return true;
    };

    std::array<std::array<GfVec3d, 4>, 3> rests = {
        kUnitRest, kUnitRest, kUnitRest};

    // Coincident goal with a degenerate root aim handle: atomic
    // pass-through flagged degenerate, never NaN.
    RigExecPointFrame collapsedRoot;
    collapsedRoot.points = {GfVec3d(0, 0, 0), GfVec3d(0, 0, 0),
                            GfVec3d(0, 0, 0), GfVec3d(0, 0, 0)};
    RigExecPointFrame goalAtRoot;
    goalAtRoot.points = kUnitRest;
    goalAtRoot.points[0] = GfVec3d(0, 0, 0);
    auto frames = RigExecSolveTwoBoneIk(
        collapsedRoot, collapsedRoot, goalAtRoot, rests, params);
    CHECK(allFinite(frames));
    CHECK(frames[0].IsDegenerate());

    // Pole exactly on the aim line, degenerate rest up: still finite.
    RigExecPointFrame root;
    root.points = kUnitRest;
    RigExecPointFrame goal;
    goal.points = kUnitRest;
    goal.points[0] = GfVec3d(6, 0, 0);
    RigExecPointFrame poleOnAim;
    poleOnAim.points = kUnitRest;
    poleOnAim.points[0] = GfVec3d(3, 0, 0);
    std::array<std::array<GfVec3d, 4>, 3> degRests = rests;
    // Rest up parallel to the aim direction.
    degRests[0][2] = degRests[0][0] + GfVec3d(1, 0, 0);
    frames = RigExecSolveTwoBoneIk(root, goal, poleOnAim, degRests, params);
    CHECK(allFinite(frames));
    // Bone lengths preserved.
    CHECK(std::abs((frames[1].Origin() - frames[0].Origin()).GetLength() - 4)
          < 1e-9);
}

static void
TestSvdPerturbationStability()
{
    // Repeated singular values with reflection: tiny perturbations must
    // not move the pinned negative stretch between axes (spec §5.4).
    for (double eps : {0.0, 1e-9, -1e-9, 3e-9}) {
        GfMatrix4d m(1.0);
        m.SetScale(GfVec3d(1, 1, -1));
        m = m * GfMatrix4d(GfRotation(GfVec3d(1, 0, 0), eps), GfVec3d(0));
        const RigExecPointFrame frame = RigExecMatrixToPoints(kUnitRest, m);
        RigExecTransformParams params;
        CHECK(RigExecPointsToParams(kUnitRest, frame.points, RigExecAxis::Z,
                                    &params));
        CHECK(params.scale[2] < 0);
        CHECK(params.scale[0] > 0 && params.scale[1] > 0);
        const GfMatrix4d rebuilt = RigExecParamsToMatrix(params);
        CHECK(Near(rebuilt.TransformAffine(GfVec3d(1, 2, 3)),
                   m.TransformAffine(GfVec3d(1, 2, 3)), 1e-6));
    }
}

static void
TestAimConstraintKernel()
{
    // Non-X aim: the Y landmark aims at the target; up (Z) is preserved;
    // the side landmark completes the frame with input handedness.
    RigExecPointFrame input;
    input.points = kUnitRest;
    const RigExecPointFrame aimed = RigExecApplyAimConstraint(
        input, GfVec3d(5, 0, 0), 1.0, /*aimLandmarkIndex=*/2);
    CHECK(Near(aimed.Y() - aimed.Origin(), GfVec3d(1, 0, 0), 1e-9));
    CHECK(Near(aimed.Z() - aimed.Origin(), GfVec3d(0, 0, 1), 1e-9));
    CHECK(Near(aimed.X() - aimed.Origin(), GfVec3d(0, -1, 0), 1e-9));

    // Reflected input keeps its handedness through the rebuild.
    RigExecPointFrame mirrored;
    mirrored.points = kUnitRest;
    mirrored.points[1] = GfVec3d(-1, 0, 0);  // side landmark flipped
    const RigExecPointFrame aimedMirrored = RigExecApplyAimConstraint(
        mirrored, GfVec3d(5, 0, 0), 1.0, 2);
    CHECK(Near(aimedMirrored.X() - aimedMirrored.Origin(),
               GfVec3d(0, 1, 0), 1e-9));

    // Non-finite weight fails atomically with a degenerate flag.
    const RigExecPointFrame bad = RigExecApplyAimConstraint(
        input, GfVec3d(5, 0, 0), std::nan(""), 1);
    CHECK(bad.IsDegenerate());

    // Overshooting weight clamps to the target direction.
    const RigExecPointFrame clamped = RigExecApplyAimConstraint(
        input, GfVec3d(0, 5, 0), 2.0, 1);
    CHECK(Near(clamped.X() - clamped.Origin(), GfVec3d(0, 1, 0), 1e-9));
}

static void
TestFbxPositionConstraintKernel()
{
    const RigExecPointFrame input = ConstraintFrame(
        GfVec3d(2, 3, 4), GfVec3d(12, -8, 23),
        GfVec3d(2, 3, 4), GfVec3d(0.1, -0.05, 0.07));
    RigExecConstraintSource a, b;
    a.frame = ConstraintFrame(GfVec3d(10, 20, 30));
    a.normalizedWeight = 1.0;
    // Per-source offsets belong to Parent and are deliberately ignored by
    // Position, which has one FBX Translation offset.
    a.translationOffset = GfVec3d(1000);
    b.frame = ConstraintFrame(GfVec3d(30, 40, 50));
    b.normalizedWeight = 3.0;

    RigExecPositionConstraintParams params;
    params.offset = GfVec3d(1, 2, 3);
    params.affect = {true, false, true};
    params.weight = 0.25;
    const RigExecPointFrame output =
        RigExecApplyPositionConstraint(input, {a, b}, params);

    // Relative source weights first give (25,35,45), the global offset gives
    // (26,37,48), then the independent global weight blends selected axes.
    CHECK(Near(output.Origin(), GfVec3d(8, 3, 15), 1e-9));
    CHECK(SameLinearPart(output, input, 1e-9));

    // An inert source is not inspected and zero total influence is exact.
    RigExecConstraintSource inert;
    inert.normalizedWeight = 0.0;
    inert.frame.points[0][0] = std::nan("");
    CHECK(RigExecApplyPositionConstraint(input, {inert}, params) == input);
    params.weight = 0.0;
    CHECK(RigExecApplyPositionConstraint(input, {a, b}, params) == input);
}

static void
TestFbxRotationConstraintKernel()
{
    const RigExecPointFrame input = ConstraintFrame(
        GfVec3d(3, 4, 5), GfVec3d(0, 0, 170));
    RigExecConstraintSource a, b;
    a.frame = ConstraintFrame(GfVec3d(0), GfVec3d(0, 0, -170));
    a.normalizedWeight = 1.0;
    // RotationConstraint uses its one global offset, not Parent offsets.
    a.rotationOffsetDegrees = GfVec3d(0, 0, 1000);
    b.frame = ConstraintFrame(GfVec3d(0), GfVec3d(0, 0, 150));
    b.normalizedWeight = 3.0;

    RigExecRotationConstraintParams params;
    params.offsetDegrees = GfVec3d(0, 0, 20);
    params.affect = {false, false, true};
    params.weight = 0.5;
    const RigExecPointFrame output =
        RigExecApplyRotationConstraint(input, {a, b}, params);

    // From 170 degrees the shortest deltas are +20 and -20.  Their 1:3
    // weighted mean is -10; the global +20 offset makes +10, and global
    // weight 0.5 lands at 175 degrees rather than crossing the long arc.
    const double radians = 175.0 * kPi / 180.0;
    const GfVec3d outputX =
        (output.X() - output.Origin()).GetNormalized();
    CHECK(Near(outputX, GfVec3d(std::cos(radians), std::sin(radians), 0),
               1e-8));
    RigExecTransformParams before, after;
    CHECK(ConstraintParams(input, &before));
    CHECK(ConstraintParams(output, &after));
    CHECK(Near(after.translation, before.translation, 1e-8));
    CHECK(Near(after.scale, before.scale, 1e-8));
    CHECK(Near(after.shear, before.shear, 1e-8));

    // At full global weight, source aggregation must not depend on the
    // constrained object's prior rotation. The source-anchored average of
    // +170 and -170 degrees is the shared half-turn, not zero degrees.
    RigExecConstraintSource branchA, branchB;
    branchA.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(0, 0, 170));
    branchB.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(0, 0, -170));
    RigExecRotationConstraintParams branchParams;
    const RigExecPointFrame fromZero = RigExecApplyRotationConstraint(
        ConstraintFrame(), {branchA, branchB}, branchParams);
    const RigExecPointFrame fromNearBranch = RigExecApplyRotationConstraint(
        ConstraintFrame(GfVec3d(0), GfVec3d(0, 0, 170)),
        {branchA, branchB}, branchParams);
    CHECK(SameLinearPart(fromZero, fromNearBranch, 1e-8));
    CHECK(Near(
        (fromZero.X() - fromZero.Origin()).GetNormalized(),
        GfVec3d(-1, 0, 0), 1e-8));

    const RigExecPointFrame affineInput = ConstraintFrame(
        GfVec3d(-2, 7, 1), GfVec3d(12, -8, 23),
        GfVec3d(2, 3, 4), GfVec3d(0.08, -0.03, 0.04));
    const RigExecPointFrame affineOutput = RigExecApplyRotationConstraint(
        affineInput, {a, b}, params);
    CHECK(ConstraintParams(affineInput, &before));
    CHECK(ConstraintParams(affineOutput, &after));
    CHECK(Near(after.translation, before.translation, 1e-8));
    CHECK(Near(after.scale, before.scale, 1e-8));
    CHECK(Near(after.shear, before.shear, 1e-8));

    // Every FBX Euler order round-trips a nontrivial source orientation.
    const RigExecEulerOrder orders[] = {
        RigExecEulerOrder::XYZ, RigExecEulerOrder::XZY,
        RigExecEulerOrder::YXZ, RigExecEulerOrder::YZX,
        RigExecEulerOrder::ZXY, RigExecEulerOrder::ZYX};
    for (const RigExecEulerOrder order : orders) {
        RigExecConstraintSource source;
        source.frame = ConstraintFrame(
            GfVec3d(0), GfVec3d(20, -30, 40), GfVec3d(1),
            GfVec3d(0), order);
        RigExecRotationConstraintParams orderParams;
        orderParams.rotationOrder = order;
        const RigExecPointFrame ordered = RigExecApplyRotationConstraint(
            ConstraintFrame(), {source}, orderParams);
        CHECK(SameLinearPart(ordered, source.frame, 1e-8));
    }

    // Axis masks operate on Euler components in the selected order.
    RigExecConstraintSource maskedSource;
    maskedSource.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(35, 45, 55));
    RigExecRotationConstraintParams maskedParams;
    maskedParams.affect = {false, true, false};
    const RigExecPointFrame masked = RigExecApplyRotationConstraint(
        ConstraintFrame(), {maskedSource}, maskedParams);
    CHECK(SameLinearPart(
        masked, ConstraintFrame(GfVec3d(0), GfVec3d(0, 45, 0)), 1e-8));
}

static void
TestFbxScaleConstraintKernel()
{
    const RigExecPointFrame input = ConstraintFrame(
        GfVec3d(2, -3, 4), GfVec3d(10, 20, -15),
        GfVec3d(2, 3, 4), GfVec3d(0.1, -0.04, 0.06));
    RigExecConstraintSource a, b;
    a.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(0), GfVec3d(4, 6, 8));
    a.normalizedWeight = 1.0;
    b.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(0), GfVec3d(8, 10, 12));
    b.normalizedWeight = 3.0;

    RigExecScaleConstraintParams params;
    params.offset = GfVec3d(1, -1, 2);  // additive; FBX default is zero
    params.affect = {true, false, true};
    params.weight = 0.5;
    const RigExecPointFrame output =
        RigExecApplyScaleConstraint(input, {a, b}, params);

    RigExecTransformParams before, after;
    CHECK(ConstraintParams(input, &before));
    CHECK(ConstraintParams(output, &after));
    // Weighted source scale (7,9,11), plus offset (1,-1,2), then global
    // half-weight.  Y is masked and remains the input scale.
    CHECK(Near(after.scale, GfVec3d(5, 3, 8.5), 1e-8));
    CHECK(Near(after.translation, before.translation, 1e-8));
    CHECK(Near(after.shear, before.shear, 1e-8));
    CHECK(Near(after.rotation.Transform(GfVec3d(1, 0, 0)),
               before.rotation.Transform(GfVec3d(1, 0, 0)), 1e-8));

    RigExecScaleConstraintParams defaults;
    RigExecConstraintSource four;
    four.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(0), GfVec3d(4));
    const RigExecPointFrame defaultOffset =
        RigExecApplyScaleConstraint(ConstraintFrame(), {four}, defaults);
    CHECK(std::abs((defaultOffset.X() - defaultOffset.Origin()).GetLength() -
                   4.0) < 1e-9);
}

/// rigExec:blendShear is opt-in. Off, a constraint that governs every
/// scale axis keeps the shear its input inherited -- the FBX behaviour and
/// what every rig authored before the flag existed. On, the shear blends
/// toward the sources' like the scale does, and a partial scale mask
/// leaves it alone either way.
static void
TestConstraintShearBlendIsOptIn()
{
    const GfVec3d inputShear(0.12, -0.05, 0.08);
    const RigExecPointFrame input = ConstraintFrame(
        GfVec3d(1, 2, 3), GfVec3d(10, 0, 5), GfVec3d(2, 2, 2), inputShear);
    RigExecConstraintSource clean;
    clean.frame = ConstraintFrame(GfVec3d(0), GfVec3d(0), GfVec3d(3, 3, 3));
    clean.normalizedWeight = 1.0;

    RigExecScaleConstraintParams scale;
    scale.affect = {true, true, true};
    RigExecTransformParams kept, blended, partial;
    CHECK(ConstraintParams(
        RigExecApplyScaleConstraint(input, {clean}, scale), &kept));
    CHECK(Near(kept.shear, inputShear, 1e-8));
    scale.blendShear = true;
    CHECK(ConstraintParams(
        RigExecApplyScaleConstraint(input, {clean}, scale), &blended));
    CHECK(Near(blended.shear, GfVec3d(0), 1e-8));
    CHECK(Near(blended.scale, kept.scale, 1e-8));
    scale.affect = {true, false, true};
    CHECK(ConstraintParams(
        RigExecApplyScaleConstraint(input, {clean}, scale), &partial));
    CHECK(Near(partial.shear, inputShear, 1e-8));

    RigExecParentConstraintParams parent;
    parent.scaleAxes = {true, true, true};
    CHECK(ConstraintParams(
        RigExecApplyParentConstraint(input, {clean}, parent), &kept));
    CHECK(Near(kept.shear, inputShear, 1e-8));
    parent.blendShear = true;
    CHECK(ConstraintParams(
        RigExecApplyParentConstraint(input, {clean}, parent), &blended));
    CHECK(Near(blended.shear, GfVec3d(0), 1e-8));
}

static void
TestFbxParentConstraintKernel()
{
    const RigExecPointFrame input = ConstraintFrame(
        GfVec3d(2, 4, 6), GfVec3d(0), GfVec3d(2, 3, 4),
        GfVec3d(0.09, -0.03, 0.05));
    RigExecConstraintSource a, b;
    a.frame = ConstraintFrame(
        GfVec3d(10, 20, 30), GfVec3d(0, 0, 10),
        GfVec3d(4, 5, 6));
    a.normalizedWeight = 1.0;
    b.frame = ConstraintFrame(
        GfVec3d(20, 30, 40), GfVec3d(0, 0, 30),
        GfVec3d(8, 9, 10));
    b.normalizedWeight = 3.0;

    RigExecParentConstraintParams params;
    params.translationAxes = {true, false, true};
    params.rotationAxes = {false, false, true};
    params.scaleAxes = {true, false, true};
    params.weight = 0.5;
    const RigExecPointFrame output =
        RigExecApplyParentConstraint(input, {a, b}, params);

    RigExecTransformParams before, after;
    CHECK(ConstraintParams(input, &before));
    CHECK(ConstraintParams(output, &after));
    CHECK(Near(after.translation, GfVec3d(9.75, 4, 21.75), 1e-8));
    CHECK(Near(after.scale, GfVec3d(4.5, 3, 6.5), 1e-8));
    CHECK(Near(after.shear, before.shear, 1e-8));
    const double radians = 12.5 * kPi / 180.0;
    CHECK(Near(after.rotation.Transform(GfVec3d(1, 0, 0)),
               GfVec3d(std::cos(radians), std::sin(radians), 0), 1e-8));

    // A Parent offset is local to its source. In row-vector convention a
    // +X translation under a +90-degree Z source lands on world +Y, and the
    // non-commuting rotations compose as offset * source.
    RigExecConstraintSource localOffset;
    localOffset.frame = ConstraintFrame(
        GfVec3d(10, 0, 0), GfVec3d(0, 0, 90));
    localOffset.translationOffset = GfVec3d(2, 0, 0);
    localOffset.rotationOffsetDegrees = GfVec3d(30, 0, 0);
    RigExecParentConstraintParams localParams;
    const RigExecPointFrame locallyComposed = RigExecApplyParentConstraint(
        ConstraintFrame(), {localOffset}, localParams);
    CHECK(Near(locallyComposed.Origin(), GfVec3d(10, 2, 0), 1e-8));
    const GfMatrix4d expectedLocal =
        EulerMatrix(GfVec3d(30, 0, 0), RigExecEulerOrder::XYZ) *
        GfMatrix4d(
            GfRotation(GfVec3d(0, 0, 1), 90), GfVec3d(10, 0, 0));
    CHECK(SameLinearPart(
        locallyComposed,
        RigExecMatrixToPoints(kUnitRest, expectedLocal), 1e-8));

    // Parent rotation aggregation has the same full-weight independence from
    // the constrained input as RotationConstraint.
    RigExecConstraintSource branchA, branchB;
    branchA.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(0, 0, 170));
    branchB.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(0, 0, -170));
    RigExecParentConstraintParams branchParams;
    const RigExecPointFrame fromZero = RigExecApplyParentConstraint(
        ConstraintFrame(), {branchA, branchB}, branchParams);
    const RigExecPointFrame fromNearBranch = RigExecApplyParentConstraint(
        ConstraintFrame(GfVec3d(0), GfVec3d(0, 0, 170)),
        {branchA, branchB}, branchParams);
    CHECK(SameLinearPart(fromZero, fromNearBranch, 1e-8));
}

static void
TestFbxAimConstraintKernel()
{
    // Arbitrary local axes: local +Y aims to world +X while local +Z is
    // pinned to the supplied world-up direction.
    RigExecAimConstraintParams params;
    params.localAimVector = GfVec3d(0, 1, 0);
    params.localUpVector = GfVec3d(0, 0, 1);
    params.worldUpDirection = GfVec3d(0, 0, 1);
    const RigExecPointFrame aimed = RigExecApplyAimConstraint(
        ConstraintFrame(), GfVec3d(5, 0, 0), params);
    CHECK(Near(aimed.Y() - aimed.Origin(), GfVec3d(1, 0, 0), 1e-8));
    CHECK(Near(aimed.Z() - aimed.Origin(), GfVec3d(0, 0, 1), 1e-8));
    CHECK(Near(aimed.X() - aimed.Origin(), GfVec3d(0, -1, 0), 1e-8));

    // With no explicit world up, the input up is preserved; global weight
    // is independent and yields the halfway orientation.
    params = RigExecAimConstraintParams{};
    params.weight = 0.5;
    const RigExecPointFrame half = RigExecApplyAimConstraint(
        ConstraintFrame(), GfVec3d(0, 5, 0), params);
    CHECK(Near(half.X() - half.Origin(),
               GfVec3d(std::sqrt(0.5), std::sqrt(0.5), 0), 1e-8));

    // FBX worldUpType=None is minimum swing and performs no roll correction.
    // It is intentionally distinct from the legacy preserveInputUp behavior,
    // and does not consume localUpVector at all.
    const GfVec3d diagonalTarget(4, 3, 2);
    params = RigExecAimConstraintParams{};
    params.preserveInputUp = false;
    params.localUpVector = params.localAimVector;  // dormant for None
    const RigExecPointFrame minimumSwing = RigExecApplyAimConstraint(
        ConstraintFrame(), diagonalTarget, params);
    const GfRotation expectedSwing(
        GfVec3d(1, 0, 0), diagonalTarget.GetNormalized());
    CHECK(Near(
        (minimumSwing.X() - minimumSwing.Origin()).GetNormalized(),
        diagonalTarget.GetNormalized(), 1e-8));
    CHECK(Near(
        (minimumSwing.Y() - minimumSwing.Origin()).GetNormalized(),
        expectedSwing.TransformDir(GfVec3d(0, 1, 0)), 1e-8));
    params = RigExecAimConstraintParams{};
    const RigExecPointFrame preservedUp = RigExecApplyAimConstraint(
        ConstraintFrame(), diagonalTarget, params);
    CHECK(!Near(
        (minimumSwing.Y() - minimumSwing.Origin()).GetNormalized(),
        (preservedUp.Y() - preservedUp.Origin()).GetNormalized(), 1e-4));

    // Rotation offset is applied after aiming and respects the Euler mask.
    params = RigExecAimConstraintParams{};
    params.rotationOffsetDegrees = GfVec3d(0, 0, 90);
    const RigExecPointFrame offset = RigExecApplyAimConstraint(
        ConstraintFrame(), GfVec3d(5, 0, 0), params);
    CHECK(Near(offset.X() - offset.Origin(), GfVec3d(0, 1, 0), 1e-8));
    params.affectRotation = {true, true, false};
    const RigExecPointFrame masked = RigExecApplyAimConstraint(
        ConstraintFrame(), GfVec3d(5, 0, 0), params);
    CHECK(SameLinearPart(masked, ConstraintFrame(), 1e-8));

    // Changing rotation preserves all other decomposed components, including
    // input shear, and works in a non-default Euler order.
    const RigExecPointFrame affineInput = ConstraintFrame(
        GfVec3d(2, 3, 4), GfVec3d(15, -10, 20), GfVec3d(2, 3, 4),
        GfVec3d(0.08, -0.03, 0.05), RigExecEulerOrder::ZYX);
    params = RigExecAimConstraintParams{};
    params.rotationOrder = RigExecEulerOrder::ZYX;
    const RigExecPointFrame affineOutput = RigExecApplyAimConstraint(
        affineInput, GfVec3d(2, 8, 4), params);
    RigExecTransformParams before, after;
    CHECK(ConstraintParams(affineInput, &before));
    CHECK(ConstraintParams(affineOutput, &after));
    CHECK(Near(after.translation, before.translation, 1e-8));
    CHECK(Near(after.scale, before.scale, 1e-8));
    CHECK(Near(after.shear, before.shear, 1e-8));
}

static void
TestFbxConstraintFailures()
{
    const RigExecPointFrame input = ConstraintFrame();
    RigExecConstraintSource source;
    source.frame = ConstraintFrame(GfVec3d(5), GfVec3d(10, 20, 30),
                                   GfVec3d(2));

    RigExecPositionConstraintParams position;
    source.normalizedWeight = std::nan("");
    RigExecPointFrame failed =
        RigExecApplyPositionConstraint(input, {source}, position);
    CHECK(failed.IsDegenerate() && AllFinite(failed));
    source.normalizedWeight = 1.0;
    source.frame.points[2][0] = std::numeric_limits<double>::infinity();
    failed = RigExecApplyPositionConstraint(input, {source}, position);
    CHECK(failed.IsDegenerate() && AllFinite(failed));

    source.frame = ConstraintFrame(GfVec3d(5), GfVec3d(10, 20, 30),
                                   GfVec3d(2));
    source.frame.flags |= RigExecPointFrameDegenerate;
    RigExecRotationConstraintParams rotation;
    failed = RigExecApplyRotationConstraint(input, {source}, rotation);
    CHECK(failed.IsDegenerate() && AllFinite(failed));
    source.frame = ConstraintFrame(GfVec3d(5), GfVec3d(10, 20, 30),
                                   GfVec3d(2));
    rotation.weight = std::nan("");
    failed = RigExecApplyRotationConstraint(input, {source}, rotation);
    CHECK(failed.IsDegenerate() && AllFinite(failed));
    rotation.weight = 1.0;

    source.frame = ConstraintFrame(GfVec3d(0), GfVec3d(0), GfVec3d(2));
    RigExecScaleConstraintParams scale;
    scale.offset[1] = std::numeric_limits<double>::infinity();
    failed = RigExecApplyScaleConstraint(input, {source}, scale);
    CHECK(failed.IsDegenerate() && AllFinite(failed));

    RigExecParentConstraintParams parent;
    source.normalizedWeight = -1.0;
    failed = RigExecApplyParentConstraint(input, {source}, parent);
    CHECK(failed.IsDegenerate() && AllFinite(failed));
    source.normalizedWeight = 1.0;
    parent.rotationOrder = static_cast<RigExecEulerOrder>(99);
    failed = RigExecApplyParentConstraint(input, {source}, parent);
    CHECK(failed.IsDegenerate() && AllFinite(failed));

    RigExecAimConstraintParams aim;
    aim.localAimVector = GfVec3d(0);
    failed = RigExecApplyAimConstraint(input, GfVec3d(1, 0, 0), aim);
    CHECK(failed.IsDegenerate() && AllFinite(failed));
    aim = RigExecAimConstraintParams{};
    aim.localUpVector = aim.localAimVector;
    failed = RigExecApplyAimConstraint(input, GfVec3d(1, 0, 0), aim);
    CHECK(failed.IsDegenerate() && AllFinite(failed));
    aim = RigExecAimConstraintParams{};
    failed = RigExecApplyAimConstraint(input, input.Origin(), aim);
    CHECK(failed.IsDegenerate() && AllFinite(failed));
    aim.worldUpDirection = GfVec3d(1, 0, 0);
    failed = RigExecApplyAimConstraint(input, GfVec3d(1, 0, 0), aim);
    CHECK(failed.IsDegenerate() && AllFinite(failed));
    aim = RigExecAimConstraintParams{};
    failed = RigExecApplyAimConstraint(
        input, GfVec3d(std::nan(""), 0, 0), aim);
    CHECK(failed.IsDegenerate() && AllFinite(failed));

    // Enable is intentionally evaluator-side: the pure kernels have no
    // enabled switch.  Global weight zero is independently an exact
    // pass-through, even if dormant source data is malformed.
    source.frame.points[0][0] = std::nan("");
    position.weight = 0.0;
    CHECK(RigExecApplyPositionConstraint(input, {source}, position) == input);
    rotation.weight = 0.0;
    CHECK(RigExecApplyRotationConstraint(input, {source}, rotation) == input);
    scale.weight = 0.0;
    CHECK(RigExecApplyScaleConstraint(input, {source}, scale) == input);
    parent.weight = 0.0;
    CHECK(RigExecApplyParentConstraint(input, {source}, parent) == input);
    aim = RigExecAimConstraintParams{};
    aim.weight = 0.0;
    CHECK(RigExecApplyAimConstraint(
              input, GfVec3d(std::nan(""), 0, 0), aim) == input);
}

static void
TestConstraintEnvelopeExactEndpoints()
{
    // Zero is a dormant common envelope, including signed-zero payloads and
    // malformed operation-specific inputs that must not be inspected.
    RigExecPointFrame preceding = ConstraintFrame();
    preceding.points[0][1] = -0.0;
    preceding.points[2][2] = -0.0;
    preceding.flags |= RigExecPointFrameAffine;
    RigExecConstraintSource malformed;
    malformed.normalizedWeight = std::numeric_limits<double>::quiet_NaN();

    RigExecPositionConstraintParams position;
    position.weight = -1.0;
    CHECK(SameFrameBits(
        RigExecApplyPositionConstraint(preceding, {malformed}, position),
        preceding));
    RigExecRotationConstraintParams rotation;
    rotation.weight = 0.0;
    CHECK(SameFrameBits(
        RigExecApplyRotationConstraint(preceding, {malformed}, rotation),
        preceding));
    RigExecScaleConstraintParams scale;
    scale.weight = 0.0;
    CHECK(SameFrameBits(
        RigExecApplyScaleConstraint(preceding, {malformed}, scale),
        preceding));
    RigExecParentConstraintParams parent;
    parent.weight = 0.0;
    CHECK(SameFrameBits(
        RigExecApplyParentConstraint(preceding, {malformed}, parent),
        preceding));
    RigExecAimConstraintParams aim;
    aim.weight = 0.0;
    CHECK(SameFrameBits(
        RigExecApplyAimConstraint(
            preceding, GfVec3d(std::nan(""), 0, 0), aim),
        preceding));
    CHECK(SameFrameBits(
        RigExecApplyAimConstraint(
            preceding, GfVec3d(std::nan(""), 0, 0), 0.0, 1),
        preceding));

    // These exactly representable inputs make a + 1*(b-a) round away from b.
    // Full-strength Position and Parent must select b itself.
    constexpr double inputTranslation = 0x1.6d2667059ba63p+15;
    constexpr double targetTranslation = -0x1.c3b1534cb93f1p-15;
    const RigExecPointFrame translatedInput =
        ConstraintFrame(GfVec3d(inputTranslation, 0, 0));
    RigExecConstraintSource translatedSource;
    translatedSource.frame =
        ConstraintFrame(GfVec3d(targetTranslation, 0, 0));
    position = RigExecPositionConstraintParams{};
    const RigExecPointFrame positioned = RigExecApplyPositionConstraint(
        translatedInput, {translatedSource}, position);
    CHECK(SameBits(positioned.Origin()[0], targetTranslation));

    parent = RigExecParentConstraintParams{};
    const RigExecPointFrame parented = RigExecApplyParentConstraint(
        translatedInput, {translatedSource}, parent);
    CHECK(SameBits(parented.Origin()[0], targetTranslation));

    // The same cancellation exists in pose scale channels.
    constexpr double inputScale = 0x1.8e6763ad2707ap-4;
    constexpr double targetScale = 0x1.dd56d4ae13c38p-8;
    const RigExecPointFrame scaledInput = ConstraintFrame(
        GfVec3d(0), GfVec3d(0), GfVec3d(inputScale, 2, 3));
    RigExecConstraintSource scaledSource;
    scaledSource.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(0), GfVec3d(targetScale, 4, 5));
    scale = RigExecScaleConstraintParams{};
    const RigExecPointFrame scaled = RigExecApplyScaleConstraint(
        scaledInput, {scaledSource}, scale);
    CHECK(SameBits(scaled.X()[0], targetScale));

    parent = RigExecParentConstraintParams{};
    parent.scaleAxes = {true, true, true};
    const RigExecPointFrame parentScaled = RigExecApplyParentConstraint(
        scaledInput, {scaledSource}, parent);
    CHECK(SameBits(parentScaled.X()[0], targetScale));

    // A full shortest-arc rotation selects the source Euler candidate, not the
    // equivalent input+delta representation (+190 degrees in this case). Its
    // constrained X axis therefore carries the source candidate's exact bits.
    RigExecConstraintSource rotationSource;
    rotationSource.frame = ConstraintFrame(
        GfVec3d(0), GfVec3d(0, 0, -170));
    rotation = RigExecRotationConstraintParams{};
    const RigExecPointFrame fromOppositeBranch =
        RigExecApplyRotationConstraint(
            ConstraintFrame(GfVec3d(0), GfVec3d(0, 0, 170)),
            {rotationSource}, rotation);
    CHECK(SameBits(
        fromOppositeBranch.X()[0], rotationSource.frame.X()[0]));
    CHECK(SameBits(
        fromOppositeBranch.X()[1], rotationSource.frame.X()[1]));

    // Aim uses the same masked Euler envelope after constructing its fully
    // aimed candidate. Keep the aim itself dormant here and use an offset to
    // cross the same Euler branch, so this pins Aim's endpoint as well.
    const RigExecPointFrame aimInput = ConstraintFrame(
        GfVec3d(0), GfVec3d(0, 0, 170));
    aim = RigExecAimConstraintParams{};
    aim.preserveInputUp = false;
    aim.rotationOffsetDegrees = GfVec3d(0, 0, -340);
    const RigExecPointFrame aimOffset = RigExecApplyAimConstraint(
        aimInput, aimInput.Origin() +
            (aimInput.X() - aimInput.Origin()) * 5.0,
        aim);
    CHECK(SameBits(aimOffset.X()[0], rotationSource.frame.X()[0]));
    CHECK(SameBits(aimOffset.X()[1], rotationSource.frame.X()[1]));

    // The legacy aim overload has the same endpoint rule: its authored aim
    // landmark selects the normalized target direction without rotating that
    // direction through a numerically approximate full-angle operation.
    const RigExecPointFrame legacyAimed = RigExecApplyAimConstraint(
        ConstraintFrame(), GfVec3d(0, 5, 0), 1.0, 1);
    CHECK(legacyAimed.X() - legacyAimed.Origin() == GfVec3d(0, 1, 0));
}

static void
TestGeometryKernels()
{
    // Volume correction: doubling a unit cube's bound restores it fully
    // at strength 1 and half-way (by volume ratio) at intermediate
    // strengths; planar sets (zero volume) are untouched.
    {
        std::vector<GfVec3f> cube = {
            {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
            {1, 1, 0}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};
        const double reference = RigExecBoundVolume(cube.data(), cube.size());
        for (GfVec3f &p : cube) p *= 2.0f;
        RigExecApplyVolumeCorrect(&cube, reference, 1.0);
        CHECK(std::abs(RigExecBoundVolume(cube.data(), cube.size()) -
                       reference) < 1e-5);
        std::vector<GfVec3f> planar = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
        const std::vector<GfVec3f> planarBefore = planar;
        RigExecApplyVolumeCorrect(&planar, 0.0, 1.0);
        CHECK(planar == planarBefore);
    }

    // Laplacian smoothing at strength 1 moves each quad corner to the
    // average of its two edge neighbors.
    {
        std::vector<GfVec3f> quad = {
            {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
        RigExecApplyLaplacianSmooth(&quad, {4}, {0, 1, 2, 3}, 1.0);
        CHECK(Near(GfVec3d(quad[0]), GfVec3d(0.5, 0.5, 0), 1e-6));
    }

    // Lattice: a rest cage produces identity; translating the whole cage
    // translates every bound point by the same delta.
    {
        std::vector<GfVec3f> cage;
        for (int z = 0; z < 2; ++z)
            for (int y = 0; y < 2; ++y)
                for (int x = 0; x < 2; ++x)
                    cage.push_back(GfVec3f(float(x), float(y), float(z)));
        std::vector<GfVec3f> pts = {{0.5f, 0.5f, 0.5f}, {0.25f, 0.5f, 0.75f}};
        const std::vector<GfVec3f> rest = pts;
        std::vector<GfVec3f> identity = pts;
        RigExecApplyLattice(&identity, rest, cage, cage, GfVec3i(2, 2, 2));
        CHECK(Near(GfVec3d(identity[0]), GfVec3d(rest[0]), 1e-6));
        std::vector<GfVec3f> moved = pts;
        std::vector<GfVec3f> posedCage = cage;
        for (GfVec3f &c : posedCage) c += GfVec3f(0, 3, 0);
        RigExecApplyLattice(&moved, rest, cage, posedCage, GfVec3i(2, 2, 2));
        CHECK(Near(GfVec3d(moved[0]), GfVec3d(rest[0]) + GfVec3d(0, 3, 0),
                   1e-5));
        CHECK(Near(GfVec3d(moved[1]), GfVec3d(rest[1]) + GfVec3d(0, 3, 0),
                   1e-5));
    }

    // Surface projection: a point above a unit quad lands on it.
    {
        std::vector<GfVec3f> pts = {{0.25f, 0.25f, 2.0f}};
        RigExecApplySurfaceProject(
            &pts,
            {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}},
            {4}, {0, 1, 2, 3}, 1.0);
        CHECK(Near(GfVec3d(pts[0]), GfVec3d(0.25, 0.25, 0), 1e-5));
    }

    // RMF sampling on a straight-line control polygon: samples are
    // colinear, equally spaced in arc length, and frames orthonormal.
    {
        // Zero- and one-point drivers have no segment to sample.  They must
        // fail closed rather than entering the linear interpolation branch
        // (the one-point case previously indexed controlPoints[-1]).
        CHECK(RigExecSampleCurveRMF({}, 5).GetSize() == 0);
        CHECK(RigExecSampleCurveRMF({{1, 2, 3}}, 5).GetSize() == 0);

        const auto samples = RigExecSampleCurveRMF(
            {{0, 0, 0}, {3, 0, 0}, {6, 0, 0}, {9, 0, 0}}, 5);
        CHECK(samples.GetSize() == 5);
        for (size_t k = 0; k < samples.GetSize(); ++k) {
            CHECK(std::abs(samples.positions[k][1]) < 1e-5);
            CHECK(std::abs(GfDot(samples.tangents[k],
                                 samples.normals[k])) < 1e-5);
            CHECK(std::abs(samples.normals[k].GetLength() - 1) < 1e-5);
        }
        // Ribbon transport with posed == rest is identity.
        std::vector<GfVec3f> pts = {{4, 1, 0}, {5, -1, 2}};
        const std::vector<GfVec3f> before = pts;
        RigExecApplyRibbonTransport(
            &pts, {{0.3f, 0.0f}, {0.7f, 0.0f}}, samples, samples);
        CHECK(Near(GfVec3d(pts[0]), GfVec3d(before[0]), 1e-5));
        CHECK(Near(GfVec3d(pts[1]), GfVec3d(before[1]), 1e-5));
    }

    // Normals/extent kernels on the unit quad.
    {
        const auto normals = RigExecComputeVertexNormals(
            {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}}, {4}, {0, 1, 2, 3});
        CHECK(normals.size() == 4);
        CHECK(Near(GfVec3d(normals[0]), GfVec3d(0, 0, 1), 1e-6));

        // Two pentagons meeting along a seam, both flat in z = 0 and both
        // wound counter-clockwise, so every vertex normal is unambiguously
        // +Z. Vertex 2 is the seam's middle, and in BOTH faces it lands
        // next to the fan anchor: a per-triangle kernel -- which this one
        // used to be -- therefore gives it exactly one sliver triangle from
        // each face, the two are mirror images, and they cancel to a zero
        // normal on a perfectly valid manifold vertex. puppetA
        // (chars/puppetA) has one such vertex on its face.
        const auto seam = RigExecComputeVertexNormals(
            {{0, 0, 0}, {0, 1, 0}, {0.05f, 0.5f, 0},      // the seam
             {1, 0.2f, 0}, {1, 0.8f, 0},                  // +x side
             {-1, 0.2f, 0}, {-1, 0.8f, 0}},               // -x side
            {5, 5},
            {0, 3, 4, 1, 2,
             0, 2, 1, 6, 5});
        CHECK(seam.size() == 7);
        for (size_t i = 0; i < seam.size(); ++i) {
            CHECK(Near(GfVec3d(seam[i]), GfVec3d(0, 0, 1), 1e-6));
        }
        const auto extent = RigExecComputeExtent(
            {{0, 0, 0}, {1, 2, 3}}, {1.0f, 1.0f});
        CHECK(extent.size() == 2);
        CHECK(Near(GfVec3d(extent[0]), GfVec3d(-0.5, -0.5, -0.5), 1e-6));
        CHECK(Near(GfVec3d(extent[1]), GfVec3d(1.5, 2.5, 3.5), 1e-6));
    }
}

static void
TestSurfaceOffsets()
{
    const std::vector<GfVec3f> rest = {{0,0,0}, {4,0,0}, {4,1,0}, {0,1,0}};
    const std::vector<int> counts = {4}, indices = {0,1,2,3};
    const std::vector<GfVec3f> deltas = {{1,2,3}, {0,0,2}, {-3,1,0}, {0,2,0}};
    GfMatrix4d rotation(1.0);
    rotation.SetRotate(GfRotation(GfVec3d(1,0,0), 90));
    GfMatrix4d scale(1.0);
    scale.SetScale(GfVec3d(0.25,9,2));
    GfMatrix4d transform = scale * rotation;
    transform.SetTranslateOnly(GfVec3d(30,-20,10));
    std::vector<GfVec3f> posed, result;
    for (const auto &p : rest) posed.push_back(GfVec3f(transform.TransformAffine(GfVec3d(p))));
    CHECK(RigExecTransportSurfaceOffsets(rest, posed, counts, indices, deltas, &result));
    CHECK(result.size() == deltas.size());
    for (size_t i = 0; i < result.size(); ++i) {
        CHECK(Near(GfVec3d(result[i]), rotation.TransformDir(GfVec3d(deltas[i])), 1e-5));
        CHECK(std::abs(result[i].GetLength() - deltas[i].GetLength()) < 1e-5);
    }
    // The posed longest edge switches from X to Y after nonuniform scaling;
    // correspondence must still use the edge selected on the REST surface.
    CHECK(Near(GfVec3d(result[3]), GfVec3d(0,0,2), 1e-5));
    CHECK(RigExecTransportSurfaceOffsets(rest, rest, counts, indices, deltas, &result));
    for (size_t i = 0; i < result.size(); ++i) CHECK(Near(GfVec3d(result[i]), GfVec3d(deltas[i]), 1e-6));

    const std::vector<GfVec3f> square = {{0,0,0},{1,0,0},{1,1,0},{0,1,0}};
    const std::vector<GfVec3f> skewed = {{0,0,0},{1,1,0},{1,2,0},{0,1,0}};
    const std::vector<GfVec3f> one = {{1,0,0},{0,0,0},{0,0,0},{0,0,0}};
    CHECK(RigExecTransportSurfaceOffsets(square, skewed, counts, indices, one, &result));
    CHECK(Near(GfVec3d(result[0]), GfVec3d(std::sqrt(0.5),std::sqrt(0.5),0), 1e-6));
    std::vector<GfVec3f> reordered;
    CHECK(RigExecTransportSurfaceOffsets(square, skewed, counts, {1,2,3,0}, one, &reordered));
    CHECK(result == reordered);

    auto isolatedRest = rest, isolatedPosed = posed, isolatedDeltas = deltas;
    isolatedRest.push_back(GfVec3f(7)); isolatedPosed.push_back(GfVec3f(9));
    isolatedDeltas.push_back(GfVec3f(0));
    CHECK(RigExecTransportSurfaceOffsets(isolatedRest, isolatedPosed, counts, indices,
                                         isolatedDeltas, &result));
    CHECK(result.back() == GfVec3f(0));
    const std::vector<GfVec3f> sentinel = {{123,456,789}};
    result = sentinel;
    isolatedDeltas.back() = GfVec3f(1,0,0);
    CHECK(!RigExecTransportSurfaceOffsets(isolatedRest, isolatedPosed, counts, indices,
                                          isolatedDeltas, &result));
    CHECK(result == sentinel); // failure after other vertices never partially writes
    CHECK(!RigExecTransportSurfaceOffsets(rest, std::vector<GfVec3f>(4,GfVec3f(0)),
                                          counts, indices, deltas, &result));
    CHECK(!RigExecTransportSurfaceOffsets(rest, posed, {3}, indices, deltas, &result));
    CHECK(!RigExecTransportSurfaceOffsets(rest, posed, counts, {0,1,2,8}, deltas, &result));
    CHECK(!RigExecTransportSurfaceOffsets(rest, posed, {-1}, {}, deltas, &result));
    CHECK(!RigExecTransportSurfaceOffsets(rest, posed, counts, indices, {}, &result));
    auto invalid = deltas;
    invalid[0][1] = std::numeric_limits<float>::quiet_NaN();
    CHECK(!RigExecTransportSurfaceOffsets(rest, posed, counts, indices, invalid, &result));
    CHECK(result == sentinel);
    CHECK(RigExecTransportSurfaceOffsets({}, {}, {}, {}, {}, &result) && result.empty());
}

static void
TestSimdParity()
{
    // The SIMD weighted-matrix kernel must match the scalar reference
    // within the spec 13.2 bulk-float tolerance (1e-6 x character scale)
    // across rotation, scale, shear, and varied weights.
    GfMatrix4d shear(1.0);
    shear[1][0] = 0.3;
    const GfMatrix4d m =
        GfMatrix4d(1.0).SetScale(GfVec3d(1.5, 0.8, 2.0)) * shear *
        GfMatrix4d(GfRotation(GfVec3d(1, 1, 0), 33.0), GfVec3d(2, -1, 4));
    std::vector<GfVec3f> in;
    std::vector<float> weights;
    for (int i = 0; i < 257; ++i) {
        in.push_back(GfVec3f(
            float(i % 17) - 8.0f, float(i % 5) * 2.0f, float(i % 11)));
        weights.push_back(float(i % 101) / 100.0f);
    }
    std::vector<GfVec3f> simd(in.size());
    RigExecApplyWeightedMatrixSimd(
        in.data(), simd.data(), weights.data(), in.size(), m);
    const double scale = 20.0;  // point-set extent
    for (size_t i = 0; i < in.size(); ++i) {
        const GfVec3d scalar = RigExecApplyWeightedMatrix(
            GfVec3d(in[i]), m, weights[i]);
        CHECK((GfVec3d(simd[i]) - scalar).GetLength() <= 1e-6 * scale);
    }

    // Endpoint selection must bypass interpolation exactly. With the old
    // q + (moved - q) expression, a full-weight collapse from 1e20 to 1
    // loses the candidate to cancellation (typically producing zero).
    GfMatrix4d collapse(0.0);
    collapse[3][0] = 1.0;
    collapse[3][1] = -2.0;
    collapse[3][2] = 3.0;
    collapse[3][3] = 1.0;
    const GfVec3f endpointIn[] = {
        GfVec3f(1.0e20f, -1.0e20f, 1.0e20f),
        GfVec3f(1.0e20f, -1.0e20f, 1.0e20f)};
    GfVec3f endpointOut[2];
    const float endpointWeights[] = {0.0f, 1.0f};
    RigExecApplyWeightedMatrixSimd(
        endpointIn, endpointOut, endpointWeights, 2, collapse);
    CHECK(SameVec3fBits(endpointOut[0], endpointIn[0]));
    CHECK(SameVec3fBits(endpointOut[1], GfVec3f(1.0f, -2.0f, 3.0f)));
}

static void
TestSimdScalarWeightMatchesArray()
{
    // The constant-weight SIMD form is the array form with a uniform
    // field and its per-point weight branch hoisted: bit-exact in all
    // three arms, in place as well as out of place.
    GfMatrix4d shear(1.0);
    shear[1][0] = 0.3;
    const GfMatrix4d m =
        GfMatrix4d(1.0).SetScale(GfVec3d(1.5, 0.8, 2.0)) * shear *
        GfMatrix4d(GfRotation(GfVec3d(1, 1, 0), 33.0), GfVec3d(2, -1, 4));
    std::vector<GfVec3f> in;
    for (int i = 0; i < 257; ++i) {
        in.push_back(GfVec3f(
            float(i % 17) - 8.0f, float(i % 5) * 2.0f, float(i % 11)));
    }
    for (const float weight : {0.0f, 0.5f, 1.0f}) {
        const std::vector<float> uniform(in.size(), weight);
        std::vector<GfVec3f> byArray(in.size()), byScalar(in.size());
        RigExecApplyWeightedMatrixSimd(
            in.data(), byArray.data(), uniform.data(), in.size(), m);
        RigExecApplyWeightedMatrixSimd(
            in.data(), byScalar.data(), weight, in.size(), m);
        for (size_t i = 0; i < in.size(); ++i) {
            CHECK(SameVec3fBits(byArray[i], byScalar[i]));
        }
    }
    for (const float weight : {0.0f, 0.5f, 1.0f}) {
        std::vector<GfVec3f> inPlace = in;
        RigExecApplyWeightedMatrixSimd(
            inPlace.data(), inPlace.data(), weight, inPlace.size(), m);
        const std::vector<float> uniform(in.size(), weight);
        std::vector<GfVec3f> byArray(in.size());
        RigExecApplyWeightedMatrixSimd(
            in.data(), byArray.data(), uniform.data(), in.size(), m);
        CHECK(SameVec3fArrayBits(inPlace, byArray));
    }
    // Endpoints through the scalar form: the collapse the old
    // q + (moved - q) expression lost to cancellation.
    GfMatrix4d collapse(0.0);
    collapse[3][0] = 1.0;
    collapse[3][1] = -2.0;
    collapse[3][2] = 3.0;
    collapse[3][3] = 1.0;
    const GfVec3f big(1.0e20f, -1.0e20f, 1.0e20f);
    GfVec3f atZero = big, atOne = big;
    RigExecApplyWeightedMatrixSimd(&big, &atZero, 0.0f, 1, collapse);
    RigExecApplyWeightedMatrixSimd(&big, &atOne, 1.0f, 1, collapse);
    CHECK(SameVec3fBits(atZero, big));
    CHECK(SameVec3fBits(atOne, GfVec3f(1.0f, -2.0f, 3.0f)));
}

static void
TestWireRestEvalsMatchDirectEvaluation()
{
    // M19: a caller-supplied rest table answers exactly the per-point
    // rest evaluation, whole-range and chunked; a short table fails
    // the call, and a null table with a stale count still evaluates.
    const std::vector<GfVec3f> rest = {{0, 0, 0}, {2, 0, 0}};
    const std::vector<GfVec3f> posed = {{0, 1, 0}, {2, 2, 0}};
    const std::vector<double> knots = {0, 0, 1, 1};
    const RigExecNurbsCurve restCurve{&rest, 2, &knots};
    const RigExecNurbsCurve posedCurve{&posed, 2, &knots};
    const std::vector<GfVec2f> binds = {
        {0.0f, 0.0f}, {0.25f, 0.0f}, {0.5f, 0.5f}, {1.0f, 0.0f}};
    const std::vector<GfVec3f> points = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 1}};
    std::vector<GfVec3f> byNull = points;
    CHECK(RigExecApplyWire(&byNull, restCurve, posedCurve, binds.data(),
                           binds.size(), 1.0, 0, points.size()));
    std::vector<GfVec3f> table(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        table[i] = restCurve.Evaluate(binds[i][0]);
    }
    std::vector<GfVec3f> byTable = points;
    CHECK(RigExecApplyWire(&byTable, restCurve, posedCurve, binds.data(),
                           binds.size(), 1.0, 0, points.size(),
                           table.data(), table.size()));
    CHECK(SameVec3fArrayBits(byTable, byNull));
    CHECK(byTable != points);
    std::vector<GfVec3f> chunked = points;
    CHECK(RigExecApplyWire(&chunked, restCurve, posedCurve, binds.data(),
                           binds.size(), 1.0, 0, 2,
                           table.data(), table.size()));
    CHECK(RigExecApplyWire(&chunked, restCurve, posedCurve, binds.data(),
                           binds.size(), 1.0, 2, points.size(),
                           table.data(), table.size()));
    CHECK(SameVec3fArrayBits(chunked, byNull));
    std::vector<GfVec3f> shorted = points;
    CHECK(!RigExecApplyWire(&shorted, restCurve, posedCurve, binds.data(),
                            binds.size(), 1.0, 0, points.size(),
                            table.data(), table.size() - 1));
    CHECK(shorted == points);
    std::vector<GfVec3f> nullCount = points;
    CHECK(RigExecApplyWire(&nullCount, restCurve, posedCurve, binds.data(),
                           binds.size(), 1.0, 0, points.size(),
                           nullptr, 37));
    CHECK(SameVec3fArrayBits(nullCount, byNull));
}

static void
TestPartialDecompositionMatchesOneShot()
{
    // M53: the two-phase form answers exactly the one-shot form on the
    // open interval; a pure rotation takes exactly its fraction of the
    // angle, a pure translation its fraction of the offset.
    GfMatrix4d spin(1.0);
    spin.SetRotate(GfRotation(GfVec3d(0, 0, 1), 90.0));
    GfMatrix4d shear(1.0);
    shear[1][0] = 0.3;
    const GfMatrix4d screw =
        GfMatrix4d(1.0).SetScale(GfVec3d(1.5, 0.8, 2.0)) * shear *
        GfMatrix4d(GfRotation(GfVec3d(1, 1, 0), 33.0), GfVec3d(2, -1, 4));
    GfMatrix4d slide(1.0);
    slide.SetTranslateOnly(GfVec3d(2, -1, 4));
    const GfMatrix4d scale = GfMatrix4d(1.0).SetScale(GfVec3d(2, 3, 4));
    for (const GfMatrix4d &m :
         {spin, screw, slide, scale, GfMatrix4d(1.0)}) {
        const RigExecPartialDecomposition d =
            RigExecDecomposePartialTransform(m);
        for (const double w : {0.125, 0.5, 0.9}) {
            CHECK(SameMatrix4dBits(
                RigExecApplyPartialDecomposition(d, w),
                RigExecPartialTransform(m, w)));
        }
    }
    // Fixed pin: the screw at w=0.5, frozen from the oracle-validated
    // implementation so a future arithmetic change breaks loudly.
    const double expectScrew[4][4] = {
        {1.2167756076626171, -0.066519034337087904, -0.27844629404632132, 0},
        {0.085383733153661448, 0.89983904049750907, 0.1581504968872646, 0},
        {0.31390621695305815, -0.28274485034013319, 1.4392769839625572, 0},
        {-0.45275297278926885, -0.76922663930483481, 3.1293735108362148, 1},
    };
    const GfMatrix4d screwHalf = RigExecPartialTransform(screw, 0.5);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            CHECK(SameBits(screwHalf[i][j], expectScrew[i][j]));
        }
    }
    const GfMatrix4d half = RigExecApplyPartialDecomposition(
        RigExecDecomposePartialTransform(spin), 0.5);
    GfMatrix4d expect(1.0);
    expect.SetRotate(GfRotation(GfVec3d(0, 0, 1), 45.0));
    CHECK(NearMatrix4d(half, expect, 1e-12));
    const GfMatrix4d halfSlide = RigExecApplyPartialDecomposition(
        RigExecDecomposePartialTransform(slide), 0.5);
    CHECK(Near(halfSlide.ExtractTranslation(), GfVec3d(1, -0.5, 2), 1e-12));
}

static void
TestSurfaceProjectorBaseNormalsMemo()
{
    // M18: the base-normals memo answers a fresh computation after the
    // base changes: each base solved twice, with the other base solved
    // between, gives identical shaders. The tilted second base has
    // different normals, so a memo that never invalidated would answer
    // wrong; the counting callback proves each base computed once and
    // the posed side once per solve.
    const std::vector<GfVec3f> quad = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    const std::vector<int> counts = {4}, indices = {0, 1, 2, 3};
    RigExecSurfaceProjectorInputs<GfMatrix4d, GfVec3d> in;
    in.rayOrigin = GfVec3d(0.25, 0.25, 8.0);
    in.rayDirection = GfVec3d(0, 0, -1);
    in.rayUp = GfVec3d(0, 1, 0);
    in.shaderOffset.SetIdentity();
    in.sourceBase.SetIdentity();
    in.sourceFinal.SetIdentity();
    in.sourceSpaceBase.SetIdentity();
    in.sourceSpaceFinal.SetIdentity();
    in.spaceFinal.SetIdentity();
    in.worldToMesh.SetIdentity();
    const std::vector<GfVec3f> baseA = quad;
    const std::vector<GfVec3f> posedA = {
        {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0}};
    // Tilted 30 degrees about X as well as lifted: different normals.
    const std::vector<GfVec3f> baseB = {
        {0, 0, 5}, {1, 0, 5}, {1, 0.8660254f, 5.5f}, {0, 0.8660254f, 5.5f}};
    const std::vector<GfVec3f> posedB = {
        {0, 2, 5}, {1, 2, 5}, {1, 2.8660254f, 5.5f}, {0, 2.8660254f, 5.5f}};
    int baseACalls = 0, baseBCalls = 0, posedCalls = 0;
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> cacheA,cacheB;
    auto compute = [&](const std::vector<GfVec3f> &points,
                        const std::vector<int> &faceCounts,
                        const std::vector<int> &faceIndices) {
        if (&points == &baseA) {
            ++baseACalls;
        } else if (&points == &baseB) {
            ++baseBCalls;
        } else {
            ++posedCalls;
        }
        return RigExecComputeVertexNormals(points, faceCounts, faceIndices);
    };
    // Retention is explicitly owned by each consuming revision. The pure
    // solve has no process-global memo; posed reads remain fresh here.
    auto counting = [&](const std::vector<GfVec3f> &points,
                        const std::vector<int> &faceCounts,
                        const std::vector<int> &faceIndices) {
        if(&points==&baseA)return cacheA.VertexNormals(false,points,faceCounts,faceIndices,compute);
        if(&points==&baseB)return cacheB.VertexNormals(false,points,faceCounts,faceIndices,compute);
        return compute(points,faceCounts,faceIndices);
    };
    GfMatrix4d a1, b1, a2, b2;
    std::vector<std::string> diagnostics;
    CHECK(RigExecSolveSurfaceProjectorT(
        in, baseA, posedA, counts, indices,
        counting, "a1", &a1, &diagnostics));
    CHECK(RigExecSolveSurfaceProjectorT(
        in, baseB, posedB, counts, indices,
        counting, "b1", &b1, &diagnostics));
    CHECK(RigExecSolveSurfaceProjectorT(
        in, baseA, posedA, counts, indices,
        counting, "a2", &a2, &diagnostics));
    CHECK(RigExecSolveSurfaceProjectorT(
        in, baseB, posedB, counts, indices,
        counting, "b2", &b2, &diagnostics));
    CHECK(SameMatrix4dBits(a1, a2));
    CHECK(SameMatrix4dBits(b1, b2));
    CHECK(!SameMatrix4dBits(a1, b1));
    CHECK(baseACalls == 1);
    CHECK(baseBCalls == 1);
    CHECK(posedCalls == 4);
}

static void
TestLaplacianSmoothBorrowMatchesCopy()
{
    // A lent read side answers exactly the owned copy; a short range
    // passes through rather than reading out of bounds.
    const std::vector<GfVec3f> quad = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    std::vector<GfVec3f> borrowed = quad, owned = quad;
    RigExecApplyLaplacianSmooth(
        &borrowed, {4}, {0, 1, 2, 3}, 1.0, quad.data(), quad.size());
    RigExecApplyLaplacianSmooth(&owned, {4}, {0, 1, 2, 3}, 1.0);
    CHECK(SameVec3fArrayBits(borrowed, owned));
    CHECK(borrowed != quad);
    std::vector<GfVec3f> shorted = quad;
    RigExecApplyLaplacianSmooth(
        &shorted, {4}, {0, 1, 2, 3}, 1.0, quad.data(), quad.size() - 1);
    CHECK(SameVec3fArrayBits(shorted, quad));
}

// Linear blend skinning (RigExecSkinMover): the scalar reference against
// analytic expectations, then the SSE kernel against the scalar one.
static void
TestLinearBlendSkin()
{
    const GfVec3d p(1, 2, 3);
    GfMatrix4d t1(1.0), t2(1.0);
    t1.SetTranslate(GfVec3d(10, 0, 0));
    t2.SetTranslate(GfVec3d(0, 10, 0));
    const GfMatrix4d rt = GfMatrix4d(GfRotation(GfVec3d(0, 0, 1), 90.0),
                                     GfVec3d(1, 2, 3));
    const std::vector<GfMatrix4d> transforms = {t1, t2, rt};

    auto layoutFor = [&](const std::vector<int> &indices,
                         const std::vector<float> &weights, size_t elementSize) {
        RigExecSkinLayout layout;
        layout.transforms = transforms.data();
        layout.transformCount = transforms.size();
        layout.indices = indices.data();
        layout.weights = weights.data();
        layout.indexCount = indices.size();
        layout.elementSize = elementSize;
        layout.pointCount = indices.size() / elementSize;
        return layout;
    };

    // Single influence at weight 1: exactly the transform, no blending.
    {
        const std::vector<int> indices = {2};
        const std::vector<float> weights = {1.0f};
        const RigExecSkinLayout layout = layoutFor(indices, weights, 1);
        CHECK(layout.Validate());
        CHECK(Near(RigExecApplyLinearBlendSkin(p, layout, 0),
                   rt.TransformAffine(p)));
        CHECK(Near(RigExecApplyLinearBlendSkin(p, layout, 0),
                   RigExecApplyWeightedMatrix(p, rt, 1.0)));
    }
    // Two translations at 0.5 / 0.5: the analytic midpoint.
    {
        const std::vector<int> indices = {0, 1};
        const std::vector<float> weights = {0.5f, 0.5f};
        const RigExecSkinLayout layout = layoutFor(indices, weights, 2);
        CHECK(layout.Validate());
        CHECK(Near(RigExecApplyLinearBlendSkin(p, layout, 0),
                   p + GfVec3d(5, 5, 0)));
    }
    // Weights summing below one: the complement stays with the rest point,
    // so 0.25 / 0.25 moves a quarter of each way and keeps half of p --
    // NOT the bare sum 0.25 T1 p + 0.25 T2 p, which would halve p itself.
    {
        const std::vector<int> indices = {0, 1};
        const std::vector<float> weights = {0.25f, 0.25f};
        const RigExecSkinLayout layout = layoutFor(indices, weights, 2);
        CHECK(Near(RigExecApplyLinearBlendSkin(p, layout, 0),
                   p + GfVec3d(2.5, 2.5, 0)));
    }
    // All-zero weights leave the point where it was.
    {
        const std::vector<int> indices = {0, 1};
        const std::vector<float> weights = {0.0f, 0.0f};
        const RigExecSkinLayout layout = layoutFor(indices, weights, 2);
        CHECK(RigExecApplyLinearBlendSkin(p, layout, 0) == p);
    }
    // Shape validation: range, sign, finiteness, and length agreement.
    {
        std::string error;
        const std::vector<int> bad = {3};
        const std::vector<float> one = {1.0f};
        CHECK(!layoutFor(bad, one, 1).Validate(&error));
        CHECK(!error.empty());
        const std::vector<int> ok = {0};
        const std::vector<float> negative = {-0.5f};
        CHECK(!layoutFor(ok, negative, 1).Validate());
        const std::vector<float> nan = {
            std::numeric_limits<float>::quiet_NaN()};
        CHECK(!layoutFor(ok, nan, 1).Validate());
        RigExecSkinLayout short_ = layoutFor(ok, one, 1);
        short_.pointCount = 2;
        CHECK(!short_.Validate());
        RigExecSkinLayout zeroSlots = layoutFor(ok, one, 1);
        zeroSlots.elementSize = 0;
        CHECK(!zeroSlots.Validate());
    }

    // SIMD parity against the scalar reference over a mixed layout of
    // rotations, shears and scales (spec 13.4: within 1e-6 x extent).
    {
        const GfMatrix4d shear(1, 0, 0, 0, 0.3, 1, 0, 0, 0, 0.2, 1, 0, 0, 0, 0, 1);
        std::vector<GfMatrix4d> many = {
            GfMatrix4d(1.0).SetScale(GfVec3d(1.5, 0.8, 2.0)) * shear *
                GfMatrix4d(GfRotation(GfVec3d(1, 1, 0), 33.0), GfVec3d(2, -1, 4)),
            GfMatrix4d(GfRotation(GfVec3d(0, 1, 0), -70.0), GfVec3d(-3, 5, 1)),
            GfMatrix4d(1.0).SetTranslate(GfVec3d(7, 7, 7)),
            GfMatrix4d(GfRotation(GfVec3d(1, 0, 0), 120.0), GfVec3d(0, 0, 0))};
        std::vector<GfVec3f> in;
        std::vector<int> indices;
        std::vector<float> weights;
        const size_t elementSize = 3;
        for (int i = 0; i < 257; ++i) {
            in.push_back(GfVec3f(
                float(i % 17) - 8.0f, float(i % 5) * 2.0f, float(i % 11)));
            for (size_t k = 0; k < elementSize; ++k) {
                indices.push_back(int((i + k) % many.size()));
                // Sums vary above and below one on purpose.
                weights.push_back(float((i * 7 + k * 13) % 50) / 100.0f);
            }
        }
        RigExecSkinLayout layout;
        layout.transforms = many.data();
        layout.transformCount = many.size();
        layout.indices = indices.data();
        layout.weights = weights.data();
        layout.indexCount = indices.size();
        layout.elementSize = elementSize;
        layout.pointCount = in.size();
        CHECK(layout.Validate());
        std::vector<GfVec3f> simd(in.size());
        RigExecApplyLinearBlendSkinSimd(in.data(), simd.data(), layout);
        std::vector<GfVec3f> scalar(in.size());
        RigExecApplyLinearBlendSkin(in.data(), scalar.data(), layout);
        const double scale = 20.0;  // point-set extent
        for (size_t i = 0; i < in.size(); ++i) {
            CHECK((GfVec3d(simd[i]) - GfVec3d(scalar[i])).GetLength() <=
                  1e-6 * scale);
        }
        // In-place aliasing is part of the contract for both kernels.
        std::vector<GfVec3f> aliased = in;
        RigExecApplyLinearBlendSkinSimd(aliased.data(), aliased.data(), layout);
        CHECK(aliased == simd);
        aliased = in;
        RigExecApplyLinearBlendSkin(aliased.data(), aliased.data(), layout);
        CHECK(aliased == scalar);
    }
}

static void
TestWeightedMatrix()
{
    GfMatrix4d t(1.0);
    t.SetTranslate(GfVec3d(0, 2, 0));
    const GfVec3d p(1, 1, 1);
    // Weight zero: bit-exact pass-through.
    CHECK(RigExecApplyWeightedMatrix(p, t, 0.0) == p);
    // Weight one: full transform.
    CHECK(Near(RigExecApplyWeightedMatrix(p, t, 1.0), GfVec3d(1, 3, 1)));
    // Half weight blends translation.
    CHECK(Near(RigExecApplyWeightedMatrix(p, t, 0.5), GfVec3d(1, 2, 1)));
}

// Property-domain math movers: the five float/vec3f operations, the two
// matrix ones, and the uniform weight rule that wraps all of them.
static void
TestPropertyMath()
{
    // Unknown operations are rejected, not defaulted: the operation selects
    // the kernel, and quietly computing a different one is the failure mode
    // the return value exists to prevent.
    RigExecPropertyOp parsed;
    CHECK(RigExecParsePropertyOp(TfToken("clamp"), &parsed));
    CHECK(parsed == RigExecPropertyOp::Clamp);
    CHECK(!RigExecParsePropertyOp(TfToken("smoothstep"), &parsed));
    CHECK(!RigExecParsePropertyOp(TfToken(), &parsed));

    auto floatParams = [](RigExecPropertyOp op, float value, float lo,
                          float hi, float w) {
        RigExecPropertyMathParams<float> p;
        p.op = op;
        p.value = value;
        p.min = lo;
        p.max = hi;
        p.weight = w;
        return p;
    };

    CHECK(std::abs(RigExecApplyFloatMath(
              2.0f, floatParams(RigExecPropertyOp::Add, 3, 0, 1, 1)) -
                   5.0f) < 1e-6f);
    CHECK(std::abs(RigExecApplyFloatMath(
              2.0f, floatParams(RigExecPropertyOp::Multiply, 3, 0, 1, 1)) -
                   6.0f) < 1e-6f);
    CHECK(std::abs(RigExecApplyFloatMath(
              2.5f, floatParams(RigExecPropertyOp::Clamp, 0, 0, 1, 1)) -
                   1.0f) < 1e-6f);
    CHECK(std::abs(RigExecApplyFloatMath(
              -0.25f, floatParams(RigExecPropertyOp::Clamp, 0, 0, 1, 1))) <
          1e-6f);
    CHECK(std::abs(RigExecApplyFloatMath(
              0.5f, floatParams(RigExecPropertyOp::Blend, 1, 0, 1, 1)) -
                   1.0f) < 1e-6f);

    // remap normalizes [min, max] -> [0, 1] and deliberately does NOT clamp:
    // an out-of-range input stays out of range so a following clamp mover is
    // the thing that bounds it, and the two operations stay distinguishable.
    CHECK(std::abs(RigExecApplyFloatMath(
              5.0f, floatParams(RigExecPropertyOp::Remap, 0, 0, 10, 1)) -
                   0.5f) < 1e-6f);
    CHECK(RigExecApplyFloatMath(
              20.0f, floatParams(RigExecPropertyOp::Remap, 0, 0, 10, 1)) >
          1.5f);
    // A zero span has no meaningful normalization; 0 rather than an infinity.
    CHECK(std::abs(RigExecApplyFloatMath(
              5.0f, floatParams(RigExecPropertyOp::Remap, 0, 3, 3, 1))) <
          1e-6f);

    // The weight is the uniform mover blend: 0 is a pass-through, 1 is the
    // operation outright, and the midpoint is halfway between them.
    CHECK(std::abs(RigExecApplyFloatMath(
              2.0f, floatParams(RigExecPropertyOp::Add, 4, 0, 1, 0)) -
                   2.0f) < 1e-6f);
    CHECK(std::abs(RigExecApplyFloatMath(
              2.0f, floatParams(RigExecPropertyOp::Add, 4, 0, 1, 0.5f)) -
                   4.0f) < 1e-6f);

    // Vec3f is component-wise in every operation, bounds included.
    RigExecPropertyMathParams<GfVec3f> v;
    v.op = RigExecPropertyOp::Add;
    v.value = GfVec3f(0, 2, 0);
    CHECK(Near(GfVec3d(RigExecApplyVec3fMath(GfVec3f(0, 3, 0), v)),
               GfVec3d(0, 5, 0), 1e-6));
    v.op = RigExecPropertyOp::Clamp;
    v.min = GfVec3f(0, 0, 0);
    v.max = GfVec3f(1, 10, 1);
    CHECK(Near(GfVec3d(RigExecApplyVec3fMath(GfVec3f(-1, 5, 3), v)),
               GfVec3d(0, 5, 1), 1e-6));

    // Matrix: multiply post-multiplies (row-vector convention), so the
    // authored value is applied AFTER the incoming matrix.
    GfMatrix4d base(1.0), offset(1.0), out(1.0);
    base.SetTranslate(GfVec3d(1, 0, 0));
    offset.SetTranslate(GfVec3d(0, 5, 0));
    CHECK(RigExecApplyMatrixMath(base, RigExecPropertyOp::Multiply, offset,
                                 1.0f, &out));
    CHECK(Near(out.ExtractTranslation(), GfVec3d(1, 5, 0), 1e-9));
    // Weight 0 leaves the incoming matrix exactly where it was.
    CHECK(RigExecApplyMatrixMath(base, RigExecPropertyOp::Multiply, offset,
                                 0.0f, &out));
    CHECK(Near(out.ExtractTranslation(), GfVec3d(1, 0, 0), 1e-9));
    CHECK(RigExecApplyMatrixMath(base, RigExecPropertyOp::Blend, offset, 1.0f,
                                 &out));
    CHECK(Near(out.ExtractTranslation(), GfVec3d(0, 5, 0), 1e-9));
    // add/clamp/remap have no matrix meaning and are refused rather than
    // approximated.
    CHECK(!RigExecApplyMatrixMath(base, RigExecPropertyOp::Add, offset, 1.0f,
                                  &out));
    CHECK(!RigExecApplyMatrixMath(base, RigExecPropertyOp::Clamp, offset, 1.0f,
                                  &out));
}

// The property kernels instantiated with plain non-Gf types compute the
// same bits as the Gf entry points the evaluators call: what the zero-USD
// runtime relies on when it instantiates them with its own types.
namespace {
struct KernelVec2 {
    float v[2];
    float operator[](size_t i) const { return v[i]; }
    float &operator[](size_t i) { return v[i]; }
};
struct KernelVec3 {
    float v[3];
    float operator[](size_t i) const { return v[i]; }
    float &operator[](size_t i) { return v[i]; }
};
struct KernelMat4 {
    double m[4][4];
    const double *operator[](size_t r) const { return m[r]; }
    double *operator[](size_t r) { return m[r]; }
    // Each element summed left to right, as GfMatrix4d::operator*= does.
    friend KernelMat4 operator*(const KernelMat4 &a, const KernelMat4 &b)
    {
        KernelMat4 out;
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                out.m[r][c] = a.m[r][0] * b.m[0][c] + a.m[r][1] * b.m[1][c] +
                              a.m[r][2] * b.m[2][c] + a.m[r][3] * b.m[3][c];
            }
        }
        return out;
    }
};
}  // namespace

static void
TestPropertyMathKernelTypes()
{
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::vector<float> scalars = {0.0f, -0.0f, 0.5f, 1.0f, -1.5f,
                                        0.3f, 2.75f, 1e30f, -inf, inf, nan,
                                        1e-40f};
    const std::vector<float> weights = {-0.5f, 0.0f, 0.25f, 0.5f, 0.7f,
                                        1.0f, 1.5f, nan};
    const RigExecPropertyOp ops[] = {
        RigExecPropertyOp::Add, RigExecPropertyOp::Multiply,
        RigExecPropertyOp::Clamp, RigExecPropertyOp::Remap,
        RigExecPropertyOp::Blend, RigExecPropertyOp::Curve};
    // One key, a short table (linear scan) and a long one (binary search),
    // each with and without tangents.
    std::vector<std::vector<GfVec2f>> keySets = {
        {GfVec2f(0.5f, 2.0f)},
        {GfVec2f(-1.0f, 0.0f), GfVec2f(0.0f, 0.25f), GfVec2f(2.0f, 3.0f)},
        {}};
    for (int i = 0; i < 12; ++i) {
        keySets[2].push_back(
            GfVec2f(float(i) * 0.37f - 2.0f, std::sin(float(i)) * 1.3f));
    }
    size_t compared = 0;
    for (const std::vector<GfVec2f> &keys : keySets) {
        std::vector<GfVec2f> tangents;
        for (size_t i = 0; i < keys.size(); ++i) {
            tangents.push_back(
                GfVec2f(0.1f * float(i) - 0.3f, 0.7f - 0.2f * float(i)));
        }
        std::vector<KernelVec2> plainKeys, plainTangents;
        for (size_t i = 0; i < keys.size(); ++i) {
            plainKeys.push_back(KernelVec2{{keys[i][0], keys[i][1]}});
            plainTangents.push_back(
                KernelVec2{{tangents[i][0], tangents[i][1]}});
        }
        CHECK(RigExecValidateLinearKeys(keys.data(), keys.size()) ==
              RigExecValidateLinearKeysKernel(plainKeys.data(),
                                              plainKeys.size()));
        for (int withTangents = 0; withTangents < 2; ++withTangents) {
            for (float x : scalars) {
                for (float w : weights) {
                    for (RigExecPropertyOp op : ops) {
                        RigExecPropertyMathParams<float> gf;
                        RigExecPropertyMathKernelParams<float, KernelVec2>
                            plain;
                        gf.op = plain.op = op;
                        gf.value = plain.value = 0.75f;
                        gf.min = plain.min = -0.25f;
                        gf.max = plain.max = x == 0.3f ? -0.25f : 1.25f;
                        gf.weight = plain.weight = w;
                        gf.keys = keys.data();
                        plain.keys = plainKeys.data();
                        gf.keyCount = plain.keyCount = keys.size();
                        if (withTangents) {
                            gf.tangents = tangents.data();
                            plain.tangents = plainTangents.data();
                            gf.tangentCount = plain.tangentCount =
                                tangents.size();
                        }
                        CHECK(SameBits(
                            RigExecApplyFloatMath(x, gf),
                            RigExecApplyFloatMathKernel(x, plain)));
                        ++compared;

                        RigExecPropertyMathParams<GfVec3f> gf3;
                        RigExecPropertyMathKernelParams<KernelVec3,
                                                        KernelVec2>
                            plain3;
                        gf3.op = plain3.op = op;
                        gf3.value = GfVec3f(0.75f, x, -2.0f);
                        plain3.value = KernelVec3{{0.75f, x, -2.0f}};
                        gf3.min = GfVec3f(-0.25f, 0.0f, x);
                        plain3.min = KernelVec3{{-0.25f, 0.0f, x}};
                        gf3.max = GfVec3f(1.25f, 0.0f, 3.0f);
                        plain3.max = KernelVec3{{1.25f, 0.0f, 3.0f}};
                        gf3.weight = plain3.weight = w;
                        const GfVec3f base3(x, 0.4f, -x);
                        const GfVec3f a = RigExecApplyVec3fMath(base3, gf3);
                        const KernelVec3 b = RigExecApplyVec3fMathKernel(
                            KernelVec3{{x, 0.4f, -x}}, plain3);
                        CHECK(SameBits(a[0], b[0]) && SameBits(a[1], b[1]) &&
                              SameBits(a[2], b[2]));
                    }
                }
                CHECK(SameBits(
                    RigExecEvaluateHermiteKeys(
                        keys.data(), withTangents ? tangents.data() : nullptr,
                        keys.size(), x),
                    RigExecEvaluateHermiteKeysKernel(
                        plainKeys.data(),
                        withTangents ? plainTangents.data() : nullptr,
                        plainKeys.size(), x)));
                CHECK(SameBits(
                    RigExecEvaluateLinearKeys(keys.data(), keys.size(), x),
                    RigExecEvaluateLinearKeysKernel(plainKeys.data(),
                                                    plainKeys.size(), x)));
            }
        }
    }
    CHECK(compared > 1000);

    // Matrices: every operation and weight, including the refused ones.
    GfMatrix4d base(1.0), value(1.0);
    base.SetRotate(GfRotation(GfVec3d(1, 2, 3).GetNormalized(), 37.0));
    base.SetTranslateOnly(GfVec3d(1.25, -3.5, 0.1));
    value.SetRotate(GfRotation(GfVec3d(-2, 0.5, 1).GetNormalized(), -71.0));
    value.SetTranslateOnly(GfVec3d(-0.3, 7.0, 2.2));
    KernelMat4 plainBase, plainValue;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            plainBase.m[r][c] = base[r][c];
            plainValue.m[r][c] = value[r][c];
        }
    }
    for (float w : weights) {
        for (RigExecPropertyOp op : ops) {
            GfMatrix4d gfOut(2.0);
            KernelMat4 plainOut;
            for (size_t r = 0; r < 4; ++r) {
                for (size_t c = 0; c < 4; ++c) {
                    plainOut.m[r][c] = gfOut[r][c];
                }
            }
            const bool gfOk =
                RigExecApplyMatrixMath(base, op, value, w, &gfOut);
            const bool plainOk = RigExecApplyMatrixMathKernel(
                plainBase, op, plainValue, w, &plainOut);
            CHECK(gfOk == plainOk);
            for (size_t r = 0; r < 4; ++r) {
                for (size_t c = 0; c < 4; ++c) {
                    CHECK(SameBits(gfOut[r][c], plainOut.m[r][c]));
                }
            }
        }
    }
}

static void
TestAvarScaleNormalization()
{
    const double floor = RigExecAvarScaleFloor;
    CHECK(floor == 1e-4);
    CHECK(SameBits(RigExecNormalizeAvarScale(2.0), 2.0));
    CHECK(SameBits(RigExecNormalizeAvarScale(-2.0), -2.0));
    CHECK(SameBits(RigExecNormalizeAvarScale(floor), floor));
    CHECK(SameBits(RigExecNormalizeAvarScale(-floor), -floor));
    CHECK(SameBits(RigExecNormalizeAvarScale(0.5 * floor), floor));
    CHECK(SameBits(RigExecNormalizeAvarScale(-0.5 * floor), -floor));
    CHECK(SameBits(RigExecNormalizeAvarScale(0.0), floor));
    CHECK(SameBits(RigExecNormalizeAvarScale(-0.0), -floor));
    CHECK(RigExecNormalizeAvarScale(
              std::numeric_limits<double>::infinity()) == 1.0);
    CHECK(RigExecNormalizeAvarScale(
              -std::numeric_limits<double>::infinity()) == 1.0);
    CHECK(RigExecNormalizeAvarScale(
              std::numeric_limits<double>::quiet_NaN()) == 1.0);
}

// ---- Batch 8: spatial acceleration (H7/M41/M49) ----

static uint32_t _AccelRandState = 0x12345678u;
static float
_AccelRandFloat(float lo, float hi)
{
    _AccelRandState = _AccelRandState * 1664525u + 1013904223u;
    return lo + (hi - lo) *
                 (float(_AccelRandState >> 8) * (1.0f / 16777216.0f));
}

// Closest-point-on-triangle transcribed from Ericson, Real-Time
// Collision Detection, 5.1.5 (the reference the kernel cites), as the
// brute-force oracle for traversal equivalence. The known-answer pins
// in TestTriangleBvhMatchesBruteForce validate the transcription.
static GfVec3f
_OracleClosestPointOnTriangle(
    const GfVec3f &p, const GfVec3f &a, const GfVec3f &b, const GfVec3f &c)
{
    const GfVec3f ab = b - a, ac = c - a, ap = p - a;
    const float d1 = GfDot(ab, ap), d2 = GfDot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const GfVec3f bp = p - b;
    const float d3 = GfDot(ab, bp), d4 = GfDot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const float v = d1 / (d1 - d3);
        return a + ab * v;
    }
    const GfVec3f cp = p - c;
    const float d5 = GfDot(ab, cp), d6 = GfDot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom, w = vc * denom;
    return a + ab * v + ac * w;
}

static void
TestFanTriangulationBuilders()
{
    // H7/M41: quad fan, skip, truncation, and the flavor split.
    {
        const std::vector<int> counts = {4}, indices = {0, 1, 2, 3};
        const RigExecFanTrisH7 h7 =
            RigExecBuildFanTrisH7(counts, indices, 4);
        CHECK(!h7.truncated);
        CHECK(h7.tris.size() == 2);
        CHECK(h7.tris[0].a == 0 && h7.tris[0].b == 1 &&
              h7.tris[0].c == 2);
        CHECK(h7.tris[1].a == 0 && h7.tris[1].b == 2 &&
              h7.tris[1].c == 3);
        const RigExecFanTrisStrict strict =
            RigExecBuildFanTrisStrict(counts, indices, 4);
        CHECK(strict.valid);
        CHECK(strict.tris.size() == 2);
    }
    // Out-of-range triple: H7 skips it, strict fails the build.
    {
        const std::vector<int> counts = {3, 3};
        const std::vector<int> indices = {0, 1, 2, 0, 1, 9};
        const RigExecFanTrisH7 h7 =
            RigExecBuildFanTrisH7(counts, indices, 4);
        CHECK(!h7.truncated);
        CHECK(h7.tris.size() == 1);
        const RigExecFanTrisStrict strict =
            RigExecBuildFanTrisStrict(counts, indices, 4);
        CHECK(!strict.valid);
    }
    // Overrun: H7 truncates after the complete triangles, strict fails.
    {
        const std::vector<int> counts = {4};
        const std::vector<int> indices = {0, 1, 2};
        const RigExecFanTrisH7 h7 =
            RigExecBuildFanTrisH7(counts, indices, 4);
        CHECK(h7.truncated);
        CHECK(h7.tris.size() == 1);
        const RigExecFanTrisStrict strict =
            RigExecBuildFanTrisStrict(counts, indices, 4);
        CHECK(!strict.valid);
    }
    // Trailing indices: H7 has no trailing check, strict fails.
    {
        const std::vector<int> counts = {3};
        const std::vector<int> indices = {0, 1, 2, 0};
        const RigExecFanTrisH7 h7 =
            RigExecBuildFanTrisH7(counts, indices, 4);
        CHECK(!h7.truncated);
        CHECK(h7.tris.size() == 1);
        const RigExecFanTrisStrict strict =
            RigExecBuildFanTrisStrict(counts, indices, 4);
        CHECK(!strict.valid);
    }
    // Empty counts: H7 walks nothing, strict accepts only empty indices.
    {
        const std::vector<int> counts;
        const std::vector<int> indices;
        CHECK(!RigExecBuildFanTrisH7(counts, indices, 4).truncated);
        CHECK(RigExecBuildFanTrisStrict(counts, indices, 4).valid);
        const std::vector<int> trailing = {0};
        CHECK(!RigExecBuildFanTrisStrict(counts, trailing, 4).valid);
    }
    // Short faces and negative counts fail strict; H7 walks what it can.
    {
        const std::vector<int> counts = {2};
        const std::vector<int> indices = {0, 1};
        CHECK(!RigExecBuildFanTrisStrict(counts, indices, 4).valid);
        const RigExecFanTrisH7 h7 =
            RigExecBuildFanTrisH7(counts, indices, 4);
        CHECK(!h7.truncated);
        CHECK(h7.tris.empty());
        const std::vector<int> negative = {-1, 3};
        const std::vector<int> idx = {0, 1, 2};
        CHECK(!RigExecBuildFanTrisStrict(negative, idx, 4).valid);
        // A negative count moves the offset below zero: defined
        // truncation instead of the wrapped out-of-bounds read.
        CHECK(RigExecBuildFanTrisH7(negative, idx, 4).truncated);
    }
}

static void
TestTriangleCandidatesCannotUseGeometricPruning()
{
    // A closest-point callback has no certified computed-value AABB. This
    // deliberately outside-box result models an unsafe numerical branch:
    // even a very distant candidate must be evaluated after a nearer hit.
    std::vector<GfVec3f> points{
        GfVec3f(-1,-1,1), GfVec3f(1,-1,1), GfVec3f(0,1,1)};
    std::vector<RigExecFanTri> tris{{0,1,2}};
    for(int i=0;i<80;++i) {
        const int base=int(points.size()); const float x=1000.0f+float(i)*8.0f;
        points.push_back(GfVec3f(x,0,0)); points.push_back(GfVec3f(x+1,0,0));
        points.push_back(GfVec3f(x,1,0)); tris.push_back({base,base+1,base+2});
    }
    RigExecTriangleBvh<GfVec3f> index;
    CHECK(index.Build(points.data(),points.size(),tris.data(),tris.size()));
    size_t calls=0; bool ordinalOrder=true;
    const GfVec3f query(0.0f);
    const auto result=index.QueryNearest(query,points.data(),tris.data(),
        [&](const GfVec3f &p,const GfVec3f &a,const GfVec3f &b,const GfVec3f &c) {
            ordinalOrder = ordinalOrder && size_t(&a-points.data())/3 == calls;
            ++calls;
            return a[0]>100.0f ? p : _OracleClosestPointOnTriangle(p,a,b,c);
        });
    CHECK(calls==tris.size() && ordinalOrder);
    CHECK(SameVec3fBits(result,query));
}

static void
TestTrianglePathologicalFloatCandidates()
{
    const float tiny=std::numeric_limits<float>::denorm_min();
    const float huge=std::numeric_limits<float>::max()/4.0f;
    const float translated=std::ldexp(1.0f,60);
    const float translatedStep=std::ldexp(1.0f,38);
    const std::vector<GfVec3f> points{
        GfVec3f(0,0,0),GfVec3f(1,0,0),GfVec3f(2,tiny,0),
        GfVec3f(0,0,0),GfVec3f(tiny,0,0),GfVec3f(0,tiny,0),
        GfVec3f(-huge,0,0),GfVec3f(huge,0,0),GfVec3f(0,huge,0),
        GfVec3f(translated,translated,0),
        GfVec3f(translated+translatedStep,translated,0),
        GfVec3f(translated+2*translatedStep,translated+translatedStep,0),
        GfVec3f(1,1,1),GfVec3f(1,1,1),GfVec3f(1,1,1),
        GfVec3f(-0.0f,0,0),GfVec3f(-0.0f,1,0),GfVec3f(-0.0f,0,1)};
    std::vector<RigExecFanTri> tris;
    for(size_t i=0;i<points.size();i+=3) tris.push_back({int(i),int(i+1),int(i+2)});
    RigExecTriangleBvh<GfVec3f> index;
    CHECK(index.Build(points.data(),points.size(),tris.data(),tris.size()));
    const std::vector<GfVec3f> queries{
        GfVec3f(0.5f,tiny,1),GfVec3f(tiny,tiny,tiny),GfVec3f(huge,huge,huge),
        GfVec3f(translated,translated,translatedStep),GfVec3f(-0.0f,0,0),
        GfVec3f(std::numeric_limits<float>::infinity(),0,0),
        GfVec3f(std::numeric_limits<float>::quiet_NaN(),0,0)};
    for(const auto &query:queries) {
        float bestDistance=std::numeric_limits<float>::max(); GfVec3f best=query;
        for(const auto &tri:tris) {
            const auto candidate=_OracleClosestPointOnTriangle(query,
                points[size_t(tri.a)],points[size_t(tri.b)],points[size_t(tri.c)]);
            const float distance=(candidate-query).GetLengthSq();
            if(distance<bestDistance) { bestDistance=distance; best=candidate; }
        }
        size_t calls=0;
        const auto result=index.QueryNearest(query,points.data(),tris.data(),
            [&](const GfVec3f &p,const GfVec3f &a,const GfVec3f &b,const GfVec3f &c) {
                ++calls; return _OracleClosestPointOnTriangle(p,a,b,c);
            });
        CHECK(calls==tris.size());
        CHECK(SameVec3fBits(result,best));
    }
}

static void
TestTriangleBvhMatchesBruteForce()
{
    // H7: the BVH answers the nested fan loop bit-for-bit.
    // Known-answer pins validate the oracle transcription first: above
    // the face, the vertex region, and an edge region. All dyadic, so
    // every intermediate is exact.
    const GfVec3f a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
    CHECK(SameVec3fBits(_OracleClosestPointOnTriangle(
        GfVec3f(0.25f, 0.25f, 1.0f), a, b, c),
        GfVec3f(0.25f, 0.25f, 0.0f)));
    CHECK(SameVec3fBits(
        _OracleClosestPointOnTriangle(GfVec3f(-1, 0, 0), a, b, c), a));
    CHECK(SameVec3fBits(
        _OracleClosestPointOnTriangle(GfVec3f(0.5f, -1, 0), a, b, c),
        GfVec3f(0.5f, 0, 0)));
    // Exact tie: parallel tris at z=0 and z=2, query at z=1 over the
    // overlap. Both distances are exactly 1; the first triangle wins.
    {
        const std::vector<GfVec3f> surface = {
            GfVec3f(0, 0, 0), GfVec3f(2, 0, 0), GfVec3f(0, 2, 0),
            GfVec3f(0, 0, 2), GfVec3f(2, 0, 2), GfVec3f(0, 2, 2)};
        const std::vector<int> counts = {3, 3};
        const std::vector<int> indices = {0, 1, 2, 3, 4, 5};
        const RigExecFanTrisH7 fan =
            RigExecBuildFanTrisH7(counts, indices, surface.size());
        CHECK(!fan.truncated && fan.tris.size() == 2);
        RigExecTriangleBvh<GfVec3f> bvh;
        CHECK(bvh.Build(surface.data(), surface.size(), fan.tris.data(),
                        fan.tris.size()));
        const GfVec3f q = bvh.QueryNearest(
            GfVec3f(0.5f, 0.5f, 1.0f), surface.data(), fan.tris.data(),
            _OracleClosestPointOnTriangle);
        CHECK(SameBits(q[2], 0.0f));  // first triangle's plane
    }
    // Random soup: BVH with the oracle kernel vs the nested oracle loop.
    {
        _AccelRandState = 0xC0FFEEu;
        const int grid = 12;
        std::vector<GfVec3f> surface;
        for (int y = 0; y <= grid; ++y) {
            for (int x = 0; x <= grid; ++x) {
                surface.push_back(GfVec3f(float(x), float(y),
                                          _AccelRandFloat(-0.5f, 0.5f)));
            }
        }
        std::vector<int> counts;
        std::vector<int> indices;
        for (int y = 0; y < grid; ++y) {
            for (int x = 0; x < grid; ++x) {
                const int v00 = y * (grid + 1) + x;
                counts.push_back(4);
                indices.push_back(v00);
                indices.push_back(v00 + 1);
                indices.push_back(v00 + grid + 2);
                indices.push_back(v00 + grid + 1);
            }
        }
        // A degenerate fan triangle joins the soup.
        counts.push_back(3);
        indices.push_back(0);
        indices.push_back(0);
        indices.push_back(0);
        const RigExecFanTrisH7 fan =
            RigExecBuildFanTrisH7(counts, indices, surface.size());
        CHECK(!fan.truncated);
        CHECK(fan.tris.size() == size_t(grid * grid * 2 + 1));
        RigExecTriangleBvh<GfVec3f> bvh;
        CHECK(bvh.Build(surface.data(), surface.size(), fan.tris.data(),
                        fan.tris.size()));
        for (int i = 0; i < 200; ++i) {
            const GfVec3f p(_AccelRandFloat(-1.0f, float(grid) + 1.0f),
                            _AccelRandFloat(-1.0f, float(grid) + 1.0f),
                            _AccelRandFloat(-2.0f, 2.0f));
            float bestDistSq = std::numeric_limits<float>::max();
            GfVec3f best = p;
            for (const RigExecFanTri &tri : fan.tris) {
                const GfVec3f q = _OracleClosestPointOnTriangle(
                    p, surface[size_t(tri.a)], surface[size_t(tri.b)],
                    surface[size_t(tri.c)]);
                const float distSq = (q - p).GetLengthSq();
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    best = q;
                }
            }
            const GfVec3f got = bvh.QueryNearest(
                p, surface.data(), fan.tris.data(),
                _OracleClosestPointOnTriangle);
            CHECK(SameVec3fBits(got, best));
        }
        // A non-finite query prunes nothing and wins nothing: identity.
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const GfVec3f nanQ = bvh.QueryNearest(
            GfVec3f(nan, 0.0f, 0.0f), surface.data(), fan.tris.data(),
            _OracleClosestPointOnTriangle);
        CHECK(SameBits(nanQ[0], nan) && SameBits(nanQ[1], 0.0f) &&
              SameBits(nanQ[2], 0.0f));
    }
    // Non-finite surface points, empty input, and bad indices refuse
    // the build; the caller then runs its verbatim loop.
    {
        RigExecTriangleBvh<GfVec3f> bvh;
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const std::vector<GfVec3f> bad = {
            GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(nan, 0, 0)};
        const RigExecFanTri tri{0, 1, 2};
        CHECK(!bvh.Build(bad.data(), bad.size(), &tri, 1));
        const std::vector<GfVec3f> good = {
            GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0)};
        CHECK(!bvh.Build(good.data(), good.size(), &tri, 0));
        const RigExecFanTri oob{0, 1, 9};
        CHECK(!bvh.Build(good.data(), good.size(), &oob, 1));
    }
}

// Full H7 loop transcribed from the pre-acceleration kernel as the
// entry-point oracle: truncation, skips, clamp, and blend included.
static void
_OracleApplySurfaceProject(
    std::vector<GfVec3f> *points,
    const std::vector<GfVec3f> &surfacePoints,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    double weight)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (points->empty() || surfacePoints.empty() || w <= 0.0) {
        return;
    }
    for (GfVec3f &p : *points) {
        float bestDistSq = std::numeric_limits<float>::max();
        GfVec3f best = p;
        size_t offset = 0;
        for (int faceCount : faceVertexCounts) {
            for (int c = 1; c + 1 < faceCount; ++c) {
                const size_t i2 = offset + c + 1;
                if (i2 >= faceVertexIndices.size()) {
                    return;
                }
                const int ia = faceVertexIndices[offset];
                const int ib = faceVertexIndices[offset + c];
                const int ic = faceVertexIndices[i2];
                if (ia < 0 || ib < 0 || ic < 0 ||
                    size_t(ia) >= surfacePoints.size() ||
                    size_t(ib) >= surfacePoints.size() ||
                    size_t(ic) >= surfacePoints.size()) {
                    continue;
                }
                const GfVec3f q = _OracleClosestPointOnTriangle(
                    p, surfacePoints[ia], surfacePoints[ib],
                    surfacePoints[ic]);
                const float distSq = (q - p).GetLengthSq();
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    best = q;
                }
            }
            offset += faceCount;
        }
        p = p + (best - p) * float(w);
    }
}

static void
TestSurfaceProjectEntryPaths()
{
    // H7: the public entry matches the oracle loop on the fallback path
    // (small soups), the BVH path (large soups), both sides of each
    // gate, and every defined edge.
    _AccelRandState = 0xBEEF01u;
    const int grid = 6;  // 72 fan tris: over the BVH gate.
    std::vector<GfVec3f> surface;
    for (int y = 0; y <= grid; ++y) {
        for (int x = 0; x <= grid; ++x) {
            surface.push_back(GfVec3f(float(x), float(y),
                                      _AccelRandFloat(-1.0f, 1.0f)));
        }
    }
    std::vector<int> counts;
    std::vector<int> indices;
    for (int y = 0; y < grid; ++y) {
        for (int x = 0; x < grid; ++x) {
            const int v00 = y * (grid + 1) + x;
            counts.push_back(4);
            indices.push_back(v00);
            indices.push_back(v00 + 1);
            indices.push_back(v00 + grid + 2);
            indices.push_back(v00 + grid + 1);
        }
    }
    std::vector<GfVec3f> queries;
    for (int i = 0; i < 24; ++i) {
        queries.push_back(GfVec3f(_AccelRandFloat(-1.0f, 7.0f),
                                  _AccelRandFloat(-1.0f, 7.0f),
                                  _AccelRandFloat(-2.0f, 2.0f)));
    }
    auto checkBoth = [&](const std::vector<GfVec3f> &in,
                         const std::vector<int> &faceCounts,
                         const std::vector<int> &faceIndices, double w) {
        std::vector<GfVec3f> got = in, want = in;
        RigExecApplySurfaceProject(&got, surface, faceCounts, faceIndices,
                                   w);
        _OracleApplySurfaceProject(&want, surface, faceCounts, faceIndices,
                                   w);
        CHECK(SameVec3fArrayBits(got, want));
    };
    // BVH path: 72 tris, 24 queries.
    checkBoth(queries, counts, indices, 0.5);
    checkBoth(queries, counts, indices, 1.0);
    checkBoth(queries, counts, indices, 2.0);  // clamped to 1
    // Fallback path: few tris.
    {
        const std::vector<int> fewCounts(counts.begin(),
                                         counts.begin() + 4);
        const std::vector<int> fewIndices(indices.begin(),
                                          indices.begin() + 16);
        checkBoth(queries, fewCounts, fewIndices, 0.5);  // 8 tris
    }
    // Both sides of each gate: 63/64 tris x 3/4 queries.
    for (size_t tris : {size_t(63), size_t(64)}) {
        std::vector<int> gateCounts(counts.begin(),
                                    counts.begin() + tris / 2);
        std::vector<int> gateIndices(indices.begin(),
                                     indices.begin() + tris / 2 * 4);
        if (tris % 2) {
            gateCounts.push_back(3);
            gateIndices.push_back(0);
            gateIndices.push_back(1);
            gateIndices.push_back(2);
        }
        for (size_t nq : {size_t(3), size_t(4)}) {
            const std::vector<GfVec3f> fewQueries(queries.begin(),
                                                  queries.begin() + nq);
            checkBoth(fewQueries, gateCounts, gateIndices, 0.75);
        }
    }
    // Truncated topology modifies nothing.
    {
        const std::vector<int> badCounts = {4};
        const std::vector<int> badIndices = {0, 1, 2};
        checkBoth(queries, badCounts, badIndices, 0.5);
        std::vector<GfVec3f> untouched = queries;
        RigExecApplySurfaceProject(&untouched, surface, badCounts,
                                   badIndices, 0.5);
        CHECK(SameVec3fArrayBits(untouched, queries));
    }
    // Skipped triples and empty counts.
    {
        const std::vector<int> skipCounts = {3, 3};
        const std::vector<int> skipIndices = {0, 1, 2, 0, 1, 99};
        checkBoth(queries, skipCounts, skipIndices, 0.5);
        checkBoth(queries, {}, {}, 0.5);
    }
    // Non-finite surface points take the verbatim fallback.
    {
        std::vector<GfVec3f> nanSurface = surface;
        nanSurface[10] = GfVec3f(
            std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
        std::vector<GfVec3f> got = queries, want = queries;
        RigExecApplySurfaceProject(&got, nanSurface, counts, indices, 0.5);
        _OracleApplySurfaceProject(&want, nanSurface, counts, indices, 0.5);
        CHECK(SameVec3fArrayBits(got, want));
    }
    // Non-finite queries on the BVH path: identity per point.
    {
        std::vector<GfVec3f> nanQueries = queries;
        nanQueries[3] = GfVec3f(
            std::numeric_limits<float>::quiet_NaN(), 1.0f, 2.0f);
        checkBoth(nanQueries, counts, indices, 0.5);
    }
    // Weight edges: zero and negative leave points alone.
    checkBoth(queries, counts, indices, 0.0);
    checkBoth(queries, counts, indices, -1.0);
    {
        std::vector<GfVec3f> empty;
        RigExecApplySurfaceProject(&empty, surface, counts, indices, 0.5);
        CHECK(empty.empty());
    }
}

static void
TestTriangleBvhRoundingTies()
{
    // H7: float rounding can push a box bound strictly above an exact
    // float-distance tie (sol counterexample); the sound pad keeps the
    // first triangle winning. 64 degenerate tris in two clusters, four
    // queries: the BVH path, pinned against the oracle loop.
    const float posX = float(43.74845504760742);
    const float negX = float(-29.79381561279297);
    const float qx = float(6.977319717407227);
    const std::vector<GfVec3f> surface = {
        GfVec3f(posX, 0, 0), GfVec3f(negX, 0, 0)};
    std::vector<int> counts;
    std::vector<int> indices;
    for (int i = 0; i < 32; ++i) {
        counts.push_back(3);
        indices.push_back(0);
        indices.push_back(0);
        indices.push_back(0);
    }
    for (int i = 0; i < 32; ++i) {
        counts.push_back(3);
        indices.push_back(1);
        indices.push_back(1);
        indices.push_back(1);
    }
    const std::vector<GfVec3f> queries = {
        GfVec3f(qx, 0, 0), GfVec3f(qx, 0, 0), GfVec3f(qx, 0, 0),
        GfVec3f(qx, 0, 0)};
    std::vector<GfVec3f> got = queries, want = queries;
    RigExecApplySurfaceProject(&got, surface, counts, indices, 1.0);
    _OracleApplySurfaceProject(&want, surface, counts, indices, 1.0);
    CHECK(SameVec3fArrayBits(got, want));
    // The winner is the FIRST triangle's cluster, not the pruned one.
    for (const GfVec3f &p : got) {
        CHECK(p[0] > 0.0f);
    }
}

static bool
_SameHit(const RigExecSurfaceHit &a, const RigExecSurfaceHit &b)
{
    return a.a == b.a && a.b == b.b && a.c == b.c &&
           SameBits(a.u, b.u) && SameBits(a.v, b.v) &&
           SameBits(a.distance, b.distance);
}

// Pre-change per-face raycast transcribed as the independent oracle
// for the shared-triangle loop: the wrapper delegates to the loop under
// test, so agreement between the two alone would be circular. The exact
// pin below validates the transcription.
static bool
_OracleRaycastSurface(
    const std::vector<GfVec3f> &points,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices,
    const GfVec3d &origin, const GfVec3d &direction,
    RigExecSurfaceHit *hit)
{
    if (!hit || points.empty() || faceVertexCounts.empty()) return false;
    GfVec3d dir = direction;
    const double dirLength = dir.GetLength();
    if (!(dirLength > 0) || !std::isfinite(dirLength)) return false;
    dir /= dirLength;
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(origin[axis])) return false;
    }

    auto toDouble = [](const GfVec3f &v) {
        return GfVec3d(double(v[0]), double(v[1]), double(v[2]));
    };
    double nearest = std::numeric_limits<double>::infinity();
    int hitA = -1, hitB = -1, hitC = -1;
    double hitU = 0, hitV = 0;

    size_t offset = 0;
    for (int count : faceVertexCounts) {
        if (count < 3 ||
            static_cast<size_t>(count) > faceVertexIndices.size() - offset) {
            return false;
        }
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceVertexIndices[offset + corner];
            if (index < 0 || static_cast<size_t>(index) >= points.size()) {
                return false;
            }
        }
        const int origin0 = faceVertexIndices[offset];
        for (int corner = 1; corner + 1 < count; ++corner) {
            const int ia = origin0;
            const int ib = faceVertexIndices[offset + corner];
            const int ic = faceVertexIndices[offset + corner + 1];
            const GfVec3d a = toDouble(points[ia]);
            const GfVec3d b = toDouble(points[ib]);
            const GfVec3d c = toDouble(points[ic]);
            const GfVec3d e1 = b - a, e2 = c - a;
            const GfVec3d p = dir ^ e2;
            const double det = e1 * p;
            if (std::abs(det) < 1e-16) continue;
            const double inv = 1.0 / det;
            const GfVec3d t = origin - a;
            const double u = (t * p) * inv;
            if (u < 0.0 || u > 1.0) continue;
            const GfVec3d q = t ^ e1;
            const double v = (dir * q) * inv;
            if (v < 0.0 || u + v > 1.0) continue;
            const double distance = (e2 * q) * inv;
            if (distance <= 1e-9 || distance >= nearest) continue;
            nearest = distance;
            hitA = ia; hitB = ib; hitC = ic;
            hitU = u; hitV = v;
        }
        offset += static_cast<size_t>(count);
    }
    if (offset != faceVertexIndices.size()) return false;
    if (hitA < 0 || !std::isfinite(nearest)) return false;

    hit->a = hitA; hit->b = hitB; hit->c = hitC;
    hit->u = hitU; hit->v = hitV;
    hit->distance = nearest;
    return true;
}

static void
TestRaycastSharedTriangulation()
{
    // M41: the shared-triangle loop answers the per-face loop exactly,
    // and the strict builder reproduces every validation verdict.
    // Exact pin: hand-computed Moller-Trumbore through (0.5, 0.5).
    const std::vector<GfVec3f> tri = {
        GfVec3f(0, 0, 0), GfVec3f(2, 0, 0), GfVec3f(0, 2, 0)};
    const std::vector<int> counts = {3}, indices = {0, 1, 2};
    const RigExecFanTrisStrict fan =
        RigExecBuildFanTrisStrict(counts, indices, tri.size());
    CHECK(fan.valid && fan.tris.size() == 1);
    const GfVec3d origin(0.5, 0.5, 4.0), direction(0, 0, -1);
    RigExecSurfaceHit hit;
    CHECK(RigExecRaycastSurfaceT(tri, counts, indices, origin, direction,
                                 &hit));
    CHECK(hit.a == 0 && hit.b == 1 && hit.c == 2);
    CHECK(SameBits(hit.u, 0.25) && SameBits(hit.v, 0.25) &&
          SameBits(hit.distance, 4.0));
    RigExecSurfaceHit shared;
    CHECK(RigExecRaycastSurfaceTrisT(tri, fan, origin, direction,
                                     &shared));
    CHECK(_SameHit(shared, hit));
    RigExecSurfaceHit oracle;
    CHECK(_OracleRaycastSurface(tri, counts, indices, origin, direction,
                                &oracle));
    CHECK(_SameHit(oracle, hit));
    // Random mesh, random rays: agree on hits, misses, and degenerates.
    _AccelRandState = 0x5EED04u;
    const int grid = 8;
    std::vector<GfVec3f> surface;
    for (int y = 0; y <= grid; ++y) {
        for (int x = 0; x <= grid; ++x) {
            surface.push_back(GfVec3f(float(x), float(y),
                                      _AccelRandFloat(-1.0f, 1.0f)));
        }
    }
    std::vector<int> gridCounts;
    std::vector<int> gridIndices;
    for (int y = 0; y < grid; ++y) {
        for (int x = 0; x < grid; ++x) {
            const int v00 = y * (grid + 1) + x;
            gridCounts.push_back(4);
            gridIndices.push_back(v00);
            gridIndices.push_back(v00 + 1);
            gridIndices.push_back(v00 + grid + 2);
            gridIndices.push_back(v00 + grid + 1);
        }
    }
    // A zero-area face joins the mesh: the det guard skips it on both.
    gridCounts.push_back(3);
    gridIndices.push_back(0);
    gridIndices.push_back(0);
    gridIndices.push_back(0);
    const RigExecFanTrisStrict gridFan = RigExecBuildFanTrisStrict(
        gridCounts, gridIndices, surface.size());
    CHECK(gridFan.valid);
    for (int i = 0; i < 120; ++i) {
        const GfVec3d o(_AccelRandFloat(-2.0f, 10.0f),
                        _AccelRandFloat(-2.0f, 10.0f),
                        _AccelRandFloat(-4.0f, 6.0f));
        GfVec3d d(_AccelRandFloat(-1.0f, 1.0f),
                  _AccelRandFloat(-1.0f, 1.0f),
                  _AccelRandFloat(-1.0f, 1.0f));
        if (i % 10 == 0) {
            d = GfVec3d(0, 0, 0);  // degenerate direction
        }
        RigExecSurfaceHit h0{-1, -1, -1, -1.0, -1.0, -1.0};
        RigExecSurfaceHit h1{-1, -1, -1, -1.0, -1.0, -1.0};
        RigExecSurfaceHit h2{-1, -1, -1, -1.0, -1.0, -1.0};
        const bool r0 = _OracleRaycastSurface(surface, gridCounts,
                                              gridIndices, o, d, &h0);
        const bool r1 = RigExecRaycastSurfaceT(surface, gridCounts,
                                               gridIndices, o, d, &h1);
        const bool r2 = RigExecRaycastSurfaceTrisT(surface, gridFan, o, d,
                                                   &h2);
        CHECK(r0 == r1 && r1 == r2);
        CHECK(_SameHit(h0, h1));
        CHECK(_SameHit(h1, h2));
    }
    // Non-finite rays and null hit: false without touching the hit.
    {
        RigExecSurfaceHit h{-1, -1, -1, -1.0, -1.0, -1.0};
        const RigExecSurfaceHit sentinel = h;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        CHECK(!RigExecRaycastSurfaceT(tri, counts, indices, origin,
                                      GfVec3d(nan, 0, 0), &h));
        CHECK(_SameHit(h, sentinel));
        CHECK(!RigExecRaycastSurfaceTrisT(tri, fan, origin,
                                          GfVec3d(nan, 0, 0), &h));
        CHECK(_SameHit(h, sentinel));
        CHECK(!RigExecRaycastSurfaceT(tri, counts, indices,
                                      GfVec3d(nan, 0, 0), direction, &h));
        CHECK(_SameHit(h, sentinel));
        CHECK(!RigExecRaycastSurfaceT(tri, counts, indices, origin,
                                      direction, nullptr));
    }
    // Every invalid flavor: false with the hit untouched, on both paths.
    {
        const std::vector<std::vector<int>> badCountsList = {
            {2}, {4}, {3}, {3}};
        const std::vector<std::vector<int>> badIndicesList = {
            {0, 1}, {0, 1, 2}, {0, 1, 9}, {0, 1, 2, 0}};
        for (size_t k = 0; k < badCountsList.size(); ++k) {
            const RigExecFanTrisStrict badFan =
                RigExecBuildFanTrisStrict(badCountsList[k],
                                          badIndicesList[k], tri.size());
            CHECK(!badFan.valid);
            RigExecSurfaceHit h0{1, 2, 3, 4.0, 5.0, 6.0};
            RigExecSurfaceHit h1{1, 2, 3, 4.0, 5.0, 6.0};
            RigExecSurfaceHit h2{1, 2, 3, 4.0, 5.0, 6.0};
            CHECK(!_OracleRaycastSurface(tri, badCountsList[k],
                                         badIndicesList[k], origin,
                                         direction, &h0));
            CHECK(!RigExecRaycastSurfaceT(tri, badCountsList[k],
                                          badIndicesList[k], origin,
                                          direction, &h1));
            CHECK(!RigExecRaycastSurfaceTrisT(tri, badFan, origin,
                                              direction, &h2));
            CHECK(h0.a == 1 && h1.a == 1 && h2.a == 1);
        }
        // Empty counts: the wrapper rejects before delegating, while the
        // empty fan itself casts nothing. Both false either way.
        const RigExecFanTrisStrict emptyFan =
            RigExecBuildFanTrisStrict({}, {}, tri.size());
        CHECK(emptyFan.valid && emptyFan.tris.empty());
        RigExecSurfaceHit h0{1, 2, 3, 4.0, 5.0, 6.0};
        RigExecSurfaceHit h1{1, 2, 3, 4.0, 5.0, 6.0};
        RigExecSurfaceHit h2{1, 2, 3, 4.0, 5.0, 6.0};
        CHECK(!_OracleRaycastSurface(tri, {}, {}, origin, direction,
                                     &h0));
        CHECK(!RigExecRaycastSurfaceT(tri, {}, {}, origin, direction,
                                      &h1));
        CHECK(!RigExecRaycastSurfaceTrisT(tri, emptyFan, origin,
                                          direction, &h2));
        CHECK(h0.a == 1 && h1.a == 1 && h2.a == 1);
    }
}

static void
TestSolverSharedTriangulation()
{
    // M41: every cast of a solve shares one fan triangulation. The
    // up-front guard rejects mismatched point counts, so a single
    // validation serves base and posed alike; these pins hold the guard
    // and prove the shared posed cast discriminates hit from miss.
    const std::vector<GfVec3f> quad = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    const std::vector<int> counts = {4}, indices = {0, 1, 2, 3};
    RigExecSurfaceProjectorInputs<GfMatrix4d, GfVec3d> in;
    in.rayOrigin = GfVec3d(0.25, 0.25, 8.0);
    in.rayDirection = GfVec3d(0, 0, -1);
    in.rayUp = GfVec3d(0, 1, 0);
    in.shaderOffset.SetIdentity();
    in.sourceBase.SetIdentity();
    in.sourceFinal.SetIdentity();
    in.sourceSpaceBase.SetIdentity();
    in.sourceSpaceFinal.SetIdentity();
    in.spaceFinal.SetIdentity();
    in.worldToMesh.SetIdentity();
    RigExecSurfaceProjectorInputs<GfMatrix4d, GfVec3d> reproject = in;
    reproject.reproject = true;
    const std::vector<GfVec3f> posed = {
        {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0}};
    // The guard: mismatched counts fail before any cast, in both
    // directions. The shared triangulation relies on it.
    {
        const std::vector<GfVec3f> shortBase(quad.begin(),
                                             quad.begin() + 3);
        std::vector<GfVec3f> longPosed = posed;
        longPosed.push_back(GfVec3f(9, 9, 9));
        GfMatrix4d ignored;
        std::vector<std::string> diags;
        CHECK(!RigExecSolveSurfaceProjectorT(
            reproject, shortBase, posed, counts, indices,
            static_cast<RigExecVertexNormalsFn>(
            &RigExecComputeVertexNormals), "guard", &ignored, &diags));
        CHECK(!RigExecSolveSurfaceProjectorT(
            reproject, quad, longPosed, counts, indices,
            static_cast<RigExecVertexNormalsFn>(
            &RigExecComputeVertexNormals), "guard", &ignored, &diags));
        CHECK(diags.size() == 2);
        for (const std::string &d : diags) {
            CHECK(d.find("no usable points or topology") !=
                  std::string::npos);
        }
    }
    // Reproject through the shared fan: a hittable posed surface
    // casts clean, a missed one falls back to the material point.
    auto hasMissMessage = [](const std::vector<std::string> &diags) {
        for (const std::string &d : diags) {
            if (d.find("re-cast missed") != std::string::npos) {
                return true;
            }
        }
        return false;
    };
    {
        const std::vector<GfVec3f> liftedPosed = {
            {0, 0, 5}, {1, 0, 5}, {1, 1, 5}, {0, 1, 5}};
        GfMatrix4d viaHit, viaFallback;
        std::vector<std::string> hitDiags, fallbackDiags;
        CHECK(RigExecSolveSurfaceProjectorT(
            reproject, quad, liftedPosed, counts, indices,
            static_cast<RigExecVertexNormalsFn>(
            &RigExecComputeVertexNormals), "hit", &viaHit, &hitDiags));
        CHECK(!hasMissMessage(hitDiags));
        CHECK(RigExecSolveSurfaceProjectorT(
            reproject, quad, posed, counts, indices,
            static_cast<RigExecVertexNormalsFn>(
            &RigExecComputeVertexNormals), "miss", &viaFallback,
            &fallbackDiags));
        CHECK(hasMissMessage(fallbackDiags));
    }
}

static void
TestMeshAdjacencyBuild()
{
    // H1: the sorted-vector adjacency answers the legacy set-based
    // verdicts: sorted unique neighbors, a repeated corner once, and
    // the same invalid-topology failures with the output untouched.
    RigExecMeshAdjacency adjacency;
    CHECK(RigExecBuildMeshAdjacency(
        4, {4}, {0, 1, 2, 3}, &adjacency));
    CHECK(adjacency.valid);
    CHECK(adjacency.pointCount == 4);
    CHECK((adjacency.neighbors[0] == std::vector<int>{1, 3}));
    CHECK((adjacency.neighbors[1] == std::vector<int>{0, 2}));
    CHECK((adjacency.neighbors[2] == std::vector<int>{1, 3}));
    CHECK((adjacency.neighbors[3] == std::vector<int>{0, 2}));
    CHECK(adjacency.Covers(4));
    CHECK(!adjacency.Covers(3));
    // A repeated corner contributes once, as the set insert did.
    CHECK(RigExecBuildMeshAdjacency(
        3, {3}, {0, 0, 1}, &adjacency));
    CHECK((adjacency.neighbors[0] == std::vector<int>{0, 1}));
    CHECK((adjacency.neighbors[1] == std::vector<int>{0}));
    CHECK(adjacency.neighbors[2].empty());
    // Invalid inputs fail with the output untouched.
    const RigExecMeshAdjacency before = adjacency;
    CHECK(!RigExecBuildMeshAdjacency(
        4, {4}, {0, 1, 2, 9}, &adjacency));
    CHECK(!RigExecBuildMeshAdjacency(
        4, {4}, {0, 1}, &adjacency));
    CHECK(!RigExecBuildMeshAdjacency(
        4, {4}, {0, 1, 2, 3}, nullptr));
    CHECK(adjacency.valid == before.valid &&
          adjacency.pointCount == before.pointCount &&
          adjacency.neighbors == before.neighbors);
    // An empty mesh builds valid.
    CHECK(RigExecBuildMeshAdjacency(0, {}, {}, &adjacency));
    CHECK(adjacency.valid);
    CHECK(adjacency.Covers(0));
}

static void
TestLaplacianSmoothCachedMatchesDirect()
{
    // H1: the cached-adjacency smoother answers the direct kernel
    // bit-identically, and passes invalid topology through the same way.
    const std::vector<GfVec3f> quad = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    RigExecMeshAdjacency adjacency;
    CHECK(RigExecBuildMeshAdjacency(
        4, {4}, {0, 1, 2, 3}, &adjacency));
    std::vector<GfVec3f> cached = quad, direct = quad;
    RigExecApplyLaplacianSmoothWithAdjacency(&cached, adjacency, 1.0);
    RigExecApplyLaplacianSmooth(&direct, {4}, {0, 1, 2, 3}, 1.0);
    CHECK(SameVec3fArrayBits(cached, direct));
    CHECK(cached != quad);
    // The borrowed read side through the cached form.
    std::vector<GfVec3f> borrowed = quad;
    RigExecApplyLaplacianSmoothWithAdjacency(
        &borrowed, adjacency, 1.0, quad.data(), quad.size());
    CHECK(SameVec3fArrayBits(borrowed, direct));
    // Invalid topology passes through on both paths.
    RigExecMeshAdjacency invalid;
    CHECK(!RigExecBuildMeshAdjacency(
        4, {4}, {0, 1, 2, 9}, &invalid));
    std::vector<GfVec3f> cachedInvalid = quad, directInvalid = quad;
    RigExecApplyLaplacianSmoothWithAdjacency(
        &cachedInvalid, invalid, 1.0);
    RigExecApplyLaplacianSmooth(
        &directInvalid, {4}, {0, 1, 2, 9}, 1.0);
    CHECK(SameVec3fArrayBits(cachedInvalid, quad));
    CHECK(SameVec3fArrayBits(directInvalid, quad));
    // An entry covering another point count passes through.
    const std::vector<GfVec3f> trio(quad.begin(), quad.begin() + 3);
    std::vector<GfVec3f> mismatch = trio;
    RigExecApplyLaplacianSmoothWithAdjacency(&mismatch, adjacency, 1.0);
    CHECK(SameVec3fArrayBits(mismatch, trio));
}

static void
TestSurfaceProjectCachedAccelMatchesDirect()
{
    // H1: the cached fan + BVH answers the per-call build
    // bit-identically across the gate sides and the fallback edges.
    const std::vector<GfVec3f> quad = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    const std::vector<GfVec3f> dest = {
        {0.25f, 0.25f, 1}, {0.75f, 0.75f, -1}, {2, 2, 2},
        {-1, -1, -1}, {0.5f, 0.5f, 0.25f}};
    // Small fan: the verbatim loop on both paths.
    RigExecSurfaceAccel accel;
    RigExecBuildSurfaceAccel({4}, {0, 1, 2, 3}, quad, &accel);
    CHECK(accel.fan && !accel.bvh);
    std::vector<GfVec3f> cached = dest, direct = dest;
    RigExecApplySurfaceProject(
        &cached, quad, {4}, {0, 1, 2, 3}, 1.0, &accel);
    RigExecApplySurfaceProject(&direct, quad, {4}, {0, 1, 2, 3}, 1.0);
    CHECK(SameVec3fArrayBits(cached, direct));
    CHECK(cached != dest);
    // Truncated: no point modified on either path.
    RigExecSurfaceAccel truncated;
    RigExecBuildSurfaceAccel({4}, {0, 1}, quad, &truncated);
    CHECK(truncated.fan && truncated.fan->truncated);
    std::vector<GfVec3f> cachedTrunc = dest, directTrunc = dest;
    RigExecApplySurfaceProject(
        &cachedTrunc, quad, {4}, {0, 1}, 1.0, &truncated);
    RigExecApplySurfaceProject(&directTrunc, quad, {4}, {0, 1}, 1.0);
    CHECK(SameVec3fArrayBits(cachedTrunc, dest));
    CHECK(SameVec3fArrayBits(directTrunc, dest));
    // Big fan: the BVH on both paths, bit-identical.
    std::vector<GfVec3f> surface;
    std::vector<int> counts, indices;
    for (int y = 0; y < 9; ++y) {
        for (int x = 0; x < 9; ++x) {
            surface.emplace_back(float(x), float(y), 0.1f * (x + y));
        }
    }
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            const int a = y * 9 + x;
            counts.push_back(4);
            indices.insert(indices.end(), {a, a + 1, a + 10, a + 9});
        }
    }
    RigExecSurfaceAccel big;
    RigExecBuildSurfaceAccel(counts, indices, surface, &big);
    CHECK(big.fan && big.bvh);
    std::vector<GfVec3f> bigCached = dest, bigDirect = dest;
    RigExecApplySurfaceProject(
        &bigCached, surface, counts, indices, 0.5, &big);
    RigExecApplySurfaceProject(
        &bigDirect, surface, counts, indices, 0.5);
    CHECK(SameVec3fArrayBits(bigCached, bigDirect));
    CHECK(bigCached != dest);
    // Non-finite surface: the BVH stays null and the verbatim
    // fallback answers on both paths.
    std::vector<GfVec3f> nanSurface = surface;
    nanSurface[40][2] = std::numeric_limits<float>::quiet_NaN();
    RigExecSurfaceAccel nanAccel;
    RigExecBuildSurfaceAccel(counts, indices, nanSurface, &nanAccel);
    CHECK(nanAccel.fan && !nanAccel.bvh);
    std::vector<GfVec3f> nanCached = dest, nanDirect = dest;
    RigExecApplySurfaceProject(
        &nanCached, nanSurface, counts, indices, 1.0, &nanAccel);
    RigExecApplySurfaceProject(
        &nanDirect, nanSurface, counts, indices, 1.0);
    CHECK(SameVec3fArrayBits(nanCached, nanDirect));
    // An entry built for another surface falls back to the per-call
    // build rather than answering wrong.
    std::vector<GfVec3f> other = quad;
    other.emplace_back(2, 2, 2);
    std::vector<GfVec3f> fellBack = dest, fellDirect = dest;
    RigExecApplySurfaceProject(
        &fellBack, other, {4}, {0, 1, 2, 3}, 1.0, &accel);
    RigExecApplySurfaceProject(
        &fellDirect, other, {4}, {0, 1, 2, 3}, 1.0);
    CHECK(SameVec3fArrayBits(fellBack, fellDirect));
    // Same cardinality, stale truncation verdict: the truncated entry
    // must not suppress projection onto the valid topology.
    std::vector<GfVec3f> staleBack = dest, staleDirect = dest;
    RigExecApplySurfaceProject(
        &staleBack, quad, {4}, {0, 1, 2, 3}, 1.0, &truncated);
    RigExecApplySurfaceProject(
        &staleDirect, quad, {4}, {0, 1, 2, 3}, 1.0);
    CHECK(SameVec3fArrayBits(staleBack, staleDirect));
    CHECK(staleBack != dest);
    // The other direction: a stale valid entry must not mask a
    // truncated current topology.
    std::vector<GfVec3f> maskBack = dest, maskDirect = dest;
    RigExecApplySurfaceProject(
        &maskBack, quad, {4}, {0, 1}, 1.0, &accel);
    RigExecApplySurfaceProject(
        &maskDirect, quad, {4}, {0, 1}, 1.0);
    CHECK(SameVec3fArrayBits(maskBack, dest));
    CHECK(SameVec3fArrayBits(maskDirect, dest));
    // Same cardinality, animated points on the BVH path: the stale
    // bounds must not answer.
    std::vector<GfVec3f> moved = surface;
    for (GfVec3f &p : moved) {
        p[2] += 10.0f;
    }
    std::vector<GfVec3f> movedBack = dest, movedDirect = dest;
    RigExecApplySurfaceProject(
        &movedBack, moved, counts, indices, 0.5, &big);
    RigExecApplySurfaceProject(
        &movedDirect, moved, counts, indices, 0.5);
    CHECK(SameVec3fArrayBits(movedBack, movedDirect));
    CHECK(movedDirect != bigCached);
    // Same cardinality, degenerate topology on the BVH path: the stale
    // triangles must not answer.
    const std::vector<int> collapsedCounts(counts.size(), 4);
    const std::vector<int> collapsedIndices(indices.size(), 0);
    std::vector<GfVec3f> collapsedBack = dest, collapsedDirect = dest;
    RigExecApplySurfaceProject(
        &collapsedBack, surface, collapsedCounts, collapsedIndices,
        0.5, &big);
    RigExecApplySurfaceProject(
        &collapsedDirect, surface, collapsedCounts, collapsedIndices,
        0.5);
    CHECK(SameVec3fArrayBits(collapsedBack, collapsedDirect));
    CHECK(collapsedDirect != bigCached);
}

static void
TestSpanOverloadsMatchVectorForms()
{
    // H2: every span overload answers its vector form bit-identically,
    // including the invalid-input verdicts. The spans borrow the vectors
    // below for each call only.
    const std::vector<GfVec3f> quad = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    const std::vector<int> counts = {4};
    const std::vector<int> indices = {0, 1, 2, 3};
    // Adjacency.
    {
        RigExecMeshAdjacency viaSpan, viaVector;
        CHECK(RigExecBuildMeshAdjacency(
            4, counts.data(), counts.size(), indices.data(),
            indices.size(), &viaSpan));
        CHECK(RigExecBuildMeshAdjacency(4, counts, indices, &viaVector));
        CHECK(viaSpan.valid == viaVector.valid &&
              viaSpan.pointCount == viaVector.pointCount &&
              viaSpan.neighbors == viaVector.neighbors);
        // Invalid topology fails on both paths with the output untouched.
        RigExecMeshAdjacency bad = viaSpan;
        CHECK(!RigExecBuildMeshAdjacency(
            4, counts.data(), counts.size(), indices.data(), 2, &bad));
        CHECK(bad.neighbors == viaSpan.neighbors);
        // A null range with a nonzero size fails closed; with size zero
        // it is the empty range.
        CHECK(!RigExecBuildMeshAdjacency(
            4, nullptr, 1, indices.data(), indices.size(), &bad));
        CHECK(RigExecBuildMeshAdjacency(
            0, nullptr, 0, nullptr, 0, &bad));
        CHECK(bad.valid && bad.Covers(0));
    }
    // Fan triangulation, span-vs-vector directly.
    {
        const RigExecFanTrisH7 spanFan = RigExecBuildFanTrisH7(
            counts.data(), counts.size(), indices.data(),
            indices.size(), quad.size());
        const RigExecFanTrisH7 vectorFan =
            RigExecBuildFanTrisH7(counts, indices, quad.size());
        CHECK(spanFan.truncated == vectorFan.truncated);
        CHECK(spanFan.tris.size() == vectorFan.tris.size());
        CHECK(spanFan.tris.size() == 2);
        for (size_t i = 0; i < spanFan.tris.size(); ++i) {
            CHECK(spanFan.tris[i].a == vectorFan.tris[i].a &&
                  spanFan.tris[i].b == vectorFan.tris[i].b &&
                  spanFan.tris[i].c == vectorFan.tris[i].c);
        }
        // The overrun truncation matches too.
        const std::vector<int> shortIndices(indices.begin(),
                                             indices.begin() + 2);
        const RigExecFanTrisH7 spanTrunc = RigExecBuildFanTrisH7(
            counts.data(), counts.size(), shortIndices.data(),
            shortIndices.size(), quad.size());
        const RigExecFanTrisH7 vectorTrunc =
            RigExecBuildFanTrisH7(counts, shortIndices, quad.size());
        CHECK(spanTrunc.truncated && vectorTrunc.truncated);
        CHECK(spanTrunc.tris.empty() && vectorTrunc.tris.empty());
        // A null range with a nonzero size truncates; with size zero
        // it is the empty range (span-only verdicts).
        const RigExecFanTrisH7 nullCounts = RigExecBuildFanTrisH7(
            nullptr, 1, indices.data(), indices.size(), quad.size());
        CHECK(nullCounts.truncated && nullCounts.tris.empty());
        const RigExecFanTrisH7 nullIndices = RigExecBuildFanTrisH7(
            counts.data(), counts.size(), nullptr, 4, quad.size());
        CHECK(nullIndices.truncated && nullIndices.tris.empty());
        const RigExecFanTrisH7 emptyRange = RigExecBuildFanTrisH7(
            nullptr, 0, nullptr, 0, 0);
        CHECK(!emptyRange.truncated && emptyRange.tris.empty());
    }
    // Smooth.
    {
        std::vector<GfVec3f> span = quad, vector = quad;
        RigExecApplyLaplacianSmooth(
            &span, counts.data(), counts.size(), indices.data(),
            indices.size(), 1.0);
        RigExecApplyLaplacianSmooth(&vector, counts, indices, 1.0);
        CHECK(SameVec3fArrayBits(span, vector));
        CHECK(span != quad);
        // The borrowed read side through the span form.
        std::vector<GfVec3f> borrowed = quad;
        RigExecApplyLaplacianSmooth(
            &borrowed, counts.data(), counts.size(), indices.data(),
            indices.size(), 1.0, quad.data(), quad.size());
        CHECK(SameVec3fArrayBits(borrowed, vector));
        // Invalid topology passes through on both paths.
        const std::vector<int> badIndices = {0, 1, 2, 9};
        std::vector<GfVec3f> spanBad = quad, vectorBad = quad;
        RigExecApplyLaplacianSmooth(
            &spanBad, counts.data(), counts.size(), badIndices.data(),
            badIndices.size(), 1.0);
        RigExecApplyLaplacianSmooth(&vectorBad, counts, badIndices, 1.0);
        CHECK(SameVec3fArrayBits(spanBad, quad));
        CHECK(SameVec3fArrayBits(vectorBad, quad));
    }
    // Normals.
    {
        const std::vector<GfVec3f> span = RigExecComputeVertexNormals(
            quad.data(), quad.size(), counts.data(), counts.size(),
            indices.data(), indices.size());
        const std::vector<GfVec3f> vector =
            RigExecComputeVertexNormals(quad, counts, indices);
        CHECK(SameVec3fArrayBits(span, vector));
        CHECK(!span.empty());
        // The invalid-topology verdict (zero normals) matches too.
        const std::vector<int> shortIndices(indices.begin(),
                                             indices.begin() + 2);
        const std::vector<GfVec3f> spanBad = RigExecComputeVertexNormals(
            quad.data(), quad.size(), counts.data(), counts.size(),
            shortIndices.data(), shortIndices.size());
        const std::vector<GfVec3f> vectorBad =
            RigExecComputeVertexNormals(quad, counts, shortIndices);
        CHECK(SameVec3fArrayBits(spanBad, vectorBad));
    }
    // Extent: per-point widths, one width, no widths, empty points.
    {
        const std::vector<float> perPoint = {2, 2, 2, 2};
        const std::vector<float> one = {4};
        const std::vector<float> none;
        for (const std::vector<float> *w : {&perPoint, &one, &none}) {
            const std::vector<GfVec3f> span = RigExecComputeExtent(
                quad.data(), quad.size(), w->data(), w->size());
            const std::vector<GfVec3f> vector =
                RigExecComputeExtent(quad, *w);
            CHECK(SameVec3fArrayBits(span, vector));
            CHECK(span.size() == 2);
        }
        const std::vector<GfVec3f> spanEmpty = RigExecComputeExtent(
            nullptr, 0, nullptr, 0);
        const std::vector<GfVec3f> vectorEmpty = RigExecComputeExtent(
            std::vector<GfVec3f>(), std::vector<float>());
        CHECK(spanEmpty.empty() && vectorEmpty.empty());
    }
    // Lattice.
    {
        const GfVec3i divisions(2, 2, 2);
        const std::vector<GfVec3f> restCage = {
            {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
            {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};
        std::vector<GfVec3f> posedCage = restCage;
        posedCage[7] = GfVec3f(2, 2, 2);
        const std::vector<GfVec3f> rest = {
            {0.25f, 0.25f, 0.25f}, {0.75f, 0.75f, 0.75f}};
        std::vector<GfVec3f> span = rest, vector = rest;
        RigExecApplyLattice(
            &span, rest.data(), rest.size(), restCage.data(),
            restCage.size(), posedCage.data(), posedCage.size(),
            divisions);
        RigExecApplyLattice(
            &vector, rest, restCage, posedCage, divisions);
        CHECK(SameVec3fArrayBits(span, vector));
        CHECK(span != rest);
        // A cage mismatch passes through on both paths.
        const std::vector<GfVec3f> shortCage(restCage.begin(),
                                             restCage.end() - 1);
        std::vector<GfVec3f> spanBad = rest, vectorBad = rest;
        RigExecApplyLattice(
            &spanBad, rest.data(), rest.size(), shortCage.data(),
            shortCage.size(), posedCage.data(), posedCage.size(),
            divisions);
        RigExecApplyLattice(
            &vectorBad, rest, shortCage, posedCage, divisions);
        CHECK(SameVec3fArrayBits(spanBad, rest));
        CHECK(SameVec3fArrayBits(vectorBad, rest));
    }
    // Surface accel + project, both gate sides and the truncation edge.
    {
        const std::vector<GfVec3f> dest = {
            {0.25f, 0.25f, 1}, {0.75f, 0.75f, -1}, {2, 2, 2},
            {-1, -1, -1}, {0.5f, 0.5f, 0.25f}};
        RigExecSurfaceAccel spanAccel, vectorAccel;
        RigExecBuildSurfaceAccel(
            counts.data(), counts.size(), indices.data(), indices.size(),
            quad.data(), quad.size(), &spanAccel);
        RigExecBuildSurfaceAccel(counts, indices, quad, &vectorAccel);
        CHECK(spanAccel.fan && vectorAccel.fan);
        CHECK(spanAccel.fan->truncated == vectorAccel.fan->truncated);
        CHECK(spanAccel.fan->tris.size() ==
              vectorAccel.fan->tris.size());
        for (size_t i = 0; i < spanAccel.fan->tris.size(); ++i) {
            const RigExecFanTri &a = spanAccel.fan->tris[i];
            const RigExecFanTri &b = vectorAccel.fan->tris[i];
            CHECK(a.a == b.a && a.b == b.b && a.c == b.c);
        }
        CHECK(!spanAccel.bvh && !vectorAccel.bvh);
        CHECK(spanAccel.counts == vectorAccel.counts &&
              spanAccel.indices == vectorAccel.indices &&
              spanAccel.points == vectorAccel.points);
        std::vector<GfVec3f> span = dest, vector = dest;
        RigExecApplySurfaceProject(
            &span, quad.data(), quad.size(), counts.data(), counts.size(),
            indices.data(), indices.size(), 1.0, &spanAccel);
        RigExecApplySurfaceProject(
            &vector, quad, counts, indices, 1.0, &vectorAccel);
        CHECK(SameVec3fArrayBits(span, vector));
        CHECK(span != dest);
        // The unretained path matches too.
        std::vector<GfVec3f> spanDirect = dest, vectorDirect = dest;
        RigExecApplySurfaceProject(
            &spanDirect, quad.data(), quad.size(), counts.data(),
            counts.size(), indices.data(), indices.size(), 1.0);
        RigExecApplySurfaceProject(
            &vectorDirect, quad, counts, indices, 1.0);
        CHECK(SameVec3fArrayBits(spanDirect, vectorDirect));
        CHECK(SameVec3fArrayBits(spanDirect, span));
        // Truncated: no point modified on either path.
        const std::vector<int> shortIndices(indices.begin(),
                                             indices.begin() + 2);
        std::vector<GfVec3f> spanTrunc = dest, vectorTrunc = dest;
        RigExecApplySurfaceProject(
            &spanTrunc, quad.data(), quad.size(), counts.data(),
            counts.size(), shortIndices.data(), shortIndices.size(),
            1.0);
        RigExecApplySurfaceProject(
            &vectorTrunc, quad, counts, shortIndices, 1.0);
        CHECK(SameVec3fArrayBits(spanTrunc, dest));
        CHECK(SameVec3fArrayBits(vectorTrunc, dest));
        // Truncated builds match too: same verdict, no BVH, same keys.
        RigExecSurfaceAccel spanTruncAccel, vectorTruncAccel;
        RigExecBuildSurfaceAccel(
            counts.data(), counts.size(), shortIndices.data(),
            shortIndices.size(), quad.data(), quad.size(),
            &spanTruncAccel);
        RigExecBuildSurfaceAccel(
            counts, shortIndices, quad, &vectorTruncAccel);
        CHECK(spanTruncAccel.fan && vectorTruncAccel.fan);
        CHECK(spanTruncAccel.fan->truncated);
        CHECK(vectorTruncAccel.fan->truncated);
        CHECK(!spanTruncAccel.bvh && !vectorTruncAccel.bvh);
        CHECK(spanTruncAccel.counts == vectorTruncAccel.counts &&
              spanTruncAccel.indices == vectorTruncAccel.indices &&
              spanTruncAccel.points == vectorTruncAccel.points);
        // A null accel is a no-op (span-only verdict).
        RigExecBuildSurfaceAccel(
            counts.data(), counts.size(), indices.data(),
            indices.size(), quad.data(), quad.size(), nullptr);
        // Above the BVH gate the span path projects through the BVH
        // bit-identically to the vector path.
        std::vector<GfVec3f> bigSurface;
        std::vector<int> bigCounts, bigIndices;
        for (int y = 0; y < 9; ++y) {
            for (int x = 0; x < 9; ++x) {
                bigSurface.emplace_back(
                    float(x), float(y), 0.1f * (x + y));
            }
        }
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                const int a = y * 9 + x;
                bigCounts.push_back(4);
                bigIndices.insert(
                    bigIndices.end(), {a, a + 1, a + 10, a + 9});
            }
        }
        RigExecSurfaceAccel spanBig, vectorBig;
        RigExecBuildSurfaceAccel(
            bigCounts.data(), bigCounts.size(), bigIndices.data(),
            bigIndices.size(), bigSurface.data(), bigSurface.size(),
            &spanBig);
        RigExecBuildSurfaceAccel(
            bigCounts, bigIndices, bigSurface, &vectorBig);
        CHECK(spanBig.fan && spanBig.bvh);
        CHECK(vectorBig.fan && vectorBig.bvh);
        CHECK(spanBig.counts == vectorBig.counts &&
              spanBig.indices == vectorBig.indices &&
              spanBig.points == vectorBig.points);
        std::vector<GfVec3f> spanProj = dest, vectorProj = dest;
        RigExecApplySurfaceProject(
            &spanProj, bigSurface.data(), bigSurface.size(),
            bigCounts.data(), bigCounts.size(), bigIndices.data(),
            bigIndices.size(), 0.5, &spanBig);
        RigExecApplySurfaceProject(
            &vectorProj, bigSurface, bigCounts, bigIndices, 0.5,
            &vectorBig);
        CHECK(SameVec3fArrayBits(spanProj, vectorProj));
        CHECK(spanProj != dest);
    }
    // RMF sampler + ribbon transport.
    {
        const std::vector<GfVec3f> cvs = {
            {0, 0, 0}, {1, 0, 0}, {2, 1, 0}, {3, 1, 0}, {4, 0, 0}};
        const RigExecCurveFrameSamples spanSamples =
            RigExecSampleCurveRMF(cvs.data(), cvs.size(), 9);
        const RigExecCurveFrameSamples vectorSamples =
            RigExecSampleCurveRMF(cvs, 9);
        CHECK(spanSamples.GetSize() == 9);
        CHECK(SameVec3fArrayBits(
            spanSamples.positions, vectorSamples.positions));
        CHECK(SameVec3fArrayBits(
            spanSamples.tangents, vectorSamples.tangents));
        CHECK(SameVec3fArrayBits(
            spanSamples.normals, vectorSamples.normals));
        CHECK(SameVec3fArrayBits(
            spanSamples.binormals, vectorSamples.binormals));
        CHECK(spanSamples.parameters == vectorSamples.parameters);
        // The short-curve linear rule matches too.
        const std::vector<GfVec3f> two = {cvs[0], cvs[1]};
        const RigExecCurveFrameSamples spanTwo =
            RigExecSampleCurveRMF(two.data(), two.size(), 5);
        const RigExecCurveFrameSamples vectorTwo =
            RigExecSampleCurveRMF(two, 5);
        CHECK(SameVec3fArrayBits(
            spanTwo.positions, vectorTwo.positions));
        // Invalid inputs publish no frames on either path.
        const RigExecCurveFrameSamples spanFew =
            RigExecSampleCurveRMF(cvs.data(), cvs.size(), 1);
        const RigExecCurveFrameSamples vectorFew =
            RigExecSampleCurveRMF(cvs, 1);
        CHECK(spanFew.GetSize() == 0 && vectorFew.GetSize() == 0);
        const std::vector<GfVec3f> one = {cvs[0]};
        const RigExecCurveFrameSamples spanOne =
            RigExecSampleCurveRMF(one.data(), one.size(), 9);
        const RigExecCurveFrameSamples vectorOne =
            RigExecSampleCurveRMF(one, 9);
        CHECK(spanOne.GetSize() == 0 && vectorOne.GetSize() == 0);
        // A null range publishes no frames (span-only verdict).
        const RigExecCurveFrameSamples spanNull =
            RigExecSampleCurveRMF(nullptr, 5, 9);
        CHECK(spanNull.GetSize() == 0);
        // Ribbon transport over shifted frames moves both paths alike.
        RigExecCurveFrameSamples posedSamples = vectorSamples;
        for (GfVec3f &p : posedSamples.positions) {
            p += GfVec3f(0, 0, 1);
        }
        const std::vector<GfVec2f> binds = {
            {0.0f, 0.0f}, {0.5f, 0.0f}, {1.0f, 0.0f}};
        const std::vector<GfVec3f> pts = {
            {0, 1, 0}, {2, 2, 0}, {4, 1, 0}};
        std::vector<GfVec3f> span = pts, vector = pts;
        RigExecApplyRibbonTransport(
            &span, binds.data(), binds.size(), vectorSamples,
            posedSamples);
        RigExecApplyRibbonTransport(
            &vector, binds, vectorSamples, posedSamples);
        CHECK(SameVec3fArrayBits(span, vector));
        CHECK(span != pts);
        // A bind mismatch passes through on both paths.
        const std::vector<GfVec2f> shortBinds(binds.begin(),
                                              binds.begin() + 2);
        std::vector<GfVec3f> spanBad = pts, vectorBad = pts;
        RigExecApplyRibbonTransport(
            &spanBad, shortBinds.data(), shortBinds.size(),
            vectorSamples, posedSamples);
        RigExecApplyRibbonTransport(
            &vectorBad, shortBinds, vectorSamples, posedSamples);
        CHECK(SameVec3fArrayBits(spanBad, pts));
        CHECK(SameVec3fArrayBits(vectorBad, pts));
    }
}

int
main()
{
    TestPointsToMatrixRoundTrip();
    TestReflectionRoundTrip();
    TestSingularReference();
    TestOrthogonalReconstruction();
    TestDegenerateFallbacks();
    TestFkChain();
    TestTwoBoneIk();
    TestBlendFrames();
    TestTwistDistribution();
    TestParamsRoundTrip();
    TestTwistSideScaleInteraction();
    TestAimUpAxisSelection();
    TestIkDegeneracies();
    TestSvdPerturbationStability();
    TestAimConstraintKernel();
    TestFbxPositionConstraintKernel();
    TestFbxRotationConstraintKernel();
    TestFbxScaleConstraintKernel();
    TestFbxParentConstraintKernel();
    TestConstraintShearBlendIsOptIn();
    TestFbxAimConstraintKernel();
    TestFbxConstraintFailures();
    TestConstraintEnvelopeExactEndpoints();
    TestGeometryKernels();
    TestSurfaceOffsets();
    TestSimdParity();
    TestSimdScalarWeightMatchesArray();
    TestLaplacianSmoothBorrowMatchesCopy();
    TestWireRestEvalsMatchDirectEvaluation();
    TestPartialDecompositionMatchesOneShot();
    TestSurfaceProjectorBaseNormalsMemo();
    TestFanTriangulationBuilders();
    TestTriangleCandidatesCannotUseGeometricPruning();
    TestTrianglePathologicalFloatCandidates();
    TestTriangleBvhMatchesBruteForce();
    TestTriangleBvhRoundingTies();
    TestSurfaceProjectEntryPaths();
    TestRaycastSharedTriangulation();
    TestSolverSharedTriangulation();
    TestMeshAdjacencyBuild();
    TestLaplacianSmoothCachedMatchesDirect();
    TestSurfaceProjectCachedAccelMatchesDirect();
    TestSpanOverloadsMatchVectorForms();
    TestWeightedMatrix();
    TestLinearBlendSkin();
    TestPropertyMath();
    TestPropertyMathKernelTypes();
    TestAvarScaleNormalization();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecMath: all tests passed\n");
    return 0;
}
