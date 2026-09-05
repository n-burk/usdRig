//
// RigExec math conformance tests (spec §5, §14.3 exit criteria):
// Points -> Matrix -> Points round trips including reflection and shear;
// reconstruction policies; IK reach/stretch; blend endpoints; twist
// distribution; weighted matrix movement.
//
#include "rigExecMath/avarScale.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/rotation.h"

#include <cmath>
#include <cstring>
#include <cstdio>
#include <limits>

using namespace rigExec;

static int failures = 0;

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
    args.twist = M_PI / 2;
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
    args.twist = M_PI / 2;

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
    const double radians = 175.0 * M_PI / 180.0;
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
    const double radians = 12.5 * M_PI / 180.0;
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
    TestFbxAimConstraintKernel();
    TestFbxConstraintFailures();
    TestConstraintEnvelopeExactEndpoints();
    TestGeometryKernels();
    TestSurfaceOffsets();
    TestSimdParity();
    TestWeightedMatrix();
    TestPropertyMath();
    TestAvarScaleNormalization();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecMath: all tests passed\n");
    return 0;
}
