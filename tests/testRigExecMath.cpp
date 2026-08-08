//
// RigExec math conformance tests (spec §5, §14.3 exit criteria):
// Points -> Matrix -> Points round trips including reflection and shear;
// reconstruction policies; IK reach/stretch; blend endpoints; twist
// distribution; weighted matrix movement.
//
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/rotation.h"

#include <cmath>
#include <cstdio>

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

static const std::array<GfVec3d, 4> kUnitRest = {
    GfVec3d(0, 0, 0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};

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
    TestGeometryKernels();
    TestSimdParity();
    TestWeightedMatrix();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecMath: all tests passed\n");
    return 0;
}
