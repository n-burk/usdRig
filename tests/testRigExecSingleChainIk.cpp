//
// Arbitrary-length FBX-style SingleChainIK math conformance tests.
//
#include "rigExecMath/singleChainIk.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

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
Near(double a, double b, double tolerance = 1e-8)
{
    return std::abs(a - b) <= tolerance;
}

static bool
Near(const GfVec3d &a, const GfVec3d &b, double tolerance = 1e-8)
{
    return (a - b).GetLength() <= tolerance;
}

static GfVec3d
UnitX(const RigExecPointFrame &frame)
{
    const GfVec3d axis = frame.X() - frame.Origin();
    return axis / axis.GetLength();
}

static GfVec3d
UnitY(const RigExecPointFrame &frame)
{
    const GfVec3d axis = frame.Y() - frame.Origin();
    return axis / axis.GetLength();
}

static RigExecPointFrame
MakeFrame(
    const GfVec3d &origin,
    const GfVec3d &xDirection = GfVec3d(1.0, 0.0, 0.0),
    const GfVec3d &upCandidate = GfVec3d(0.0, 1.0, 0.0),
    const GfVec3d &handleLengths = GfVec3d(1.0))
{
    const GfVec3d x = xDirection.GetNormalized();
    GfVec3d up = upCandidate - x * GfDot(x, upCandidate);
    if (up.GetLength() < 1e-12) {
        up = GfVec3d(0.0, 0.0, 1.0) -
             x * GfDot(x, GfVec3d(0.0, 0.0, 1.0));
    }
    up.Normalize();
    const GfVec3d side = GfCross(x, up).GetNormalized();

    RigExecPointFrame frame;
    frame.points = {
        origin,
        origin + x * handleLengths[0],
        origin + up * handleLengths[1],
        origin + side * handleLengths[2],
    };
    return frame;
}

static std::vector<RigExecPointFrame>
MakeChain(const std::vector<GfVec3d> &origins)
{
    std::vector<RigExecPointFrame> frames;
    frames.reserve(origins.size());
    for (size_t i = 0; i < origins.size(); ++i) {
        GfVec3d x(1.0, 0.0, 0.0);
        if (i + 1 < origins.size()) {
            x = origins[i + 1] - origins[i];
        } else if (i > 0) {
            x = origins[i] - origins[i - 1];
        }
        frames.push_back(MakeFrame(origins[i], x));
    }
    return frames;
}

static RigExecPointFrame
Translated(RigExecPointFrame frame, const GfVec3d &translation)
{
    for (GfVec3d &point : frame.points) point += translation;
    return frame;
}

static RigExecPointFrame
Scaled(RigExecPointFrame frame, double scale)
{
    for (GfVec3d &point : frame.points) point *= scale;
    return frame;
}

static bool
AllFinite(const std::vector<RigExecPointFrame> &frames)
{
    for (const RigExecPointFrame &frame : frames) {
        if (!frame.IsValid() || frame.IsDegenerate()) return false;
        for (const GfVec3d &point : frame.points) {
            for (int axis = 0; axis < 3; ++axis) {
                if (!std::isfinite(point[axis])) return false;
            }
        }
    }
    return true;
}

static void
CheckLengths(
    const std::vector<RigExecPointFrame> &before,
    const std::vector<RigExecPointFrame> &after,
    double tolerance = 1e-8)
{
    CHECK(before.size() == after.size());
    if (before.size() != after.size()) return;
    for (size_t i = 0; i + 1 < before.size(); ++i) {
        const double expected =
            (before[i + 1].Origin() - before[i].Origin()).GetLength();
        const double actual =
            (after[i + 1].Origin() - after[i].Origin()).GetLength();
        CHECK(Near(actual, expected, tolerance));
    }
}

static void
TestTwoJointReachableAndEndOrientation()
{
    const std::vector<RigExecPointFrame> chain =
        MakeChain({GfVec3d(0.0), GfVec3d(1.0, 0.0, 0.0)});
    const RigExecPointFrame effector = MakeFrame(
        GfVec3d(0.0, 1.0, 0.0),
        GfVec3d(0.0, 0.0, 1.0), GfVec3d(0.0, 1.0, 0.0));

    RigExecSingleChainIkParams params;
    params.mode = RigExecSingleChainIkMode::SingleChain;
    const auto solved =
        RigExecSolveSingleChainIk(chain, effector, params);
    const auto repeated =
        RigExecSolveSingleChainIk(chain, effector, params);

    CHECK(AllFinite(solved));
    CHECK(solved == repeated);
    CHECK(Near(solved.front().Origin(), chain.front().Origin()));
    CHECK(Near(solved.back().Origin(), effector.Origin()));
    CheckLengths(chain, solved);
    CHECK(Near(UnitX(solved.front()), GfVec3d(0.0, 1.0, 0.0)));
    CHECK(Near(UnitX(solved.back()), UnitX(effector)));
    CHECK(Near(UnitY(solved.back()), UnitY(effector)));

    // At full strength the solved end position and basis are the candidate,
    // not values rotated toward and normalized one more time with weight 1.
    // This simple orthogonal case therefore has an exact expected frame.
    RigExecPointFrame exactEnd;
    exactEnd.points = {
        GfVec3d(0.0, 1.0, 0.0), GfVec3d(0.0, 1.0, 1.0),
        GfVec3d(0.0, 2.0, 0.0), GfVec3d(-1.0, 1.0, 0.0)};
    CHECK(solved.back() == exactEnd);
}

static void
TestLongerReachableRotatePlane()
{
    const std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(3.0, 0.0, 0.0)});
    const RigExecPointFrame effector =
        MakeFrame(GfVec3d(2.0, 0.0, 0.0));

    RigExecSingleChainIkParams params;
    params.mode = RigExecSingleChainIkMode::RotatePlane;
    params.pole = GfVec3d(0.0, 1.0, 0.0);
    const auto solved =
        RigExecSolveSingleChainIk(chain, effector, params);
    const auto repeated =
        RigExecSolveSingleChainIk(chain, effector, params);

    CHECK(AllFinite(solved));
    CHECK(solved == repeated);
    CHECK(Near(solved.back().Origin(), effector.Origin(), 1e-7));
    CheckLengths(chain, solved, 1e-7);
    CHECK(solved[1].Origin()[1] > 1e-5 ||
          solved[2].Origin()[1] > 1e-5);
    for (const RigExecPointFrame &frame : solved) {
        // root->goal and pole span XY, so every solved joint lies in XY.
        CHECK(std::abs(frame.Origin()[2]) < 1e-8);
    }
    for (size_t i = 0; i + 1 < solved.size(); ++i) {
        const GfVec3d downChain =
            (solved[i + 1].Origin() - solved[i].Origin()).GetNormalized();
        CHECK(Near(UnitX(solved[i]), downChain));
    }
}

static void
TestUnreachableGoal()
{
    const std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(3.0, 0.0, 0.0)});
    const RigExecPointFrame effector =
        MakeFrame(GfVec3d(10.0, 0.0, 0.0));

    RigExecSingleChainIkParams params;
    params.mode = RigExecSingleChainIkMode::SingleChain;
    const auto solved =
        RigExecSolveSingleChainIk(chain, effector, params);

    CHECK(AllFinite(solved));
    CheckLengths(chain, solved);
    CHECK(Near(solved.back().Origin(), GfVec3d(3.0, 0.0, 0.0)));

    // A two-segment chain whose longer segment cannot fold close enough to
    // reach the root settles at its minimum reach while keeping both lengths.
    const std::vector<RigExecPointFrame> unequal = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(2.0, 0.0, 0.0),
        GfVec3d(3.0, 0.0, 0.0)});
    const RigExecPointFrame atRoot = MakeFrame(GfVec3d(0.0));
    const auto folded =
        RigExecSolveSingleChainIk(unequal, atRoot, params);
    CHECK(AllFinite(folded));
    CheckLengths(unequal, folded, 1e-7);
    CHECK(Near((folded.back().Origin() - folded.front().Origin()).GetLength(),
               1.0, 1e-7));
}

static void
TestPoleModeDistinction()
{
    const std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(3.0, 0.0, 0.0)});
    const RigExecPointFrame effector =
        MakeFrame(GfVec3d(2.0, 0.0, 0.0));

    RigExecSingleChainIkParams positive;
    positive.mode = RigExecSingleChainIkMode::RotatePlane;
    positive.pole = GfVec3d(0.0, 1.0, 0.0);
    RigExecSingleChainIkParams negative = positive;
    negative.pole = GfVec3d(0.0, -1.0, 0.0);
    const auto rotatePositive =
        RigExecSolveSingleChainIk(chain, effector, positive);
    const auto rotateNegative =
        RigExecSolveSingleChainIk(chain, effector, negative);
    CHECK(rotatePositive[1].Origin()[1] > 1e-5);
    CHECK(rotateNegative[1].Origin()[1] < -1e-5);

    positive.mode = RigExecSingleChainIkMode::SingleChain;
    negative.mode = RigExecSingleChainIkMode::SingleChain;
    const auto singlePositive =
        RigExecSolveSingleChainIk(chain, effector, positive);
    const auto singleNegative =
        RigExecSolveSingleChainIk(chain, effector, negative);
    CHECK(singlePositive == singleNegative);
}

static void
TestModeOrientationSemantics()
{
    const std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(3.0, 0.0, 0.0)});
    const RigExecPointFrame upY = MakeFrame(
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(0.0, 1.0, 0.0));
    const RigExecPointFrame upZ = MakeFrame(
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(0.0, 0.0, 1.0));

    RigExecSingleChainIkParams single;
    single.mode = RigExecSingleChainIkMode::SingleChain;
    single.pole = GfVec3d(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    single.twistDegrees = std::numeric_limits<double>::quiet_NaN();
    const auto singleY = RigExecSolveSingleChainIk(chain, upY, single);
    const auto singleZ = RigExecSolveSingleChainIk(chain, upZ, single);
    CHECK(AllFinite(singleY) && AllFinite(singleZ));
    CHECK(std::abs(singleY[1].Origin()[1]) > 1e-5);
    CHECK(std::abs(singleY[1].Origin()[2]) < 1e-8);
    CHECK(std::abs(singleZ[1].Origin()[2]) > 1e-5);
    CHECK(std::abs(singleZ[1].Origin()[1]) < 1e-8);
    CHECK(!Near(singleY[1].Origin(), singleZ[1].Origin(), 1e-5));
    CHECK(Near(UnitY(singleY.back()), UnitY(upY), 1e-8));
    CHECK(Near(UnitY(singleZ.back()), UnitY(upZ), 1e-8));

    RigExecSingleChainIkParams rotate;
    rotate.mode = RigExecSingleChainIkMode::RotatePlane;
    rotate.pole = GfVec3d(0.0, 1.0, 0.0);
    const auto rotateY = RigExecSolveSingleChainIk(chain, upY, rotate);
    const auto rotateZ = RigExecSolveSingleChainIk(chain, upZ, rotate);
    CHECK(rotateY == rotateZ);
    CHECK(!Near(UnitY(rotateZ.back()), UnitY(upZ), 1e-4));
    RigExecPointFrame malformedOrientation = upY;
    malformedOrientation.points[1][0] =
        std::numeric_limits<double>::quiet_NaN();
    CHECK(RigExecSolveSingleChainIk(
              chain, malformedOrientation, rotate) == rotateY);
}

static void
TestAbsolutePolePointTranslationInvariance()
{
    const std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(3.0, 0.0, 0.0)});
    const RigExecPointFrame effector = MakeFrame(GfVec3d(2.0, 0.0, 0.0));
    RigExecSingleChainIkParams params;
    params.mode = RigExecSingleChainIkMode::RotatePlane;
    params.pole = GfVec3d(0.0, 1.0, 0.0);
    const auto base = RigExecSolveSingleChainIk(chain, effector, params);

    const GfVec3d translation(0.0, 0.0, 10.0);
    std::vector<RigExecPointFrame> shiftedChain;
    for (const RigExecPointFrame &frame : chain) {
        shiftedChain.push_back(Translated(frame, translation));
    }
    params.pole += translation;
    const auto shifted = RigExecSolveSingleChainIk(
        shiftedChain, Translated(effector, translation), params);
    CHECK(AllFinite(base) && AllFinite(shifted));
    for (size_t i = 0; i < base.size(); ++i) {
        CHECK(Near(shifted[i].Origin(), base[i].Origin() + translation, 1e-7));
        CHECK(Near(UnitX(shifted[i]), UnitX(base[i]), 1e-8));
        CHECK(Near(UnitY(shifted[i]), UnitY(base[i]), 1e-8));
    }
}

static void
TestTwistAndScalePreservation()
{
    std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(3.0, 0.0, 0.0)});
    chain[0] = MakeFrame(
        chain[0].Origin(), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(0.0, 1.0, 0.0), GfVec3d(2.0, 3.0, 4.0));
    const RigExecPointFrame effector =
        MakeFrame(GfVec3d(2.0, 0.0, 0.0));

    RigExecSingleChainIkParams untwisted;
    untwisted.mode = RigExecSingleChainIkMode::RotatePlane;
    untwisted.pole = GfVec3d(0.0, 1.0, 0.0);
    RigExecSingleChainIkParams twisted = untwisted;
    twisted.twistDegrees = 90.0;

    const auto zero =
        RigExecSolveSingleChainIk(chain, effector, untwisted);
    const auto ninety =
        RigExecSolveSingleChainIk(chain, effector, twisted);
    CHECK(AllFinite(zero) && AllFinite(ninety));
    CHECK(Near(zero.front().Origin(), ninety.front().Origin()));
    CHECK(Near(zero.back().Origin(), ninety.back().Origin(), 1e-7));
    CHECK(!Near(zero[1].Origin(), ninety[1].Origin(), 1e-5));
    CHECK(std::abs(zero[1].Origin()[1]) > 1e-5);
    CHECK(std::abs(zero[1].Origin()[2]) < 1e-8);
    CHECK(std::abs(ninety[1].Origin()[2]) > 1e-5);
    CHECK(std::abs(ninety[1].Origin()[1]) < 1e-8);
    CheckLengths(chain, zero, 1e-7);
    CheckLengths(chain, ninety, 1e-7);

    CHECK(Near((ninety[0].X() - ninety[0].Origin()).GetLength(), 2.0));
    CHECK(Near((ninety[0].Y() - ninety[0].Origin()).GetLength(), 3.0));
    CHECK(Near((ninety[0].Z() - ninety[0].Origin()).GetLength(), 4.0));
}

static void
TestGlobalWeightEndpoints()
{
    const std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(3.0, 0.0, 0.0)});
    const RigExecPointFrame effector = MakeFrame(
        GfVec3d(1.5, 1.0, 0.0), GfVec3d(0.0, 0.0, 1.0));

    RigExecSingleChainIkParams params;
    params.mode = RigExecSingleChainIkMode::RotatePlane;
    params.pole = GfVec3d(0.0, 1.0, 0.0);
    params.weight = 0.0;
    CHECK(RigExecSolveSingleChainIk(chain, effector, params) == chain);
    std::vector<RigExecPointFrame> signedZeroChain = chain;
    signedZeroChain[1].points[2][2] = -0.0;
    const auto dormant =
        RigExecSolveSingleChainIk(signedZeroChain, effector, params);
    CHECK(dormant.size() == signedZeroChain.size());
    CHECK(dormant.size() > 1 && std::signbit(dormant[1].points[2][2]));

    params.weight = 1.0;
    const auto full = RigExecSolveSingleChainIk(chain, effector, params);
    CHECK(Near(full.back().Origin(), effector.Origin(), 1e-7));
    // RotatePlane consumes only the effector position. Its end orientation is
    // transported with the terminal segment, not copied from the effector.
    CHECK(!Near(UnitX(full.back()), UnitX(effector), 1e-4));
    CheckLengths(chain, full, 1e-7);

    // Partial weights preserve the chain too (no world-space position lerp).
    params.weight = 0.5;
    const auto half = RigExecSolveSingleChainIk(chain, effector, params);
    CHECK(AllFinite(half));
    CheckLengths(chain, half, 1e-7);

    // Finite out-of-range weights normalize to the endpoints.
    params.weight = -5.0;
    CHECK(RigExecSolveSingleChainIk(chain, effector, params) == chain);
    params.weight = 5.0;
    const auto over = RigExecSolveSingleChainIk(chain, effector, params);
    CHECK(Near(over.back().Origin(), full.back().Origin(), 1e-7));
}

static void
TestScaleInvariance()
{
    const std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(3.0, 0.0, 0.0)});
    const RigExecPointFrame effector = MakeFrame(GfVec3d(1.5, 1.0, 0.0));
    RigExecSingleChainIkParams params;
    params.mode = RigExecSingleChainIkMode::RotatePlane;
    params.pole = GfVec3d(0.0, 2.0, 0.0);
    const auto reference = RigExecSolveSingleChainIk(chain, effector, params);
    CHECK(AllFinite(reference));

    for (const double scale : {1e-12, 1e12}) {
        std::vector<RigExecPointFrame> scaledChain;
        for (const RigExecPointFrame &frame : chain) {
            scaledChain.push_back(Scaled(frame, scale));
        }
        RigExecSingleChainIkParams scaledParams = params;
        scaledParams.pole *= scale;
        const auto solved = RigExecSolveSingleChainIk(
            scaledChain, Scaled(effector, scale), scaledParams);
        CHECK(AllFinite(solved));
        for (size_t i = 0; i < reference.size(); ++i) {
            CHECK(Near(
                solved[i].Origin() / scale, reference[i].Origin(), 1e-6));
            CHECK(Near(UnitX(solved[i]), UnitX(reference[i]), 1e-7));
            CHECK(Near(UnitY(solved[i]), UnitY(reference[i]), 1e-7));
        }
    }
}

static void
TestWeightContinuityAndDormantInputs()
{
    std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(1.0, 0.0, 0.0),
        GfVec3d(2.0, 0.0, 0.0), GfVec3d(3.0, 0.0, 0.0)});
    // The local X axis deliberately does not point at the child. An
    // infinitesimal constraint weight must not snap it to the segment.
    chain[0] = MakeFrame(
        chain[0].Origin(), GfVec3d(0.0, 1.0, 0.0),
        GfVec3d(0.0, 0.0, 1.0));
    const RigExecPointFrame effector = MakeFrame(GfVec3d(1.5, 1.0, 0.0));
    RigExecSingleChainIkParams params;
    params.mode = RigExecSingleChainIkMode::RotatePlane;
    params.pole = GfVec3d(0.0, 2.0, 0.0);
    params.weight = 1e-9;
    const auto tiny = RigExecSolveSingleChainIk(chain, effector, params);
    CHECK(AllFinite(tiny));
    CHECK(Near(UnitX(tiny[0]), UnitX(chain[0]), 1e-7));
    for (size_t i = 0; i < chain.size(); ++i) {
        CHECK(Near(tiny[i].Origin(), chain[i].Origin(), 1e-7));
    }

    // Zero weight is an exact early pass-through even when every dormant
    // external input is malformed.
    RigExecPointFrame badEffector = effector;
    badEffector.points[0][0] = std::numeric_limits<double>::quiet_NaN();
    params.weight = 0.0;
    params.pole = GfVec3d(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    params.twistDegrees = std::numeric_limits<double>::quiet_NaN();
    CHECK(RigExecSolveSingleChainIk(chain, badEffector, params) == chain);
}

static void
TestArbitraryLengthSingleChain()
{
    const std::vector<RigExecPointFrame> chain = MakeChain({
        GfVec3d(0.0, 0.0, 0.0), GfVec3d(0.9, 0.2, 0.1),
        GfVec3d(1.8, 0.5, -0.1), GfVec3d(2.6, 0.2, 0.4),
        GfVec3d(3.5, 0.4, 0.2), GfVec3d(4.3, 0.1, 0.5)});
    const RigExecPointFrame effector =
        MakeFrame(GfVec3d(3.0, 1.0, 0.8));
    RigExecSingleChainIkParams params;
    params.mode = RigExecSingleChainIkMode::SingleChain;
    params.pole = GfVec3d(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    params.twistDegrees = std::numeric_limits<double>::quiet_NaN();

    // SingleChain ignores pole and twist completely, including unusable
    // values.
    const auto solved =
        RigExecSolveSingleChainIk(chain, effector, params);
    CHECK(AllFinite(solved));
    CHECK(Near(solved.back().Origin(), effector.Origin(), 1e-7));
    CheckLengths(chain, solved, 1e-7);
}

static void
TestDegenerateInputs()
{
    const RigExecPointFrame frame = MakeFrame(GfVec3d(0.0));
    const RigExecPointFrame effector = MakeFrame(GfVec3d(1.0, 0.0, 0.0));
    RigExecSingleChainIkParams params;

    const auto tooShort = RigExecSolveSingleChainIk({frame}, effector, params);
    CHECK(tooShort.size() == 1 && tooShort[0].IsDegenerate());

    std::vector<RigExecPointFrame> collapsed = {frame, frame};
    const auto zeroSegment =
        RigExecSolveSingleChainIk(collapsed, effector, params);
    CHECK(zeroSegment.size() == 2);
    CHECK(zeroSegment[0].IsDegenerate() && zeroSegment[1].IsDegenerate());

    const std::vector<RigExecPointFrame> chain =
        MakeChain({GfVec3d(0.0), GfVec3d(1.0, 0.0, 0.0)});
    params.weight = std::numeric_limits<double>::quiet_NaN();
    const auto badWeight =
        RigExecSolveSingleChainIk(chain, effector, params);
    CHECK(badWeight[0].IsDegenerate() && badWeight[1].IsDegenerate());

    params.weight = 1.0;
    params.mode = RigExecSingleChainIkMode::RotatePlane;
    params.pole = GfVec3d(
        0.0, std::numeric_limits<double>::infinity(), 0.0);
    const auto badPole =
        RigExecSolveSingleChainIk(chain, effector, params);
    CHECK(badPole[0].IsDegenerate() && badPole[1].IsDegenerate());

    RigExecPointFrame badEffector = effector;
    badEffector.points[0][0] =
        std::numeric_limits<double>::quiet_NaN();
    params.pole = GfVec3d(0.0, 1.0, 0.0);
    const auto badGoal =
        RigExecSolveSingleChainIk(chain, badEffector, params);
    CHECK(badGoal[0].IsDegenerate() && badGoal[1].IsDegenerate());
}

int
main()
{
    TestTwoJointReachableAndEndOrientation();
    TestLongerReachableRotatePlane();
    TestUnreachableGoal();
    TestPoleModeDistinction();
    TestModeOrientationSemantics();
    TestAbsolutePolePointTranslationInvariance();
    TestTwistAndScalePreservation();
    TestGlobalWeightEndpoints();
    TestScaleInvariance();
    TestWeightContinuityAndDormantInputs();
    TestArbitraryLengthSingleChain();
    TestDegenerateInputs();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecSingleChainIk: all tests passed\n");
    return 0;
}
