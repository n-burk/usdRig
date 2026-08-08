//
// RigExec volumetric weight-field conformance tests (spec §4.1 weight
// objects, volumetric extension).
//
// Covers the distance-to-weight remap, the baked falloff profiles, the
// three distance functions, and weight-object composition.
//
#include "rigExecMath/weightFields.h"

#include <cmath>
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
Near(float a, float b, float tol = 1e-5f)
{
    return std::abs(a - b) <= tol;
}

// A linear params block over [0, 1] with no curve: the remap under test
// in isolation from any profile.
static RigExecFalloffParams
LinearBand(float lo, float hi)
{
    RigExecFalloffParams p;
    p.falloffMin = lo;
    p.falloffMax = hi;
    return p;
}

static void
TestFalloffEndpoints()
{
    const RigExecFalloffParams p = LinearBand(0.0f, 1.0f);

    // Fully on at falloffMin, fully off at falloffMax, linear between.
    CHECK(Near(RigExecEvaluateFalloff(0.0f, p), 1.0f));
    CHECK(Near(RigExecEvaluateFalloff(1.0f, p), 0.0f));
    CHECK(Near(RigExecEvaluateFalloff(0.5f, p), 0.5f));
    CHECK(Near(RigExecEvaluateFalloff(0.25f, p), 0.75f));

    // Outside the band the normalized parameter clamps, so the field is
    // total over its whole domain (spec §4.1: a weight object is a TOTAL
    // field, never undefined at an element).
    CHECK(Near(RigExecEvaluateFalloff(-10.0f, p), 1.0f));
    CHECK(Near(RigExecEvaluateFalloff(10.0f, p), 0.0f));
}

static void
TestFalloffBandPlacement()
{
    // A band that does not start at zero.
    const RigExecFalloffParams p = LinearBand(2.0f, 6.0f);
    CHECK(Near(RigExecEvaluateFalloff(2.0f, p), 1.0f));
    CHECK(Near(RigExecEvaluateFalloff(6.0f, p), 0.0f));
    CHECK(Near(RigExecEvaluateFalloff(4.0f, p), 0.5f));
    CHECK(Near(RigExecEvaluateFalloff(0.0f, p), 1.0f));

    // A REVERSED band is legal and simply flips the ramp: no special
    // case, the signed denominator does it.
    const RigExecFalloffParams r = LinearBand(6.0f, 2.0f);
    CHECK(Near(RigExecEvaluateFalloff(6.0f, r), 1.0f));
    CHECK(Near(RigExecEvaluateFalloff(2.0f, r), 0.0f));
    CHECK(Near(RigExecEvaluateFalloff(4.0f, r), 0.5f));

    // A degenerate band is a hard step at the shared distance.
    const RigExecFalloffParams d = LinearBand(3.0f, 3.0f);
    CHECK(Near(RigExecEvaluateFalloff(2.9f, d), 1.0f));
    CHECK(Near(RigExecEvaluateFalloff(3.0f, d), 0.0f));
    CHECK(Near(RigExecEvaluateFalloff(3.1f, d), 0.0f));
}

static void
TestFalloffInvertAndStrength()
{
    RigExecFalloffParams p = LinearBand(0.0f, 1.0f);

    p.invert = 1.0f;
    CHECK(Near(RigExecEvaluateFalloff(0.0f, p), 0.0f));
    CHECK(Near(RigExecEvaluateFalloff(1.0f, p), 1.0f));
    CHECK(Near(RigExecEvaluateFalloff(0.25f, p), 0.25f));

    // invert is a float so it sweeps rather than popping: at 0.5 the two
    // ends meet and the field is flat.
    p.invert = 0.5f;
    CHECK(Near(RigExecEvaluateFalloff(0.0f, p), 0.5f));
    CHECK(Near(RigExecEvaluateFalloff(1.0f, p), 0.5f));
    CHECK(Near(RigExecEvaluateFalloff(0.3f, p), 0.5f));

    // strength scales the result and is deliberately NOT clamped here:
    // rangePolicy owns that decision.
    p.invert = 0.0f;
    p.strength = 2.0f;
    CHECK(Near(RigExecEvaluateFalloff(0.0f, p), 2.0f));
    CHECK(Near(RigExecEvaluateFalloff(0.5f, p), 1.0f));
    p.strength = -1.0f;
    CHECK(Near(RigExecEvaluateFalloff(0.0f, p), -1.0f));
}

static void
TestFalloffProfiles()
{
    // Every profile pins both ends and stays inside [0, 1].
    const RigExecFalloffProfile profiles[] = {
        RigExecFalloffProfile::Linear, RigExecFalloffProfile::Smooth,
        RigExecFalloffProfile::EaseIn, RigExecFalloffProfile::EaseOut};
    for (RigExecFalloffProfile profile : profiles) {
        const std::vector<float> lut = RigExecBuildFalloffLut(profile);
        CHECK(lut.size() == RigExecFalloffLutSize);
        CHECK(Near(lut.front(), 0.0f));
        CHECK(Near(lut.back(), 1.0f));
        for (size_t i = 1; i < lut.size(); ++i) {
            CHECK(lut[i] >= lut[i - 1] - 1e-6f);  // monotone non-decreasing
            CHECK(lut[i] >= -1e-6f && lut[i] <= 1.0f + 1e-6f);
        }
    }

    // Shape checks that distinguish the profiles from each other.
    const std::vector<float> linear =
        RigExecBuildFalloffLut(RigExecFalloffProfile::Linear);
    const std::vector<float> smooth =
        RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
    const std::vector<float> easeIn =
        RigExecBuildFalloffLut(RigExecFalloffProfile::EaseIn);
    const std::vector<float> easeOut =
        RigExecBuildFalloffLut(RigExecFalloffProfile::EaseOut);

    CHECK(Near(RigExecSampleFalloffLut(linear, 0.5f), 0.5f));
    CHECK(Near(RigExecSampleFalloffLut(smooth, 0.5f), 0.5f));
    CHECK(Near(RigExecSampleFalloffLut(easeIn, 0.5f), 0.25f, 1e-3f));
    CHECK(Near(RigExecSampleFalloffLut(easeOut, 0.5f), 0.75f, 1e-3f));
    // Smooth sits below linear in the first half and above it in the
    // second: that IS the S-curve.
    CHECK(RigExecSampleFalloffLut(smooth, 0.25f) <
          RigExecSampleFalloffLut(linear, 0.25f));
    CHECK(RigExecSampleFalloffLut(smooth, 0.75f) >
          RigExecSampleFalloffLut(linear, 0.75f));

    // Constant is the hard step.
    const std::vector<float> constant =
        RigExecBuildFalloffLut(RigExecFalloffProfile::Constant);
    CHECK(Near(RigExecSampleFalloffLut(constant, 0.0f), 0.0f));
    CHECK(Near(RigExecSampleFalloffLut(constant, 0.5f), 1.0f));
    CHECK(Near(RigExecSampleFalloffLut(constant, 1.0f), 1.0f));

    // A degenerate table is the identity, so an unauthored curve costs
    // nothing and means "linear".
    CHECK(Near(RigExecSampleFalloffLut({}, 0.3f), 0.3f));
    CHECK(Near(RigExecSampleFalloffLut({0.7f}, 0.3f), 0.3f));

    // Sampling clamps outside [0, 1] rather than extrapolating.
    CHECK(Near(RigExecSampleFalloffLut(linear, -2.0f), 0.0f));
    CHECK(Near(RigExecSampleFalloffLut(linear, 5.0f), 1.0f));

    // An explicitly authored non-monotone curve survives intact: the
    // remap must not assume a ramp.
    const std::vector<float> hump = {0.0f, 1.0f, 0.0f};
    CHECK(Near(RigExecSampleFalloffLut(hump, 0.5f), 1.0f));
    CHECK(Near(RigExecSampleFalloffLut(hump, 0.25f), 0.5f));
    CHECK(Near(RigExecSampleFalloffLut(hump, 0.75f), 0.5f));
}

static void
TestFalloffWithCurve()
{
    RigExecFalloffParams p = LinearBand(0.0f, 4.0f);
    p.curve = RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);

    // Endpoints are pinned by the profile, so the curve cannot move them.
    CHECK(Near(RigExecEvaluateFalloff(0.0f, p), 1.0f));
    CHECK(Near(RigExecEvaluateFalloff(4.0f, p), 0.0f));
    // Midpoint of a symmetric S-curve is still the midpoint.
    CHECK(Near(RigExecEvaluateFalloff(2.0f, p), 0.5f, 1e-3f));
    // A quarter in, smoothstep holds the field higher than linear would.
    CHECK(RigExecEvaluateFalloff(1.0f, p) > 0.75f);
}

static void
TestDistanceFunctions()
{
    CHECK(Near(RigExecSphereDistance(GfVec3f(0, 0, 0)), 0.0f));
    CHECK(Near(RigExecSphereDistance(GfVec3f(3, 4, 0)), 5.0f));

    // Plane distance is SIGNED, and picks its axis.
    CHECK(Near(RigExecPlaneDistance(GfVec3f(1, 2, 3), 0), 1.0f));
    CHECK(Near(RigExecPlaneDistance(GfVec3f(1, 2, 3), 1), 2.0f));
    CHECK(Near(RigExecPlaneDistance(GfVec3f(1, 2, 3), 2), 3.0f));
    CHECK(Near(RigExecPlaneDistance(GfVec3f(0, -7, 0), 1), -7.0f));
    CHECK(Near(RigExecPlaneDistance(GfVec3f(1, 2, 3), 9), 0.0f));

    // Segment distance: perpendicular foot inside, clamped past the ends.
    const GfVec3f a(0, 0, 0), b(10, 0, 0);
    CHECK(Near(RigExecSegmentDistance(GfVec3f(5, 3, 0), a, b), 3.0f));
    CHECK(Near(RigExecSegmentDistance(GfVec3f(-4, 3, 0), a, b), 5.0f));
    CHECK(Near(RigExecSegmentDistance(GfVec3f(14, 3, 0), a, b), 5.0f));
    CHECK(Near(RigExecSegmentDistance(GfVec3f(0, 2, 0), a, a), 2.0f));

    // Polyline distance takes the nearest segment; an L-bend is the
    // case a per-segment minimum has to get right.
    const GfVec3f bend[3] = {GfVec3f(0, 0, 0), GfVec3f(10, 0, 0),
                             GfVec3f(10, 10, 0)};
    CHECK(Near(RigExecCurveDistance(GfVec3f(5, 2, 0), bend, 3), 2.0f));
    CHECK(Near(RigExecCurveDistance(GfVec3f(12, 5, 0), bend, 3), 2.0f));
    CHECK(Near(RigExecCurveDistance(GfVec3f(10, 0, 0), bend, 3), 0.0f));
    // One point degenerates to a sphere about it; none is an empty field.
    CHECK(Near(RigExecCurveDistance(GfVec3f(0, 3, 0), bend, 1), 3.0f));
    CHECK(std::isinf(RigExecCurveDistance(GfVec3f(0, 0, 0), bend, 0)));
}

static void
TestSphereField()
{
    const std::vector<GfVec3f> points = {
        GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0), GfVec3f(1, 0, 0),
        GfVec3f(0, 2, 0)};
    RigExecFalloffParams p = LinearBand(0.0f, 1.0f);

    std::vector<float> w;
    RigExecSphereWeightField(points, GfMatrix4d(1.0), p, &w);
    CHECK(w.size() == points.size());
    CHECK(Near(w[0], 1.0f));
    CHECK(Near(w[1], 0.5f));
    CHECK(Near(w[2], 0.0f));
    CHECK(Near(w[3], 0.0f));

    // A non-uniform world-to-local makes the iso-surfaces ellipsoidal,
    // which is what lets the volume's own transform carry the shape
    // instead of a per-axis radius authoring surface.
    GfMatrix4d squash(1.0);
    squash.SetScale(GfVec3d(0.5, 1, 1));  // local x = world x / 2
    const std::vector<GfVec3f> probes = {
        GfVec3f(2, 0, 0), GfVec3f(0, 1, 0), GfVec3f(1, 0, 0)};
    RigExecSphereWeightField(probes, squash, p, &w);
    CHECK(Near(w[0], 0.0f));  // 2 along x reaches the boundary
    CHECK(Near(w[1], 0.0f));  // 1 along y also reaches it
    CHECK(Near(w[2], 0.5f));  // 1 along x is only halfway
}

static void
TestPlaneField()
{
    // A band straddling zero authors a GRADIENT ACROSS the plane, which
    // is the whole point of the signed distance.
    const std::vector<GfVec3f> points = {
        GfVec3f(0, -1, 0), GfVec3f(0, 0, 0), GfVec3f(0, 1, 0),
        GfVec3f(5, -3, 9)};
    const RigExecFalloffParams p = LinearBand(-1.0f, 1.0f);

    std::vector<float> w;
    RigExecPlaneWeightField(points, GfMatrix4d(1.0), 1, p, &w);
    CHECK(Near(w[0], 1.0f));
    CHECK(Near(w[1], 0.5f));
    CHECK(Near(w[2], 0.0f));
    // The off-axis components are ignored entirely: a plane is infinite.
    CHECK(Near(w[3], 1.0f));

    // Axis selection actually selects.
    RigExecPlaneWeightField(points, GfMatrix4d(1.0), 0, p, &w);
    CHECK(Near(w[0], 0.5f));
    CHECK(Near(w[3], 0.0f));
}

// A BOUNDED plane is a patch, not a half-space: the field is zero outside
// the in-plane rectangle and completely unchanged inside it.
static void
TestPlaneBounds()
{
    // Measured along y, so the in-plane axes are U = z and V = x.
    // The first three points sit on the axis; the last is far out in x.
    const std::vector<GfVec3f> points = {
        GfVec3f(0, -1, 0), GfVec3f(0, 0, 0), GfVec3f(0, 1, 0),
        GfVec3f(5, -3, 9)};
    const RigExecFalloffParams p = LinearBand(-1.0f, 1.0f);

    RigExecPlaneBounds bounds;
    bounds.extentU = 2.0f;  // z
    bounds.extentV = 2.0f;  // x

    std::vector<float> w;
    RigExecPlaneWeightField(points, GfMatrix4d(1.0), 1, p, &w, &bounds);
    // Inside the rectangle nothing moved: the bound CLIPS the field, it
    // does not reshape it, so no iso-surface can shift when bounds go on.
    CHECK(Near(w[0], 1.0f));
    CHECK(Near(w[1], 0.5f));
    CHECK(Near(w[2], 0.0f));
    // Outside it, zero -- not the 1.0 the infinite plane gave this point.
    CHECK(Near(w[3], 0.0f));

    // Containment is per axis, and it is HARD: exactly on the edge is in,
    // a hair past it is out, with no ramp between.
    const std::vector<GfVec3f> edge = {
        GfVec3f(2, -1, 0), GfVec3f(2.001f, -1, 0),
        GfVec3f(0, -1, 2), GfVec3f(0, -1, 2.001f)};
    RigExecPlaneWeightField(edge, GfMatrix4d(1.0), 1, p, &w, &bounds);
    CHECK(Near(w[0], 1.0f));
    CHECK(Near(w[1], 0.0f));
    CHECK(Near(w[2], 1.0f));
    CHECK(Near(w[3], 0.0f));

    // A rectangle, not a square: U and V are independent half-extents.
    bounds.extentU = 10.0f;  // z: reaches the far point's z = 9
    bounds.extentV = 1.0f;   // x: does not reach its x = 5
    RigExecPlaneWeightField(points, GfMatrix4d(1.0), 1, p, &w, &bounds);
    CHECK(Near(w[3], 0.0f));
    bounds.extentV = 6.0f;
    RigExecPlaneWeightField(points, GfMatrix4d(1.0), 1, p, &w, &bounds);
    CHECK(Near(w[3], 1.0f));

    // Null bounds is the infinite plane, byte for byte with the call that
    // omits the argument entirely.
    std::vector<float> unbounded;
    RigExecPlaneWeightField(points, GfMatrix4d(1.0), 1, p, &unbounded,
                            nullptr);
    CHECK(Near(unbounded[3], 1.0f));

    // The predicate itself, including the axis rotation of the U/V pair:
    // measured along x the in-plane axes are y and z.
    RigExecPlaneBounds tight;
    tight.extentU = 1.0f;
    tight.extentV = 3.0f;
    CHECK(RigExecPlaneWithinBounds(GfVec3f(99, 1, 3), 0, tight));
    CHECK(!RigExecPlaneWithinBounds(GfVec3f(99, 1.5f, 3), 0, tight));
    CHECK(!RigExecPlaneWithinBounds(GfVec3f(99, 1, 3.5f), 0, tight));
    // An out-of-range axis contains nothing, matching the 0.0 distance
    // RigExecPlaneDistance returns for it: neither invents a field.
    CHECK(!RigExecPlaneWithinBounds(GfVec3f(0, 0, 0), 7, tight));
}

static void
TestCurveField()
{
    const std::vector<GfVec3f> curve = {GfVec3f(0, 0, 0), GfVec3f(10, 0, 0)};
    const std::vector<GfVec3f> points = {
        GfVec3f(5, 0, 0), GfVec3f(5, 1, 0), GfVec3f(5, 2, 0),
        GfVec3f(-5, 0, 0)};
    const RigExecFalloffParams p = LinearBand(0.0f, 2.0f);

    std::vector<float> w;
    RigExecCurveWeightField(points, curve, GfMatrix4d(1.0), p, &w);
    CHECK(w.size() == points.size());
    CHECK(Near(w[0], 1.0f));  // on the curve
    CHECK(Near(w[1], 0.5f));
    CHECK(Near(w[2], 0.0f));
    CHECK(Near(w[3], 0.0f));  // 5 past the end, clamped to the endpoint

    // Losing the curve target empties the field rather than filling it:
    // an author who breaks the relationship sees the influence vanish.
    RigExecCurveWeightField(points, {}, GfMatrix4d(1.0), p, &w);
    CHECK(w.size() == points.size());
    for (float x : w) {
        CHECK(Near(x, 0.0f));
    }
}

static void
TestCombine()
{
    const std::vector<float> a = {1.0f, 0.5f, 0.0f};
    const std::vector<float> b = {0.5f, 0.5f, 1.0f};
    std::vector<float> out;

    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Multiply, {a, b}, 3, &out));
    CHECK(Near(out[0], 0.5f) && Near(out[1], 0.25f) && Near(out[2], 0.0f));

    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Add, {a, b}, 3, &out));
    CHECK(Near(out[0], 1.5f) && Near(out[1], 1.0f) && Near(out[2], 1.0f));

    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Max, {a, b}, 3, &out));
    CHECK(Near(out[0], 1.0f) && Near(out[1], 0.5f) && Near(out[2], 1.0f));

    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Min, {a, b}, 3, &out));
    CHECK(Near(out[0], 0.5f) && Near(out[1], 0.5f) && Near(out[2], 0.0f));

    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Average, {a, b}, 3, &out));
    CHECK(Near(out[0], 0.75f) && Near(out[1], 0.5f) && Near(out[2], 0.5f));

    // Subtract seeds from the FIRST input -- "a minus the rest" -- and is
    // therefore order dependent, as documented.
    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Subtract, {a, b}, 3, &out));
    CHECK(Near(out[0], 0.5f) && Near(out[1], 0.0f) && Near(out[2], -1.0f));
    std::vector<float> swapped;
    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Subtract, {b, a}, 3, &swapped));
    CHECK(!Near(swapped[2], out[2]));

    // The commutative modes really are permutation proof.
    for (RigExecWeightCombine mode :
         {RigExecWeightCombine::Multiply, RigExecWeightCombine::Add,
          RigExecWeightCombine::Max, RigExecWeightCombine::Min,
          RigExecWeightCombine::Average}) {
        std::vector<float> ab, ba;
        CHECK(RigExecCombineWeightFields(mode, {a, b}, 3, &ab));
        CHECK(RigExecCombineWeightFields(mode, {b, a}, 3, &ba));
        for (size_t i = 0; i < 3; ++i) {
            CHECK(Near(ab[i], ba[i]));
        }
    }

    // Overlay seeds from the first input too.
    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Overlay, {a, b}, 3, &out));
    CHECK(Near(out[0], 1.0f));   // a >= 0.5: 1 - 2(0)(0.5) = 1
    CHECK(Near(out[1], 0.5f));   // a >= 0.5: 1 - 2(0.5)(0.5) = 0.5
    CHECK(Near(out[2], 0.0f));   // a <  0.5: 2(0)(1) = 0

    // No inputs is the mode's identity, not an empty field: a combine
    // whose targets all went missing must still publish a total field.
    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Multiply, {}, 3, &out));
    CHECK(out.size() == 3 && Near(out[0], 1.0f));
    CHECK(RigExecCombineWeightFields(RigExecWeightCombine::Add, {}, 3, &out));
    CHECK(out.size() == 3 && Near(out[0], 0.0f));

    // A single input passes through unchanged under every mode that
    // seeds from an identity.
    CHECK(RigExecCombineWeightFields(
        RigExecWeightCombine::Multiply, {a}, 3, &out));
    CHECK(Near(out[0], a[0]) && Near(out[1], a[1]) && Near(out[2], a[2]));

    // A length mismatch is a structural error, NOT a truncated fold: a
    // short input would silently read as identity over its missing tail.
    const std::vector<float> shortField = {1.0f, 0.5f};
    CHECK(!RigExecCombineWeightFields(
        RigExecWeightCombine::Multiply, {a, shortField}, 3, &out));
    CHECK(out.empty());
}

int
main()
{
    TestFalloffEndpoints();
    TestFalloffBandPlacement();
    TestFalloffInvertAndStrength();
    TestFalloffProfiles();
    TestFalloffWithCurve();
    TestDistanceFunctions();
    TestSphereField();
    TestPlaneField();
    TestPlaneBounds();
    TestCurveField();
    TestCombine();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecWeightFields: all tests passed\n");
    return 0;
}
