// RigExec end-to-end tests for volumetric weight objects (spec §4.1
// volumetric extension): sphere, plane, and curve fields driving real
// matrix movers, weight-object composition, the authored falloff spline,
// and the base/preceding read phases of the sampled points.
// Every case also asserts pose.moverGraphParityMismatches == 0, which is
// the assertion that actually matters: it means the OpenExec
// computeWeightPacket kernels and the CPU oracle independently computed
// the same field. A volumetric weight that only worked on one of those
// paths would still move points, just not the same points.
// argv[1] = path to the examples directory; the codeless schema plugin is
// expected at <examples>/../plugin/rigExecSchema/resources.
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "rigExec/weightPackets.h"
#include "rigExec/frameExtraction.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecPoseCompare.h"
#include <limits>

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/errorMark.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/ts/knot.h"
#include "pxr/base/ts/spline.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;

static int failures = 0;

// Whether this run asks for the BAKED program instead of the CPU oracle.
// The suite's default is cpuParityMode, which is what it was written for:
// every expectation below is checked against an independently written CPU
// resolver as well as against exec. That mode deliberately turns the baked
// program OFF -- the oracle and the program are alternatives, not peers, and
// the program shares exec's kernels -- so the whole of this suite's coverage
// (every shape, both plane bounds, invert, strength, sampleSource, every
// combine mode, both sample phases) was unreachable from the program.
// So the same binary is registered a second time with this set, and with
// RIGEXEC_EVALUATION_MODE=parity and RIGEXEC_BAKE_REQUIRED=1 beside it: the
// assertions are the same, the rig must bake, and the two paths are compared
// exactly in every generation. Nothing is given up -- the default
// registration still runs the oracle over all of it.
static bool
BakedPathRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_TEST_BAKED_PATH", false);
    return requested;
}

// The vacuity guard, in whichever mode is running.
// "The parity harness ran" is what says a case tested a rig that actually
// built a revision rather than passing because nothing happened. With the
// oracle off there are no agreements to count, and what says the same thing
// is the program having been used at all.
static void
CheckTheHarnessRan(const char *label, const RigExecRigPose &pose)
{
    if (pose.moverGraphParityMismatches != 0) {
        std::printf("%s: %zu graph/CPU parity MISMATCHES\n", label,
                    size_t(pose.moverGraphParityMismatches));
        ++failures;
    }
    if (pose.bakedParityMismatches != 0) {
        std::printf("%s: %zu baked parity mismatch(es)\n", label,
                    size_t(pose.bakedParityMismatches));
        ++failures;
    }
    if (!BakedPathRequested() && pose.moverGraphParityAgreements == 0) {
        std::printf("%s: parity harness never ran\n", label);
        ++failures;
    }
}

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

static bool
Near(const GfVec3f &a, const GfVec3f &b, float tol = 1e-4f)
{
    return (a - b).GetLength() <= tol;
}

namespace {

// The fixture every case shares: a four-point mesh, a rig, and a joint
// posed as a pure +2Y translation, so a resolved weight w shows up
// directly as a +2w Y displacement and the expected values stay readable.
struct Fixture {
    UsdStageRefPtr stage;
    VtVec3fArray base;
    UsdPrim joint;

    explicit Fixture(const VtVec3fArray &points, const GfVec3d &translate)
    {
        stage = UsdStage::CreateInMemory();
        base = points;
        UsdPrim mesh =
            stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Points"));
        mesh.CreateAttribute(TfToken("points"),
                             SdfValueTypeNames->Point3fArray).Set(base);
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        joint = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/J"),
                                  TfToken("RigExecJoint"));
        PlaceAt(joint, translate);
    }

    static SdfPath Target() { return SdfPath("/Asset/Geom/M.points"); }

    // Puts \p prim at \p at, through the TRANSLATE AVARS.
    // An authored posed:space would say the same thing in one attribute --
    // it is what this fixture used to do -- and it is the one placement the
    // baked program declines, because a posed:space is whatever an arbitrary
    // computation says from the middle of the pose walk and there is no
    // epoch-constant summary of that. Over an identity rest with no default
    // and no posed parent, a translation composed from avars is the same
    // frame to the bit; saying it this way is what lets this suite's
    // coverage reach both evaluation paths (see BakedPathRequested).
    static void PlaceAt(const UsdPrim &prim, const GfVec3d &at)
    {
        static const char *const names[3] = {"avars:tx", "avars:ty",
                                             "avars:tz"};
        for (int k = 0; k < 3; ++k) {
            prim.CreateAttribute(TfToken(names[k]),
                                 SdfValueTypeNames->Double).Set(at[k]);
        }
    }

    /// Undoes PlaceAt, for the case that places by rest instead.
    static void ClearPlacement(const UsdPrim &prim)
    {
        for (const char *name : {"avars:tx", "avars:ty", "avars:tz"}) {
            if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
                a.Clear();
            }
        }
    }

    // Places a volume weight the same way the joint above is placed.
    UsdPrim MakeVolume(
        const char *name, const TfToken &type, const GfVec3d &at,
        float falloffMin, float falloffMax)
    {
        UsdPrim v = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name), type);
        PlaceAt(v, at);
        v.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({Target()});
        v.CreateAttribute(TfToken("inputs:falloffMin"),
                          SdfValueTypeNames->Float).Set(falloffMin);
        v.CreateAttribute(TfToken("inputs:falloffMax"),
                          SdfValueTypeNames->Float).Set(falloffMax);
        // Linear unless a case says otherwise: it keeps the expected
        // values arithmetic rather than smoothstep evaluations.
        v.CreateAttribute(TfToken("rigExec:falloffProfile"),
                          SdfValueTypeNames->Token).Set(TfToken("linear"));
        return v;
    }

    UsdPrim MakeStaticWeight(const char *name, const VtFloatArray &values)
    {
        UsdPrim w = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name),
            TfToken("RigExecStaticWeight"));
        w.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({Target()});
        w.CreateAttribute(TfToken("rigExec:representation"),
                          SdfValueTypeNames->Token).Set(TfToken("dense"));
        w.CreateAttribute(TfToken("rigExec:values"),
                          SdfValueTypeNames->FloatArray).Set(values);
        w.CreateAttribute(TfToken("rigExec:defaultWeight"),
                          SdfValueTypeNames->Float).Set(0.0f);
        return w;
    }

    UsdPrim MakeMover(const SdfPath &at, const SdfPath &weightObject)
    {
        UsdPrim m = stage->DefinePrim(at, TfToken("RigExecMatrixMover"));
        m.ApplyAPI(TfToken("RigExecMoverAPI"));
        m.CreateRelationship(TfToken("rigExec:moves")).SetTargets({Target()});
        m.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({joint.GetPath()});
        m.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weightObject});
        return m;
    }

    // Compiles, evaluates, and returns the moved points. Empty on any
    // failure, with the reason printed.
    VtVec3fArray Resolve(const char *label)
    {
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = !BakedPathRequested();
        std::vector<std::string> errors;
        if (!evaluator.Compile(&errors)) {
            for (const std::string &e : errors) {
                std::printf("%s: compile error: %s\n", label, e.c_str());
            }
            ++failures;
            return {};
        }
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        if (!pose.valid) {
            std::printf("%s: pose invalid\n", label);
            ++failures;
            return {};
        }
        for (const std::string &d : pose.diagnostics) {
            std::printf("%s: diagnostic: %s\n", label, d.c_str());
        }
        // The whole point of the exercise: exec and the CPU oracle must
        // have computed the same field -- or, in the baked registration,
        // exec and the program.
        CheckTheHarnessRan(label, pose);
        const auto it = pose.movedProperties.find(Target());
        if (it == pose.movedProperties.end()) {
            std::printf("%s: no moved points\n", label);
            ++failures;
            return {};
        }
        return it->second.Get<VtVec3fArray>();
    }
};

// Expected displacement for a weight w under the shared +2Y joint.
GfVec3f
Moved(const GfVec3f &p, float w, float amount = 2.0f)
{
    return p + GfVec3f(0, w * amount, 0);
}

}  // namespace

// A sphere placed at the origin: the field is the radial distance ramped
// between falloffMin and falloffMax.
static void
TestSphereWeight()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                           GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 2.0f);
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    const VtVec3fArray points = f.Resolve("sphere");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    // distance 0, 0.5, 1, 2 over a [0, 2] band -> 1, 0.75, 0.5, 0
    const float expected[4] = {1.0f, 0.75f, 0.5f, 0.0f};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], expected[i])));
    }
}

// A volume placed by rest:space alone, with NOTHING animated.
// This is the case every other test here avoids, and it is the one that
// matters most: MakeVolume places over an IDENTITY rest, and for that
// configuration the rest->posed delta happens to equal the
// desired placement. Author the placement on rest:space instead -- which
// is how the shipped example places its mid-body volumes -- and the two
// stop being the same thing.
// computeMatrix is documented as "the rest->posed target-local map"
// (computations.cpp), i.e. a DEFORMATION, not a location. An unanimated
// volume has posed == rest, so that map is the identity and a volume
// authored at Y=5 generates its field about the ORIGIN.
static void
TestVolumePlacedByRestSpace()
{
    // Points straddling Y=5, where the volume actually is.
    Fixture f(VtVec3fArray{GfVec3f(0, 5, 0), GfVec3f(0, 6, 0),
                           GfVec3f(0, 7, 0), GfVec3f(0, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 2.0f);
    // Replace the posed placement with a REST placement at Y=5 and leave
    // the volume unposed, so it simply sits there.
    Fixture::ClearPlacement(v);
    GfMatrix4d rest(1.0);
    rest.SetTranslate(GfVec3d(0, 5, 0));
    v.CreateAttribute(TfToken("rest:space"),
                      SdfValueTypeNames->Matrix4d).Set(rest);
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    const VtVec3fArray points = f.Resolve("rest-space-placement");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    // Distances from Y=5: 0, 1, 2, and 5 -> 1, 0.5, 0, 0.
    // If the placement collapsed to the origin the answers invert
    // completely: the point AT the volume gets 0 and the far one gets 1.
    const float expected[4] = {1.0f, 0.5f, 0.0f, 0.0f};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], expected[i])));
    }
}

// Anisotropy comes from inputs:scaleX/Y/Z, never from the transform:
// halving the X divisor doubles the reach along X.
static void
TestSphereDirectionalScaleValidation()
{
    RigExecVolumeWeightInputs inputs;
    inputs.representation = TfToken("dense");
    inputs.rangePolicy = TfToken("clamp");
    inputs.hasPlacement = true;
    inputs.targetPoints = {GfVec3f(0)};
    const TfToken type("RigExecSphereWeight");
    CHECK(RigExecBuildVolumeWeightPacket(type, inputs).valid);
    for (const float bad : {0.0f, -1.0f, std::numeric_limits<float>::infinity(),
                            std::numeric_limits<float>::quiet_NaN()}) {
        for (int side = 0; side < 2; ++side) {
            for (int axis = 0; axis < 3; ++axis) {
                auto test = inputs;
                (side ? test.negativeScales : test.positiveScales)[axis] = bad;
                CHECK(!RigExecVolumeWeightCanBuild(type, test));
                CHECK(!RigExecBuildVolumeWeightPacket(type, test).valid);
            }
        }
    }
}

static void
TestSphereDirectionalScales()
{
    const float pos[3] = {2, 3, 4}, neg[3] = {5, 6, 7};
    VtVec3fArray probes;
    for (int axis = 0; axis < 3; ++axis) {
        GfVec3f p(0), n(0);
        p[axis] = pos[axis] * 0.5f;
        n[axis] = -neg[axis] * 1.5f;
        probes.push_back(p);
        probes.push_back(n);
    }
    Fixture f(probes, GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Asymmetric", TfToken("RigExecSphereWeight"),
                             GfVec3d(0), 0.0f, 2.0f);
    const char *positive[] = {"inputs:scaleXPos", "inputs:scaleYPos", "inputs:scaleZPos"};
    const char *negative[] = {"inputs:scaleXNeg", "inputs:scaleYNeg", "inputs:scaleZNeg"};
    for (int a = 0; a < 3; ++a) {
        CHECK(v.GetAttribute(TfToken(positive[a])).Set(pos[a]));
        CHECK(v.GetAttribute(TfToken(negative[a])).Set(neg[a]));
    }
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());
    const auto points = f.Resolve("directional-sphere");
    CHECK(points.size() == probes.size());
    if (points.size() == probes.size()) {
        for (int a = 0; a < 3; ++a) {
            CHECK(Near(points[2*a], Moved(probes[2*a], 0.75f)));
            CHECK(Near(points[2*a+1], Moved(probes[2*a+1], 0.25f)));
        }
    }
}

static void
TestSphereDirectionalEdits()
{
    Fixture f(VtVec3fArray{GfVec3f(1,0,0), GfVec3f(-1,0,0)}, GfVec3d(0,2,0));
    UsdPrim v = f.MakeVolume("SignedEdit", TfToken("RigExecSphereWeight"),
                             GfVec3d(0), 0.0f, 2.0f);
    const auto pos = v.GetAttribute(TfToken("inputs:scaleXPos"));
    const auto neg = v.GetAttribute(TfToken("inputs:scaleXNeg"));
    CHECK(pos.Set(1.0f));
    CHECK(neg.Set(1.0f));
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());
    RigExecRigEvaluator evaluator(f.stage, SdfPath("/Asset/Rig"));
    evaluator.cpuParityMode = !BakedPathRequested();
    CHECK(evaluator.Compile());
    auto check = [&](UsdTimeCode time, float wp, float wn) {
        const auto pose = evaluator.Evaluate(time);
        CHECK(pose.valid);
        CheckTheHarnessRan("signed edits", pose);
        const auto it = pose.movedProperties.find(Fixture::Target());
        CHECK(it != pose.movedProperties.end());
        if (it == pose.movedProperties.end()) return;
        const auto points = it->second.Get<VtVec3fArray>();
        CHECK(points.size() == 2);
        if (points.size() != 2) return;
        CHECK(Near(points[0], Moved(f.base[0], wp)));
        CHECK(Near(points[1], Moved(f.base[1], wn)));
    };
    check(UsdTimeCode::Default(), 0.5f, 0.5f);
    const auto digest = evaluator.GetBindingEpochDigest();
    CHECK(pos.Set(2.0f));
    check(UsdTimeCode::Default(), 0.75f, 0.5f);
    CHECK(evaluator.GetBindingEpochDigest() == digest);
    CHECK(neg.Set(0.5f));
    check(UsdTimeCode::Default(), 0.75f, 0.0f);
    CHECK(evaluator.GetBindingEpochDigest() == digest);
    CHECK(pos.Set(1.0f, UsdTimeCode(1)));
    CHECK(pos.Set(2.0f, UsdTimeCode(2)));
    check(UsdTimeCode(1), 0.5f, 0.0f);
    check(UsdTimeCode(2), 0.75f, 0.0f);
}

static void
TestSphereAxisScales()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(2, 0, 0),
                           GfVec3f(0, 2, 0), GfVec3f(4, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Ellipsoid", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 2.0f);
    v.CreateAttribute(TfToken("inputs:scaleX"),
                      SdfValueTypeNames->Float).Set(2.0f);
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    const VtVec3fArray points = f.Resolve("ellipsoid");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    // local x is halved, so x=2 measures 1 and x=4 measures 2; y is
    // untouched and y=2 still measures 2.
    const float expected[4] = {1.0f, 0.5f, 0.0f, 0.0f};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], expected[i])));
    }
}

// The TRANSFORM's scale avars are not part of a volume's placement.
// A volume weight is a pose provider, so it composes like a joint -- but
// exec never binds avars:sx/sy/sz for one, and its shape is
// inputs:scaleX/Y/Z's business alone. The two say opposite things about the
// same points: a transform scale of 2 in X would HALVE the local distance
// of a point at X=2 and hand it a weight the rigid placement gives to X=1.
// So this case authors all three, expects the answers of the case that
// authors none, and is the only place either path's discard is checked --
// no shipped rig scales a volume's transform.
static void
TestVolumeIgnoresTransformScaleAvars()
{
    const VtVec3fArray points{GfVec3f(1, 0, 0), GfVec3f(2, 0, 0),
                              GfVec3f(3, 0, 0), GfVec3f(0, 1, 0)};
    // distances 1, 2, 3, 1 over a [0, 2] band -> 0.5, 0, 0, 0.5. Under a
    // composed sx=2/sy=0.5 they would be 0.75, 0.5, 0.25, 0 instead, so
    // three of the four points say which placement was used.
    const float expected[4] = {0.5f, 0.0f, 0.0f, 0.5f};
    for (const bool scaled : {false, true}) {
        Fixture f(points, GfVec3d(0, 2, 0));
        UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                                 GfVec3d(0, 0, 0), 0.0f, 2.0f);
        if (scaled) {
            const std::pair<const char *, double> avars[3] = {
                {"avars:sx", 2.0}, {"avars:sy", 0.5}, {"avars:sz", 3.0}};
            for (const auto &[name, value] : avars) {
                v.CreateAttribute(TfToken(name),
                                  SdfValueTypeNames->Double).Set(value);
            }
        }
        f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

        const VtVec3fArray moved =
            f.Resolve(scaled ? "volume-scale-avars" : "volume-no-scale-avars");
        CHECK(moved.size() == 4);
        if (moved.size() != 4) return;
        for (size_t i = 0; i < 4; ++i) {
            CHECK(Near(moved[i], Moved(f.base[i], expected[i])));
        }
    }
}

// A plane's distance is SIGNED, so a band straddling zero is a gradient
// across the plane rather than a band mirrored on both sides of it.
static void
TestPlaneWeight()
{
    Fixture f(VtVec3fArray{GfVec3f(0, -1, 0), GfVec3f(0, 0, 0),
                           GfVec3f(0, 1, 0), GfVec3f(9, -5, 9)},
              GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Plane", TfToken("RigExecPlaneWeight"),
                             GfVec3d(0, 0, 0), -1.0f, 1.0f);
    v.CreateAttribute(TfToken("rigExec:planeAxis"),
                      SdfValueTypeNames->Token).Set(TfToken("y"));
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    const VtVec3fArray points = f.Resolve("plane");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    // A plane is infinite: the last point's x and z are ignored entirely
    // and only its y = -5 matters, which clamps to fully on.
    const float expected[4] = {1.0f, 0.5f, 0.0f, 1.0f};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], expected[i])));
    }
}

// rigExec:planeBounds = `bounded` clips the field to the in-plane
// rectangle, and does nothing else: the same points inside it keep exactly
// the weights the infinite plane gave them.
// This case earns its keep through the parity assertion Resolve() makes.
// The bound lives in TWO independent implementations -- the exec kernel in
// moverKernels.cpp and the CPU oracle in rigEvaluator.cpp -- and a bound
// applied on only one of them would still move points, just not the same
// ones, which is exactly the failure moverGraphParityMismatches exists to
// catch.
static void
TestPlaneBounded()
{
    // Measured along y, so the in-plane axes are U = z and V = x. The
    // first three points sit on the axis and stay inside the rectangle;
    // the fourth is far outside it in both.
    Fixture f(VtVec3fArray{GfVec3f(0, -1, 0), GfVec3f(0, 0, 0),
                           GfVec3f(0, 1, 0), GfVec3f(9, -5, 9)},
              GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Plane", TfToken("RigExecPlaneWeight"),
                             GfVec3d(0, 0, 0), -1.0f, 1.0f);
    v.CreateAttribute(TfToken("rigExec:planeAxis"),
                      SdfValueTypeNames->Token).Set(TfToken("y"));
    v.CreateAttribute(TfToken("rigExec:planeBounds"),
                      SdfValueTypeNames->Token).Set(TfToken("bounded"));
    v.CreateAttribute(TfToken("inputs:extentU"), SdfValueTypeNames->Float)
        .Set(2.0f);
    v.CreateAttribute(TfToken("inputs:extentV"), SdfValueTypeNames->Float)
        .Set(2.0f);
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    {
        const VtVec3fArray points = f.Resolve("plane-bounded");
        CHECK(points.size() == 4);
        if (points.size() != 4) return;
        // Unchanged inside; zero outside, where the unbounded plane gave
        // this point a full 1.0 (see TestPlaneWeight).
        const float expected[4] = {1.0f, 0.5f, 0.0f, 0.0f};
        for (size_t i = 0; i < 4; ++i) {
            CHECK(Near(points[i], Moved(f.base[i], expected[i])));
        }
    }

    // The extents are per axis: widening V alone (x) is not enough,
    // because the far point is outside in U (z) as well.
    v.GetAttribute(TfToken("inputs:extentV")).Set(20.0f);
    {
        const VtVec3fArray points = f.Resolve("plane-bounded-wide-v");
        CHECK(points.size() == 4);
        if (points.size() != 4) return;
        CHECK(Near(points[3], Moved(f.base[3], 0.0f)));
    }
    v.GetAttribute(TfToken("inputs:extentU")).Set(20.0f);
    {
        const VtVec3fArray points = f.Resolve("plane-bounded-wide-uv");
        CHECK(points.size() == 4);
        if (points.size() != 4) return;
        CHECK(Near(points[3], Moved(f.base[3], 1.0f)));
    }

    // And back to unbounded, which must ignore the extents entirely.
    v.GetAttribute(TfToken("inputs:extentU")).Set(0.01f);
    v.GetAttribute(TfToken("inputs:extentV")).Set(0.01f);
    v.GetAttribute(TfToken("rigExec:planeBounds")).Set(TfToken("unbounded"));
    {
        const VtVec3fArray points = f.Resolve("plane-unbounded-again");
        CHECK(points.size() == 4);
        if (points.size() != 4) return;
        const float expected[4] = {1.0f, 0.5f, 0.0f, 1.0f};
        for (size_t i = 0; i < 4; ++i) {
            CHECK(Near(points[i], Moved(f.base[i], expected[i])));
        }
    }
}

// The structural / animatable split in the epoch digest, for the two
// properties this shape added.
// rigExec:planeBounds SELECTS WHICH FIELD FUNCTION RUNS, so it has to be
// hashed: an unhashed structural token leaves exec replaying the epoch's
// baked packet shape while the CPU oracle reads the live one. The
// extents are ordinary per-frame floats and must NOT be, because a digest
// that hashes an animatable value recompiles the whole rig on every
// mouse-move of the scrub row that drives it -- which is a rig-sized
// rebuild per frame, for a number exec re-reads for free.
// Both halves are asserted, and each half is what makes the other
// meaningful: "the digest did not change" is only good news next to a
// field that DID.
static void
TestPlaneBoundsEpochSplit()
{
    Fixture f(VtVec3fArray{GfVec3f(0, -1, 0), GfVec3f(0, 0, 0),
                           GfVec3f(0, 1, 0), GfVec3f(9, -5, 9)},
              GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Plane", TfToken("RigExecPlaneWeight"),
                             GfVec3d(0, 0, 0), -1.0f, 1.0f);
    v.CreateAttribute(TfToken("rigExec:planeAxis"),
                      SdfValueTypeNames->Token).Set(TfToken("y"));
    UsdAttribute bounds = v.CreateAttribute(
        TfToken("rigExec:planeBounds"), SdfValueTypeNames->Token);
    bounds.Set(TfToken("bounded"));
    UsdAttribute extentU = v.CreateAttribute(
        TfToken("inputs:extentU"), SdfValueTypeNames->Float);
    UsdAttribute extentV = v.CreateAttribute(
        TfToken("inputs:extentV"), SdfValueTypeNames->Float);
    extentU.Set(2.0f);
    extentV.Set(2.0f);
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    RigExecRigEvaluator evaluator(f.stage, SdfPath("/Asset/Rig"));

    evaluator.cpuParityMode = !BakedPathRequested();
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("plane-epoch: compile error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    // The far point is outside the 2 x 2 rectangle, so it reads zero.
    auto weightOfFarPoint = [&](const char *label) {
        const RigExecRigPose pose =
            evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        // Without this the whole case passes vacuously on a rig that
        // never built a revision at all.
        CheckTheHarnessRan(
            (std::string("plane-epoch ") + label).c_str(), pose);
        bool rebuilt = false;
        for (const std::string &d : pose.diagnostics) {
            if (d.find("epoch rebuilt") != std::string::npos) {
                rebuilt = true;
            }
        }
        const auto it = pose.movedProperties.find(Fixture::Target());
        float weight = -1.0f;
        if (it != pose.movedProperties.end()) {
            const VtVec3fArray points = it->second.Get<VtVec3fArray>();
            if (points.size() == 4) {
                weight = (points[3] - f.base[3])[1] * 0.5f;
            }
        }
        return std::make_pair(weight, rebuilt);
    };

    const auto first = weightOfFarPoint("bounded");
    const size_t digest = evaluator.GetBindingEpochDigest();
    CHECK(Near(GfVec3f(first.first, 0, 0), GfVec3f(0.0f, 0, 0)));

    extentU.Set(20.0f);
    extentV.Set(20.0f);
    const auto widened = weightOfFarPoint("wide extents");
    CHECK(Near(GfVec3f(widened.first, 0, 0), GfVec3f(1.0f, 0, 0)));
    if (evaluator.GetBindingEpochDigest() != digest) {
        ++failures;
        std::printf("FAIL inputs:extentU/V changed the binding-epoch "
                    "digest -- a scrub of an animatable float would "
                    "recompile the rig every frame\n");
    }
    if (widened.second) {
        ++failures;
        std::printf("FAIL inputs:extentU/V triggered an epoch rebuild\n");
    }

    // bounded kernel while the oracle runs the unbounded one.
    bounds.Set(TfToken("unbounded"));
    const auto freed = weightOfFarPoint("unbounded");
    CHECK(Near(GfVec3f(freed.first, 0, 0), GfVec3f(1.0f, 0, 0)));
    if (evaluator.GetBindingEpochDigest() == digest) {
        ++failures;
        std::printf("FAIL rigExec:planeBounds did not change the "
                    "binding-epoch digest\n");
    }
    if (!freed.second) {
        ++failures;
        std::printf("FAIL rigExec:planeBounds did not trigger an epoch "
                    "rebuild\n");
    }
}

// A bounded plane whose extents are not a rectangle.
// Both paths reject it -- the exec kernel returns an invalid packet and
// the CPU oracle reports an error -- and the two rejections have to mean
// the SAME THING downstream, or the parity harness is comparing a
// pass-through against a deformation. The failure mode this rules out is
// the quiet one: a kernel that treated a negative extent as "unbounded"
// would move points the oracle leaves alone, and nothing but parity
// would notice.
static void
TestPlaneBoundedInvalidExtents()
{
    Fixture f(VtVec3fArray{GfVec3f(0, -1, 0), GfVec3f(0, 0, 0),
                           GfVec3f(0, 1, 0), GfVec3f(9, -5, 9)},
              GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Plane", TfToken("RigExecPlaneWeight"),
                             GfVec3d(0, 0, 0), -1.0f, 1.0f);
    v.CreateAttribute(TfToken("rigExec:planeAxis"),
                      SdfValueTypeNames->Token).Set(TfToken("y"));
    v.CreateAttribute(TfToken("rigExec:planeBounds"),
                      SdfValueTypeNames->Token).Set(TfToken("bounded"));
    UsdAttribute extentU = v.CreateAttribute(
        TfToken("inputs:extentU"), SdfValueTypeNames->Float);
    v.CreateAttribute(TfToken("inputs:extentV"), SdfValueTypeNames->Float)
        .Set(2.0f);
    extentU.Set(-1.0f);  // no such rectangle
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    RigExecRigEvaluator evaluator(f.stage, SdfPath("/Asset/Rig"));

    evaluator.cpuParityMode = !BakedPathRequested();
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("plane-bad-extent: compile error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    // The rejection has to be the EXTENT, not a rig that never built a
    // revision: without this the case passes on any failure at all.
    CheckTheHarnessRan("plane-bad-extent", pose);
    // Rejected means PASS-THROUGH, not "weight zero" and not "unbounded":
    // the points come out exactly as authored.
    const auto it = pose.movedProperties.find(Fixture::Target());
    CHECK(it != pose.movedProperties.end());
    if (it != pose.movedProperties.end()) {
        const VtVec3fArray points = it->second.Get<VtVec3fArray>();
        CHECK(points.size() == 4);
        for (size_t i = 0; i < points.size() && i < 4; ++i) {
            CHECK(Near(points[i], f.base[i]));
        }
    }

    // ...and a valid extent on the same rig recovers, which is what says
    // the rejection was the extent and not a broken epoch.
    extentU.Set(2.0f);
    const RigExecRigPose fixed = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(fixed.valid);
    CHECK(fixed.moverGraphParityMismatches == 0);
    const auto fixedIt = fixed.movedProperties.find(Fixture::Target());
    CHECK(fixedIt != fixed.movedProperties.end());
    if (fixedIt != fixed.movedProperties.end()) {
        const VtVec3fArray points = fixedIt->second.Get<VtVec3fArray>();
        CHECK(points.size() == 4);
        if (points.size() == 4) {
            const float expected[4] = {1.0f, 0.5f, 0.0f, 0.0f};
            for (size_t i = 0; i < 4; ++i) {
                CHECK(Near(points[i], Moved(f.base[i], expected[i])));
            }
        }
    }
}

// A curve weight measures distance to the polyline through the curve's
// points, including past its ends, where it clamps to the endpoint.
static void
TestCurveWeight()
{
    Fixture f(VtVec3fArray{GfVec3f(5, 0, 0), GfVec3f(5, 1, 0),
                           GfVec3f(5, 2, 0), GfVec3f(-5, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim curve = f.stage->DefinePrim(SdfPath("/Asset/Rig/Curve"),
                                        TfToken("BasisCurves"));
    curve.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(10, 0, 0)});

    UsdPrim v = f.MakeVolume("Curve", TfToken("RigExecCurveWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 2.0f);
    v.CreateRelationship(TfToken("rigExec:curve"))
        .SetTargets({SdfPath("/Asset/Rig/Curve.points")});
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    const VtVec3fArray points = f.Resolve("curve");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    // distance 0, 1, 2, and 5 (clamped to the segment start).
    const float expected[4] = {1.0f, 0.5f, 0.0f, 0.0f};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], expected[i])));
    }
}

// Composition: a painted static field masked by a generated sphere field.
static void
TestCombineMultiply()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                           GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim sphere = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                                  GfVec3d(0, 0, 0), 0.0f, 2.0f);
    UsdPrim paint =
        f.MakeStaticWeight("Paint", VtFloatArray{1.0f, 1.0f, 0.5f, 1.0f});

    UsdPrim combine = f.stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Combined"),
        TfToken("RigExecCombineWeight"));
    combine.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({Fixture::Target()});
    combine.CreateAttribute(TfToken("rigExec:representation"),
                            SdfValueTypeNames->Token).Set(TfToken("dense"));
    combine.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({sphere.GetPath(), paint.GetPath()});
    combine.CreateAttribute(TfToken("rigExec:combineMode"),
                            SdfValueTypeNames->Token)
        .Set(TfToken("multiply"));
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), combine.GetPath());

    const VtVec3fArray points = f.Resolve("combine-multiply");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    // sphere 1, 0.75, 0.5, 0 times paint 1, 1, 0.5, 1
    const float expected[4] = {1.0f, 0.75f, 0.25f, 0.0f};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], expected[i])));
    }

    // The combine's OWN inputs:invert and inputs:strength are applied
    // after the fold, in two separate implementations (the exec kernel
    // and the CPU oracle). Untested, they are exactly the kind of thing
    // that drifts: same formula, two places.
    combine.CreateAttribute(TfToken("inputs:invert"),
                            SdfValueTypeNames->Float).Set(1.0f);
    combine.CreateAttribute(TfToken("inputs:strength"),
                            SdfValueTypeNames->Float).Set(0.5f);
    const VtVec3fArray shaped = f.Resolve("combine-invert-strength");
    CHECK(shaped.size() == 4);
    if (shaped.size() != 4) return;
    for (size_t i = 0; i < 4; ++i) {
        // (1 - w) * 0.5 over the folded field above.
        const float w = (1.0f - expected[i]) * 0.5f;
        CHECK(Near(shaped[i], Moved(f.base[i], w)));
    }
}

// `max` unions two volumes, which is the composition a rigger reaches for
// when one influence has to cover two regions.
static void
TestCombineMaxOfTwoSpheres()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(5, 0, 0),
                           GfVec3f(10, 0, 0), GfVec3f(20, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim a = f.MakeVolume("A", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 10.0f);
    UsdPrim b = f.MakeVolume("B", TfToken("RigExecSphereWeight"),
                             GfVec3d(10, 0, 0), 0.0f, 10.0f);
    UsdPrim combine = f.stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Union"), TfToken("RigExecCombineWeight"));
    combine.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({Fixture::Target()});
    combine.CreateAttribute(TfToken("rigExec:representation"),
                            SdfValueTypeNames->Token).Set(TfToken("dense"));
    combine.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({a.GetPath(), b.GetPath()});
    combine.CreateAttribute(TfToken("rigExec:combineMode"),
                            SdfValueTypeNames->Token).Set(TfToken("max"));
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), combine.GetPath());

    const VtVec3fArray points = f.Resolve("combine-max");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    // A gives 1, 0.5, 0, 0; B gives 0, 0.5, 1, 0. max is 1, 0.5, 1, 0.
    const float expected[4] = {1.0f, 0.5f, 1.0f, 0.0f};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], expected[i])));
    }
}

// `subtract` is the order-DEPENDENT mode, and it is the one that proves
// the exec kernel and the CPU oracle agree on what "authored order"
// means: the oracle reads rel.GetTargets() directly, while the kernel
// receives the targets through a VdfReadIterator. If exec delivered them
// in any other order (sorted by path, say) the two would disagree and the
// parity harness would fire -- which is exactly what this asserts.
// The commutative modes could never catch that.
static void
TestCombineSubtractOrder()
{
    // Two flat static fields, so the ONLY thing under test is the fold
    // order. Names chosen so path-sorted order differs from authored
    // order: authored is [B, A], sorted would be [A, B].
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                           GfVec3f(2, 0, 0), GfVec3f(3, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim a =
        f.MakeStaticWeight("Aaa", VtFloatArray{0.25f, 0.25f, 0.25f, 0.25f});
    UsdPrim b =
        f.MakeStaticWeight("Bbb", VtFloatArray{1.0f, 1.0f, 1.0f, 1.0f});

    UsdPrim combine = f.stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Diff"), TfToken("RigExecCombineWeight"));
    combine.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({Fixture::Target()});
    combine.CreateAttribute(TfToken("rigExec:representation"),
                            SdfValueTypeNames->Token).Set(TfToken("dense"));
    // Authored B first, then A: B - A = 0.75. Sorted would be A - B,
    // which clamps to 0 and would move nothing.
    combine.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({b.GetPath(), a.GetPath()});
    combine.CreateAttribute(TfToken("rigExec:combineMode"),
                            SdfValueTypeNames->Token).Set(TfToken("subtract"));
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), combine.GetPath());

    const VtVec3fArray points = f.Resolve("combine-subtract");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], 0.75f)));
    }

    // Reverse the authored order and the fold goes NEGATIVE: 0.25 - 1 =
    // -0.75, which rigExec:rangePolicy = "clamp" pulls to zero. This
    // asserts two things at once -- that the order really is authored
    // order (a sorted read would give the same answer both ways), and
    // that the exec kernel and the CPU oracle clamp identically, which
    // they do in two different functions.
    combine.GetRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({a.GetPath(), b.GetPath()});
    const VtVec3fArray reversed = f.Resolve("combine-subtract-reversed");
    CHECK(reversed.size() == 4);
    if (reversed.size() != 4) return;
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(reversed[i], f.base[i]));
    }
}

// A combine whose inputs are ALL constant-representation. Nothing among
// the inputs knows the cardinality, so the combine has to take it from
// its own rigExec:weightTarget.
// This is not an exotic case: "constant" is the schema default for an
// authored weight object, so it is what a rigger gets the first time they
// multiply two freshly created static weights. Before the fix the exec
// kernel published an invalid packet (mover passes through, points do not
// move) while the CPU oracle resolved every constant to `count` values
// and moved them -- a parity mismatch rather than a visible error.
static void
TestCombineOfConstantInputs()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                           GfVec3f(2, 0, 0), GfVec3f(3, 0, 0)},
              GfVec3d(0, 2, 0));
    auto makeConstant = [&](const char *name, float value) {
        UsdPrim w = f.stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name),
            TfToken("RigExecStaticWeight"));
        w.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({Fixture::Target()});
        w.CreateAttribute(TfToken("rigExec:representation"),
                          SdfValueTypeNames->Token).Set(TfToken("constant"));
        w.CreateAttribute(TfToken("rigExec:defaultWeight"),
                          SdfValueTypeNames->Float).Set(value);
        return w;
    };
    UsdPrim a = makeConstant("ConstA", 0.5f);
    UsdPrim b = makeConstant("ConstB", 0.5f);

    UsdPrim combine = f.stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/AllConstant"),
        TfToken("RigExecCombineWeight"));
    combine.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({Fixture::Target()});
    combine.CreateAttribute(TfToken("rigExec:representation"),
                            SdfValueTypeNames->Token).Set(TfToken("dense"));
    combine.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({a.GetPath(), b.GetPath()});
    combine.CreateAttribute(TfToken("rigExec:combineMode"),
                            SdfValueTypeNames->Token)
        .Set(TfToken("multiply"));
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), combine.GetPath());

    const VtVec3fArray points = f.Resolve("combine-all-constant");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], 0.25f)));
    }
}

// inputs:invert exchanges the ends of the band, continuously.
static void
TestInvert()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                           GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 2.0f);
    v.CreateAttribute(TfToken("inputs:invert"),
                      SdfValueTypeNames->Float).Set(1.0f);
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    const VtVec3fArray points = f.Resolve("invert");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    const float expected[4] = {0.0f, 0.25f, 0.5f, 1.0f};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], expected[i])));
    }
}

// An authored Ts spline on rigExec:falloffCurve replaces the analytic
// remap. This is the path exec cannot read directly -- the table is baked
// at Compile and delivered as a value override -- so it is also the test
// that the override actually arrives.
static void
TestAuthoredFalloffCurve()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                           GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 2.0f);
    v.GetAttribute(TfToken("rigExec:falloffProfile")).Set(TfToken("curve"));

    // A curve that saturates early: r >= 0.5 is already fully on.
    UsdAttribute curveAttr = v.CreateAttribute(
        TfToken("rigExec:falloffCurve"), SdfValueTypeNames->Float);
    // The C++ Ts constructors take a TfType; only the Python bindings
    // accept the type NAME as a string.
    const TfType floatType = TfType::Find<float>();
    TsSpline spline(floatType);
    for (const auto &knotSpec :
         std::vector<std::pair<double, double>>{{0.0, 0.0}, {0.5, 1.0},
                                                {1.0, 1.0}}) {
        TsKnot knot(floatType);
        knot.SetTime(knotSpec.first);
        knot.SetValue(float(knotSpec.second));
        knot.SetNextInterpolation(TsInterpLinear);
        spline.SetKnot(knot);
    }
    CHECK(curveAttr.SetSpline(spline));
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    const VtVec3fArray points = f.Resolve("authored-curve");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    // r = 1 - d/2 -> 1, 0.75, 0.5, 0; the curve maps [0.5, 1] to 1 and
    // ramps [0, 0.5] linearly to 1, so r=0 -> 0 and the rest -> 1.
    const float expected[4] = {1.0f, 1.0f, 1.0f, 0.0f};
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], expected[i])));
    }
}

// The read phase on rigExec:weightTarget decides WHICH points the
// distance is measured against, and the two answers genuinely differ: a
// volume the geometry has already been moved out of grabs nothing under
// `preceding` and still grabs under `base`.
static void
TestSamplePhase(bool current)
{
    const VtVec3fArray base{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                            GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};
    Fixture f(base, GfVec3d(0, 2, 0));

    // A second joint that lifts everything +5Y first, unweighted.
    UsdPrim lift = f.stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Lift"),
                                       TfToken("RigExecJoint"));
    Fixture::PlaceAt(lift, GfVec3d(0, 5, 0));
    UsdPrim full = f.MakeStaticWeight(
        "Full", VtFloatArray{1.0f, 1.0f, 1.0f, 1.0f});

    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 3.0f);
    v.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetMetadata(TfToken("rigExecReadPhase"),
                     std::string(current ? "preceding" : "base"));

    // Composed post-order: the DEEPER mover runs first, so Lift is
    // nested inside Second and applies before it.
    UsdPrim second =
        f.MakeMover(SdfPath("/Asset/Rig/Movers/Second"), v.GetPath());
    UsdPrim first = f.MakeMover(
        SdfPath("/Asset/Rig/Movers/Second/First"), full.GetPath());
    first.GetRelationship(TfToken("rigExec:transform"))
        .SetTargets({lift.GetPath()});
    (void)second;

    const VtVec3fArray points =
        f.Resolve(current ? "readPhase-preceding" : "readPhase-base");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;

    for (size_t i = 0; i < 4; ++i) {
        const GfVec3f lifted5 = base[i] + GfVec3f(0, 5, 0);
        if (current) {
            // Measured AFTER the lift: every point is >= 5 away from a
            // volume that reaches 3, so the second mover does nothing.
            CHECK(Near(points[i], lifted5));
        } else {
            // Measured against the authored base: distances 0, 0.5, 1, 2
            // over a [0, 3] band.
            const float d = base[i].GetLength();
            const float w = 1.0f - d / 3.0f;
            CHECK(Near(points[i], lifted5 + GfVec3f(0, w * 2.0f, 0)));
        }
    }
}

// A volume weight whose target does not canonicalize to the consuming
// mover's points target must skip the mover, exactly as an authored one
// does -- the validation is type-agnostic and this proves it stayed so.
static void
TestVolumeWeightTargetMismatchSkipsMover()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim other =
        f.stage->DefinePrim(SdfPath("/Asset/Geom/Other"), TfToken("Points"));
    other.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0)});

    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 2.0f);
    v.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Geom/Other.points")});
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    RigExecRigEvaluator evaluator(f.stage, SdfPath("/Asset/Rig"));

    evaluator.cpuParityMode = !BakedPathRequested();
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.GetSkippedOperations().count(SdfPath("/Asset/Rig/Movers/M")) == 1);
    CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(!errors.empty());
}

// A curve weight naming two curves is an authoring error, not a polyline
// joining them. Rejected at compile, so neither evaluation path ever sees
// the ambiguity: exec would have flattened both targets into one stream
// with a spurious segment between them, while the CPU oracle rejects the
// pair -- and two paths quietly computing different fields is worse than
// a compile failure.
static void
TestCurveWeightRejectsTwoCurves()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)},
              GfVec3d(0, 2, 0));
    for (const char *name : {"CurveA", "CurveB"}) {
        UsdPrim c = f.stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/") + name),
            TfToken("BasisCurves"));
        c.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
            .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)});
    }
    UsdPrim v = f.MakeVolume("Curve", TfToken("RigExecCurveWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 2.0f);
    v.CreateRelationship(TfToken("rigExec:curve"))
        .SetTargets({SdfPath("/Asset/Rig/CurveA.points"),
                     SdfPath("/Asset/Rig/CurveB.points")});
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), v.GetPath());

    RigExecRigEvaluator evaluator(f.stage, SdfPath("/Asset/Rig"));

    evaluator.cpuParityMode = !BakedPathRequested();
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    CHECK(!errors.empty());
}

// A DEFAULT combine: author the prim, wire two inputs, change nothing
// else. rigExec:representation used to inherit `constant` from
// RigExecWeightObject, which the exec builder rejects while the CPU
// oracle resolved it happily -- a parity mismatch reachable by doing the
// most obvious possible thing. Every other combine test here authors
// `dense` explicitly and so could never have caught it.
static void
TestDefaultCombineNeedsNoRepresentation()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                           GfVec3f(2, 0, 0), GfVec3f(3, 0, 0)},
              GfVec3d(0, 2, 0));
    UsdPrim a =
        f.MakeStaticWeight("A", VtFloatArray{1.0f, 1.0f, 1.0f, 1.0f});
    UsdPrim b =
        f.MakeStaticWeight("B", VtFloatArray{0.5f, 0.5f, 0.5f, 0.5f});

    UsdPrim combine = f.stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Plain"),
        TfToken("RigExecCombineWeight"));
    combine.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({Fixture::Target()});
    combine.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({a.GetPath(), b.GetPath()});
    // Deliberately NO representation and NO combineMode authored.
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), combine.GetPath());

    const VtVec3fArray points = f.Resolve("combine-all-defaults");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], Moved(f.base[i], 0.5f)));
    }
}

// `current` sampling has to survive COMPOSITION. The graph tests the
// weight object a mover binds, which for a composed field is the
// combine, not the volume inside it -- so recording only the leaf left
// exec applying the reference field while the CPU oracle failed.
static void
TestCurrentPhaseThroughCombine()
{
    const VtVec3fArray base{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                            GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};
    Fixture f(base, GfVec3d(0, 2, 0));

    UsdPrim lift = f.stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Lift"),
                                       TfToken("RigExecJoint"));
    Fixture::PlaceAt(lift, GfVec3d(0, 5, 0));
    UsdPrim full = f.MakeStaticWeight(
        "Full", VtFloatArray{1.0f, 1.0f, 1.0f, 1.0f});

    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 3.0f);
    v.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetMetadata(TfToken("rigExecReadPhase"), std::string("preceding"));

    // The mover binds the COMBINE, not the sphere.
    UsdPrim combine = f.stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Wrapped"),
        TfToken("RigExecCombineWeight"));
    combine.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({Fixture::Target()});
    combine.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({v.GetPath()});

    UsdPrim second =
        f.MakeMover(SdfPath("/Asset/Rig/Movers/Second"), combine.GetPath());
    UsdPrim first = f.MakeMover(
        SdfPath("/Asset/Rig/Movers/Second/First"), full.GetPath());
    first.GetRelationship(TfToken("rigExec:transform"))
        .SetTargets({lift.GetPath()});
    (void)second;

    const VtVec3fArray points = f.Resolve("current-through-combine");
    CHECK(points.size() == 4);
    if (points.size() != 4) return;
    // Measured AFTER the +5Y lift, every point is >= 5 from a volume that
    // reaches 3, so the second mover must do nothing.
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(points[i], base[i] + GfVec3f(0, 5, 0)));
    }
}

// A composition cycle is an authoring error and must be DIAGNOSED. The
// CPU resolver recurses through the same edges with no guard of its own,
// so an undetected cycle exhausts the stack rather than answering wrong.
static void
TestCombineCycleSkipsMover()
{
    Fixture f(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)},
              GfVec3d(0, 2, 0));
    auto makeCombine = [&](const char *name) {
        UsdPrim c = f.stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name),
            TfToken("RigExecCombineWeight"));
        c.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({Fixture::Target()});
        return c;
    };
    UsdPrim a = makeCombine("CycleA");
    UsdPrim b = makeCombine("CycleB");
    a.GetRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({b.GetPath()});
    b.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({a.GetPath()});
    f.MakeMover(SdfPath("/Asset/Rig/Movers/M"), a.GetPath());

    RigExecRigEvaluator evaluator(f.stage, SdfPath("/Asset/Rig"));

    evaluator.cpuParityMode = !BakedPathRequested();
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.GetSkippedOperations().count(SdfPath("/Asset/Rig/Movers/M")) == 1);
    CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(!errors.empty());
}


// A volume weight object bound to a CONSTRAINT.
// The reachable shape of it is a GEOMETRY-DOMAIN constraint: a volumetric
// field needs a point domain (_ValidateWeightObjectDomain refuses one on a
// transform domain outright, composed inputs included), so the constraint
// that can bind a sphere is one whose rigExec:moves names a .points
// attribute. Its envelope is then per POINT and resolves on the Matrix
// revision the constraint's delta feeds -- a constraint and its revision
// being the same mover prim -- rather than as the one scalar a
// transform-domain constraint would resolve.
// Nothing in examples/ and nothing in this suite reached it, which is why
// the program refused "volume weight object on constraint" outright and
// with no parity evidence either way.
namespace {

struct ConstraintEnvelopeFixture {
    UsdStageRefPtr stage;
    VtVec3fArray base;

    // \p volumeAt places the sphere through its own avars.
    // The sphere is deliberately NOT a constraint target here. Making one is
    // a rig the program refuses ("constraint target is both exec-seeded and
    // xform-derived"), for a reason that belongs to the dynamic path rather
    // than to the bake: see TestAConstrainedVolumeWeightFallsBack in
    // testRigExecBakedMode.
    ConstraintEnvelopeFixture(const GfVec3d &volumeAt,
                              const char *readPhase)
    {
        stage = UsdStage::CreateInMemory();
        stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));

        base = VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0, 4, 0),
                            GfVec3f(0, 8, 0)};
        const UsdPrim mesh =
            stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Points"));
        mesh.CreateAttribute(TfToken("points"),
                             SdfValueTypeNames->Point3fArray).Set(base);

        // Where the geometry constraint pulls the point set to. A position
        // constraint rather than an aim: a translation shows the resolved
        // field directly in the answer, which is what the assertions read.
        const UsdGeomXform source =
            UsdGeomXform::Define(stage, SdfPath("/Asset/PullTo"));
        source.MakeMatrixXform().Set(
            GfMatrix4d(1.0).SetTranslate(GfVec3d(6, 0, 0)));

        // The sphere, painted over the three points and ramped across them,
        // so the envelope is strictly inside (0, 1) on at least one of them
        // and the placement is visible in the answer.
        const UsdPrim sphere = stage->DefinePrim(
            SdfPath("/Asset/Rig/Weights/Sphere"),
            TfToken("RigExecSphereWeight"));
        Fixture::PlaceAt(sphere, volumeAt);
        sphere.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({SdfPath("/Asset/Geom/M.points")});
        sphere.CreateAttribute(TfToken("inputs:falloffMin"),
                               SdfValueTypeNames->Float).Set(0.0f);
        sphere.CreateAttribute(TfToken("inputs:falloffMax"),
                               SdfValueTypeNames->Float).Set(8.0f);
        sphere.CreateAttribute(TfToken("rigExec:falloffProfile"),
                               SdfValueTypeNames->Token)
            .Set(TfToken("linear"));
        sphere.GetRelationship(TfToken("rigExec:weightTarget"))
            .SetMetadata(TfToken("rigExecReadPhase"), std::string(readPhase));

        // The geometry-domain constraint, and the weight object on it.
        const UsdPrim constraint = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Sweep/Pull"),
            TfToken("RigExecPositionConstraint"));
        constraint.ApplyAPI(TfToken("RigExecMoverAPI"));
        constraint.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath("/Asset/Geom/M.points")});
        constraint.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({source.GetPath()});
        constraint.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({sphere.GetPath()});
    }

    VtVec3fArray Resolve(const std::string &label)
    {
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        evaluator.cpuParityMode = !BakedPathRequested();
        std::vector<std::string> errors;
        if (!evaluator.Compile(&errors)) {
            for (const std::string &e : errors) {
                std::printf("%s: compile error: %s\n", label.c_str(),
                            e.c_str());
            }
            ++failures;
            return {};
        }
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        if (!pose.valid) {
            for (const std::string &d : pose.diagnostics) {
                std::printf("%s: diagnostic: %s\n", label.c_str(), d.c_str());
            }
            std::printf("%s: pose invalid\n", label.c_str());
            ++failures;
            return {};
        }
        CheckTheHarnessRan(label.c_str(), pose);
        const auto it =
            pose.movedProperties.find(SdfPath("/Asset/Geom/M.points"));
        if (it == pose.movedProperties.end()) {
            std::printf("%s: no moved points\n", label.c_str());
            ++failures;
            return {};
        }
        return it->second.Get<VtVec3fArray>();
    }
};

// How many of \p points the constraint moved, and how many it left alone.
std::pair<size_t, size_t>
SplitByMovement(const VtVec3fArray &points, const VtVec3fArray &base)
{
    size_t moved = 0, held = 0;
    for (size_t i = 0; i < points.size() && i < base.size(); ++i) {
        (Near(points[i], base[i]) ? held : moved) += 1;
    }
    return {moved, held};
}

}  // namespace

static void
TestVolumeWeightOnAConstraint(const char *readPhase)
{
    const std::string what =
        std::string("volume envelope on a geometry constraint, ") +
        readPhase;
    ConstraintEnvelopeFixture low(GfVec3d(0, 0, 0), readPhase);
    ConstraintEnvelopeFixture high(GfVec3d(0, 8, 0), readPhase);
    const VtVec3fArray lowPoints = low.Resolve(what + ", low");
    const VtVec3fArray highPoints = high.Resolve(what + ", high");
    if (lowPoints.size() != low.base.size() ||
        highPoints.size() != high.base.size()) {
        return;
    }
    // The envelope is partial: the constraint moved some points and not all
    // of them. An all-or-nothing field would agree between the two rigs
    // whatever the placement and prove nothing about either.
    const auto [moved, held] = SplitByMovement(lowPoints, low.base);
    CHECK(moved > 0);
    CHECK(held > 0);
    // And the sphere's own placement reaches the field: the two rigs differ
    // in nothing else.
    bool differs = false;
    for (size_t i = 0; i < lowPoints.size(); ++i) {
        differs = differs || !Near(lowPoints[i], highPoints[i]);
    }
    CHECK(differs);
}

// RigExecVolumePlacement: the decomposition of a usable final frame, and the
// identity for every frame that is not one.
static void
TestVolumePlacementGate()
{
    GfMatrix4d placed(1.0);
    placed.SetRotate(GfRotation(GfVec3d(0, 0, 1), 30.0));
    placed.SetTranslateOnly(GfVec3d(1, 2, 3));
    const RigExecPointFrame usable =
        RigExecMatrixToPoints(RigExecIdentityLandmarks(), placed);
    GfMatrix4d expected(1.0);
    CHECK(RigExecPointsToMatrix(RigExecIdentityLandmarks(), usable.points,
                                &expected));
    CHECK(RigExecVolumePlacement(usable) == expected);
    CHECK(GfIsClose(RigExecVolumePlacement(usable), placed, 1e-9));

    const GfMatrix4d identity(1.0);
    RigExecPointFrame degenerate = usable;
    degenerate.flags |= RigExecPointFrameDegenerate;
    CHECK(RigExecVolumePlacement(degenerate) == identity);

    RigExecPointFrame invalid = usable;
    invalid.flags = 0;
    CHECK(RigExecVolumePlacement(invalid) == identity);

    RigExecPointFrame notFinite = usable;
    notFinite.points[2][1] = std::numeric_limits<double>::quiet_NaN();
    CHECK(RigExecVolumePlacement(notFinite) == identity);
    notFinite = usable;
    notFinite.points[0][0] = std::numeric_limits<double>::infinity();
    CHECK(RigExecVolumePlacement(notFinite) == identity);
}

// A baked run places a volume from the program's own table. The dynamic
// walk's map is poisoned between two baked runs; the second run's cone skips
// VolumePlacements (no pose input moved) but re-runs the current-phase
// assemble (the base points are animated), which resolves its field through
// the oracle. The field and pose.weightFrames must equal a fresh evaluation.
static void
TestTheOracleReadsTheProgramsPlacements()
{
    const char *label = "oracle reads the program's placements";
    const VtVec3fArray base{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                            GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};
    Fixture f(base, GfVec3d(0, 2, 0));
    VtVec3fArray later = base;
    for (GfVec3f &p : later) {
        p += GfVec3f(0.25f, 0, 0);
    }
    const UsdAttribute points = f.stage->GetAttributeAtPath(Fixture::Target());
    points.Set(base, UsdTimeCode(1));
    points.Set(later, UsdTimeCode(2));

    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 3.0f);
    v.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetMetadata(TfToken("rigExecReadPhase"), std::string("preceding"));
    f.MakeMover(SdfPath("/Asset/Rig/Movers/Lift"), v.GetPath());

    RigExecRigEvaluator E(f.stage, SdfPath("/Asset/Rig"));
    E.SetEvaluationMode(RigExecEvaluationMode::Baked);
    E.SetProfilingEnabled(true);
    std::vector<std::string> errors;
    if (!E.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("%s: compile error: %s\n", label, e.c_str());
        }
        ++failures;
        return;
    }
    const RigExecRigPose first = E.Evaluate(UsdTimeCode(1));
    if (!first.valid || E.GetBakedGenerationCount() != 1) {
        std::printf("FAIL %s: the first generation is not baked\n", label);
        ++failures;
        return;
    }
    GfMatrix4d poison(1.0);
    poison.SetTranslate(GfVec3d(100, 0, 0));
    RigExecBakedProgramTesting::SetWalkVolumePlacements(&E, poison);

    E.ClearProfile();
    const RigExecRigPose second = E.Evaluate(UsdTimeCode(2));
    CHECK(second.valid);
    CHECK(E.GetBakedGenerationCount() == 2);
    CHECK(second.bakedParityMismatches == 0);
    size_t placements = 0, assembles = 0;
    for (const RigExecOpTraceEntry &entry : E.GetLastOpTrace()) {
        placements += entry.kind == "VolumePlacements";
        assembles += entry.kind == "RevisionStatic";
    }
    // The setup the assertion below depends on: the oracle ran, the
    // placement step did not.
    CHECK(placements == 0);
    CHECK(assembles > 0);

    RigExecRigEvaluator fresh(f.stage, SdfPath("/Asset/Rig"));
    fresh.SetEvaluationMode(RigExecEvaluationMode::ExecReference);
    CHECK(fresh.Compile(&errors));
    const RigExecRigPose reference = fresh.Evaluate(UsdTimeCode(2));
    CHECK(reference.valid);

    const auto got = second.movedProperties.find(Fixture::Target());
    const auto want = reference.movedProperties.find(Fixture::Target());
    CHECK(got != second.movedProperties.end());
    CHECK(want != reference.movedProperties.end());
    if (got == second.movedProperties.end() ||
        want == reference.movedProperties.end()) {
        return;
    }
    const VtVec3fArray gotPoints = got->second.Get<VtVec3fArray>();
    CHECK(gotPoints == want->second.Get<VtVec3fArray>());
    // Not vacuous: the field reaches the points, so a far-away placement
    // would have moved none of them.
    bool lifted = false;
    for (size_t i = 0; i < gotPoints.size() && i < later.size(); ++i) {
        lifted = lifted || !Near(gotPoints[i], later[i]);
    }
    CHECK(lifted);
    CHECK(second.weightFrames == reference.weightFrames);
    const auto frame = second.weightFrames.find(v.GetPath());
    CHECK(frame != second.weightFrames.end() && frame->second != poison);
}

// A baked run publishes the program's placements, not the dynamic walk's
// map. An unplaceable override (a computation override) sends one
// generation down the walk while the program is kept, and the walk places
// the overridden volume. After the release, a baked run at the same time
// has nothing dirty relative to the program's last run, so its cone skips
// VolumePlacements (no step reads outside the program, so that cluster is
// not always run). It must publish the program's earlier answer.
static void
TestReleasedOverridePublishesTheProgramsPlacements()
{
    // The setup is a deliberate fallback, which a RIGEXEC_BAKE_REQUIRED
    // registration reports as a failure; the default registration runs it.
    if (TfGetenvBool("RIGEXEC_BAKE_REQUIRED", false)) {
        return;
    }
    const char *label = "released override publishes program placements";
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const bool imported = stage->GetRootLayer()->ImportFromString(R"(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 10
)
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def RigExecControl "Root"
        {
            double avars:rz = 0
            double avars:rz.timeSamples = {1: 0, 2: 45}
        }
        def RigExecJoint "Joint"
        {
            matrix4d rest:space = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1))
            double avars:rz.connect = </Asset/Rig/Root.avars:rz>
        }
        def RigExecSphereWeight "Guide"
        {
            double avars:ty = 3
            float inputs:falloffMin = 0
            float inputs:falloffMax = 1
        }
    }
}
)");
    CHECK(imported);
    const SdfPath rig("/Asset/Rig"), guide("/Asset/Rig/Guide");
    RigExecRigEvaluator E(stage, rig);
    E.SetEvaluationMode(RigExecEvaluationMode::Baked);
    E.SetProfilingEnabled(true);
    std::vector<std::string> errors;
    if (!E.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("FAIL %s: compile error: %s\n", label, e.c_str());
        }
        ++failures;
        return;
    }
    CHECK(E.Evaluate(UsdTimeCode(1)).valid);
    const RigExecRigPose answer = E.Evaluate(UsdTimeCode(2));
    CHECK(answer.valid);
    if (E.GetBakedGenerationCount() != 2) {
        std::printf("FAIL %s: the first generations are not baked\n", label);
        ++failures;
        return;
    }
    const auto authored = answer.weightFrames.find(guide);
    CHECK(authored != answer.weightFrames.end() &&
          authored->second.ExtractTranslation()[1] == 3.0);
    for (const RigExecBakedStep &step :
         E.GetBakedProgram()->GetStepGraph().steps) {
        CHECK(!step.externalReads);
    }

    GfMatrix4d moved(1.0);
    moved.SetTranslate(GfVec3d(0, 50, 0));
    RigExecValueOverride drag;
    drag.prim = guide;
    drag.computation = TfToken("computePointFrame");
    drag.value =
        VtValue(RigExecMatrixToPoints(RigExecIdentityLandmarks(), moved));
    E.SetInteractiveOverrides({drag});
    const RigExecRigPose held = E.Evaluate(UsdTimeCode(2));
    CHECK(held.valid);
    CHECK(E.GetBakedGenerationCount() == 2);
    CHECK(E.GetBakedProgram() != nullptr);
    // Not vacuous: the walk's map now holds a placement the program's table
    // does not.
    const auto dragged = held.weightFrames.find(guide);
    CHECK(dragged != held.weightFrames.end() &&
          dragged->second.ExtractTranslation()[1] == 50.0);

    E.ClearInteractiveOverrides();
    E.ClearProfile();
    const RigExecRigPose released = E.Evaluate(UsdTimeCode(2));
    CHECK(released.valid);
    CHECK(E.GetBakedGenerationCount() == 3);
    for (const RigExecOpTraceEntry &entry : E.GetLastOpTrace()) {
        CHECK(entry.kind != "VolumePlacements");
    }
    if (released.weightFrames != answer.weightFrames) {
        std::printf("FAIL %s: weightFrames differ from the earlier baked "
                    "answer\n", label);
        ++failures;
    }
}

// The step of \p kind labelled exactly "<kind> <path>", or SIZE_MAX with a
// failure when there is not exactly one.
static size_t
OneOpGraphStep(const std::vector<RigExecOpGraphNode> &graph,
               const std::string &kind, const SdfPath &path)
{
    const std::string label = kind + " " + path.GetString();
    size_t found = SIZE_MAX, count = 0;
    for (const RigExecOpGraphNode &node : graph) {
        if (node.kind == kind && node.label == label) {
            found = node.step;
            ++count;
        }
    }
    if (count != 1) {
        std::printf("FAIL: expected one step labelled %s, found %zu\n",
                    label.c_str(), count);
        ++failures;
        return SIZE_MAX;
    }
    return found;
}

// volume_placements.usda's StripSmooth measures Both (max of SphereA and
// SphereB) against the current phase, so its RevisionStatic reads exactly
// the WeightFrames slots of SphereA's and SphereB's placement steps, waits
// on both steps, and neither reads nor waits on the placement of any volume
// in \p outside. The program is built under an error mark: a closure the
// build cannot resolve falls back to every volume slot through a TF_VERIFY,
// which would declare the same reads on a rig whose volumes are all in the
// closure.
static void
CheckPlacementReads(const UsdStageRefPtr &stage, const std::string &label,
                    const std::vector<SdfPath> &outside)
{
    const SdfPath rig("/PlacementAsset/Rig");
    const SdfPath smooth("/PlacementAsset/Rig/GeometryMovers/StripSmooth");
    RigExecRigEvaluator E(stage, rig);
    E.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    TfErrorMark mark;
    const bool compiled = E.Compile(&errors);
    const bool valid = compiled && E.Evaluate(UsdTimeCode(1.0)).valid;
    if (!compiled || !valid || E.GetBakedGenerationCount() != 1) {
        std::printf("FAIL %s: no baked generation (compiled %d, valid %d)\n",
                    label.c_str(), int(compiled), int(valid));
        for (const std::string &e : errors) {
            std::printf("  compile error: %s\n", e.c_str());
        }
        ++failures;
        return;
    }
    if (!mark.IsClean()) {
        for (const TfError &error : mark) {
            std::printf("FAIL %s: build posted: %s\n", label.c_str(),
                        error.GetCommentary().c_str());
        }
        ++failures;
        mark.Clear();
    }
    const std::vector<RigExecOpGraphNode> graph = E.GetOpGraph();
    const size_t smoothStatic = OneOpGraphStep(graph, "RevisionStatic", smooth);
    const size_t placeA = OneOpGraphStep(
        graph, "VolumePlacements",
        SdfPath("/PlacementAsset/Rig/Joints/A/SphereA"));
    const size_t placeB = OneOpGraphStep(
        graph, "VolumePlacements",
        SdfPath("/PlacementAsset/Rig/Joints/B/SphereB"));
    std::vector<size_t> placeOutside;
    for (const SdfPath &volume : outside) {
        placeOutside.push_back(
            OneOpGraphStep(graph, "VolumePlacements", volume));
    }
    if (smoothStatic == SIZE_MAX || placeA == SIZE_MAX ||
        placeB == SIZE_MAX) {
        return;
    }
    using Range = std::pair<uint32_t, uint32_t>;
    const auto placementsOf = [](const std::vector<RigExecOpSlotRange> &rs) {
        std::set<Range> out;
        for (const RigExecOpSlotRange &r : rs) {
            if (r.domain == "WeightFrames") {
                out.insert({r.first, r.last});
            }
        }
        return out;
    };
    std::set<Range> expected = placementsOf(graph[placeA].writes);
    const std::set<Range> writesB = placementsOf(graph[placeB].writes);
    CHECK(expected.size() == 1 && writesB.size() == 1);
    expected.insert(writesB.begin(), writesB.end());
    CHECK(expected.size() == 2);
    const RigExecOpGraphNode &reader = graph[smoothStatic];
    const std::set<Range> read = placementsOf(reader.reads);
    if (read != expected) {
        std::printf("FAIL %s: StripSmooth's RevisionStatic reads %zu "
                    "WeightFrames range(s), not exactly SphereA's and "
                    "SphereB's\n", label.c_str(), read.size());
        ++failures;
    }
    const std::set<size_t> preds(reader.preds.begin(), reader.preds.end());
    CHECK(preds.count(placeA) == 1);
    CHECK(preds.count(placeB) == 1);
    for (const size_t place : placeOutside) {
        if (place == SIZE_MAX) {
            continue;
        }
        const std::set<Range> writes = placementsOf(graph[place].writes);
        CHECK(writes.size() == 1);
        for (const Range &w : writes) {
            CHECK(!read.count(w));
        }
        CHECK(!preds.count(place));
    }
}

// tests/fixtures/volume_placements.usda: two volumes with one placement step
// each. SphereB rides B, which BAim revises after BFK, so its final
// placement (pose.weightFrames, the oracle) differs from its base placement
// (BSkin's packet). A and B are driven by separate solvers in separate
// hierarchies, so a drag on one control re-runs its own volume's placement
// step and not the other's.
static void
TestVolumePlacementsAreIndependent(const std::string &examplesDir)
{
    const char *label = "volume placements";
    const std::string stagePath =
        examplesDir + "/../tests/fixtures/volume_placements.usda";
    const SdfPath rig("/PlacementAsset/Rig");
    const SdfPath sphereA("/PlacementAsset/Rig/Joints/A/SphereA");
    const SdfPath sphereB("/PlacementAsset/Rig/Joints/B/SphereB");
    const SdfPath aCtl("/PlacementAsset/Rig/Controls/ACtl");
    const SdfPath bCtl("/PlacementAsset/Rig/Controls/BCtl");
    const SdfPath smooth("/PlacementAsset/Rig/GeometryMovers/StripSmooth");
    const SdfPath skin = smooth.AppendChild(TfToken("BSkin"));
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    // The same rig with BAim disabled: B's final frame equals its base.
    const UsdStageRefPtr unaimedStage = UsdStage::Open(stagePath);
    CHECK(stage && unaimedStage);
    if (!stage || !unaimedStage) {
        return;
    }
    unaimedStage->SetEditTarget(unaimedStage->GetSessionLayer());
    const UsdPrim aim = unaimedStage->GetPrimAtPath(
        SdfPath("/PlacementAsset/Rig/BFollow/BAim"));
    CHECK(aim && aim.GetAttribute(TfToken("inputs:enabled")).Set(false));

    std::vector<std::string> errors;
    const auto open = [&](const UsdStageRefPtr &s,
                          RigExecEvaluationMode mode) {
        auto E = std::make_unique<RigExecRigEvaluator>(s, rig);
        if (!E->Compile(&errors)) {
            for (const std::string &e : errors) {
                std::printf("FAIL %s: compile error: %s\n", label, e.c_str());
            }
            ++failures;
            return std::unique_ptr<RigExecRigEvaluator>();
        }
        E->SetEvaluationMode(mode);
        E->SetPublishWeightFields(true);
        return E;
    };
    const auto baked = open(stage, RigExecEvaluationMode::Baked);
    const auto walk = open(stage, RigExecEvaluationMode::Dynamic);
    const auto unaimed = open(unaimedStage, RigExecEvaluationMode::Dynamic);
    if (!baked || !walk || !unaimed) {
        return;
    }
    CHECK(baked->IsBakeable());
    baked->SetProfilingEnabled(true);

    const auto frameOf = [](const RigExecRigPose &pose, const SdfPath &path) {
        const auto found = pose.weightFrames.find(path);
        return found == pose.weightFrames.end() ? GfMatrix4d(0.0)
                                                : found->second;
    };
    for (const double frame : {1.0, 5.0, 10.0}) {
        const std::string where = TfStringPrintf("%s frame %g", label, frame);
        const size_t before = baked->GetBakedGenerationCount();
        const RigExecRigPose live = baked->Evaluate(UsdTimeCode(frame));
        const RigExecRigPose dynamic = walk->Evaluate(UsdTimeCode(frame));
        const RigExecRigPose base = unaimed->Evaluate(UsdTimeCode(frame));
        CHECK(live.valid && dynamic.valid && base.valid);
        CHECK(baked->GetBakedGenerationCount() == before + 1);
        rigExecTest::CompareEveryMap(&failures, where, dynamic, live);
        CHECK(live.weightFrames.size() == 2);
        // weightFrames is the FINAL placement: the aim moves SphereB's and
        // leaves SphereA's alone.
        CHECK(frameOf(live, sphereB) != frameOf(base, sphereB));
        CHECK(frameOf(live, sphereA) == frameOf(base, sphereA));
        // BSkin's packet is placed at SphereB's BASE frame, which the aim
        // does not revise: the same field with and without it.
        const auto field = live.weightFields.find(sphereB);
        const auto baseField = base.weightFields.find(sphereB);
        CHECK(field != live.weightFields.end() &&
              baseField != base.weightFields.end());
        if (field != live.weightFields.end() &&
            baseField != base.weightFields.end()) {
            CHECK(field->second.weights == baseField->second.weights);
            bool reaches = false;
            for (const float w : field->second.weights) {
                reaches = reaches || w > 0.0f;
            }
            CHECK(reaches);
        }
    }

    // Drags at frame 5, after a repeat of it. Each control reaches its own
    // volume's placement step and not the other's; the current-phase field
    // (StripSmooth) reads both placements and re-runs either way.
    CHECK(baked->Evaluate(UsdTimeCode(5.0)).valid);
    const RigExecRigPose still = walk->Evaluate(UsdTimeCode(5.0));
    struct Drag {
        SdfPath control, placed, kept;
        double rz;
    };
    for (const Drag &drag : {Drag{aCtl, sphereA, sphereB, 50.0},
                             Drag{bCtl, sphereB, sphereA, -40.0}}) {
        const std::string where =
            std::string(label) + " drag " + drag.control.GetName();
        const std::vector<RigExecValueOverride> overrides{
            RigExecValueOverride{drag.control, TfToken(),
                                 TfToken("avars:rz"), VtValue(drag.rz)}};
        baked->SetInteractiveOverrides(overrides);
        walk->SetInteractiveOverrides(overrides);
        baked->ClearProfile();
        const size_t before = baked->GetBakedGenerationCount();
        const RigExecRigPose live = baked->Evaluate(UsdTimeCode(5.0));
        const RigExecRigPose dynamic = walk->Evaluate(UsdTimeCode(5.0));
        CHECK(live.valid && dynamic.valid);
        CHECK(baked->GetBakedGenerationCount() == before + 1);
        rigExecTest::CompareEveryMap(&failures, where, dynamic, live);
        // The drag moved its own volume and nothing of the other.
        CHECK(frameOf(live, drag.placed) != frameOf(still, drag.placed));
        CHECK(frameOf(live, drag.kept) == frameOf(still, drag.kept));
        const std::vector<RigExecOpTraceEntry> trace = baked->GetLastOpTrace();
        // Exact labels: StripSmooth's is a prefix of BSkin's.
        const auto ran = [&trace](const std::string &kind,
                                  const SdfPath &path) {
            size_t count = 0;
            for (const RigExecOpTraceEntry &entry : trace) {
                count += entry.label == kind + " " + path.GetString();
            }
            return count;
        };
        CHECK(ran("VolumePlacements", drag.placed) == 1);
        CHECK(ran("VolumePlacements", drag.kept) == 0);
        CHECK(ran("RevisionStatic", smooth) == 1);
        // BSkin's steps run on every frame: its packet's step reads a pose
        // frame and so is always dirty (externalReads), and the cone is
        // structural. Whether the skin re-deformed is the value decision of
        // its assemble, and only a drag that reaches SphereB's base or B
        // makes it.
        bool skinExecuted = true, smoothExecuted = false;
        const RigExecBakedProgramImpl &B =
            baked->GetBakedProgram()->GetStepGraph();
        for (const auto &chain : B.chains) {
            for (const auto &revision : chain.revisions) {
                if (revision.moverPath == skin) {
                    skinExecuted = revision.executed;
                } else if (revision.moverPath == smooth) {
                    smoothExecuted = revision.executed;
                }
            }
        }
        CHECK(skinExecuted == (drag.control == bCtl));
        CHECK(smoothExecuted);
        std::printf("  %s: %zu step(s) ran, %zu revision(s) executed, "
                    "BSkin %s\n",
                    where.c_str(), trace.size(),
                    size_t(live.moverGraphRevisionsExecuted),
                    skinExecuted ? "re-deformed" : "kept");
        baked->ClearInteractiveOverrides();
        walk->ClearInteractiveOverrides();
        CHECK(baked->Evaluate(UsdTimeCode(5.0)).valid);
    }

    // The closure: on the shipped rig, and with SphereC, a volume outside
    // Both's closure, added in the session layer.
    CheckPlacementReads(stage, label, {});
    const UsdStageRefPtr thirdStage = UsdStage::Open(stagePath);
    CHECK(thirdStage);
    if (!thirdStage) {
        return;
    }
    thirdStage->SetEditTarget(thirdStage->GetSessionLayer());
    const SdfPath sphereC("/PlacementAsset/Rig/Weights/SphereC");
    const UsdPrim third =
        thirdStage->DefinePrim(sphereC, TfToken("RigExecSphereWeight"));
    CHECK(third);
    third.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
        .Set(2.0);
    third.CreateAttribute(TfToken("inputs:falloffMin"),
                          SdfValueTypeNames->Float).Set(0.5f);
    third.CreateAttribute(TfToken("inputs:falloffMax"),
                          SdfValueTypeNames->Float).Set(2.0f);
    CheckPlacementReads(thirdStage, std::string(label) + " with SphereC",
                        {sphereC});
}

static std::string
DefaultResourceDir()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return std::string();
#endif
}

int
main(int argc, char **argv)
{
    std::string resources = DefaultResourceDir();
    if (argc > 1 && resources.empty()) {
        resources = TfAbsPath(std::string(argv[1]) +
                              "/../plugin/rigExecSchema/resources");
    }
    if (!resources.empty()) {
        PlugRegistry::GetInstance().RegisterPlugins(resources);
    }

    TestSphereWeight();
    TestVolumePlacedByRestSpace();
    TestSphereAxisScales();
    TestSphereDirectionalScales();
    TestSphereDirectionalEdits();
    TestSphereDirectionalScaleValidation();
    TestVolumeIgnoresTransformScaleAvars();
    TestPlaneWeight();
    TestPlaneBounded();
    TestPlaneBoundsEpochSplit();
    TestPlaneBoundedInvalidExtents();
    TestCurveWeight();
    TestCombineMultiply();
    TestCombineMaxOfTwoSpheres();
    TestCombineSubtractOrder();
    TestCombineOfConstantInputs();
    TestDefaultCombineNeedsNoRepresentation();
    TestCurrentPhaseThroughCombine();
    TestCombineCycleSkipsMover();
    TestInvert();
    TestAuthoredFalloffCurve();
    TestSamplePhase(/* current */ false);
    TestSamplePhase(/* current */ true);
    TestVolumeWeightTargetMismatchSkipsMover();
    TestCurveWeightRejectsTwoCurves();
    TestVolumeWeightOnAConstraint("base");
    TestVolumeWeightOnAConstraint("preceding");
    TestVolumePlacementGate();
    TestTheOracleReadsTheProgramsPlacements();
    TestReleasedOverridePublishesTheProgramsPlacements();
    if (argc > 1) {
        TestVolumePlacementsAreIndependent(argv[1]);
    }

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecVolumeWeights: all tests passed\n");
    return 0;
}
