//
// RigExec end-to-end tests for volumetric weight objects (spec §4.1
// volumetric extension): sphere, plane, and curve fields driving real
// matrix movers, weight-object composition, the authored falloff spline,
// and the reference/current sample phases.
//
// Every case also asserts pose.moverGraphParityMismatches == 0, which is
// the assertion that actually matters: it means the OpenExec
// computeWeightPacket kernels and the CPU oracle independently computed
// the same field. A volumetric weight that only worked on one of those
// paths would still move points, just not the same points.
//
// argv[1] = path to the examples directory; the codeless schema plugin is
// expected at <examples>/../plugin/rigExecSchema/resources.
//
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/ts/knot.h"
#include "pxr/base/ts/spline.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <string>
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
        GfMatrix4d posed(1.0);
        posed.SetTranslate(translate);
        joint.CreateAttribute(TfToken("posed:space"),
                              SdfValueTypeNames->Matrix4d).Set(posed);
    }

    static SdfPath Target() { return SdfPath("/Asset/Geom/M.points"); }

    // Places a volume weight by authoring posed:space directly, the same
    // way the joint above is posed -- the avar path has its own coverage
    // and would only add noise here.
    UsdPrim MakeVolume(
        const char *name, const TfToken &type, const GfVec3d &at,
        float falloffMin, float falloffMax)
    {
        UsdPrim v = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name), type);
        GfMatrix4d posed(1.0);
        posed.SetTranslate(at);
        v.CreateAttribute(TfToken("posed:space"),
                          SdfValueTypeNames->Matrix4d).Set(posed);
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
        evaluator.cpuParityMode = true;
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
        // have computed the same field.
        if (pose.moverGraphParityMismatches != 0) {
            std::printf("%s: %zu graph/CPU parity MISMATCHES\n", label,
                        size_t(pose.moverGraphParityMismatches));
            ++failures;
        }
        if (pose.moverGraphParityAgreements == 0) {
            std::printf("%s: parity harness never ran\n", label);
            ++failures;
        }
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
//
// This is the case every other test here avoids, and it is the one that
// matters most: MakeVolume authors posed:space over an IDENTITY rest, and
// for that configuration the rest->posed delta happens to equal the
// desired placement. Author the placement on rest:space instead -- which
// is how the shipped example places its mid-body volumes -- and the two
// stop being the same thing.
//
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
    // posed:space unauthored, so the volume simply sits there.
    v.GetAttribute(TfToken("posed:space")).Clear();
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
//
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
//
// rigExec:planeBounds SELECTS WHICH FIELD FUNCTION RUNS, so it has to be
// hashed: an unhashed structural token leaves exec replaying the epoch's
// baked packet shape while the CPU oracle reads the live one. The
// extents are ordinary per-frame floats and must NOT be, because a digest
// that hashes an animatable value recompiles the whole rig on every
// mouse-move of the scrub row that drives it -- which is a rig-sized
// rebuild per frame, for a number exec re-reads for free.
//
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

    evaluator.cpuParityMode = true;
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
        if (pose.moverGraphParityMismatches != 0) {
            std::printf("plane-epoch %s: %zu parity MISMATCHES\n", label,
                        size_t(pose.moverGraphParityMismatches));
            ++failures;
        }
        if (pose.moverGraphParityAgreements == 0) {
            // Without this the whole case passes vacuously on a rig that
            // never built a revision at all.
            std::printf("plane-epoch %s: parity harness never ran\n", label);
            ++failures;
        }
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

    // ---- an ANIMATABLE edit: the field follows it and the epoch does not.
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

    // ---- a STRUCTURAL edit: it must re-epoch, or exec replays the
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
//
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

    evaluator.cpuParityMode = true;
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
    if (pose.moverGraphParityMismatches != 0) {
        std::printf("plane-bad-extent: %zu graph/CPU parity MISMATCHES\n",
                    size_t(pose.moverGraphParityMismatches));
        ++failures;
    }
    if (pose.moverGraphParityAgreements == 0) {
        // The rejection has to be the EXTENT, not a rig that never built
        // a revision: without this the case passes on any failure at all.
        std::printf("plane-bad-extent: parity harness never ran\n");
        ++failures;
    }
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
//
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
//
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

// rigExec:samplePhase decides WHICH points the distance is measured
// against, and the two answers genuinely differ: a volume the geometry has
// already been moved out of grabs nothing under `current` and still grabs
// under `reference`.
static void
TestSamplePhase(bool current)
{
    const VtVec3fArray base{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                            GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)};
    Fixture f(base, GfVec3d(0, 2, 0));

    // A second joint that lifts everything +5Y first, unweighted.
    UsdPrim lift = f.stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Lift"),
                                       TfToken("RigExecJoint"));
    GfMatrix4d lifted(1.0);
    lifted.SetTranslate(GfVec3d(0, 5, 0));
    lift.CreateAttribute(TfToken("posed:space"),
                         SdfValueTypeNames->Matrix4d).Set(lifted);
    UsdPrim full = f.MakeStaticWeight(
        "Full", VtFloatArray{1.0f, 1.0f, 1.0f, 1.0f});

    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 3.0f);
    v.CreateAttribute(TfToken("rigExec:samplePhase"),
                      SdfValueTypeNames->Token)
        .Set(TfToken(current ? "current" : "reference"));

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
        f.Resolve(current ? "samplePhase-current" : "samplePhase-reference");
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
// mover's points target must fail compilation, exactly as an authored one
// does -- the validation is type-agnostic and this proves it stayed so.
static void
TestVolumeWeightTargetMismatchFailsCompile()
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

    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
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

    evaluator.cpuParityMode = true;
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
    GfMatrix4d lifted(1.0);
    lifted.SetTranslate(GfVec3d(0, 5, 0));
    lift.CreateAttribute(TfToken("posed:space"),
                         SdfValueTypeNames->Matrix4d).Set(lifted);
    UsdPrim full = f.MakeStaticWeight(
        "Full", VtFloatArray{1.0f, 1.0f, 1.0f, 1.0f});

    UsdPrim v = f.MakeVolume("Sphere", TfToken("RigExecSphereWeight"),
                             GfVec3d(0, 0, 0), 0.0f, 3.0f);
    v.CreateAttribute(TfToken("rigExec:samplePhase"),
                      SdfValueTypeNames->Token).Set(TfToken("current"));

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
TestCombineCycleFailsCompile()
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

    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(!evaluator.Compile(&errors));
    CHECK(!errors.empty());
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
    TestCombineCycleFailsCompile();
    TestInvert();
    TestAuthoredFalloffCurve();
    TestSamplePhase(/* current */ false);
    TestSamplePhase(/* current */ true);
    TestVolumeWeightTargetMismatchFailsCompile();
    TestCurveWeightRejectsTwoCurves();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecVolumeWeights: all tests passed\n");
    return 0;
}
