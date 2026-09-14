//
// The pose-interpolator phase: the conventional poseInterpolator, evaluated.
//
// A RigExecPoseInterpolator reads the FINAL local rotation of a driver and
// publishes one float per authored RigExecPose. It is not a mover -- a
// mover's inputs are resolved by the property chains, which run before exec
// does and therefore cannot see the pose -- so it is its own phase of the
// evaluate, sitting between the pose walk that produces the driver's frame
// and the geometry chains that consume the weights.
//
// What each case here is protecting, in the order a failure would be found:
//
//   * the weights themselves, at rest and at a pose. An interpolator
//     standing on one of its own poses must read 1.000000 there and zero
//     everywhere else: that is what the radial basis function interpolates
//     exactly, and a neutral that does not read 1 with the rig standing
//     still is the failure mode that moved the biped's skin two millimetres
//     before the poses were rebased.
//
//   * LOCAL, not world. The driver's parent is rotated underneath it and the
//     weights must not move. This is the one that catches a
//     `world * parent^-1` written the other way round, which is invisible on
//     a rig whose driver sits at the root and wrong on every rig that does
//     not.
//
//   * THE CONSUMER, and with it the ordering. A RigExecBlendInput's
//     inputs:weight carries a single authored connection to
//     <pose>.outputs:weight, and RigExecResolvedInputs::GetAttribute follows
//     it into the values this phase resolved in memory. If the phase ran
//     after the geometry chains instead of before them, that read would be
//     answered with the attribute's AUTHORED zero -- no error raised
//     anywhere, every corrective silently off -- so the deformation is
//     measured here rather than assumed.
//
//   * the two enables, which the schema deliberately treats differently: a
//     disabled INTERPOLATOR publishes zeros (shape-preserving: a corrective
//     that is off has to be off, not frozen), while a disabled POSE is left
//     out of the solve entirely, because leaving it in would keep it in
//     every other pose's matrix row and switching one off would quietly
//     change all the others. The second of those needs a new binding epoch,
//     so it is also the check that the epoch digest hashes a pose's enable.
//
//   * parity with libs/rigExecMath/rbf.h driven directly. The solver is
//     already pinned against the studio's Python at 3.41e-11
//     (testRigExecRbf); what this adds is that the EVALUATOR hands it the
//     rotation it is supposed to.
//
#include "rigExec/rigEvaluator.h"
#include "rigExecMath/rbf.h"

#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

using namespace rigExec;

static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

namespace {

const SdfPath kRig("/Asset/Rig");
const SdfPath kParent("/Asset/Rig/Controls/Shoulder");
const SdfPath kDriver("/Asset/Rig/Controls/Shoulder/Driver");
const SdfPath kInterpolator("/Asset/Rig/PoseInterpolators/Swing");
const SdfPath kMesh("/Asset/Geom/Mesh.points");

// 45 degrees, which is both the spacing of the poses and their falloff width:
// wide enough that the kernels meet in the middle, narrow enough that each
// pose still owns its own place.
constexpr double kSpacing = 0.78539816339744830961;
constexpr float kDelta = 10.0f;   ///< what the corrective moves the mesh by

SdfPath
PoseWeight(const char *name)
{
    return kInterpolator.AppendChild(TfToken(name))
        .AppendProperty(TfToken("outputs:weight"));
}

GfQuatf
AboutZ(double degrees)
{
    const double half = GfDegreesToRadians(degrees) * 0.5;
    return GfQuatf(float(std::cos(half)),
                   GfVec3f(0.0f, 0.0f, float(std::sin(half))));
}

VtVec3fArray
BasePoints()
{
    return VtVec3fArray{GfVec3f(0.0f, 0.0f, 0.0f),
                        GfVec3f(1.0f, 0.0f, 0.0f)};
}

VtVec3fArray
TargetPoints()
{
    VtVec3fArray points = BasePoints();
    for (GfVec3f &p : points) {
        p[2] += kDelta;
    }
    return points;
}

// Two nested controls, an interpolator on the inner one with three poses
// 45 degrees apart about Z, and a blend channel wired to the FORWARD pose's
// weight so the mesh says what the weight was.
//
//   Shoulder            the driver's parent, so the delta is a LOCAL one
//     Driver            avars:rz is the whole of the input
//   Swing               neutral / Forward (+45) / Back (-45), whole-rotation
//   Movers/Blend        one channel, one sample, +10 in Z
UsdStageRefPtr
MakeRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(kRig, TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));
    stage->DefinePrim(kParent, TfToken("RigExecControl"));
    stage->DefinePrim(kDriver, TfToken("RigExecControl"));

    const UsdPrim mesh = stage->DefinePrim(kMesh.GetPrimPath(),
                                           TfToken("Mesh"));
    mesh.GetAttribute(TfToken("points")).Set(BasePoints());
    const UsdPrim target = stage->DefinePrim(SdfPath("/Asset/Targets/Full"),
                                             TfToken("Points"));
    target.CreateAttribute(TfToken("points"),
                           SdfValueTypeNames->Point3fArray)
        .Set(TargetPoints());

    stage->DefinePrim(SdfPath("/Asset/Rig/PoseInterpolators"),
                      TfToken("Scope"));
    const UsdPrim interpolator =
        stage->DefinePrim(kInterpolator, TfToken("RigExecPoseInterpolator"));
    interpolator.CreateRelationship(TfToken("rigExec:driver"))
        .SetTargets({kDriver});
    interpolator.GetAttribute(TfToken("rigExec:kernel"))
        .Set(TfToken("gaussian"));
    interpolator.GetAttribute(TfToken("rigExec:twistAxis")).Set(TfToken("Z"));

    const auto addPose = [&](const char *name, double degrees) {
        const UsdPrim pose = stage->DefinePrim(
            kInterpolator.AppendChild(TfToken(name)), TfToken("RigExecPose"));
        // Whole rotations: the point of this rig is the delta, not the
        // swing/twist split, which testRigExecRbf already pins.
        pose.GetAttribute(TfToken("rigExec:poseType")).Set(TfToken("whole"));
        pose.GetAttribute(TfToken("rigExec:rotation")).Set(AboutZ(degrees));
        pose.GetAttribute(TfToken("rigExec:rotationRadius"))
            .Set(float(kSpacing));
        return pose;
    };
    addPose("neutral", 0.0);
    const UsdPrim forward = addPose("Forward", 45.0);
    addPose("Back", -45.0);

    // The consumer. inputs:weight is CONNECTED rather than authored: that
    // connection is the whole path from the phase to the deformation.
    const UsdPrim input = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/Corrective"),
        TfToken("RigExecBlendInput"));
    UsdAttribute weight = input.GetAttribute(TfToken("inputs:weight"));
    weight.Set(0.0f);
    weight.AddConnection(
        forward.GetPath().AppendProperty(TfToken("outputs:weight")));
    const UsdPrim sample = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/Corrective/Full"),
        TfToken("RigExecBlendSample"));
    sample.GetAttribute(TfToken("rigExec:activation")).Set(1.0f);
    sample.CreateRelationship(TfToken("rigExec:targetPoints"))
        .SetTargets({SdfPath("/Asset/Targets/Full.points")});
    input.CreateRelationship(TfToken("rigExec:samples"))
        .SetTargets({sample.GetPath()});

    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Blend"), TfToken("RigExecBlendShapeMover"));
    blend.ApplyAPI(TfToken("RigExecMoverAPI"));
    blend.GetRelationship(TfToken("rigExec:moves")).SetTargets({kMesh});
    blend.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    blend.CreateRelationship(TfToken("rigExec:blendInputs"))
        .SetTargets({input.GetPath()});
    return stage;
}

// The same three poses handed straight to the solver, so the evaluator's
// answer can be compared against the maths rather than against itself.
RigExecRbfSolver
ReferenceSolver()
{
    RigExecRbfSolverDesc desc;
    desc.kernel = RigExecRbfKernel::Gaussian;
    desc.twistAxis = GfVec3d(0.0, 0.0, 1.0);
    // Spelled the way the evaluator spells it: the authored quatf, widened,
    // and through RigExecRbfEulerFromQuaternion. Not "the same rotation" --
    // the same floating-point value, which is what a 1e-6 parity needs.
    for (double degrees : {0.0, 45.0, -45.0}) {
        const GfQuatf q = AboutZ(degrees);
        desc.poses.push_back(RigExecRbfEulerFromQuaternion(
            GfQuatd(q.GetReal(), GfVec3d(q.GetImaginary()))));
    }
    desc.poseTypes.assign(3, RigExecRbfPoseType::Whole);
    RigExecRbfSolver solver(desc);
    solver.SetSolvedTable(std::vector<double>(3, kSpacing), {}, {});
    solver.Solve();
    return solver;
}

bool
CompileOrReport(RigExecRigEvaluator *evaluator)
{
    std::vector<std::string> errors;
    if (!evaluator->Compile(&errors)) {
        ++failures;
        for (const std::string &error : errors) {
            std::printf("FAIL compile: %s\n", error.c_str());
        }
        return false;
    }
    return true;
}

// One published weight, or a NaN that fails every comparison below rather
// than a zero that would pass some of them.
double
Weight(const RigExecRigPose &pose, const char *name)
{
    const auto found = pose.movedProperties.find(PoseWeight(name));
    if (found == pose.movedProperties.end() ||
        !found->second.IsHolding<float>()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return found->second.UncheckedGet<float>();
}

void
CheckWeights(const RigExecRigPose &pose, double neutral, double forward,
             double back, const char *what, double tolerance = 1e-5)
{
    CHECK(pose.valid);
    const double gotNeutral = Weight(pose, "neutral");
    const double gotForward = Weight(pose, "Forward");
    const double gotBack = Weight(pose, "Back");
    if (std::abs(gotNeutral - neutral) > tolerance ||
        std::abs(gotForward - forward) > tolerance ||
        std::abs(gotBack - back) > tolerance) {
        ++failures;
        std::printf("FAIL %s: neutral %.8f (want %.8f), Forward %.8f (want "
                    "%.8f), Back %.8f (want %.8f)\n",
                    what, gotNeutral, neutral, gotForward, forward, gotBack,
                    back);
    }
}

// What the corrective did to the mesh, in Z, which is weight * kDelta when
// the channel resolved through its connection and 0 when it did not.
double
MeshDisplacement(const RigExecRigPose &pose)
{
    const auto found = pose.movedProperties.find(kMesh);
    if (found == pose.movedProperties.end() ||
        !found->second.IsHolding<VtVec3fArray>()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const VtVec3fArray points = found->second.UncheckedGet<VtVec3fArray>();
    if (points.size() != 2) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return points[0][2];
}

void
SetRotation(const UsdStageRefPtr &stage, const SdfPath &control,
            double degrees)
{
    stage->GetPrimAtPath(control).GetAttribute(TfToken("avars:rz"))
        .Set(degrees);
}

// --------------------------------------------------------------------------

// A rig standing still reads its neutral at 1 and everything else at 0, and
// a rig standing on a pose reads THAT pose at 1. Exactly, not approximately:
// the basis function interpolates its own poses.
void
TestTheWeightsAtRestAndAtAPose()
{
    UsdStageRefPtr stage = MakeRig();
    RigExecRigEvaluator evaluator(stage, kRig);
    if (!CompileOrReport(&evaluator)) {
        return;
    }
    CheckWeights(evaluator.Evaluate(UsdTimeCode::Default()),
                 1.0, 0.0, 0.0, "rest");

    SetRotation(stage, kDriver, 45.0);
    CheckWeights(evaluator.Evaluate(UsdTimeCode::Default()),
                 0.0, 1.0, 0.0, "driver on Forward");

    SetRotation(stage, kDriver, -45.0);
    CheckWeights(evaluator.Evaluate(UsdTimeCode::Default()),
                 0.0, 0.0, 1.0, "driver on Back");

    // Half way between neutral and Forward the two share the rig and Back is
    // pushed negative -- which is a real instruction to lean away from it,
    // not an error, and is why allowNegativeWeights defaults on. The three
    // still sum to one; that is what normalization is.
    SetRotation(stage, kDriver, 22.5);
    const RigExecRigPose between = evaluator.Evaluate(UsdTimeCode::Default());
    const double sum = Weight(between, "neutral") +
                       Weight(between, "Forward") + Weight(between, "Back");
    CHECK(std::abs(sum - 1.0) < 1e-5);
    CHECK(Weight(between, "neutral") > 0.3);
    CHECK(Weight(between, "Forward") > 0.3);
}

// The delta is the driver's LOCAL rotation. Rotating its parent moves the
// driver through the world and must not move a single weight.
void
TestTheDeltaIsLocalToTheDriversParent()
{
    UsdStageRefPtr stage = MakeRig();
    RigExecRigEvaluator evaluator(stage, kRig);
    if (!CompileOrReport(&evaluator)) {
        return;
    }
    SetRotation(stage, kDriver, 45.0);
    const RigExecRigPose alone = evaluator.Evaluate(UsdTimeCode::Default());
    CheckWeights(alone, 0.0, 1.0, 0.0, "driver on Forward, parent at rest");

    SetRotation(stage, kParent, 30.0);
    const RigExecRigPose carried = evaluator.Evaluate(UsdTimeCode::Default());
    CheckWeights(carried, 0.0, 1.0, 0.0,
                 "driver on Forward, parent turned 30 degrees");

    // And the parent alone leaves the driver on its neutral.
    SetRotation(stage, kDriver, 0.0);
    CheckWeights(evaluator.Evaluate(UsdTimeCode::Default()),
                 1.0, 0.0, 0.0, "parent turned, driver at its rest");
}

// The weight reaches the deformation, through the connection and in the
// right order. This is the ordering assertion made behavioural: a phase
// running after the chains would leave the mesh at rest with no diagnostic.
void
TestTheWeightReachesTheBlendChannel()
{
    UsdStageRefPtr stage = MakeRig();
    RigExecRigEvaluator evaluator(stage, kRig);
    if (!CompileOrReport(&evaluator)) {
        return;
    }
    const RigExecRigPose rest = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(std::abs(MeshDisplacement(rest)) < 1e-5);

    SetRotation(stage, kDriver, 45.0);
    const RigExecRigPose posed = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(std::abs(MeshDisplacement(posed) - kDelta) < 1e-4);

    // And part way, so the mesh is following the weight rather than a
    // threshold: the displacement is the Forward weight times the delta.
    SetRotation(stage, kDriver, 22.5);
    const RigExecRigPose between = evaluator.Evaluate(UsdTimeCode::Default());
    const double expected = Weight(between, "Forward") * kDelta;
    CHECK(std::abs(MeshDisplacement(between) - expected) < 1e-3);
    CHECK(MeshDisplacement(between) > 0.3 * kDelta);
}

// A disabled interpolator publishes zeros -- all of them, including the
// neutral -- and the corrective goes with it.
void
TestADisabledInterpolatorPublishesZeros()
{
    UsdStageRefPtr stage = MakeRig();
    RigExecRigEvaluator evaluator(stage, kRig);
    if (!CompileOrReport(&evaluator)) {
        return;
    }
    SetRotation(stage, kDriver, 45.0);
    CheckWeights(evaluator.Evaluate(UsdTimeCode::Default()),
                 0.0, 1.0, 0.0, "enabled");

    stage->GetPrimAtPath(kInterpolator)
        .GetAttribute(TfToken("inputs:enabled")).Set(false);
    const RigExecRigPose off = evaluator.Evaluate(UsdTimeCode::Default());
    CheckWeights(off, 0.0, 0.0, 0.0, "interpolator disabled");
    CHECK(std::abs(MeshDisplacement(off)) < 1e-5);

    stage->GetPrimAtPath(kInterpolator)
        .GetAttribute(TfToken("inputs:enabled")).Set(true);
    CheckWeights(evaluator.Evaluate(UsdTimeCode::Default()),
                 0.0, 1.0, 0.0, "enabled again");
}

// A disabled POSE is left out of the solve, which changes what the others
// read -- so it has to reach a recompile. The epoch digest hashes it for
// exactly this reason; without that the solve keeps its stale inverse and
// this test reads the three-pose answer forever.
void
TestADisabledPoseLeavesTheSolve()
{
    UsdStageRefPtr stage = MakeRig();
    RigExecRigEvaluator evaluator(stage, kRig);
    if (!CompileOrReport(&evaluator)) {
        return;
    }
    SetRotation(stage, kDriver, 45.0);
    CheckWeights(evaluator.Evaluate(UsdTimeCode::Default()),
                 0.0, 1.0, 0.0, "three poses");

    stage->GetPrimAtPath(kInterpolator.AppendChild(TfToken("Back")))
        .GetAttribute(TfToken("inputs:enabled")).Set(false);
    const RigExecRigPose two = evaluator.Evaluate(UsdTimeCode::Default());
    // Back is silenced outright, and Forward still owns its own pose.
    CheckWeights(two, 0.0, 1.0, 0.0, "Back disabled, driver on Forward");

    // Between the two surviving poses the answer is NOT what three poses
    // gave, which is the observable half of "left out of the solve entirely".
    SetRotation(stage, kDriver, 22.5);
    const RigExecRigPose twoBetween =
        evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(std::abs(Weight(twoBetween, "Back")) < 1e-9);
    CHECK(std::abs(Weight(twoBetween, "neutral") +
                   Weight(twoBetween, "Forward") - 1.0) < 1e-5);
}

// The evaluator's weights against the solver driven directly. testRigExecRbf
// pins the solver against the studio's Python; this pins the rotation the
// evaluator feeds it.
void
TestParityWithTheSolverDrivenDirectly()
{
    UsdStageRefPtr stage = MakeRig();
    RigExecRigEvaluator evaluator(stage, kRig);
    if (!CompileOrReport(&evaluator)) {
        return;
    }
    const RigExecRbfSolver reference = ReferenceSolver();
    for (double degrees : {0.0, 10.0, 22.5, 45.0, -33.0, 70.0}) {
        SetRotation(stage, kDriver, degrees);
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        const GfQuatf q = AboutZ(degrees);
        std::vector<double> expected;
        reference.Evaluate(
            RigExecRbfEulerFromQuaternion(
                GfQuatd(q.GetReal(), GfVec3d(q.GetImaginary()))),
            nullptr, &expected, true);
        CHECK(expected.size() == 3);
        if (expected.size() != 3) {
            continue;
        }
        // 1e-6, not tighter: the evaluator publishes a FLOAT, because
        // inputs:weight is a float and RigExecResolvedInputs answers only
        // the type the value actually holds.
        const char *names[3] = {"neutral", "Forward", "Back"};
        for (size_t i = 0; i < 3; ++i) {
            const double got = Weight(pose, names[i]);
            if (std::abs(got - expected[i]) > 1e-6) {
                ++failures;
                std::printf("FAIL parity at %.1f deg, %s: %.9f vs %.9f\n",
                            degrees, names[i], got, expected[i]);
            }
        }
    }
}

// Structural refusals. Each of these is a rig that would otherwise compile
// and drive nothing, which is the failure this whole phase exists to end.
void
TestStructuralValidation()
{
    {   // A driver that publishes no frame.
        UsdStageRefPtr stage = MakeRig();
        stage->DefinePrim(SdfPath("/Asset/Rig/NotAProvider"),
                          TfToken("Scope"));
        stage->GetPrimAtPath(kInterpolator)
            .GetRelationship(TfToken("rigExec:driver"))
            .SetTargets({SdfPath("/Asset/Rig/NotAProvider")});
        RigExecRigEvaluator evaluator(stage, kRig);
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(!errors.empty());
    }
    {   // No driver at all.
        UsdStageRefPtr stage = MakeRig();
        stage->GetPrimAtPath(kInterpolator)
            .GetRelationship(TfToken("rigExec:driver")).SetTargets({});
        RigExecRigEvaluator evaluator(stage, kRig);
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
    }
    {   // A child that is not a pose.
        UsdStageRefPtr stage = MakeRig();
        stage->DefinePrim(kInterpolator.AppendChild(TfToken("Stray")),
                          TfToken("Scope"));
        RigExecRigEvaluator evaluator(stage, kRig);
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
    }
    {   // outputs:weight is a source; an authored connection on it says
        // otherwise.
        UsdStageRefPtr stage = MakeRig();
        stage->GetPrimAtPath(kInterpolator.AppendChild(TfToken("Forward")))
            .GetAttribute(TfToken("outputs:weight"))
            .AddConnection(PoseWeight("Back"));
        RigExecRigEvaluator evaluator(stage, kRig);
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
    }
}

}  // namespace

int
main()
{
    // The codeless schema has to be loadable before a RigExecPose means
    // anything; ctest runs this with no plugin path set.
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);

    TestTheWeightsAtRestAndAtAPose();
    TestTheDeltaIsLocalToTheDriversParent();
    TestTheWeightReachesTheBlendChannel();
    TestADisabledInterpolatorPublishesZeros();
    TestADisabledPoseLeavesTheSolve();
    TestParityWithTheSolverDrivenDirectly();
    TestStructuralValidation();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecPoseInterpolator: all tests passed\n");
    return 0;
}
