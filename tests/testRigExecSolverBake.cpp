// Canonical solver execution on focused fixtures. Numeric assertions check
// solver formulas; separately initialized programs and frozen jobs check
// cache freshness and deterministic publication.
// They are also where the authoring errors live. Every solver computation
// answers a malformed binding with an empty aggregate and a warning that
// never reaches the pose, so the only observable consequence is that the
// joints it names fall back to their rest chains -- and the bake has to
// answer with the same empty aggregate rather than with a partial solve or a
// refusal. No shipped example is malformed, so these are the only rigs in
// the tree where that agreement is checked at all.
// argv[1] = path to the examples directory (for the schema plugin).
// argv[2] = --space-rest-candidates lists the space-rest refresh's
// candidate solvers instead of running the suite.
#include "rigExecExampleFixtures.h"
#include "rigExecPoseCompare.h"

#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

const SdfPath kRigPath("/Asset/Rig");

/// A rig builder, so each fixture can be built twice over.
using MakeStage = std::function<UsdStageRefPtr()>;

// The check every fixture is run through.

/// Separate canonical evaluators repeat the same input history and must
/// publish exactly equal generations. This checks deterministic publication
/// and cache behavior; the numeric solver checks below establish correctness.
void
CheckEvaluatorConsistency(const char *what, const MakeStage &make,
            const std::vector<double> &frames, bool expectBaked,
            bool guides = true)
{
    const UsdStageRefPtr baselineStage = make();
    const UsdStageRefPtr bakedStage = make();
    CHECK(baselineStage && bakedStage);
    if (!baselineStage || !bakedStage) return;

    RigExecRigEvaluator baseline(baselineStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    // A separately initialized canonical evaluator supplies the baseline.

    std::vector<std::string> errors;
    if (!baseline.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }

    // A headless consumer skips the guide request outright, and the program
    // has to skip its publication with it -- while still running the solvers
    // it would have published, because the toggle can move without the epoch
    // moving.
    baseline.SetSolverGuidesEnabled(guides);
    baked.SetSolverGuidesEnabled(guides);

    std::vector<std::string> reasons;
    const bool bakeable = baked.IsBakeable(&reasons);
    if (bakeable != expectBaked) {
        ++failures;
        std::printf("FAIL %s: the rig is %sbakeable\n", what,
                    bakeable ? "" : "not ");
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
    }

    // Each frame twice, then the list backwards: a repeated frame is what
    // makes a run skip steps, and a backwards sweep is what makes it skip
    // different ones. A cone that publishes last frame's answer shows up
    // here and nowhere else.
    std::vector<double> sweep = frames;
    sweep.insert(sweep.end(), frames.rbegin(), frames.rend());
    sweep.insert(sweep.end(), frames.begin(), frames.end());
    size_t generations = 0;
    for (const double frame : sweep) {
        const std::string where =
            std::string(what) + " frame " + std::to_string(frame);
        const RigExecRigPose a = baseline.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose b = baked.Evaluate(UsdTimeCode(frame));
        CHECK(a.valid && b.valid);

        ++generations;
        // The two evaluators are at the same point in their lives -- the
        // same fixture, the same frames, in the same order -- so the mover
        // graph counters are comparable and the whole generation is.
        rigExecTest::ComparePose(&failures, where, a, b);
    }
    if (expectBaked && baked.GetBakedGenerationCount() != generations) {
        ++failures;
        std::printf("FAIL %s: %zu of %zu generation(s) came from the "
                    "program\n", what, baked.GetBakedGenerationCount(),
                    generations);
    }
}

/// Compares canonical evaluators while an interactive override stands on
/// \p prim.\p attribute -- the drag a gizmo makes, on a solver's own input.
///
/// Every per-frame input a new solver reads has to reach the program through
/// the binding table, or a drag that lands on it will be answered with the
/// value the bake captured. That is invisible to a frame sweep, because the
/// authored value is what a sweep reads.
void
CheckDrag(const char *what, const MakeStage &make, const SdfPath &prim,
          const char *attribute, const VtValue &held)
{
    const UsdStageRefPtr baselineStage = make();
    const UsdStageRefPtr bakedStage = make();
    CHECK(baselineStage && bakedStage);
    if (!baselineStage || !bakedStage) return;

    RigExecRigEvaluator baseline(baselineStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    std::vector<std::string> errors;
    if (!baseline.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        return;
    }


    const std::vector<RigExecValueOverride> drag{
        RigExecValueOverride{prim, TfToken(), TfToken(attribute), held}};
    // Settled, then held, then released: the release is the half a drag test
    // usually forgets, and the one a value captured at Build survives.
    const RigExecRigPose settledA = baseline.Evaluate(UsdTimeCode(2.0));
    const RigExecRigPose settledB = baked.Evaluate(UsdTimeCode(2.0));
    rigExecTest::ComparePose(&failures, std::string(what) + " settled",
                             settledA, settledB);
    baseline.SetInteractiveOverrides(drag);
    baked.SetInteractiveOverrides(drag);
    const RigExecRigPose heldA = baseline.Evaluate(UsdTimeCode(2.0));
    const RigExecRigPose heldB = baked.Evaluate(UsdTimeCode(2.0));
    rigExecTest::ComparePose(&failures, std::string(what) + " held", heldA,
                             heldB);
    baseline.ClearInteractiveOverrides();
    baked.ClearInteractiveOverrides();
    const RigExecRigPose freedA = baseline.Evaluate(UsdTimeCode(2.0));
    const RigExecRigPose freedB = baked.Evaluate(UsdTimeCode(2.0));
    rigExecTest::ComparePose(&failures, std::string(what) + " released",
                             freedA, freedB);
    // The drag has to MOVE something, or the comparison above is two
    // identical generations agreeing about nothing.
    if (heldA.solverFrames == settledA.solverFrames &&
        heldA.jointFramesFinal == settledA.jointFramesFinal) {
        ++failures;
        std::printf("FAIL %s: the drag moved nothing on the dynamic path\n",
                    what);
    }
    // And all three generations have to come FROM the program. An override
    // the program cannot place sends the generation down the dynamic path,
    // where the two sides agree for the wrong reason.
    if (baked.GetBakedGenerationCount() != 3) {
        ++failures;
        std::printf("FAIL %s: %zu of 3 generation(s) came from the "
                    "program\n", what, baked.GetBakedGenerationCount());
    }
}

// Authoring helpers.

UsdPrim
Define(const UsdStageRefPtr &stage, const char *path, const char *type)
{
    return stage->DefinePrim(SdfPath(path), TfToken(type));
}

void
SetTargets(const UsdPrim &prim, const char *relationship,
           const SdfPathVector &targets)
{
    UsdRelationship rel = prim.GetRelationship(TfToken(relationship));
    if (!rel) rel = prim.CreateRelationship(TfToken(relationship));
    rel.SetTargets(targets);
}

/// A rest transform with its origin at \p y up the world Y axis.
GfMatrix4d
RestAt(double y)
{
    GfMatrix4d m(1.0);
    m.SetTranslateOnly(GfVec3d(0, y, 0));
    return m;
}

/// The stage skeleton every fixture starts from: a root, two FK controls and
/// the two joints they pose, keyed so the frames of a sweep differ.
UsdStageRefPtr
MakeFkSpine()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    Define(stage, "/Asset", "Xform");
    Define(stage, "/Asset/Rig", "RigExecRoot");

    const UsdPrim rootCtl =
        Define(stage, "/Asset/Rig/Controls/RootCtl", "RigExecControl");
    const UsdPrim chestCtl =
        Define(stage, "/Asset/Rig/Controls/ChestCtl", "RigExecControl");
    chestCtl.GetAttribute(TfToken("rest:space")).Set(RestAt(4.0));
    // Keyed, and on two channels, so that a frame moves both the twist the
    // distribution unwraps and the bend the ribbon's driver follows.
    UsdAttribute rz = chestCtl.GetAttribute(TfToken("avars:rz"));
    rz.Set(0.0, UsdTimeCode(1.0));
    rz.Set(25.0, UsdTimeCode(3.0));
    rz.Set(-10.0, UsdTimeCode(5.0));
    UsdAttribute rspin = chestCtl.GetAttribute(TfToken("avars:rspin"));
    rspin.Set(0.0, UsdTimeCode(1.0));
    rspin.Set(68.755, UsdTimeCode(5.0));

    const UsdPrim root =
        Define(stage, "/Asset/Rig/Joints/Root", "RigExecJoint");
    const UsdPrim chest =
        Define(stage, "/Asset/Rig/Joints/Root/Chest", "RigExecJoint");
    chest.GetAttribute(TfToken("rest:space")).Set(RestAt(4.0));

    const UsdPrim fk = Define(stage, "/Asset/Rig/Solvers/SpineFK",
                              "RigExecFkChain");
    SetTargets(fk, "rigExec:controls",
               {rootCtl.GetPath(), chestCtl.GetPath()});
    SetTargets(fk, "rigExec:joints", {root.GetPath(), chest.GetPath()});
    return stage;
}

// RigExecTwistDistribution.

/// The spine plus a twist distribution posing a middle joint, which is
/// example 05's solver with its ribbon and its geometry left off.
UsdStageRefPtr
MakeTwistRig()
{
    const UsdStageRefPtr stage = MakeFkSpine();
    const UsdPrim mid =
        Define(stage, "/Asset/Rig/Joints/TwistMid", "RigExecJoint");
    mid.GetAttribute(TfToken("rest:space")).Set(RestAt(2.0));

    const UsdPrim twist = Define(stage, "/Asset/Rig/Solvers/SpineTwist",
                                 "RigExecTwistDistribution");
    twist.GetAttribute(TfToken("rigExec:count")).Set(5);
    twist.GetAttribute(TfToken("rigExec:weights"))
        .Set(VtFloatArray{0.0f, 0.25f, 0.5f, 0.75f, 1.0f});
    SetTargets(twist, "rigExec:start",
               {SdfPath("/Asset/Rig/Joints/Root")});
    SetTargets(twist, "rigExec:end",
               {SdfPath("/Asset/Rig/Joints/Root/Chest")});
    SetTargets(twist, "rigExec:joints", {mid.GetPath()});
    twist.GetAttribute(TfToken("rigExec:jointElements")).Set(VtIntArray{2});
    return stage;
}

/// The same, with the extra winding keyed: inputs:twistTurns is the one
/// per-frame input the distribution reads off its own prim, so a keyed one
/// is what proves the bake bound it rather than folding it.
UsdStageRefPtr
MakeTwistRigWithKeyedTurns()
{
    const UsdStageRefPtr stage = MakeTwistRig();
    UsdAttribute turns =
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineTwist"))
            .GetAttribute(TfToken("inputs:twistTurns"));
    turns.Set(0.0, UsdTimeCode(1.0));
    turns.Set(1.5, UsdTimeCode(5.0));
    return stage;
}

/// Nothing authored on rigExec:weights, so the count decides the sample
/// positions -- the ramp the shared kernel owns.
UsdStageRefPtr
MakeTwistRigWithUnauthoredWeights()
{
    const UsdStageRefPtr stage = MakeTwistRig();
    const UsdPrim twist =
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineTwist"));
    twist.GetAttribute(TfToken("rigExec:weights")).Set(VtFloatArray{});
    twist.GetAttribute(TfToken("rigExec:count")).Set(3);
    twist.GetAttribute(TfToken("rigExec:jointElements")).Set(VtIntArray{1});
    return stage;
}

/// A single sample: `count` of one is the branch of the ramp that answers a
/// lone position at the start rather than dividing by zero.
UsdStageRefPtr
MakeTwistRigWithOneSample()
{
    const UsdStageRefPtr stage = MakeTwistRigWithUnauthoredWeights();
    const UsdPrim twist =
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineTwist"));
    twist.GetAttribute(TfToken("rigExec:count")).Set(1);
    twist.GetAttribute(TfToken("rigExec:jointElements")).Set(VtIntArray{0});
    return stage;
}

/// rigExec:end wired to nothing. The computation's end frame is a REQUIRED
/// input, so it publishes an empty aggregate and the joint falls back to its
/// rest chain with the diagnostic that carries -- which the program has to
/// reproduce rather than solve half a distribution.
UsdStageRefPtr
MakeTwistRigWithNoEnd()
{
    const UsdStageRefPtr stage = MakeTwistRig();
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineTwist")),
               "rigExec:end", {});
    return stage;
}

// RigExecRibbon.

/// A native driver curve under /Asset/Geom, with a bind-time default and,
/// when \p animated, a bend keyed over the sweep.
///
/// The default matters on its own: the sampler pairs each posed sample with
/// a REST sample read at UsdTimeCode::Default, and a curve that answers
/// nothing there publishes no frames at all.
UsdPrim
DefineDriverCurve(const UsdStageRefPtr &stage, bool animated,
                  bool withDefault = true)
{
    const UsdPrim curve =
        Define(stage, "/Asset/Geom/SpineCurve", "BasisCurves");
    curve.CreateAttribute(TfToken("type"), SdfValueTypeNames->Token)
        .Set(TfToken("cubic"));
    curve.CreateAttribute(TfToken("basis"), SdfValueTypeNames->Token)
        .Set(TfToken("bspline"));
    curve.CreateAttribute(TfToken("wrap"), SdfValueTypeNames->Token)
        .Set(TfToken("nonperiodic"));
    curve.CreateAttribute(TfToken("curveVertexCounts"),
                          SdfValueTypeNames->IntArray).Set(VtIntArray{4});
    const VtVec3fArray rest{GfVec3f(0, 0, 0), GfVec3f(0, 2.7f, 0),
                            GfVec3f(0, 5.3f, 0), GfVec3f(0, 8, 0)};
    UsdAttribute points = curve.CreateAttribute(
        TfToken("points"), SdfValueTypeNames->Point3fArray);
    if (withDefault) {
        points.Set(rest);
    }
    if (animated) {
        points.Set(rest, UsdTimeCode(1.0));
        points.Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0.4f, 2.7f, 0),
                                GfVec3f(1.4f, 5.3f, 0), GfVec3f(2.8f, 7.6f, 0)},
                   UsdTimeCode(3.0));
        points.Set(rest, UsdTimeCode(5.0));
    }
    return curve;
}

/// The ribbon, with a joint riding its middle sample. Example 05's solver
/// with the curve mover and the guide emitter left off -- and with a joint
/// bound, because a ribbon nothing consumes is the guide-only case instead.
UsdStageRefPtr
MakeRibbonRig(bool animated = true, bool withDefault = true)
{
    const UsdStageRefPtr stage = MakeFkSpine();
    const UsdPrim curve = DefineDriverCurve(stage, animated, withDefault);
    const UsdPrim rider =
        Define(stage, "/Asset/Rig/Joints/RibbonMid", "RigExecJoint");
    rider.GetAttribute(TfToken("rest:space")).Set(RestAt(4.0));

    const UsdPrim ribbon =
        Define(stage, "/Asset/Rig/Solvers/SpineRibbon", "RigExecRibbon");
    ribbon.GetAttribute(TfToken("rigExec:sampleCount")).Set(5);
    SetTargets(ribbon, "rigExec:driverCurve", {curve.GetPath()});
    SetTargets(ribbon, "rigExec:startFrame",
               {SdfPath("/Asset/Rig/Joints/Root")});
    SetTargets(ribbon, "rigExec:endFrame",
               {SdfPath("/Asset/Rig/Joints/Root/Chest")});
    SetTargets(ribbon, "rigExec:joints", {rider.GetPath()});
    ribbon.GetAttribute(TfToken("rigExec:jointElements")).Set(VtIntArray{2});
    return stage;
}

/// The driver curve keyed. This is the only per-frame input of the ribbon
/// that is scene data, and the only reason the prologue reads the stage.
UsdStageRefPtr
MakeAnimatedRibbonRig()
{
    return MakeRibbonRig(/* animated = */ true);
}

/// The same curve with no time samples at all: one value at every time code,
/// folded at Build, and a cone that never has to look at it.
UsdStageRefPtr
MakeStaticRibbonRig()
{
    return MakeRibbonRig(/* animated = */ false);
}

/// Time samples and NO default. The rest read answers nothing, so the
/// sampler publishes an empty aggregate however good the live curve is, and
/// the joint falls back -- which is the dynamic path's behaviour and not an
/// obvious one to re-derive.
UsdStageRefPtr
MakeRibbonRigWithNoBindPose()
{
    return MakeRibbonRig(/* animated = */ true, /* withDefault = */ false);
}

/// rigExec:driverCurve wired to nothing, so the compiler resolves no points
/// attribute and both curves are empty.
UsdStageRefPtr
MakeRibbonRigWithNoDriver()
{
    const UsdStageRefPtr stage = MakeRibbonRig();
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineRibbon")),
               "rigExec:driverCurve", {});
    return stage;
}

/// A ribbon that names no joint and drives no geometry: it is in no solver
/// batch, and the only thing that reads it is the guide request.
UsdStageRefPtr
MakeGuideOnlyRibbonRig()
{
    const UsdStageRefPtr stage = MakeRibbonRig();
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineRibbon")),
               "rigExec:joints", {});
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineRibbon"))
        .GetAttribute(TfToken("rigExec:jointElements")).Set(VtIntArray{});
    stage->RemovePrim(SdfPath("/Asset/Rig/Joints/RibbonMid"));
    return stage;
}

/// The same for a twist distribution, whose endpoints are provider frames:
/// a guide-only solver reads the FINAL frame of each, which is the whole
/// reason its Solve step runs after the walk rather than inside it.
UsdStageRefPtr
MakeGuideOnlyTwistRig()
{
    const UsdStageRefPtr stage = MakeTwistRig();
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineTwist")),
               "rigExec:joints", {});
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineTwist"))
        .GetAttribute(TfToken("rigExec:jointElements")).Set(VtIntArray{});
    stage->RemovePrim(SdfPath("/Asset/Rig/Joints/TwistMid"));
    return stage;
}

/// A guide-only blend of two guide-only solvers. This is the only fixture
/// that can tell whether the dependency ORDER among them is right: the blend
/// reads both aggregates, and reading one a step later fills it would
/// publish last run's frames.
UsdStageRefPtr
MakeGuideOnlyBlendRig()
{
    const UsdStageRefPtr stage = MakeGuideOnlyTwistRig();
    const UsdPrim second = Define(stage, "/Asset/Rig/Solvers/SecondTwist",
                                  "RigExecTwistDistribution");
    second.GetAttribute(TfToken("rigExec:count")).Set(5);
    second.GetAttribute(TfToken("inputs:twistTurns")).Set(0.75);
    SetTargets(second, "rigExec:start",
               {SdfPath("/Asset/Rig/Joints/Root/Chest")});
    SetTargets(second, "rigExec:end", {SdfPath("/Asset/Rig/Joints/Root")});

    const UsdPrim blend = Define(stage, "/Asset/Rig/Solvers/GuideBlend",
                                 "RigExecBlendPointFrames");
    SetTargets(blend, "rigExec:inputA",
               {SdfPath("/Asset/Rig/Solvers/SpineTwist")});
    SetTargets(blend, "rigExec:inputB", {second.GetPath()});
    blend.GetAttribute(TfToken("inputs:weight")).Set(0.35f);
    return stage;
}

// The latent guards: RigExecTwoBoneIk, RigExecSplineIk and the blend.
// Every one of these malformed bindings makes the computation warn and
// publish an EMPTY aggregate; the warning never reaches the pose, so the
// only observable consequence is that the joints fall back to their rest
// chains and say so. No shipped rig is malformed, so the bake's agreement
// with that is checked here and nowhere else.

/// A two-bone IK leg with its three controls and its three bound joints:
/// example 02's solver with the geometry left off.
UsdStageRefPtr
MakeTwoBoneIkRig()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    Define(stage, "/Asset", "Xform");
    Define(stage, "/Asset/Rig", "RigExecRoot");

    const UsdPrim hipRoot =
        Define(stage, "/Asset/Rig/Controls/HipRoot", "RigExecControl");
    hipRoot.GetAttribute(TfToken("rest:space")).Set(RestAt(8.0));
    const UsdPrim footIk =
        Define(stage, "/Asset/Rig/Controls/FootIK", "RigExecControl");
    UsdAttribute ty = footIk.GetAttribute(TfToken("avars:ty"));
    ty.Set(0.0, UsdTimeCode(1.0));
    ty.Set(1.5, UsdTimeCode(3.0));
    ty.Set(0.5, UsdTimeCode(5.0));
    const UsdPrim kneePole =
        Define(stage, "/Asset/Rig/Controls/KneePole", "RigExecControl");
    GfMatrix4d poleRest(1.0);
    poleRest.SetTranslateOnly(GfVec3d(0, 4, 3));
    kneePole.GetAttribute(TfToken("rest:space")).Set(poleRest);

    const UsdPrim hip = Define(stage, "/Asset/Rig/Joints/Hip", "RigExecJoint");
    hip.GetAttribute(TfToken("rest:space")).Set(RestAt(8.0));
    GfMatrix4d down(1.0);
    down.SetTranslateOnly(GfVec3d(4, 0, 0));
    const UsdPrim knee =
        Define(stage, "/Asset/Rig/Joints/Hip/Knee", "RigExecJoint");
    knee.GetAttribute(TfToken("rest:space")).Set(down);
    const UsdPrim ankle =
        Define(stage, "/Asset/Rig/Joints/Hip/Knee/Ankle", "RigExecJoint");
    ankle.GetAttribute(TfToken("rest:space")).Set(down);

    const UsdPrim ik =
        Define(stage, "/Asset/Rig/Solvers/LegIK", "RigExecTwoBoneIk");
    SetTargets(ik, "rigExec:rootControl", {hipRoot.GetPath()});
    SetTargets(ik, "rigExec:effectorControl", {footIk.GetPath()});
    SetTargets(ik, "rigExec:poleControl", {kneePole.GetPath()});
    SetTargets(ik, "rigExec:joints",
               {hip.GetPath(), knee.GetPath(), ankle.GetPath()});
    ik.GetAttribute(TfToken("rigExec:preferredBendRadians")).Set(0.3);
    return stage;
}

/// rigExec:joints reduced to two, so the solver binds two of the three chain
/// slots and cannot measure its lower bone.
UsdStageRefPtr
MakeTwoBoneIkRigWithTwoJoints()
{
    const UsdStageRefPtr stage = MakeTwoBoneIkRig();
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/LegIK")),
               "rigExec:joints",
               {SdfPath("/Asset/Rig/Joints/Hip"),
                SdfPath("/Asset/Rig/Joints/Hip/Knee")});
    return stage;
}

/// rigExec:poleControl aimed at a plain Xform. It publishes no
/// computePointFrame, so the pole input is unbound and the computation
/// publishes nothing.
UsdStageRefPtr
MakeTwoBoneIkRigWithANonProviderPole()
{
    const UsdStageRefPtr stage = MakeTwoBoneIkRig();
    const UsdPrim marker = Define(stage, "/Asset/Geom/Marker", "Xform");
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/LegIK")),
               "rigExec:poleControl", {marker.GetPath()});
    return stage;
}

/// A spline IK spine of five joints over three controls.
UsdStageRefPtr
MakeSplineIkRig()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    Define(stage, "/Asset", "Xform");
    Define(stage, "/Asset/Rig", "RigExecRoot");

    const UsdPrim rootCtl =
        Define(stage, "/Asset/Rig/Controls/RootCtl", "RigExecControl");
    const UsdPrim midCtl =
        Define(stage, "/Asset/Rig/Controls/MidCtl", "RigExecControl");
    midCtl.GetAttribute(TfToken("rest:space")).Set(RestAt(4.0));
    UsdAttribute midTx = midCtl.GetAttribute(TfToken("avars:tx"));
    midTx.Set(0.0, UsdTimeCode(1.0));
    midTx.Set(1.2, UsdTimeCode(3.0));
    midTx.Set(-0.4, UsdTimeCode(5.0));
    const UsdPrim endCtl =
        Define(stage, "/Asset/Rig/Controls/EndCtl", "RigExecControl");
    endCtl.GetAttribute(TfToken("rest:space")).Set(RestAt(8.0));
    UsdAttribute endRz = endCtl.GetAttribute(TfToken("avars:rz"));
    endRz.Set(0.0, UsdTimeCode(1.0));
    endRz.Set(20.0, UsdTimeCode(5.0));

    SdfPathVector joints;
    std::string path = "/Asset/Rig/Joints";
    for (int k = 0; k < 5; ++k) {
        path += "/S" + std::to_string(k);
        const UsdPrim joint = Define(stage, path.c_str(), "RigExecJoint");
        joint.GetAttribute(TfToken("rest:space")).Set(RestAt(k == 0 ? 0 : 2));
        joints.push_back(joint.GetPath());
    }

    const UsdPrim spline =
        Define(stage, "/Asset/Rig/Solvers/SpineIk", "RigExecSplineIk");
    SetTargets(spline, "rigExec:rootControl", {rootCtl.GetPath()});
    SetTargets(spline, "rigExec:midControl", {midCtl.GetPath()});
    SetTargets(spline, "rigExec:endControl", {endCtl.GetPath()});
    SetTargets(spline, "rigExec:joints", joints);
    spline.GetAttribute(TfToken("rigExec:restLength")).Set(TfToken("curve"));
    spline.GetAttribute(TfToken("rigExec:volumeWeights"))
        .Set(VtFloatArray{0.2f, 0.4f, 0.5f, 0.35f, 0.1f});
    return stage;
}

/// A token no kernel implements, on each of the two the computation parses.
UsdStageRefPtr
MakeSplineIkRigWithUnsupportedRestLength()
{
    const UsdStageRefPtr stage = MakeSplineIkRig();
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineIk"))
        .GetAttribute(TfToken("rigExec:restLength")).Set(TfToken("spring"));
    return stage;
}

UsdStageRefPtr
MakeSplineIkRigWithUnsupportedRootTangent()
{
    const UsdStageRefPtr stage = MakeSplineIkRig();
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineIk"))
        .GetAttribute(TfToken("rigExec:rootTangent")).Set(TfToken("wobble"));
    return stage;
}

// Four more of the computations' guards have no fixture, and cannot have
// one: COMPILE rejects every one of them before a program is built, with the
// error each was probed for --
//   rigExec:volumeWeights of the wrong length: "rigExec:volumeWeights length
//     3 must equal rigExec:joints length 5 (or be empty)"
//   a remap filling one chain slot twice: "rigExec:jointElements fills chain
//     slot 0 twice"
//   a remap entry outside the chain: "element 9 for <joint> is out of range
//     (solver produces 5 frames)"
//   a remap of the wrong LENGTH, on either solver that reads one:
//     "rigExec:jointElements length 2 must equal rigExec:joints length 3"
// (the last one authored as a CUSTOM array, since RigExecTwoBoneIk does not
// declare jointElements -- its positions are its elements).
// The bake's `degenerate` for all four is therefore defensive rather than
// reachable. It stays, because the computations check them at runtime and
// the two paths should answer the same hypothetical the same way; this
// comment is so the next reader does not spend an afternoon trying to
// author one.

/// An IK/FK blend over the two-bone leg: example 03's shape, with an FK
/// chain over the same three joints and a blend in front of both.
UsdStageRefPtr
MakeBlendRig()
{
    const UsdStageRefPtr stage = MakeTwoBoneIkRig();
    const UsdPrim fkRoot =
        Define(stage, "/Asset/Rig/Controls/FkHip", "RigExecControl");
    fkRoot.GetAttribute(TfToken("rest:space")).Set(RestAt(8.0));
    UsdAttribute rz = fkRoot.GetAttribute(TfToken("avars:rz"));
    rz.Set(0.0, UsdTimeCode(1.0));
    rz.Set(30.0, UsdTimeCode(5.0));
    GfMatrix4d down(1.0);
    down.SetTranslateOnly(GfVec3d(4, 0, 0));
    const UsdPrim fkKnee =
        Define(stage, "/Asset/Rig/Controls/FkHip/FkKnee", "RigExecControl");
    fkKnee.GetAttribute(TfToken("rest:space")).Set(down);
    const UsdPrim fkAnkle = Define(
        stage, "/Asset/Rig/Controls/FkHip/FkKnee/FkAnkle", "RigExecControl");
    fkAnkle.GetAttribute(TfToken("rest:space")).Set(down);

    const UsdPrim fk =
        Define(stage, "/Asset/Rig/Solvers/LegFK", "RigExecFkChain");
    SetTargets(fk, "rigExec:controls",
               {fkRoot.GetPath(), fkKnee.GetPath(), fkAnkle.GetPath()});
    SetTargets(fk, "rigExec:joints",
               {SdfPath("/Asset/Rig/Joints/Hip"),
                SdfPath("/Asset/Rig/Joints/Hip/Knee"),
                SdfPath("/Asset/Rig/Joints/Hip/Knee/Ankle")});

    const UsdPrim blend =
        Define(stage, "/Asset/Rig/Solvers/Blend", "RigExecBlendPointFrames");
    SetTargets(blend, "rigExec:inputA", {fk.GetPath()});
    SetTargets(blend, "rigExec:inputB",
               {SdfPath("/Asset/Rig/Solvers/LegIK")});
    UsdAttribute weight = blend.GetAttribute(TfToken("inputs:weight"));
    weight.Set(0.0f, UsdTimeCode(1.0));
    weight.Set(1.0f, UsdTimeCode(5.0));
    // The blend is what poses the joints now; its two inputs name them only
    // for their rests.
    SetTargets(blend, "rigExec:joints",
               {SdfPath("/Asset/Rig/Joints/Hip"),
                SdfPath("/Asset/Rig/Joints/Hip/Knee"),
                SdfPath("/Asset/Rig/Joints/Hip/Knee/Ankle")});
    return stage;
}

/// An FK chain one of whose controls is a plain Xform. The computation reads
/// its controls through a read iterator, so the target contributes no input
/// at all: the chain SHORTENS, every later element renumbers, and the joint
/// bound to the element past the new end falls back.
UsdStageRefPtr
MakeFkChainWithANonProviderControl()
{
    const UsdStageRefPtr stage = MakeBlendRig();
    stage->RemovePrim(SdfPath("/Asset/Rig/Solvers/LegIK"));
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/Blend")),
               "rigExec:inputB", {SdfPath("/Asset/Rig/Solvers/LegFK")});
    const UsdPrim marker = Define(stage, "/Asset/Geom/Marker", "Xform");
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/LegFK")),
               "rigExec:controls",
               {SdfPath("/Asset/Rig/Controls/FkHip"), marker.GetPath(),
                SdfPath("/Asset/Rig/Controls/FkHip/FkKnee/FkAnkle")});
    return stage;
}

/// rigExec:rotationBlend authored to the one thing the schema does not
/// allow: the computation rejects the token rather than substituting for it.
UsdStageRefPtr
MakeBlendRigWithLinearRotation()
{
    const UsdStageRefPtr stage = MakeBlendRig();
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/Blend"))
        .GetAttribute(TfToken("rigExec:rotationBlend"))
        .Set(TfToken("linear"));
    return stage;
}

/// rigExec:inputB aimed at a prim that publishes no aggregate -- a control.
/// That is the computation's null pointer, and a null pointer passes the
/// OTHER input through unchanged, rests and all.
///
/// The IK goes with it, and that is now a CHOICE rather than a requirement:
/// a solver nothing consumes writes the joints its rigExec:joints names, and
/// two solvers writing one joint is a legal stack whose last writer wins. So
/// leaving the IK in place would still compile -- it would just add a second
/// writer to all three joints and change what this fixture measures. Removing
/// it keeps the fixture about the null input and nothing else.
UsdStageRefPtr
MakeBlendRigWithANonSolverInput()
{
    const UsdStageRefPtr stage = MakeBlendRig();
    stage->RemovePrim(SdfPath("/Asset/Rig/Solvers/LegIK"));
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/Blend")),
               "rigExec:inputB",
               {SdfPath("/Asset/Rig/Controls/KneePole")});
    return stage;
}

/// Both at once: an unsupported rigExec:rotationBlend AND an input that
/// publishes no aggregate.
///
/// The ORDER of the computation's two checks is the whole fixture. It
/// returns the surviving input before it ever looks at the token, so the
/// blend passes the FK chain through and poses the three joints -- while a
/// bake that treated the token as a whole-solver degeneracy publishes
/// nothing and drops all three to their rest chains. Each half alone is
/// above, and each half alone agrees; only the conjunction tells the two
/// orderings apart.
UsdStageRefPtr
MakeBlendRigWithLinearRotationAndANonSolverInput()
{
    const UsdStageRefPtr stage = MakeBlendRigWithANonSolverInput();
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/Blend"))
        .GetAttribute(TfToken("rigExec:rotationBlend"))
        .Set(TfToken("linear"));
    return stage;
}

// The driver curve, which is the one solver input that is scene data.

/// The driver curve's bind pose is folded into bake state, so a drag on its
/// points must be REFUSED rather than placed.
///
/// This captured raw source is outside the scalar override contract, so
/// placing an override must report failure before executing the graph.
///
/// Asked of the PROGRAM rather than through Evaluate on purpose: a
/// unsupported override must be detected at the input boundary.
void
CheckRibbonPointsOverrideIsRefused()
{
    const UsdStageRefPtr stage = MakeAnimatedRibbonRig();
    RigExecRigEvaluator rig(stage, kRigPath);
    std::vector<std::string> errors;
    if (!rig.Compile(&errors)) {
        ++failures;
        std::printf("FAIL a drag on the driver curve: the fixture does not "
                    "compile\n");
        return;
    }
    std::vector<std::string> reasons;
    const std::unique_ptr<RigExecBakedProgram> program =
        RigExecBakedProgram::Build(&rig, &reasons);
    CHECK(program != nullptr);
    if (!program) return;
    const VtVec3fArray held{GfVec3f(0, 0, 0), GfVec3f(1.0f, 2.7f, 0),
                            GfVec3f(2.0f, 5.3f, 0), GfVec3f(3.0f, 8, 0)};
    CHECK(!program->SetOverrides({RigExecValueOverride{
        SdfPath("/Asset/Geom/SpineCurve"), TfToken(), TfToken("points"),
        VtValue(held)}}));
    // Two reasons refuse it and either is enough -- the path is folded, and
    // it is in no binding table -- which is the point: no future rewiring
    // of one of them can make this drag placeable quietly.
    // The control: rigExec:sampleCount is a per-frame input of the same
    // solver and IS placeable, so a program that refused everything would
    // pass the line above while saying nothing.
    CHECK(program->SetOverrides({RigExecValueOverride{
        SdfPath("/Asset/Rig/Solvers/SpineRibbon"), TfToken(),
        TfToken("rigExec:sampleCount"), VtValue(int(4))}}));
}

/// A ribbon whose rigExec:driverCurve names a prim that has no `points` at
/// all -- a plain Xform, where `points` is not even a schema attribute, so
/// the path the compiler resolves names nothing on this stage.
///
/// A BasisCurves cannot pose this question: `points` is builtin there, so
/// the attribute exists whether or not anything is authored on it, and the
/// bake reads it and registers it like any other.
UsdStageRefPtr
MakeRibbonRigWithNoPointsYet()
{
    const UsdStageRefPtr stage = MakeRibbonRig();
    const UsdPrim future = Define(stage, "/Asset/Geom/FutureCurve", "Xform");
    SetTargets(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/SpineRibbon")),
               "rigExec:driverCurve", {future.GetPath()});
    return stage;
}

/// Authors that prim's points, keyed the way DefineDriverCurve keys them.
void
AuthorDriverPoints(const UsdStageRefPtr &stage)
{
    const VtVec3fArray rest{GfVec3f(0, 0, 0), GfVec3f(0, 2.7f, 0),
                            GfVec3f(0, 5.3f, 0), GfVec3f(0, 8, 0)};
    UsdAttribute points =
        stage->GetPrimAtPath(SdfPath("/Asset/Geom/FutureCurve"))
            .CreateAttribute(TfToken("points"),
                             SdfValueTypeNames->Point3fArray);
    points.Set(rest);
    points.Set(rest, UsdTimeCode(1.0));
    points.Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0.4f, 2.7f, 0),
                            GfVec3f(1.4f, 5.3f, 0), GfVec3f(2.8f, 7.6f, 0)},
               UsdTimeCode(3.0));
    points.Set(rest, UsdTimeCode(5.0));
}

/// The driver curve's points CREATED after the program was built.
///
/// The bake read no attribute and captured two empty curves, so nothing
/// about that path is in the program's invalidation index -- and the day
/// the attribute appears the dynamic path starts sampling it. What catches
/// that is the epoch digest, not the index: creating a property is a
/// structural edit, so the epoch recompiles and the program is rebuilt with
/// it. This is the fixture that says so; a program that answered the new
/// curve with the empty aggregate it baked would fail here.
void
CheckRibbonPointsCreatedAfterTheBake()
{
    const char *const what = "the driver curve's points created later";
    const UsdStageRefPtr baselineStage = MakeRibbonRigWithNoPointsYet();
    const UsdStageRefPtr bakedStage = MakeRibbonRigWithNoPointsYet();
    CHECK(baselineStage && bakedStage);
    if (!baselineStage || !bakedStage) return;

    RigExecRigEvaluator baseline(baselineStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    std::vector<std::string> errors;
    if (!baseline.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        return;
    }

    const RigExecRigPose before = baked.Evaluate(UsdTimeCode(3.0));
    CHECK(before.valid);
    if (baked.GetBakedGenerationCount() != 1) {
        ++failures;
        std::printf("FAIL %s: the first generation was not the program's\n",
                    what);
        return;
    }

    AuthorDriverPoints(baselineStage);
    AuthorDriverPoints(bakedStage);

    const RigExecRigPose a = baseline.Evaluate(UsdTimeCode(3.0));
    const RigExecRigPose b = baked.Evaluate(UsdTimeCode(3.0));
    CHECK(a.valid && b.valid);

    rigExecTest::ComparePose(&failures, what, a, b);
    if (baked.GetBakedGenerationCount() != 2) {
        ++failures;
        std::printf("FAIL %s: the generation after the edit was not the "
                    "program's\n", what);
    }
    if (b.jointFramesFinal == before.jointFramesFinal) {
        ++failures;
        std::printf("FAIL %s: the new curve moved nothing\n", what);
    }
}

/// Compares canonical evaluators across an edit to the driver curve's points made
/// AFTER the program was built.
///
/// The bind-pose half of that curve is folded into bake state, so an edit
/// to it has to REBUILD the program: one that kept the rest it captured
/// would sample against the bind pose it was baked with, and no frame sweep
/// would ever say so. The live half is read per frame, and an edit to a
/// time sample has to be followed too.
void
CheckRibbonPointsEdit(const char *what, bool editDefault)
{
    const UsdStageRefPtr baselineStage = MakeAnimatedRibbonRig();
    const UsdStageRefPtr bakedStage = MakeAnimatedRibbonRig();
    CHECK(baselineStage && bakedStage);
    if (!baselineStage || !bakedStage) return;

    RigExecRigEvaluator baseline(baselineStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    std::vector<std::string> errors;
    if (!baseline.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        return;
    }

    const RigExecRigPose before = baked.Evaluate(UsdTimeCode(3.0));
    CHECK(before.valid);
    // The program answered that generation, so what follows is a comparison
    // between separately initialized canonical programs.
    if (baked.GetBakedGenerationCount() != 1) {
        ++failures;
        std::printf("FAIL %s: the first generation was not the program's\n",
                    what);
        return;
    }

    const VtVec3fArray edited{GfVec3f(0, 0, 0), GfVec3f(1.1f, 2.7f, 0),
                              GfVec3f(2.3f, 5.3f, 0), GfVec3f(3.5f, 7.6f, 0)};
    for (const UsdStageRefPtr &stage : {baselineStage, bakedStage}) {
        UsdAttribute points = stage->GetAttributeAtPath(
            SdfPath("/Asset/Geom/SpineCurve.points"));
        CHECK(points);
        if (editDefault) {
            points.Set(edited);
        } else {
            points.Set(edited, UsdTimeCode(3.0));
        }
    }

    const RigExecRigPose a = baseline.Evaluate(UsdTimeCode(3.0));
    const RigExecRigPose b = baked.Evaluate(UsdTimeCode(3.0));
    CHECK(a.valid && b.valid);

    rigExecTest::ComparePose(&failures, what, a, b);
    // A folded value the edit moved REBUILDS the program; it does not make
    // it refuse the rig, and a generation answered dynamically would have
    // agreed with the dynamic path for the wrong reason.
    if (baked.GetBakedGenerationCount() != 2) {
        ++failures;
        std::printf("FAIL %s: the generation after the edit was not the "
                    "program's\n", what);
    }
    // And the edit has to have MOVED something, or nothing above followed
    // anything.
    if (b.jointFramesFinal == before.jointFramesFinal) {
        ++failures;
        std::printf("FAIL %s: the edit moved nothing\n", what);
    }
}

// TestRefreshSolverRestsOnBuildStateIsIdentity.
//
// The Solve step rebuilds a solver's rest description (RefreshSolverRests)
// only on a run on which one of its own rests moved. Skipping it is sound
// only if a refresh over unchanged rests reproduces, bit for bit, the
// description the solver already holds. Each refresh is compared with the
// description the solver holds now and, wherever the solver's rests are
// bit-identical to the ones Build measured from, with the Build description.
// A run that already refreshed the solver makes the first comparison a
// determinism check only; the second is the one a per-solver skip relies on,
// including on a run that moved another solver's rests. A rest ref read LIVE
// from `fin` differs from Build's authored rest by design and refreshes
// every run, so each field fed by one is excluded.

using SolverDesc = RigExecBakedProgramImpl::Solver;
using RestPoints = std::array<GfVec3d, 4>;

bool
SameBits(double a, double b)
{
    uint64_t x = 0, y = 0;
    std::memcpy(&x, &a, sizeof(x));
    std::memcpy(&y, &b, sizeof(y));
    return x == y;
}

bool
SameBits(const GfVec3d &a, const GfVec3d &b)
{
    return SameBits(a[0], b[0]) && SameBits(a[1], b[1]) &&
           SameBits(a[2], b[2]);
}

bool
SameBits(const RestPoints &a, const RestPoints &b)
{
    for (size_t i = 0; i < a.size(); ++i) {
        if (!SameBits(a[i], b[i])) return false;
    }
    return true;
}

bool
SameBits(const RigExecPointFrame &a, const RigExecPointFrame &b)
{
    return a.flags == b.flags && SameBits(a.points, b.points);
}

template <class T>
bool
SameBits(const std::vector<T> &a, const std::vector<T> &b)
{
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (!SameBits(a[i], b[i])) return false;
    }
    return true;
}

bool
SameBits(const RigExecSplineIkRest &a, const RigExecSplineIkRest &b)
{
    return SameBits(std::vector<GfVec3d>(a.cvs.begin(), a.cvs.end()),
                    std::vector<GfVec3d>(b.cvs.begin(), b.cvs.end())) &&
           SameBits(a.rootControl, b.rootControl) &&
           SameBits(a.midControl, b.midControl) &&
           SameBits(a.endControl, b.endControl) &&
           SameBits(a.joints, b.joints) &&
           SameBits(a.segmentLengths, b.segmentLengths) &&
           SameBits(a.restArcLength, b.restArcLength) &&
           SameBits(a.volumeWeights, b.volumeWeights);
}

std::string
Describe(double v)
{
    char text[64];
    std::snprintf(text, sizeof(text), "%.17g (%a)", v, v);
    return text;
}

std::string
Describe(const RestPoints &p)
{
    std::string out;
    for (const GfVec3d &v : p) {
        char text[160];
        std::snprintf(text, sizeof(text), "(%.17g %.17g %.17g)", v[0], v[1],
                      v[2]);
        out += text;
    }
    return out;
}

/// Field-by-field comparison of one held description with its refresh.
struct RestDiff {
    std::string where;
    /// False counts mismatches without failing the test.
    bool report = true;
    size_t compared = 0;
    size_t excluded = 0;
    size_t differing = 0;

    /// \p live excludes the field; \p detail is printed only on a mismatch.
    void Field(const std::string &field, bool live, bool same,
               const std::function<std::string()> &detail = {})
    {
        if (live) {
            ++excluded;
            return;
        }
        ++compared;
        if (same) return;
        ++differing;
        if (!report) return;
        ++failures;
        std::printf("FAIL %s: %s differs from the refresh%s%s\n",
                    where.c_str(), field.c_str(), detail ? ": " : "",
                    detail ? detail().c_str() : "");
    }
};

/// Compares every field RefreshSolverRests writes, over \p held and
/// \p refreshed, into \p diff.
void
CompareRestDescription(const SolverDesc &held, const SolverDesc &refreshed,
                       RestDiff *diff)
{
    const auto liveRef = [&held](size_t k) {
        return k < held.restIsLive.size() && held.restIsLive[k];
    };
    // Whether a live ref feeds element \p e of an element-remapped table.
    const auto liveElement = [&](int e) {
        for (size_t k = 0; k < held.restRefs.size(); ++k) {
            if (held.restRefs[k].second == e && liveRef(k)) return true;
        }
        return false;
    };
    const auto points = [](const RestPoints &a, const RestPoints &b) {
        return [&a, &b] {
            return "held " + Describe(a) + " refreshed " + Describe(b);
        };
    };
    const auto scalar = [](double a, double b) {
        return [a, b] {
            return "held " + Describe(a) + " refreshed " + Describe(b);
        };
    };

    diff->Field("jointRests.size", false,
                held.jointRests.size() == refreshed.jointRests.size());
    for (size_t k = 0; k < held.jointRests.size() &&
                       k < refreshed.jointRests.size();
         ++k) {
        diff->Field("jointRests[" + std::to_string(k) + "]", liveRef(k),
                    SameBits(held.jointRests[k], refreshed.jointRests[k]),
                    points(held.jointRests[k], refreshed.jointRests[k]));
    }

    if (held.type == "RigExecFkChain") {
        diff->Field("controlRests.size", false,
                    held.controlRests.size() ==
                        refreshed.controlRests.size());
        for (size_t k = 0; k < held.controlRests.size() &&
                           k < refreshed.controlRests.size();
             ++k) {
            diff->Field("controlRests[" + std::to_string(k) + "]", false,
                        SameBits(held.controlRests[k],
                                 refreshed.controlRests[k]),
                        points(held.controlRests[k],
                               refreshed.controlRests[k]));
        }
        diff->Field("startRest", false,
                    SameBits(held.startRest, refreshed.startRest),
                    points(held.startRest, refreshed.startRest));
    } else if (held.type == "RigExecTwoBoneIk") {
        bool anyLive = false;
        for (int e = 0; e < 3; ++e) {
            const bool live = liveElement(e);
            anyLive = anyLive || live;
            diff->Field("ikRests[" + std::to_string(e) + "]", live,
                        SameBits(held.ikRests[size_t(e)],
                                 refreshed.ikRests[size_t(e)]),
                        points(held.ikRests[size_t(e)],
                               refreshed.ikRests[size_t(e)]));
        }
        // Measured from ikRests, so a live element excludes them too.
        diff->Field("upperLengthBase", anyLive,
                    SameBits(held.upperLengthBase, refreshed.upperLengthBase),
                    scalar(held.upperLengthBase, refreshed.upperLengthBase));
        diff->Field("lowerLengthBase", anyLive,
                    SameBits(held.lowerLengthBase, refreshed.lowerLengthBase),
                    scalar(held.lowerLengthBase, refreshed.lowerLengthBase));
        diff->Field("ikParams.upperLength", anyLive,
                    SameBits(held.ikParams.upperLength,
                             refreshed.ikParams.upperLength),
                    scalar(held.ikParams.upperLength,
                           refreshed.ikParams.upperLength));
        diff->Field("ikParams.lowerLength", anyLive,
                    SameBits(held.ikParams.lowerLength,
                             refreshed.ikParams.lowerLength),
                    scalar(held.ikParams.lowerLength,
                           refreshed.ikParams.lowerLength));
        if (held.spaceSlot >= 0) {
            diff->Field("spaceRest", false,
                        SameBits(held.spaceRest, refreshed.spaceRest),
                        points(held.spaceRest, refreshed.spaceRest));
        }
    } else if (held.type == "RigExecSplineIk") {
        bool anyLive = false;
        diff->Field("splineJointRests.size", false,
                    held.splineJointRests.size() ==
                        refreshed.splineJointRests.size());
        diff->Field("splineRestFrames.size", false,
                    held.splineRestFrames.size() ==
                        refreshed.splineRestFrames.size());
        for (size_t e = 0; e < held.splineJointRests.size() &&
                           e < refreshed.splineJointRests.size();
             ++e) {
            const bool live = liveElement(int(e));
            anyLive = anyLive || live;
            diff->Field("splineJointRests[" + std::to_string(e) + "]", live,
                        SameBits(held.splineJointRests[e],
                                 refreshed.splineJointRests[e]),
                        points(held.splineJointRests[e],
                               refreshed.splineJointRests[e]));
        }
        for (size_t e = 0; e < held.splineRestFrames.size() &&
                           e < refreshed.splineRestFrames.size();
             ++e) {
            diff->Field("splineRestFrames[" + std::to_string(e) + "]",
                        liveElement(int(e)),
                        SameBits(held.splineRestFrames[e],
                                 refreshed.splineRestFrames[e]),
                        points(held.splineRestFrames[e].points,
                               refreshed.splineRestFrames[e].points));
        }
        diff->Field("splineRootRest", false,
                    SameBits(held.splineRootRest, refreshed.splineRootRest),
                    points(held.splineRootRest.points,
                           refreshed.splineRootRest.points));
        diff->Field("splineMidRest", false,
                    SameBits(held.splineMidRest, refreshed.splineMidRest),
                    points(held.splineMidRest.points,
                           refreshed.splineMidRest.points));
        diff->Field("splineEndRest", false,
                    SameBits(held.splineEndRest, refreshed.splineEndRest),
                    points(held.splineEndRest.points,
                           refreshed.splineEndRest.points));
        // Built from the joint rest frames, so a live element excludes it.
        diff->Field("splineRest", anyLive,
                    SameBits(held.splineRest, refreshed.splineRest),
                    scalar(held.splineRest.restArcLength,
                           refreshed.splineRest.restArcLength));
        if (held.spaceSlot >= 0) {
            diff->Field("spaceRest", false,
                        SameBits(held.spaceRest, refreshed.spaceRest),
                        points(held.spaceRest, refreshed.spaceRest));
        }
    } else if (held.type == "RigExecTwistDistribution") {
        diff->Field("twistStartRest", false,
                    SameBits(held.twistStartRest, refreshed.twistStartRest),
                    points(held.twistStartRest, refreshed.twistStartRest));
        diff->Field("twistEndRest", false,
                    SameBits(held.twistEndRest, refreshed.twistEndRest),
                    points(held.twistEndRest, refreshed.twistEndRest));
    }
}

/// Field counts over one fixture.
struct RestCounts {
    /// Against the description the solver holds.
    size_t compared = 0;
    size_t excluded = 0;
    /// Against the Build description, where the rests are Build's.
    size_t sinceBuild = 0;
    /// The subset of `sinceBuild` on a run on which the rest tier moved a
    /// rest: another solver's, since this one's are still Build's.
    size_t sinceBuildRecomposed = 0;
};

/// Whether \p s measures from bit-identical rests in \p a and \p b. Both
/// programs must number their slots alike.
bool
SameRestInputs(const SolverDesc &s, const RigExecBakedProgramImpl &a,
               const RigExecBakedProgramImpl &b)
{
    for (const int slot : s.restSlots) {
        if (!SameBits(a.restPts[size_t(slot)], b.restPts[size_t(slot)]) ||
            !SameBits(a.restFrames[size_t(slot)],
                      b.restFrames[size_t(slot)])) {
            return false;
        }
    }
    return true;
}

/// Compares the refresh of every solver of \p program that measures from a
/// rest with the description it holds and, where its rests are still
/// \p build's, with \p build's description. Returns how many solvers it
/// compared.
size_t
CompareProgramRests(const std::string &where,
                    const RigExecBakedProgram &program,
                    const RigExecBakedProgramImpl *build, RestCounts *counts)
{
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    size_t solvers = 0;
    for (size_t i = 0; i < B.solvers.size(); ++i) {
        const SolverDesc &held = B.solvers[i];
        if (held.restSlots.empty()) continue;
        ++solvers;
        const std::string name = where + " " + held.path.GetString() + " (" +
                                 held.type.GetString() + ")";
        const SolverDesc refreshed =
            RigExecBakedProgramTesting::RefreshedSolverRests(program, i);
        RestDiff diff;
        diff.where = name;
        CompareRestDescription(held, refreshed, &diff);
        counts->compared += diff.compared;
        counts->excluded += diff.excluded;
        if (!build) continue;
        if (i >= build->solvers.size() ||
            build->solvers[i].path != held.path ||
            build->solvers[i].restSlots != held.restSlots ||
            build->paths != B.paths) {
            ++failures;
            std::printf("FAIL %s: the run's program and the Build reference "
                        "disagree about the solver or its slots\n",
                        name.c_str());
            continue;
        }
        if (!SameRestInputs(held, B, *build)) continue;
        RestDiff since;
        since.where = name + " against Build";
        CompareRestDescription(build->solvers[i], refreshed, &since);
        counts->sinceBuild += since.compared;
        if (!B.restMoved.empty()) {
            counts->sinceBuildRecomposed += since.compared;
        }
    }
    return solvers;
}

SdfPath
FindRigRoot(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") return prim.GetPath();
    }
    return SdfPath();
}

/// A compiled evaluator over \p stage's rig; null on failure.
std::unique_ptr<RigExecRigEvaluator>
CompileFixture(const char *what, const UsdStageRefPtr &stage)
{
    const SdfPath rig = stage ? FindRigRoot(stage) : SdfPath();
    if (rig.IsEmpty()) {
        ++failures;
        std::printf("FAIL %s: no stage or no RigExecRoot\n", what);
        return nullptr;
    }
    auto evaluator = std::make_unique<RigExecRigEvaluator>(stage, rig);
    std::vector<std::string> errors;
    if (!evaluator->Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return nullptr;
    }
    return evaluator;
}

/// Returns the fields compared against the Build description on a run
/// that moved another rest.
size_t
TestRefreshSolverRestsOnBuildStateIsIdentity(const char *what,
                                             const MakeStage &make)
{
    RestCounts counts;

    // After Build, before any run: against the Build-path description. The
    // program stays as the reference for the runs below.
    const UsdStageRefPtr buildStage = make();
    const auto buildEvaluator = CompileFixture(what, buildStage);
    if (!buildEvaluator) return 0;
    std::vector<std::string> reasons;
    const std::unique_ptr<RigExecBakedProgram> built =
        RigExecBakedProgram::Build(buildEvaluator.get(), &reasons);
    if (!built) {
        ++failures;
        std::printf("FAIL %s: the rig does not bake\n", what);
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return 0;
    }
    if (CompareProgramRests(std::string(what) + " after Build", *built,
                            nullptr, &counts) == 0) {
        ++failures;
        std::printf("FAIL %s: no solver measures from a rest\n", what);
        return 0;
    }
    const RigExecBakedProgramImpl &build = built->GetStepGraph();

    // After runs: settled, under a drag on a rest channel, and released.
    const UsdStageRefPtr stage = make();
    const auto evaluator = CompileFixture(what, stage);
    if (!evaluator) return 0;

    const std::vector<double> frames{1, 3, 7};
    const auto runAndCompare = [&](const std::string &phase) {
        for (const double frame : frames) {
            const std::string where = std::string(what) + " " + phase +
                                      " frame " +
                                      std::to_string(int(frame));
            const size_t before = evaluator->GetBakedGenerationCount();
            const RigExecRigPose pose =
                evaluator->Evaluate(UsdTimeCode(frame));
            const RigExecBakedProgram *program =
                evaluator->GetBakedProgram();
            if (!pose.valid || !program ||
                evaluator->GetBakedGenerationCount() != before + 1) {
                ++failures;
                std::printf("FAIL %s: the generation did not come from the "
                            "program\n",
                            where.c_str());
                continue;
            }
            CompareProgramRests(where, *program, &build, &counts);
        }
    };
    runAndCompare("settled");

    // The drag lands on a rest slot the first such solver reads from its
    // authored rest, not through a live ref.
    const RigExecBakedProgram *program = evaluator->GetBakedProgram();
    size_t draggedSolver = 0;
    int slot = -1;
    if (program) {
        const auto &solvers = program->GetStepGraph().solvers;
        for (size_t i = 0; i < solvers.size() && slot < 0; ++i) {
            const SolverDesc &s = solvers[i];
            std::set<int> live;
            for (size_t k = 0;
                 k < s.restRefs.size() && k < s.restIsLive.size(); ++k) {
                if (s.restIsLive[k]) live.insert(s.restRefs[k].first);
            }
            for (const int candidate : s.restSlots) {
                if (!live.count(candidate)) {
                    slot = candidate;
                    draggedSolver = i;
                    break;
                }
            }
        }
    }
    if (slot < 0) {
        ++failures;
        std::printf("FAIL %s: no authored solver rest to drag\n", what);
        return counts.sinceBuildRecomposed;
    }
    const SdfPath dragged = program->GetStepGraph().paths[size_t(slot)];
    double authored = 0.0;
    if (const UsdAttribute a = stage->GetPrimAtPath(dragged).GetAttribute(
            TfToken("rest:tx"))) {
        a.Get(&authored, UsdTimeCode(frames.front()));
    }
    evaluator->SetInteractiveOverrides({RigExecValueOverride{
        dragged, TfToken(), TfToken("rest:tx"), VtValue(authored + 0.5)}});
    runAndCompare("dragging " + dragged.GetString() + ".rest:tx");
    // The drag has to reach the dragged solver's description, or its half
    // is vacuous.
    program = evaluator->GetBakedProgram();
    RestDiff moved;
    moved.report = false;
    if (program) {
        CompareRestDescription(
            build.solvers[draggedSolver],
            program->GetStepGraph().solvers[draggedSolver], &moved);
    }
    if (moved.differing == 0) {
        ++failures;
        std::printf("FAIL %s: the drag on %s.rest:tx moved no rest of %s\n",
                    what, dragged.GetString().c_str(),
                    build.solvers[draggedSolver].path.GetText());
    }
    evaluator->ClearInteractiveOverrides();
    runAndCompare("released");

    std::printf("%s: %zu rest field(s) compared with the held description, "
                "%zu live field(s) excluded, %zu compared with Build's "
                "(%zu on a run that moved a rest); the drag moved %zu "
                "field(s)\n",
                what, counts.compared, counts.excluded, counts.sinceBuild,
                counts.sinceBuildRecomposed, moved.differing);
    return counts.sinceBuildRecomposed;
}

/// The rigs whose solvers measure from a rest, R0's table.
std::vector<std::pair<std::string, MakeStage>>
RestFixtureRigs(const std::string &examples)
{
    const std::string fixtures = examples + "/../tests/fixtures";
    const auto file = [](const std::string &path) -> MakeStage {
        return [path] { return UsdStage::Open(path); };
    };
    return {
        {"biped", file(examples + "/biped/Biped.usda")},
        {"01 fk chain tail", file(examples + "/01_FkChainTail.usda")},
        {"02 two-bone ik leg", file(examples + "/02_TwoBoneIkLeg.usda")},
        {"03 ik/fk blend clamp", file(examples + "/03_IkFkBlendClamp.usda")},
        {"05 twist ribbon spine",
         file(examples + "/05_TwistRibbonSpine.usda")},
        {"13 read phases", file(examples + "/13_ReadPhases.usda")},
        {"arm rig", file(examples + "/ArmRig.usda")},
        {"rubberhose", file(examples + "/2d/rubberhose/rubberhose_rig.usda")},
        {"bust", file(examples + "/2d/bust/bust_rig.usda")},
        {"solver_checkpoint", file(fixtures + "/solver_checkpoint.usda")},
        {"oneloop_two_limbs", file(fixtures + "/oneloop_two_limbs.usda")},
        {"oneloop_cross_domain",
         file(fixtures + "/oneloop_cross_domain.usda")},
        {"computed_ik_space", file(fixtures + "/computed_ik_space.usda")},
        {"preceding_own_chain", file(fixtures + "/preceding_own_chain.usda")},
        {"volume_placements", file(fixtures + "/volume_placements.usda")},
        {"two-bone ik rig", MakeTwoBoneIkRig},
        {"spline ik rig", MakeSplineIkRig},
        {"twist distribution rig", MakeTwistRig},
        {"ik/fk blend rig", MakeBlendRig},
    };
}

// TestASpaceRestMoveReachesTheSolve.
//
// A TwoBoneIk or SplineIk that names rigExec:space measures the space delta
// from its space slot's rest (Solver::spaceRest). Dynamic reads that rest
// from the space target's computeRestFrame on every evaluation, so the baked
// refresh has to rewrite it whenever the ladder moves it. computed_ik_space's
// arm and tail both name the Master control, whose rest is moved here by
// keyed rest channels, by a drag on one and by authored edits.

const SdfPath kIkSpaceMaster("/IkSpaceAsset/Rig/Controls/Master");
const SdfPath kIkSpaceWrist("/IkSpaceAsset/Rig/Joints/Shoulder/Elbow/Wrist");
const SdfPath kIkSpaceTailEnd(
    "/IkSpaceAsset/Rig/Joints/Tail0/Tail1/Tail2/Tail3/Tail4");

/// Canonical baseline and cached evaluators, each over its own stage.
struct EvaluatorPair {
    UsdStageRefPtr baselineStage;
    UsdStageRefPtr bakedStage;
    std::unique_ptr<RigExecRigEvaluator> baseline;
    std::unique_ptr<RigExecRigEvaluator> baked;
};

bool
MakeEvaluatorPair(const char *what, const MakeStage &make,
                  EvaluatorPair *pair)
{
    pair->baselineStage = make();
    pair->bakedStage = make();
    pair->baseline = CompileFixture(what, pair->baselineStage);
    pair->baked = CompileFixture(what, pair->bakedStage);
    if (!pair->baseline || !pair->baked) return false;


    return true;
}

/// Evaluates one generation on each side of \p pair at \p frame and
/// compares them the way CheckEvaluatorConsistency does. Returns whether they agreed;
/// \p dynamic receives the reference generation.
bool
GenerationsAgree(const std::string &where, EvaluatorPair *pair, double frame,
                 RigExecRigPose *dynamic)
{
    const int before = failures;
    const size_t generations = pair->baked->GetBakedGenerationCount();
    *dynamic = pair->baseline->Evaluate(UsdTimeCode(frame));
    const RigExecRigPose b = pair->baked->Evaluate(UsdTimeCode(frame));
    CHECK(dynamic->valid && b.valid);

    rigExecTest::ComparePose(&failures, where, *dynamic, b);
    if (pair->baked->GetBakedGenerationCount() != generations + 1) {
        ++failures;
        std::printf("FAIL %s: the generation did not come from the program\n",
                    where.c_str());
    }
    return failures == before;
}

/// Fails unless \p joint's final frame differs between \p a and \p b.
void
RequireJointMoved(const char *what, const RigExecRigPose &a,
                  const RigExecRigPose &b, const SdfPath &joint)
{
    const auto x = a.jointFramesFinal.find(joint);
    const auto y = b.jointFramesFinal.find(joint);
    if (x == a.jointFramesFinal.end() || y == b.jointFramesFinal.end() ||
        rigExecTest::SameFrame(x->second, y->second)) {
        ++failures;
        std::printf("FAIL %s: the space rest move did not move %s on the "
                    "dynamic path\n",
                    what, joint.GetText());
    }
}

void
TestASpaceRestMoveReachesTheSolve(const std::string &examples)
{
    const std::string file =
        examples + "/../tests/fixtures/computed_ik_space.usda";
    const MakeStage plain = [file] { return UsdStage::Open(file); };
    const MakeStage keyed = [file] {
        const UsdStageRefPtr stage = UsdStage::Open(file);
        if (!stage) return stage;
        UsdEditContext session(stage, stage->GetSessionLayer());
        const UsdPrim master = stage->GetPrimAtPath(kIkSpaceMaster);
        const auto key = [&master](const char *name, double last) {
            UsdAttribute a = master.CreateAttribute(
                TfToken(name), SdfValueTypeNames->Double);
            a.Set(0.0, UsdTimeCode(1.0));
            a.Set(last, UsdTimeCode(10.0));
        };
        key("rest:ry", 20.0);
        key("rest:tx", 3.0);
        return stage;
    };
    size_t generations = 0;
    size_t disagreeing = 0;
    const auto compare = [&](const std::string &where, EvaluatorPair *pair,
                             double frame, RigExecRigPose *dynamic) {
        ++generations;
        if (!GenerationsAgree(where, pair, frame, dynamic)) ++disagreeing;
    };

    // Keyed: frame 1 is Build's rest, 5 and 10 are not.
    {
        const char *what = "a keyed space rest";
        EvaluatorPair pair;
        if (MakeEvaluatorPair(what, keyed, &pair)) {
            RigExecRigPose dynamic;
            for (const double frame : {1.0, 5.0, 10.0}) {
                compare(std::string(what) + " frame " +
                            std::to_string(int(frame)),
                        &pair, frame, &dynamic);
            }
            // Against the unkeyed rig at frame 10, or both halves agree
            // about a rest nothing moved.
            const UsdStageRefPtr stillStage = plain();
            const auto still = CompileFixture(what, stillStage);
            if (still) {

                const RigExecRigPose unmoved =
                    still->Evaluate(UsdTimeCode(10.0));
                RequireJointMoved(what, dynamic, unmoved, kIkSpaceWrist);
                RequireJointMoved(what, dynamic, unmoved, kIkSpaceTailEnd);
            }
        }
    }

    // Dragged at a held frame: two drag values, then the release.
    {
        const char *what = "a dragged space rest";
        EvaluatorPair pair;
        if (MakeEvaluatorPair(what, plain, &pair)) {
            const double frame = 5.0;
            RigExecRigPose settled, dragged, other;
            compare(std::string(what) + " settled", &pair, frame, &settled);
            const auto drag = [&](double degrees) {
                const std::vector<RigExecValueOverride> overrides{
                    RigExecValueOverride{kIkSpaceMaster, TfToken(),
                                         TfToken("rest:ry"),
                                         VtValue(degrees)}};
                pair.baseline->SetInteractiveOverrides(overrides);
                pair.baked->SetInteractiveOverrides(overrides);
            };
            drag(15.0);
            compare(std::string(what) + " at 15", &pair, frame, &dragged);
            drag(25.0);
            compare(std::string(what) + " at 25", &pair, frame, &other);
            pair.baseline->ClearInteractiveOverrides();
            pair.baked->ClearInteractiveOverrides();
            compare(std::string(what) + " released", &pair, frame, &other);
            RequireJointMoved(what, dragged, settled, kIkSpaceWrist);
            RequireJointMoved(what, dragged, settled, kIkSpaceTailEnd);
        }
    }

    // Edited at a held frame: an authored rest:tx change on the space slot,
    // on a keyed ladder (the edit is routed to the ladder) and on a static
    // one (the program answers it however the notice is classified).
    for (const bool keyedLadder : {true, false}) {
        const std::string what = std::string("an edited space rest (") +
                                 (keyedLadder ? "keyed" : "static") + ")";
        EvaluatorPair pair;
        if (!MakeEvaluatorPair(what.c_str(), keyedLadder ? keyed : plain,
                               &pair)) {
            continue;
        }
        const double frame = 5.0;
        RigExecRigPose before, after;
        compare(what + " before", &pair, frame, &before);
        for (const UsdStageRefPtr &stage :
             {pair.baselineStage, pair.bakedStage}) {
            UsdEditContext session(stage, stage->GetSessionLayer());
            UsdAttribute a =
                stage->GetPrimAtPath(kIkSpaceMaster)
                    .CreateAttribute(TfToken("rest:tx"),
                                     SdfValueTypeNames->Double);
            if (keyedLadder) {
                a.Set(6.0, UsdTimeCode(10.0));
            } else {
                a.Set(1.5);
            }
        }
        compare(what + " after", &pair, frame, &after);
        RequireJointMoved(what.c_str(), after, before, kIkSpaceWrist);
        RequireJointMoved(what.c_str(), after, before, kIkSpaceTailEnd);
    }
    std::printf("a moved space rest: %zu of %zu generation(s) disagreed\n",
                disagreeing, generations);
}

// TestASolverRefreshesOnlyForItsOwnRests.
//
// The Solve step refreshes a solver's rest description only on a run on
// which a rest it measures from moved: one of its `restSlots`, the space
// slot among them. On computed_ik_space a keyed rest on the leg's Hip joint
// moves LegIK's rests and no other solver's, and a keyed rest on the Master
// control moves ArmIK's space rest alone (and TailIK's, whose space and
// spline controls sit under Master). Either way the ladder varies, which is
// the case where a program-wide gate would refresh every solver on every
// run. Counters are Solver::restRefreshes, which excludes the cone
// verifier's second pass.

const SdfPath kIkSpaceHipJoint("/IkSpaceAsset/Rig/Joints/Hip");
const SdfPath kIkSpaceArmIk("/IkSpaceAsset/Rig/Solvers/ArmIK");
const SdfPath kIkSpaceLegIk("/IkSpaceAsset/Rig/Solvers/LegIK");
const SdfPath kIkSpaceReachIk("/IkSpaceAsset/Rig/Solvers/ReachIK");
const SdfPath kIkSpaceTailIk("/IkSpaceAsset/Rig/Solvers/TailIK");

/// computed_ik_space with \p attribute on \p prim keyed {1: 0, 10: \p last}
/// in the session layer.
MakeStage
KeyedIkSpaceRest(const std::string &examples, const SdfPath &prim,
                 const char *attribute, double last)
{
    const std::string file =
        examples + "/../tests/fixtures/computed_ik_space.usda";
    const std::string name(attribute);
    return [file, prim, name, last] {
        const UsdStageRefPtr stage = UsdStage::Open(file);
        if (!stage) return stage;
        UsdEditContext session(stage, stage->GetSessionLayer());
        UsdAttribute a = stage->GetPrimAtPath(prim).CreateAttribute(
            TfToken(name), SdfValueTypeNames->Double);
        a.Set(0.0, UsdTimeCode(1.0));
        a.Set(last, UsdTimeCode(10.0));
        return stage;
    };
}

/// Evaluates \p make at frames 1, 5 and 10 against dynamic, then requires
/// a refresh of every solver in \p refreshing and none of every solver in
/// \p still.
void
CheckOwnRestRefreshes(const char *what, const MakeStage &make,
                      const std::vector<SdfPath> &refreshing,
                      const std::vector<SdfPath> &still)
{
    EvaluatorPair pair;
    if (!MakeEvaluatorPair(what, make, &pair)) return;
    RigExecRigPose dynamic;
    for (const double frame : {1.0, 5.0, 10.0}) {
        GenerationsAgree(std::string(what) + " frame " +
                             std::to_string(int(frame)),
                         &pair, frame, &dynamic);
    }
    const RigExecBakedProgram *program = pair.baked->GetBakedProgram();
    if (!program) {
        ++failures;
        std::printf("FAIL %s: no baked program\n", what);
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    if (!B.ladderVarying) {
        ++failures;
        std::printf("FAIL %s: the ladder does not vary, so the case says "
                    "nothing\n",
                    what);
    }
    const auto refreshesOf = [&B](const SdfPath &path) -> long long {
        for (const SolverDesc &s : B.solvers) {
            if (s.path == path) return (long long)s.restRefreshes;
        }
        return -1;
    };
    for (const SdfPath &path : refreshing) {
        const long long n = refreshesOf(path);
        std::printf("%s: %s refreshed %lld time(s)\n", what, path.GetText(),
                    n);
        if (n <= 0) {
            ++failures;
            std::printf("FAIL %s: %s never refreshed its rests\n", what,
                        path.GetText());
        }
    }
    for (const SdfPath &path : still) {
        const long long n = refreshesOf(path);
        std::printf("%s: %s refreshed %lld time(s)\n", what, path.GetText(),
                    n);
        if (n != 0) {
            ++failures;
            std::printf("FAIL %s: %s refreshed although none of its rests "
                        "moved\n",
                        what, path.GetText());
        }
    }
}

void
TestASolverRefreshesOnlyForItsOwnRests(const std::string &examples)
{
    CheckOwnRestRefreshes(
        "a keyed rest on the leg's hip joint",
        KeyedIkSpaceRest(examples, kIkSpaceHipJoint, "rest:tx", 1.5),
        {kIkSpaceLegIk}, {kIkSpaceArmIk, kIkSpaceReachIk, kIkSpaceTailIk});
    CheckOwnRestRefreshes(
        "a keyed rest on the arm's space",
        KeyedIkSpaceRest(examples, kIkSpaceMaster, "rest:ry", 20.0),
        {kIkSpaceArmIk, kIkSpaceTailIk}, {kIkSpaceLegIk, kIkSpaceReachIk});
}

// The space-rest candidates. The refresh writes Solver::spaceRest from
// B.restPts[spaceSlot], so it changes a value only where that rest moves
// after Build: a varying, connected or dragged rest channel on the space
// slot or a ladder ancestor, seen here both from the channels and from what
// the runs leave in restPts. Listed apart: solvers whose space slot is at or
// under one of their own joints, where dynamic's computeRestFrame overrides
// (rigEvaluatorDynamic.cpp, restInputs) can move the space rest and baked's
// ladder does not; that gap is outside the refresh.

struct CandidateRig {
    std::string name;
    MakeStage make;
    std::vector<double> frames;
    /// The registered control drag, or empty.
    SdfPath dragPrim;
    std::string dragAttribute;
};

std::vector<double>
ParseFrames(const std::string &text)
{
    std::vector<double> frames;
    size_t at = 0;
    while (at < text.size()) {
        const size_t comma = text.find(',', at);
        const std::string item = text.substr(
            at, comma == std::string::npos ? std::string::npos : comma - at);
        if (!item.empty()) frames.push_back(std::stod(item));
        if (comma == std::string::npos) break;
        at = comma + 1;
    }
    return frames;
}

void
ListSpaceRestCandidates(const std::vector<CandidateRig> &rigs)
{
    size_t spaced = 0;
    std::vector<std::string> moving;
    std::vector<std::string> underJoint;
    for (const CandidateRig &rig : rigs) {
        const UsdStageRefPtr stage = rig.make();
        if (!stage) {
            ++failures;
            std::printf("FAIL %s: the stage does not open\n",
                        rig.name.c_str());
            continue;
        }
        SdfPathVector roots;
        for (const UsdPrim &prim : stage->Traverse()) {
            if (prim.GetTypeName() == "RigExecRoot") {
                roots.push_back(prim.GetPath());
            }
        }
        for (const SdfPath &root : roots) {
            const std::string where = rig.name + " " + root.GetString();
            RigExecRigEvaluator evaluator(stage, root);
            std::vector<std::string> errors;
            if (!evaluator.Compile(&errors)) {
                std::printf("%s: does not compile, not listed\n",
                            where.c_str());
                continue;
            }
            std::vector<std::string> reasons;
            const std::unique_ptr<RigExecBakedProgram> built =
                RigExecBakedProgram::Build(&evaluator, &reasons);
            if (!built) {
                std::printf("%s: does not bake, not listed\n", where.c_str());
                continue;
            }
            const RigExecBakedProgramImpl &build = built->GetStepGraph();
            std::vector<size_t> spaceSolvers;
            for (size_t i = 0; i < build.solvers.size(); ++i) {
                if (build.solvers[i].spaceSlot >= 0) spaceSolvers.push_back(i);
            }
            if (spaceSolvers.empty()) continue;
            spaced += spaceSolvers.size();

            std::vector<std::string> why(build.solvers.size());
            for (const size_t i : spaceSolvers) {
                for (int slot = build.solvers[i].spaceSlot; slot >= 0;
                     slot = build.parent[size_t(slot)]) {
                    const SdfPath &path = build.paths[size_t(slot)];
                    const UsdPrim prim = stage->GetPrimAtPath(path);
                    if (!prim) continue;
                    for (const UsdAttribute &a : prim.GetAttributes()) {
                        const std::string name = a.GetName().GetString();
                        if (name.rfind("rest:", 0) != 0) continue;
                        if (a.ValueMightBeTimeVarying()) {
                            why[i] += " " + a.GetPath().GetString() +
                                      " varies;";
                        }
                        if (a.HasAuthoredConnections()) {
                            why[i] += " " + a.GetPath().GetString() +
                                      " is connected;";
                        }
                        if (path == rig.dragPrim &&
                            name == rig.dragAttribute) {
                            why[i] += " " + a.GetPath().GetString() +
                                      " is the registered drag;";
                        }
                    }
                }
            }

            std::vector<bool> moved(build.solvers.size(), false);
            for (const double frame : rig.frames) {
                evaluator.Evaluate(UsdTimeCode(frame));
                const RigExecBakedProgram *program =
                    evaluator.GetBakedProgram();
                if (!program) continue;
                const RigExecBakedProgramImpl &B = program->GetStepGraph();
                if (B.paths != build.paths) {
                    ++failures;
                    std::printf("FAIL %s: the run's program numbers its "
                                "slots differently from Build's\n",
                                where.c_str());
                    break;
                }
                for (const size_t i : spaceSolvers) {
                    const size_t slot = size_t(build.solvers[i].spaceSlot);
                    if (moved[i] ||
                        SameBits(B.restPts[slot], build.restPts[slot])) {
                        continue;
                    }
                    moved[i] = true;
                    char text[64];
                    std::snprintf(text, sizeof(text), "%g", frame);
                    why[i] += std::string(" its rest differs from Build's "
                                          "at frame ") +
                              text + ";";
                }
            }

            for (const size_t i : spaceSolvers) {
                const SolverDesc &s = build.solvers[i];
                const SdfPath &space = build.paths[size_t(s.spaceSlot)];
                const std::string solver = where + " " + s.path.GetString() +
                                           " (space " + space.GetString() +
                                           ")";
                std::printf("%s: names a space\n", solver.c_str());
                if (!why[i].empty()) moving.push_back(solver + ":" + why[i]);
                SdfPathVector named;
                if (const UsdPrim prim = stage->GetPrimAtPath(s.path)) {
                    if (const UsdRelationship rel = prim.GetRelationship(
                            TfToken("rigExec:joints"))) {
                        rel.GetTargets(&named);
                    }
                }
                for (const SdfPath &joint : named) {
                    if (space.HasPrefix(joint.GetPrimPath())) {
                        underJoint.push_back(
                            solver + ": at or under " + joint.GetString() +
                            (s.hasLiveRest ? " (live rest)"
                                           : " (no live rest)"));
                    }
                }
            }
        }
    }
    std::printf("space-rest candidates: %zu solver(s) name a space\n",
                spaced);
    std::printf("whose space rest can move after Build: %zu\n",
                moving.size());
    for (const std::string &line : moving) {
        std::printf("    %s\n", line.c_str());
    }
    std::printf("whose space slot is at or under one of their joints: %zu\n",
                underJoint.size());
    for (const std::string &line : underJoint) {
        std::printf("    %s\n", line.c_str());
    }
    if (spaced == 0) {
        ++failures;
        std::printf("FAIL no rig names a solver space, so the list says "
                    "nothing\n");
    }
}

}  // namespace

static std::string
SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecSolverBake <examplesDir>\n");
        return 2;
    }
    const std::string resources = SchemaResourceDir(argv[1]);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    // The affected-fixture candidates for the space-rest refresh, listed
    // by their own ctest entry.
    if (argc > 2 && std::string(argv[2]) == "--space-rest-candidates") {
        std::vector<CandidateRig> rigs;
        for (const auto &[what, make] : RestFixtureRigs(argv[1])) {
            std::vector<double> frames{1, 3, 7};
            if (const UsdStageRefPtr stage = make()) {
                if (stage->HasAuthoredTimeCodeRange()) {
                    const double start = stage->GetStartTimeCode();
                    const double end = stage->GetEndTimeCode();
                    frames = {start, 0.5 * (start + end), end};
                }
            }
            rigs.push_back({what, make, frames, SdfPath(), std::string()});
        }
        for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
            const std::string path =
                std::string(argv[1]) + "/" + fixture.stage;
            rigs.push_back(
                {std::string("example ") + fixture.stage,
                 [path] { return UsdStage::Open(path); },
                 ParseFrames(fixture.frames),
                 *fixture.controlPrim ? SdfPath(fixture.controlPrim)
                                      : SdfPath(),
                 fixture.controlAvar});
        }
        ListSpaceRestCandidates(rigs);
        if (failures) {
            std::printf("testRigExecSolverBake: %d FAILURE(S)\n", failures);
            return 1;
        }
        std::printf("testRigExecSolverBake: all tests passed\n");
        return 0;
    }

    const std::vector<double> frames{1, 2, 3, 4, 5};

    CheckEvaluatorConsistency("twist distribution", MakeTwistRig, frames, true);
    CheckEvaluatorConsistency("twist distribution with keyed turns",
                MakeTwistRigWithKeyedTurns, frames, true);
    CheckEvaluatorConsistency("twist distribution with unauthored weights",
                MakeTwistRigWithUnauthoredWeights, frames, true);
    CheckEvaluatorConsistency("twist distribution with one sample",
                MakeTwistRigWithOneSample, frames, true);
    CheckEvaluatorConsistency("twist distribution with no end", MakeTwistRigWithNoEnd,
                frames, true);

    CheckEvaluatorConsistency("ribbon on a keyed driver curve", MakeAnimatedRibbonRig,
                frames, true);
    CheckEvaluatorConsistency("ribbon on a static driver curve", MakeStaticRibbonRig,
                frames, true);
    CheckEvaluatorConsistency("ribbon whose driver curve has no bind pose",
                MakeRibbonRigWithNoBindPose, frames, true);
    CheckEvaluatorConsistency("ribbon with no driver curve", MakeRibbonRigWithNoDriver,
                frames, true);

    CheckEvaluatorConsistency("two-bone ik", MakeTwoBoneIkRig, frames, true);
    CheckEvaluatorConsistency("two-bone ik binding two joints",
                MakeTwoBoneIkRigWithTwoJoints, frames, true);

    CheckEvaluatorConsistency("two-bone ik with a non-provider pole",
                MakeTwoBoneIkRigWithANonProviderPole, frames, true);
    CheckEvaluatorConsistency("fk chain with a non-provider control",
                MakeFkChainWithANonProviderControl, frames, true);

    CheckEvaluatorConsistency("spline ik", MakeSplineIkRig, frames, true);
    CheckEvaluatorConsistency("spline ik with an unsupported restLength",
                MakeSplineIkRigWithUnsupportedRestLength, frames, true);
    CheckEvaluatorConsistency("spline ik with an unsupported rootTangent",
                MakeSplineIkRigWithUnsupportedRootTangent, frames, true);

    CheckEvaluatorConsistency("ik/fk blend", MakeBlendRig, frames, true);
    CheckEvaluatorConsistency("ik/fk blend with a linear rotation blend",
                MakeBlendRigWithLinearRotation, frames, true);
    CheckEvaluatorConsistency("ik/fk blend with a non-solver input",
                MakeBlendRigWithANonSolverInput, frames, true);
    CheckEvaluatorConsistency("ik/fk blend with a linear rotation blend AND a non-solver "
                "input",
                MakeBlendRigWithLinearRotationAndANonSolverInput, frames,
                true);

    CheckEvaluatorConsistency("guide-only ribbon", MakeGuideOnlyRibbonRig, frames, true);
    CheckEvaluatorConsistency("guide-only twist distribution", MakeGuideOnlyTwistRig,
                frames, true);
    CheckEvaluatorConsistency("guide-only blend of two guide-only solvers",
                MakeGuideOnlyBlendRig, frames, true);
    CheckEvaluatorConsistency("guide-only solvers with the guides switched off",
                MakeGuideOnlyBlendRig, frames, true,
                /* guides = */ false);

    // A blend reads its inputs' aggregates from the generation it answers,
    // whatever came before: another frame, or a drag on an input solver.
    {
        using rigExecTest::EvaluationState;
        const auto drag = [](const char *prim, const char *attribute,
                             const VtValue &held) {
            return EvaluationState{
                UsdTimeCode(2.0),
                {RigExecValueOverride{SdfPath(prim), TfToken(),
                                      TfToken(attribute), held}}};
        };
        const EvaluationState frame2{UsdTimeCode(2.0), {}};
        const EvaluationState frame5{UsdTimeCode(5.0), {}};
        const EvaluationState footDrag =
            drag("/Asset/Rig/Controls/FootIK", "avars:ty", VtValue(3.0));
        const EvaluationState turnsDrag =
            drag("/Asset/Rig/Solvers/SecondTwist", "inputs:twistTurns",
                 VtValue(0.2));
        {
            const auto check = [&](const char *what, const MakeStage &make,
                                   const EvaluationState &before) {
                rigExecTest::CheckHistoryIndependent(
                    &failures, what, make, kRigPath, before, frame2);
            };
            check("ik/fk blend after another frame", MakeBlendRig, frame5);
            check("ik/fk blend after a drag on its ik", MakeBlendRig,
                  footDrag);
            check("linear-rotation blend after another frame",
                  MakeBlendRigWithLinearRotation, frame5);
            check("linear-rotation blend after a drag on its ik",
                  MakeBlendRigWithLinearRotation, footDrag);
            check("guide-only blend after another frame",
                  MakeGuideOnlyBlendRig, frame5);
            check("guide-only blend after a drag on an input",
                  MakeGuideOnlyBlendRig, turnsDrag);
        }
    }

    // The drags: one per per-frame input this group added to a solver.
    CheckDrag("a drag on inputs:twistTurns", MakeTwistRig,
              SdfPath("/Asset/Rig/Solvers/SpineTwist"), "inputs:twistTurns",
              VtValue(0.4));
    CheckDrag("a drag on rigExec:sampleCount", MakeAnimatedRibbonRig,
              SdfPath("/Asset/Rig/Solvers/SpineRibbon"),
              "rigExec:sampleCount", VtValue(int(4)));
    CheckDrag("a drag on a guide-only solver's turns", MakeGuideOnlyBlendRig,
              SdfPath("/Asset/Rig/Solvers/SecondTwist"), "inputs:twistTurns",
              VtValue(0.2));
    CheckRibbonPointsOverrideIsRefused();

    // The driver curve edited after the bake, in both of its halves.
    CheckRibbonPointsEdit("an edit to the driver curve's bind pose",
                          /* editDefault = */ true);
    CheckRibbonPointsEdit("an edit to a driver curve time sample",
                          /* editDefault = */ false);
    CheckRibbonPointsCreatedAfterTheBake();

    // Every fixture whose solvers measure from a rest.
    {
        size_t recomposed = 0;
        for (const auto &[what, make] : RestFixtureRigs(argv[1])) {
            recomposed += TestRefreshSolverRestsOnBuildStateIsIdentity(
                what.c_str(), make);
        }
        // A skipped refresh while another solver's rests move is the case
        // the per-solver gate exists for, so it has to have been exercised.
        if (recomposed == 0) {
            ++failures;
            std::printf("FAIL no solver kept Build's rests on a run that "
                        "moved another rest\n");
        }
    }

    TestASpaceRestMoveReachesTheSolve(argv[1]);
    TestASolverRefreshesOnlyForItsOwnRests(argv[1]);

    if (failures) {
        std::printf("testRigExecSolverBake: %d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecSolverBake: all tests passed\n");
    return 0;
}
