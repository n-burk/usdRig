//
// The baked program's solver half, on rigs the shipped examples do not have.
//
// Every solver the program expresses is exercised by some example, but only
// through a rig that ALSO carries geometry the program cannot bake yet -- the
// ribbon and twist stages decline for their mover operations, so a parity run
// on them compares the dynamic path with itself and proves nothing about the
// solver. The fixtures here are the same solvers with the geometry left off,
// so the first generation each one bakes is measured.
//
// They are also where the authoring errors live. Every solver computation
// answers a malformed binding with an empty aggregate and a warning that
// never reaches the pose, so the only observable consequence is that the
// joints it names fall back to their rest chains -- and the bake has to
// answer with the same empty aggregate rather than with a partial solve or a
// refusal. No shipped example is malformed, so these are the only rigs in
// the tree where that agreement is checked at all.
//
// argv[1] = path to the examples directory (for the schema plugin).
//
#include "rigExecPoseCompare.h"

#include "rigExec/bakedProgram.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <functional>
#include <memory>
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

// ---------------------------------------------------------------------------
// The check every fixture is run through.
// ---------------------------------------------------------------------------

/// Builds \p make twice and compares the two paths over \p frames.
///
/// The baked side runs in the parity mode, so the judge is
/// RigExecComparePoses itself rather than a second opinion assembled here;
/// the shared comparator then re-checks the published generation against a
/// separate dynamic evaluator, which is what catches a domain the parity
/// mode's own publication leaves empty on both sides.
///
/// \p expectBaked is the half that stops the whole thing passing vacuously:
/// a fixture whose rig declines would agree perfectly and say nothing.
void
CheckParity(const char *what, const MakeStage &make,
            const std::vector<double> &frames, bool expectBaked,
            bool guides = true)
{
    const UsdStageRefPtr referenceStage = make();
    const UsdStageRefPtr bakedStage = make();
    CHECK(referenceStage && bakedStage);
    if (!referenceStage || !bakedStage) return;

    RigExecRigEvaluator reference(referenceStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    // The oracle by name, so the comparison stays against the exec walk
    // whatever the default mode comes to run.
    reference.SetEvaluationMode(RigExecEvaluationMode::ExecReference);
    std::vector<std::string> errors;
    if (!reference.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    baked.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    // A headless consumer skips the guide request outright, and the program
    // has to skip its publication with it -- while still running the solvers
    // it would have published, because the toggle can move without the epoch
    // moving.
    reference.SetSolverGuidesEnabled(guides);
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
        const RigExecRigPose a = reference.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose b = baked.Evaluate(UsdTimeCode(frame));
        CHECK(a.valid && b.valid);
        if (b.bakedParityMismatches) {
            ++failures;
            std::printf("FAIL %s: %zu baked parity mismatch(es)\n",
                        where.c_str(), b.bakedParityMismatches);
            for (const std::string &line : b.diagnostics) {
                std::printf("    %s\n", line.c_str());
            }
        }
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

/// Compares the two paths while an interactive override stands on
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
    const UsdStageRefPtr referenceStage = make();
    const UsdStageRefPtr bakedStage = make();
    CHECK(referenceStage && bakedStage);
    if (!referenceStage || !bakedStage) return;

    RigExecRigEvaluator reference(referenceStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    std::vector<std::string> errors;
    if (!reference.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        return;
    }
    baked.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);

    const std::vector<RigExecValueOverride> drag{
        RigExecValueOverride{prim, TfToken(), TfToken(attribute), held}};
    // Settled, then held, then released: the release is the half a drag test
    // usually forgets, and the one a value captured at Build survives.
    const RigExecRigPose settledA = reference.Evaluate(UsdTimeCode(2.0));
    const RigExecRigPose settledB = baked.Evaluate(UsdTimeCode(2.0));
    rigExecTest::ComparePose(&failures, std::string(what) + " settled",
                             settledA, settledB);
    reference.SetInteractiveOverrides(drag);
    baked.SetInteractiveOverrides(drag);
    const RigExecRigPose heldA = reference.Evaluate(UsdTimeCode(2.0));
    const RigExecRigPose heldB = baked.Evaluate(UsdTimeCode(2.0));
    rigExecTest::ComparePose(&failures, std::string(what) + " held", heldA,
                             heldB);
    reference.ClearInteractiveOverrides();
    baked.ClearInteractiveOverrides();
    const RigExecRigPose freedA = reference.Evaluate(UsdTimeCode(2.0));
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

// ---------------------------------------------------------------------------
// Authoring helpers.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// RigExecTwistDistribution.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// RigExecRibbon.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// The latent guards: RigExecTwoBoneIk, RigExecSplineIk and the blend.
//
// Every one of these malformed bindings makes the computation warn and
// publish an EMPTY aggregate; the warning never reaches the pose, so the
// only observable consequence is that the joints fall back to their rest
// chains and say so. No shipped rig is malformed, so the bake's agreement
// with that is checked here and nowhere else.
// ---------------------------------------------------------------------------

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
//
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

// ---------------------------------------------------------------------------
// The driver curve, which is the one solver input that is scene data.
// ---------------------------------------------------------------------------

/// The driver curve's bind pose is folded into bake state, so a drag on its
/// points must be REFUSED rather than placed.
///
/// The dynamic path reads that attribute straight off the stage with
/// UsdAttribute::Get, so it ignores an override on it entirely; a program
/// that placed one would answer a question no other path asks. Refusing is
/// what sends the generation down the dynamic path instead.
///
/// Asked of the PROGRAM rather than through Evaluate on purpose: a
/// deliberate fallback reports itself as a parity mismatch on the pose when
/// RIGEXEC_BAKE_REQUIRED=1, which is how this suite is run.
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
    //
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
    const UsdStageRefPtr referenceStage = MakeRibbonRigWithNoPointsYet();
    const UsdStageRefPtr bakedStage = MakeRibbonRigWithNoPointsYet();
    CHECK(referenceStage && bakedStage);
    if (!referenceStage || !bakedStage) return;

    RigExecRigEvaluator reference(referenceStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    std::vector<std::string> errors;
    if (!reference.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        return;
    }
    baked.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    const RigExecRigPose before = baked.Evaluate(UsdTimeCode(3.0));
    CHECK(before.valid);
    if (baked.GetBakedGenerationCount() != 1) {
        ++failures;
        std::printf("FAIL %s: the first generation was not the program's\n",
                    what);
        return;
    }

    AuthorDriverPoints(referenceStage);
    AuthorDriverPoints(bakedStage);

    const RigExecRigPose a = reference.Evaluate(UsdTimeCode(3.0));
    const RigExecRigPose b = baked.Evaluate(UsdTimeCode(3.0));
    CHECK(a.valid && b.valid);
    if (b.bakedParityMismatches) {
        ++failures;
        std::printf("FAIL %s: %zu baked parity mismatch(es)\n", what,
                    b.bakedParityMismatches);
        for (const std::string &line : b.diagnostics) {
            std::printf("    %s\n", line.c_str());
        }
    }
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

/// Compares the two paths across an edit to the driver curve's points made
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
    const UsdStageRefPtr referenceStage = MakeAnimatedRibbonRig();
    const UsdStageRefPtr bakedStage = MakeAnimatedRibbonRig();
    CHECK(referenceStage && bakedStage);
    if (!referenceStage || !bakedStage) return;

    RigExecRigEvaluator reference(referenceStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    std::vector<std::string> errors;
    if (!reference.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        return;
    }
    baked.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    const RigExecRigPose before = baked.Evaluate(UsdTimeCode(3.0));
    CHECK(before.valid);
    // The program answered that generation, so what follows is a comparison
    // between the two paths and not between the dynamic path and itself.
    if (baked.GetBakedGenerationCount() != 1) {
        ++failures;
        std::printf("FAIL %s: the first generation was not the program's\n",
                    what);
        return;
    }

    const VtVec3fArray edited{GfVec3f(0, 0, 0), GfVec3f(1.1f, 2.7f, 0),
                              GfVec3f(2.3f, 5.3f, 0), GfVec3f(3.5f, 7.6f, 0)};
    for (const UsdStageRefPtr &stage : {referenceStage, bakedStage}) {
        UsdAttribute points = stage->GetAttributeAtPath(
            SdfPath("/Asset/Geom/SpineCurve.points"));
        CHECK(points);
        if (editDefault) {
            points.Set(edited);
        } else {
            points.Set(edited, UsdTimeCode(3.0));
        }
    }

    const RigExecRigPose a = reference.Evaluate(UsdTimeCode(3.0));
    const RigExecRigPose b = baked.Evaluate(UsdTimeCode(3.0));
    CHECK(a.valid && b.valid);
    if (b.bakedParityMismatches) {
        ++failures;
        std::printf("FAIL %s: %zu baked parity mismatch(es)\n", what,
                    b.bakedParityMismatches);
        for (const std::string &line : b.diagnostics) {
            std::printf("    %s\n", line.c_str());
        }
    }
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

    const std::vector<double> frames{1, 2, 3, 4, 5};

    CheckParity("twist distribution", MakeTwistRig, frames, true);
    CheckParity("twist distribution with keyed turns",
                MakeTwistRigWithKeyedTurns, frames, true);
    CheckParity("twist distribution with unauthored weights",
                MakeTwistRigWithUnauthoredWeights, frames, true);
    CheckParity("twist distribution with one sample",
                MakeTwistRigWithOneSample, frames, true);
    CheckParity("twist distribution with no end", MakeTwistRigWithNoEnd,
                frames, true);

    CheckParity("ribbon on a keyed driver curve", MakeAnimatedRibbonRig,
                frames, true);
    CheckParity("ribbon on a static driver curve", MakeStaticRibbonRig,
                frames, true);
    CheckParity("ribbon whose driver curve has no bind pose",
                MakeRibbonRigWithNoBindPose, frames, true);
    CheckParity("ribbon with no driver curve", MakeRibbonRigWithNoDriver,
                frames, true);

    CheckParity("two-bone ik", MakeTwoBoneIkRig, frames, true);
    CheckParity("two-bone ik binding two joints",
                MakeTwoBoneIkRigWithTwoJoints, frames, true);

    CheckParity("two-bone ik with a non-provider pole",
                MakeTwoBoneIkRigWithANonProviderPole, frames, true);
    CheckParity("fk chain with a non-provider control",
                MakeFkChainWithANonProviderControl, frames, true);

    CheckParity("spline ik", MakeSplineIkRig, frames, true);
    CheckParity("spline ik with an unsupported restLength",
                MakeSplineIkRigWithUnsupportedRestLength, frames, true);
    CheckParity("spline ik with an unsupported rootTangent",
                MakeSplineIkRigWithUnsupportedRootTangent, frames, true);

    CheckParity("ik/fk blend", MakeBlendRig, frames, true);
    CheckParity("ik/fk blend with a linear rotation blend",
                MakeBlendRigWithLinearRotation, frames, true);
    CheckParity("ik/fk blend with a non-solver input",
                MakeBlendRigWithANonSolverInput, frames, true);
    CheckParity("ik/fk blend with a linear rotation blend AND a non-solver "
                "input",
                MakeBlendRigWithLinearRotationAndANonSolverInput, frames,
                true);

    CheckParity("guide-only ribbon", MakeGuideOnlyRibbonRig, frames, true);
    CheckParity("guide-only twist distribution", MakeGuideOnlyTwistRig,
                frames, true);
    CheckParity("guide-only blend of two guide-only solvers",
                MakeGuideOnlyBlendRig, frames, true);
    CheckParity("guide-only solvers with the guides switched off",
                MakeGuideOnlyBlendRig, frames, true,
                /* guides = */ false);

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

    if (failures) {
        std::printf("testRigExecSolverBake: %d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecSolverBake: all tests passed\n");
    return 0;
}
