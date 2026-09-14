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
            const std::vector<double> &frames, bool expectBaked)
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
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    baked.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);

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
    return stage;
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

    if (failures) {
        std::printf("testRigExecSolverBake: %d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecSolverBake: all tests passed\n");
    return 0;
}
