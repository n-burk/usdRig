//
// `uniform bool rigExec:baked` on the RigExecRoot: the rig asking to be
// answered by the baked program.
//
// The mode has three possible authors and they do not carry the same weight
// (RigExecEvaluationModeSource). This suite is about the WEAKEST of them --
// the one an asset carries -- and therefore about the precedence as much as
// about the attribute: what it does when nothing else asked, what happens to
// it when something did, and what a rig that asks and cannot be given the
// program is told. The VALUES are not this suite's subject; they are proven
// exactly, on every shipped rig, by testRigExecBakedMode and the parity
// entries. What is asserted here is that turning the program on through the
// stage reaches the same answer as turning it on through the API, and that
// nothing about the rest of the ladder moved.
//
// Run twice by ctest. The second run sets RIGEXEC_EVALUATION_MODE=dynamic
// and passes --expect-environment: the variable is read once per process, so
// "the environment outranks the attribute" cannot be tested in the same
// binary as "the attribute decides".
//
// argv[1] = path to the examples directory (only to locate the codeless
// schema plugin at <examples>/../plugin/rigExecSchema/resources; every
// fixture here is built in memory).
//
#include "rigExecPoseCompare.h"

#include "rigExec/bakedProgram.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
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

static const TfToken kBaked("rigExec:baked");
static const SdfPath kRigPath("/Asset/Rig");

// Whether the attribute is authored on the fixture, and as what. Three
// states and not two: an ABSENT attribute and an authored `false` mean the
// same thing for the mode and different things for the source, which is the
// distinction the evaluator draws with HasAuthoredValue.
enum class Authored { Nothing, False, True };

static void
CompareEveryMap(const std::string &where, const RigExecRigPose &reference,
                const RigExecRigPose &baked)
{
    rigExecTest::CompareEveryMap(&failures, where, reference, baked);
}

// One rig, small enough to read and complete enough to bake: a control, a
// joint, and a skinned slab, which is one pose domain and one geometry
// domain.
//
// \p connectedSpace builds the shape testRigExecBakedMode's
// TestANonBakeableRigFallsBack uses -- a provider whose posed:space is an
// exec answer computed from the middle of the pose walk, which is the one
// refusal no feature group is going to remove. It is here so that "the
// attribute asked and the program could not" is reachable without a fixture
// whose refusal a later phase might delete.
static UsdStageRefPtr
MakeARig(Authored baked, bool connectedSpace = false)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    const UsdPrim rig = stage->DefinePrim(kRigPath, TfToken("RigExecRoot"));
    if (baked != Authored::Nothing) {
        // Through GetAttribute rather than CreateAttribute: the schema
        // declares the attribute, and authoring it the way an artist's
        // layer does is the only authoring this feature is about.
        const UsdAttribute attribute = rig.GetAttribute(kBaked);
        if (!attribute) {
            return UsdStageRefPtr();
        }
        attribute.Set(baked == Authored::True);
    }
    const UsdPrim control = stage->DefinePrim(SdfPath("/Asset/Rig/Root"),
                                              TfToken("RigExecControl"));
    control.GetAttribute(TfToken("avars:tx")).Set(3.0);
    const UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Bone"),
                                            TfToken("RigExecJoint"));
    joint.GetAttribute(TfToken("avars:ty")).Set(2.0);
    if (connectedSpace) {
        // The connection is to the control's own posed:space, which is the
        // identity the joint would have composed anyway: the rig still
        // evaluates to the pose a plain one does, and the only thing that
        // changed is that an epoch constant became an exec answer.
        joint.CreateAttribute(TfToken("posed:space"),
                              SdfValueTypeNames->Matrix4d)
            .AddConnection(control.GetPath().AppendProperty(
                TfToken("posed:space")));
    }

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/Slab"),
                                           TfToken("Mesh"));
    VtVec3fArray points{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0)};
    mesh.GetAttribute(TfToken("points")).Set(points);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({joint.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(1);
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(VtIntArray{0, 0, 0});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1.0f, 1.0f, 1.0f});
    return stage;
}

// Compiles, reporting the errors rather than swallowing them: every fixture
// here is built in this file, so a mistake in one would otherwise read as
// the feature failing.
static bool
CompileOrReport(const char *what, RigExecRigEvaluator *rig)
{
    std::vector<std::string> errors;
    if (rig->Compile(&errors)) {
        return true;
    }
    ++failures;
    std::printf("FAIL %s: the fixture does not compile\n", what);
    for (const std::string &error : errors) {
        std::printf("    %s\n", error.c_str());
    }
    return false;
}

// A plain dynamic evaluator on an identical stage: the answer the attribute
// must not change.
static RigExecRigPose
DynamicAnswer(const char *what, const UsdStageRefPtr &stage, UsdTimeCode time)
{
    RigExecRigEvaluator rig(stage, kRigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    if (!CompileOrReport(what, &rig)) {
        return RigExecRigPose();
    }
    return rig.Evaluate(time);
}

// ---------------------------------------------------------------------------
// (a) The attribute asks, and the program answers.
// ---------------------------------------------------------------------------

static void
TestTheAttributeCompilesIntoTheProgram()
{
    const char *const what = "rigExec:baked = true";
    const UsdStageRefPtr stage = MakeARig(Authored::True);
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRigPath);
    // Nothing has been asked of the evaluator yet, so it is still in the
    // mode every evaluator starts in. The attribute is read at Compile.
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Default);
    if (!CompileOrReport(what, &rig)) return;
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Attribute);

    // A fallback is a correct answer and would make every comparison below
    // compare the dynamic path with itself.
    std::vector<std::string> reasons;
    if (!rig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not bake\n", what);
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return;
    }
    CHECK(rig.GetBakedProgramBuildCount() == 1);

    const UsdStageRefPtr referenceStage = MakeARig(Authored::Nothing);
    CHECK(referenceStage);
    if (!referenceStage) return;
    RigExecRigEvaluator referenceRig(referenceStage, kRigPath);
    referenceRig.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    if (!CompileOrReport(what, &referenceRig)) return;
    // The reference asked for nothing and authored nothing, so it is the
    // rig this feature has to leave alone.
    CHECK(referenceRig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);

    for (double frame = 1; frame <= 4; ++frame) {
        const RigExecRigPose reference =
            referenceRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose baked = rig.Evaluate(UsdTimeCode(frame));
        CHECK(reference.valid);
        CHECK(baked.valid);
        // Real generations, not two empty poses agreeing perfectly.
        CHECK(!baked.jointFramesFinal.empty());
        CHECK(!baked.movedProperties.empty());
        CompareEveryMap(std::string(what) + " frame " +
                            std::to_string(int(frame)),
                        reference, baked);
        CHECK(baked.bakedParityMismatches == 0);
        CHECK(baked.diagnostics == reference.diagnostics);
    }
    // Every one of them came from the program, which is the whole claim.
    CHECK(rig.GetBakedGenerationCount() == 4);
    CHECK(referenceRig.GetBakedGenerationCount() == 0);
}

// An authored FALSE is a rig saying "not this one", and says it loudly
// enough to keep the source: it is an answer over a reference arc that says
// otherwise, not the absence of one.
static void
TestAnAuthoredFalseKeepsTheSourceAndTheDynamicPath()
{
    const char *const what = "rigExec:baked = false";
    const UsdStageRefPtr stage = MakeARig(Authored::False);
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRigPath);
    if (!CompileOrReport(what, &rig)) return;
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Attribute);
    CHECK(rig.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(rig.GetBakedGenerationCount() == 0);
    // Nothing was built for it either: a mode of Dynamic is not a program
    // standing by unused.
    CHECK(rig.GetBakedProgramBuildCount() == 0);
}

// ---------------------------------------------------------------------------
// (b) Flipped under a running evaluator.
// ---------------------------------------------------------------------------

static void
TestFlippingTheAttributeMovesTheRig()
{
    const char *const what = "rigExec:baked flipped at runtime";
    const UsdStageRefPtr stage = MakeARig(Authored::True);
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRigPath);
    if (!CompileOrReport(what, &rig)) return;
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
    CHECK(rig.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(rig.GetBakedGenerationCount() == 1);

    const UsdAttribute attribute =
        stage->GetPrimAtPath(kRigPath).GetAttribute(kBaked);
    CHECK(attribute);
    if (!attribute) return;

    // true -> false. The notice is where this is seen: rigExec:baked is a
    // VALUE, so the epoch digest does not move and no recompile follows it.
    CHECK(attribute.Set(false));
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Attribute);
    const size_t afterTheFlip = rig.GetBakedGenerationCount();
    const RigExecRigPose dynamic = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(dynamic.valid);
    // The generation came from the dynamic path, and the program is gone
    // rather than merely unused.
    CHECK(rig.GetBakedGenerationCount() == afterTheFlip);
    // And it is the right answer, not just a differently-produced one.
    CompareEveryMap(std::string(what) + ", dropped",
                    DynamicAnswer(what, MakeARig(Authored::False),
                                  UsdTimeCode(1.0)),
                    dynamic);

    // false -> true, on the fly: no recompile, and the next generation is
    // the program's.
    CHECK(attribute.Set(true));
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Attribute);
    const RigExecRigPose rebuilt = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(rebuilt.valid);
    CHECK(rig.GetBakedGenerationCount() == afterTheFlip + 1);
    CompareEveryMap(std::string(what) + ", rebuilt", dynamic, rebuilt);
}

// ---------------------------------------------------------------------------
// (c) A caller that chose deliberately outranks the stage.
// ---------------------------------------------------------------------------

static void
TestAnExplicitModeOutranksTheAttribute()
{
    // Dynamic asked for, on a rig that asks for the program.
    {
        const char *const what = "SetEvaluationMode(Dynamic) over baked=true";
        const UsdStageRefPtr stage = MakeARig(Authored::True);
        CHECK(stage);
        if (!stage) return;
        RigExecRigEvaluator rig(stage, kRigPath);
        // The mode asked for is the one the evaluator is already in, and it
        // still has to take ownership: what the call settles is who decides.
        rig.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
        CHECK(rig.GetEvaluationModeSource() ==
              RigExecEvaluationModeSource::Explicit);
        if (!CompileOrReport(what, &rig)) return;
        CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
        CHECK(rig.GetEvaluationModeSource() ==
              RigExecEvaluationModeSource::Explicit);
        CHECK(rig.Evaluate(UsdTimeCode(1.0)).valid);
        CHECK(rig.GetBakedGenerationCount() == 0);
        CHECK(rig.GetBakedProgramBuildCount() == 0);
        // And a later edit to the attribute does not take the decision back.
        CHECK(stage->GetPrimAtPath(kRigPath).GetAttribute(kBaked).Set(false));
        CHECK(stage->GetPrimAtPath(kRigPath).GetAttribute(kBaked).Set(true));
        CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
        CHECK(rig.GetEvaluationModeSource() ==
              RigExecEvaluationModeSource::Explicit);
        CHECK(rig.Evaluate(UsdTimeCode(1.0)).valid);
        CHECK(rig.GetBakedGenerationCount() == 0);
    }
    // And the other direction: the program asked for on a rig that declined
    // it. A tool that knows what it is doing is not overruled by an asset.
    {
        const char *const what = "SetEvaluationMode(Baked) over baked=false";
        const UsdStageRefPtr stage = MakeARig(Authored::False);
        CHECK(stage);
        if (!stage) return;
        RigExecRigEvaluator rig(stage, kRigPath);
        rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
        if (!CompileOrReport(what, &rig)) return;
        CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
        CHECK(rig.GetEvaluationModeSource() ==
              RigExecEvaluationModeSource::Explicit);
        const RigExecRigPose baked = rig.Evaluate(UsdTimeCode(1.0));
        CHECK(baked.valid);
        CHECK(rig.GetBakedGenerationCount() == 1);
        CompareEveryMap(what,
                        DynamicAnswer(what, MakeARig(Authored::False),
                                      UsdTimeCode(1.0)),
                        baked);
    }
}

// ---------------------------------------------------------------------------
// (d) The rig asked and could not be given the program.
// ---------------------------------------------------------------------------

// The line the fallback publishes, exactly as
// RigExecRigEvaluator::_ReportAttributeBakeFallback assembles it.
static std::string
FallbackPrefix()
{
    return "rigExec:baked is set on " + kRigPath.GetString() +
        " but this generation was evaluated dynamically: ";
}

static void
TestAnUnbakeableRigWithTheAttributeSaysSo()
{
    const char *const what = "baked=true on a connected posed:space";
    const UsdStageRefPtr stage = MakeARig(Authored::True,
                                          /* connectedSpace = */ true);
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRigPath);
    if (!CompileOrReport(what, &rig)) return;
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Attribute);
    std::vector<std::string> reasons;
    CHECK(!rig.IsBakeable(&reasons));
    CHECK(!reasons.empty());

    const UsdStageRefPtr referenceStage =
        MakeARig(Authored::Nothing, /* connectedSpace = */ true);
    CHECK(referenceStage);
    if (!referenceStage) return;
    RigExecRigEvaluator referenceRig(referenceStage, kRigPath);
    referenceRig.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    if (!CompileOrReport(what, &referenceRig)) return;

    for (double frame = 1; frame <= 4; ++frame) {
        const std::string where =
            std::string(what) + " frame " + std::to_string(int(frame));
        const RigExecRigPose reference =
            referenceRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose fallen = rig.Evaluate(UsdTimeCode(frame));
        CHECK(fallen.valid);
        CHECK(!fallen.jointFramesFinal.empty());
        // Every published value is the dynamic path's, because it IS the
        // dynamic path's: the announcement rides on the generation and
        // changes nothing about it.
        CompareEveryMap(where, reference, fallen);

        size_t announced = 0;
        std::vector<std::string> quiet;
        for (const std::string &line : fallen.diagnostics) {
            if (line.rfind(FallbackPrefix(), 0) == 0) {
                ++announced;
                // The reason is the actionable half, and a line that named
                // no feature would be a fallback report that cannot be acted
                // on.
                CHECK(line.find("connected posed:space") != std::string::npos);
            } else {
                quiet.push_back(line);
            }
        }
        // Exactly one per generation, and nothing else on the generation
        // moved.
        CHECK(announced == 1);
        CHECK(quiet == reference.diagnostics);
        // NOT a parity mismatch. An attribute is a request; the harness's
        // failure signal belongs to RIGEXEC_BAKE_REQUIRED alone.
        CHECK(fallen.bakedParityMismatches == 0);
        CHECK(FallbackPrefix().find("baked parity mismatch") ==
              std::string::npos);
    }
    // Nothing ran baked, and the request still stands: a rig that declines
    // has not had its mode taken away.
    CHECK(rig.GetBakedGenerationCount() == 0);
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Attribute);
}

// ---------------------------------------------------------------------------
// (e) The environment outranks the attribute. Its own ctest entry, because
// RIGEXEC_EVALUATION_MODE is read once per process.
// ---------------------------------------------------------------------------

static void
TestTheEnvironmentOutranksTheAttribute()
{
    const char *const what = "RIGEXEC_EVALUATION_MODE=dynamic over baked=true";
    // The entry exists to run under that variable; a run without it would
    // assert the opposite of the feature and pass.
    const std::string requested = TfGetenv("RIGEXEC_EVALUATION_MODE", "");
    if (requested != "dynamic") {
        ++failures;
        std::printf("FAIL %s: RIGEXEC_EVALUATION_MODE is \"%s\", not "
                    "\"dynamic\"; this entry has to be registered with it\n",
                    what, requested.c_str());
        return;
    }
    const UsdStageRefPtr stage = MakeARig(Authored::True);
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator rig(stage, kRigPath);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Environment);
    if (!CompileOrReport(what, &rig)) return;
    // Compiled, which is where the attribute would have had its say, and it
    // did not get one.
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Environment);

    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(pose.valid);
    CHECK(rig.GetBakedGenerationCount() == 0);
    CHECK(rig.GetBakedProgramBuildCount() == 0);
    // Nothing was announced either: the fallback line is for a rig whose
    // ATTRIBUTE asked and did not get the program, and this one was
    // overruled before it asked.
    for (const std::string &line : pose.diagnostics) {
        CHECK(line.rfind(FallbackPrefix(), 0) != 0);
    }
    CompareEveryMap(what,
                    DynamicAnswer(what, MakeARig(Authored::Nothing),
                                  UsdTimeCode(1.0)),
                    pose);
    // The attribute cannot take the decision back mid-session either.
    CHECK(stage->GetPrimAtPath(kRigPath).GetAttribute(kBaked).Set(false));
    CHECK(stage->GetPrimAtPath(kRigPath).GetAttribute(kBaked).Set(true));
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Environment);
    CHECK(rig.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(rig.GetBakedGenerationCount() == 0);
}

static std::string
_SchemaResourceDir(const std::string &examplesDir)
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
        std::printf("usage: testRigExecBakedAttribute <examplesDir> "
                    "[--expect-environment]\n");
        return 2;
    }
    bool expectEnvironment = false;
    for (int i = 2; i < argc; ++i) {
        if (std::string(argv[i]) == "--expect-environment") {
            expectEnvironment = true;
        }
    }
    const std::string resources = _SchemaResourceDir(argv[1]);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    if (expectEnvironment) {
        TestTheEnvironmentOutranksTheAttribute();
    } else {
        TestTheAttributeCompilesIntoTheProgram();
        TestAnAuthoredFalseKeepsTheSourceAndTheDynamicPath();
        TestFlippingTheAttributeMovesTheRig();
        TestAnExplicitModeOutranksTheAttribute();
        TestAnUnbakeableRigWithTheAttributeSaysSo();
    }

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBakedAttribute: all tests passed\n");
    return 0;
}
