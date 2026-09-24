//
// What Dynamic dispatches to (unified-program spec, HP-D0: rules D1, D2, D4).
//
// RIGEXEC_DYNAMIC_RUNS_PROGRAM decides whether Dynamic -- the mode every rig
// that does not author rigExec:baked, and usdview, run in -- answers from the
// baked program or from the exec walk. The evaluator reads it once per
// process, so this suite is registered twice, with the variable unset and set,
// and every assertion is written against RigExecDynamicRunsProgram() so that
// one binary states both halves:
//
//   * off, Dynamic builds no program and is the walk, as it always was;
//   * on, Dynamic builds the program wherever Baked would, publishes the
//     program's generations, and those are exactly the oracle's
//     (ExecReference, which builds no program either way);
//   * on, a generation the program cannot answer runs the walk and says
//     nothing about it: Dynamic never asked for the program, so the walk is
//     its answer rather than a fallback;
//   * on, Compile defers the walk-only exec preparations for Dynamic exactly
//     as it does for Baked, and never for ExecReference.
//
// And what every mode that runs the program does when a run gives the
// generation back (HP-D2, rule D3), which needs no flag to be seen:
//
//   * a bail only the walk can answer -- a rig left mid-edit, whose program
//     bails on every frame -- is answered by the walk with a valid pose,
//     identical to the oracle's, and costs one build for the epoch rather
//     than one per frame; the next stage edit asks again;
//   * an unresolvable constraint target is the program's own answer: the
//     run leaves the walk's invalid pose, keeps the program, and the run
//     after it runs everything.
//
// argv[1] = path to the examples directory.
//
#include "rigExecPoseCompare.h"

#include "rigExec/bakedProgram.h"
#include "rigExec/profiler.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
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

static SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

// A control's double avar, the thing an artist drags.
static UsdAttribute
FindControlAvar(const UsdStageRefPtr &stage, const SdfPath &rig)
{
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rig))) {
        if (prim.GetTypeName() != "RigExecControl") {
            continue;
        }
        for (const UsdAttribute &attr : prim.GetAttributes()) {
            if (TfStringStartsWith(attr.GetName().GetString(), "avars:") &&
                attr.GetTypeName() == SdfValueTypeNames->Double) {
                return attr;
            }
        }
    }
    return UsdAttribute();
}

static double
FirstFrame(const UsdStageRefPtr &stage)
{
    return stage->HasAuthoredTimeCodeRange() ? stage->GetStartTimeCode()
                                             : 1.0;
}

// Whether the last Compile prepared the walk-only main request, which is
// what a deferred epoch leaves for the first walk generation.
static bool
CompilePreparedTheWalk(const RigExecRigEvaluator &rig)
{
    for (const RigExecProfileEvent &event : rig.GetProfiler().GetEvents()) {
        if (event.name == "TapPrepare main") {
            return true;
        }
    }
    return false;
}

// Nothing a Dynamic generation publishes may announce a fallback: both lines
// belong to a mode that ASKED for the program.
static bool
AnnouncesAFallback(const RigExecRigPose &pose)
{
    for (const std::string &line : pose.diagnostics) {
        if (TfStringContains(line, "bake required") ||
            TfStringContains(line, "rigExec:baked is set")) {
            return true;
        }
    }
    return false;
}

// \p pose against \p reference, with the comparator the parity mode uses.
static void
CheckIsTheOracle(const std::string &what, const RigExecRigPose &reference,
                 const RigExecRigPose &pose)
{
    RigExecRigPose judged = reference;
    judged.diagnostics.clear();
    judged.bakedParityMismatches = 0;
    RigExecComparePoses(reference, pose, &judged);
    if (judged.bakedParityMismatches != 0) {
        ++failures;
        std::printf("FAIL %s: %zu mismatch(es) against ExecReference\n",
                    what.c_str(), judged.bakedParityMismatches);
        for (const std::string &line : judged.diagnostics) {
            std::printf("    %s\n", line.c_str());
        }
    }
    CHECK(!AnnouncesAFallback(pose));
}

// A default rig (no rigExec:baked) in Dynamic, frame by frame and under a
// drag, against the oracle on its own stage.
static void
TestADefaultRigRunsTheProgramOnlyUnderTheFlag(const std::string &examplesDir,
                                              const std::string &stageName)
{
    const std::string stagePath = examplesDir + "/" + stageName;
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    const UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
    CHECK(stage && referenceStage);
    if (!stage || !referenceStage) return;
    const SdfPath rigPath = FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) return;
    const bool runs = RigExecDynamicRunsProgram();

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetProfilingEnabled(true);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
    CHECK(rig.GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Default);
    CHECK((rig.GetBakedProgram() != nullptr) == runs);
    // Deferred exactly where the program is the answer.
    CHECK(CompilePreparedTheWalk(rig) == !runs);

    RigExecRigEvaluator reference(referenceStage, rigPath);
    reference.SetProfilingEnabled(true);
    reference.SetEvaluationMode(RigExecEvaluationMode::ExecReference);
    CHECK(reference.Compile(&errors));
    CHECK(reference.GetBakedProgram() == nullptr);
    CHECK(CompilePreparedTheWalk(reference));

    const double first = FirstFrame(stage);
    const size_t frames = 6;
    for (size_t i = 0; i < frames; ++i) {
        const UsdTimeCode time(first + double(i));
        CheckIsTheOracle(stageName + " frame " + TfStringify(time.GetValue()),
                         reference.Evaluate(time), rig.Evaluate(time));
    }
    CHECK(rig.GetBakedGenerationCount() == (runs ? frames : 0));
    CHECK(reference.GetBakedGenerationCount() == 0);

    // A drag: placed by the program under the flag, as Baked places it.
    const UsdAttribute avar = FindControlAvar(stage, rigPath);
    if (avar) {
        const UsdTimeCode time(first);
        double value = 0.0;
        avar.Get(&value, time);
        const std::vector<RigExecValueOverride> drag{RigExecValueOverride{
            avar.GetPrim().GetPath(), TfToken(), avar.GetName(),
            VtValue(value + 0.25)}};
        rig.SetInteractiveOverrides(drag);
        reference.SetInteractiveOverrides(drag);
        const size_t before = rig.GetBakedGenerationCount();
        CheckIsTheOracle(stageName + " drag " + avar.GetPath().GetString(),
                         reference.Evaluate(time), rig.Evaluate(time));
        CHECK(rig.GetBakedGenerationCount() == before + (runs ? 1 : 0));

        // An override no program can place: the walk answers, silently.
        //
        // Judged against an oracle whose walk is as warm as the one that
        // answers. Under the flag every earlier generation came from the
        // program, so this is the first walk generation this evaluator runs
        // and it reports its mover graphs as created -- which a fresh oracle
        // does too, and the one that has walked every frame above does not.
        const std::vector<RigExecValueOverride> unplaceable{
            RigExecValueOverride{avar.GetPrim().GetPath(), TfToken(),
                                 TfToken("custom:notAnInput"),
                                 VtValue(1.0)}};
        const UsdStageRefPtr freshStage = UsdStage::Open(stagePath);
        RigExecRigEvaluator fresh(freshStage, rigPath);
        fresh.SetEvaluationMode(RigExecEvaluationMode::ExecReference);
        CHECK(fresh.Compile(&errors));
        RigExecRigEvaluator &judge = runs ? fresh : reference;
        rig.SetInteractiveOverrides(unplaceable);
        judge.SetInteractiveOverrides(unplaceable);
        const size_t held = rig.GetBakedGenerationCount();
        CheckIsTheOracle(stageName + " unplaceable override",
                         judge.Evaluate(time), rig.Evaluate(time));
        CHECK(rig.GetBakedGenerationCount() == held);
        rig.SetInteractiveOverrides({});
        reference.SetInteractiveOverrides({});
    }
}

// Every way Dynamic is chosen follows the flag (rule D1), and every switch
// into and out of it keeps the program exactly when the new mode runs one
// (rule D2).
static void
TestEverySourceOfDynamicFollowsTheFlag(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/ArmShotAnim.usda";
    const bool runs = RigExecDynamicRunsProgram();
    const UsdTimeCode time(1001.0);
    std::vector<std::string> errors;

    // An authored rigExec:baked = false.
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage);
        if (!stage) return;
        const SdfPath rigPath = FindRig(stage);
        const UsdAttribute baked =
            stage->GetPrimAtPath(rigPath).CreateAttribute(
                TfToken("rigExec:baked"), SdfValueTypeNames->Bool);
        CHECK(baked.Set(false));
        RigExecRigEvaluator rig(stage, rigPath);
        CHECK(rig.Compile(&errors));
        CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
        CHECK(rig.GetEvaluationModeSource() ==
              RigExecEvaluationModeSource::Attribute);
        CHECK((rig.GetBakedProgram() != nullptr) == runs);
        CHECK(rig.Evaluate(time).valid);
        CHECK(rig.GetBakedGenerationCount() == (runs ? 1u : 0u));

        // The asset asks for the program, then withdraws the request. The
        // withdrawal drops the program only where Dynamic does not run one.
        CHECK(baked.Set(true));
        CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
        CHECK(rig.Evaluate(time).valid);
        CHECK(rig.GetBakedProgram() != nullptr);
        CHECK(baked.Set(false));
        CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
        CHECK((rig.GetBakedProgram() != nullptr) == runs);
        const size_t before = rig.GetBakedGenerationCount();
        CHECK(rig.Evaluate(time).valid);
        CHECK(rig.GetBakedGenerationCount() == before + (runs ? 1 : 0));
    }

    // SetEvaluationMode, on a compiled clean epoch: each switch leaves a
    // program exactly when the mode it lands in runs one.
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage);
        if (!stage) return;
        const SdfPath rigPath = FindRig(stage);
        RigExecRigEvaluator rig(stage, rigPath);
        CHECK(rig.Compile(&errors));
        rig.SetEvaluationMode(RigExecEvaluationMode::ExecReference);
        CHECK(rig.GetBakedProgram() == nullptr);
        size_t before = rig.GetBakedGenerationCount();
        CHECK(rig.Evaluate(time).valid);
        CHECK(rig.GetBakedGenerationCount() == before);

        rig.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
        CHECK(rig.GetEvaluationModeSource() ==
              RigExecEvaluationModeSource::Explicit);
        CHECK((rig.GetBakedProgram() != nullptr) == runs);
        before = rig.GetBakedGenerationCount();
        CHECK(rig.Evaluate(time).valid);
        CHECK(rig.GetBakedGenerationCount() == before + (runs ? 1 : 0));

        rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(rig.GetBakedProgram() != nullptr);
        rig.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
        CHECK((rig.GetBakedProgram() != nullptr) == runs);
    }
}

// The mapping itself, source by source: the oracle never runs the program,
// the two program modes always do, and Dynamic follows the flag from every
// source it can be chosen by.
static void
TestTheMapping()
{
    const bool runs = RigExecDynamicRunsProgram();
    for (RigExecEvaluationModeSource source :
         {RigExecEvaluationModeSource::Default,
          RigExecEvaluationModeSource::Attribute,
          RigExecEvaluationModeSource::Environment,
          RigExecEvaluationModeSource::Explicit}) {
        CHECK(RigExecEvaluationModeRunsProgram(
            RigExecEvaluationMode::Baked, source));
        CHECK(RigExecEvaluationModeRunsProgram(
            RigExecEvaluationMode::BakedWithParityCheck, source));
        CHECK(!RigExecEvaluationModeRunsProgram(
            RigExecEvaluationMode::ExecReference, source));
        CHECK(RigExecEvaluationModeRunsProgram(
                  RigExecEvaluationMode::Dynamic, source) == runs);
    }
    // Asking is unchanged by the flag: Dynamic never asks.
    CHECK(!RigExecEvaluationModeWantsProgram(RigExecEvaluationMode::Dynamic));
}

// A blend that names three joints over a chain of three controls, and a
// fourth joint below the last that no solver writes: it rides whichever
// blended joint is its closest ancestor. Nothing is wired to the blend's
// second input, so it passes the chain through.
static const char *const kMidEditBlend = R"usda(#usda 1.0
(
    defaultPrim = "Asset"
    startTimeCode = 1
    endTimeCode = 6
    upAxis = "Y"
)

def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        uniform token rigExec:partition = "Asset"

        def Scope "Controls"
        {
            def RigExecControl "C1"
            {
                double avars:rz.timeSamples = { 1: 0, 6: 30 }
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
                def RigExecControl "C2"
                {
                    double avars:rz.timeSamples = { 1: 0, 6: 20 }
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 4, 0, 1) )
                    def RigExecControl "C3"
                    {
                        double avars:rz.timeSamples = { 1: 0, 6: 10 }
                        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 4, 0, 1) )
                    }
                }
            }
        }

        def Scope "Solvers"
        {
            def RigExecFkChain "FK"
            {
                rel rigExec:controls = [
                    </Asset/Rig/Controls/C1>,
                    </Asset/Rig/Controls/C1/C2>,
                    </Asset/Rig/Controls/C1/C2/C3>,
                ]
            }
            def RigExecBlendPointFrames "Blend"
            {
                float inputs:weight = 0
                rel rigExec:inputA = </Asset/Rig/Solvers/FK>
                rel rigExec:joints = [
                    </Asset/Rig/Joints/A>,
                    </Asset/Rig/Joints/A/B>,
                    </Asset/Rig/Joints/A/B/C>,
                ]
            }
        }

        def Scope "Joints"
        {
            def RigExecJoint "A"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
                def RigExecJoint "B"
                {
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 4, 0, 1) )
                    def RigExecJoint "C"
                    {
                        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 4, 0, 1) )
                        def RigExecJoint "Tip"
                        {
                            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1) )
                        }
                    }
                }
            }
        }
    }
}
)usda";

static UsdStageRefPtr
OpenMidEditBlend()
{
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    if (!layer || !layer->ImportFromString(kMidEditBlend)) {
        return UsdStageRefPtr();
    }
    return UsdStage::Open(layer);
}

// The chain's controls, the last one included or not. Without it the chain
// publishes two frames while the blend still names three joints: the rig a
// rigger leaves between removing a control and re-pointing the joints. No
// compile check sees it -- a blend's cardinality is its input's, known only
// when it runs -- so the epoch compiles and the program bails on the third
// joint's child every frame.
static void
SetChainControls(const UsdStageRefPtr &stage, bool withLast)
{
    SdfPathVector controls{SdfPath("/Asset/Rig/Controls/C1"),
                           SdfPath("/Asset/Rig/Controls/C1/C2")};
    if (withLast) {
        controls.push_back(SdfPath("/Asset/Rig/Controls/C1/C2/C3"));
    }
    CHECK(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers/FK"))
              .GetRelationship(TfToken("rigExec:controls"))
              .SetTargets(controls));
}

static bool
Mentions(const RigExecRigPose &pose, const std::string &text)
{
    for (const std::string &line : pose.diagnostics) {
        if (TfStringContains(line, text)) {
            return true;
        }
    }
    return false;
}

// A persistent mid-edit bail, in each mode that runs the program: the walk
// answers every frame with a valid pose identical to the oracle's, and the
// epoch builds its program once rather than once per frame only to be given
// the same generation back.
static void
TestAPersistentBailBuildsOnce(RigExecEvaluationMode mode, const char *what)
{
    const UsdStageRefPtr stage = OpenMidEditBlend();
    const UsdStageRefPtr referenceStage = OpenMidEditBlend();
    CHECK(stage && referenceStage);
    if (!stage || !referenceStage) return;
    const SdfPath rigPath("/Asset/Rig");
    const bool runs = RigExecEvaluationModeRunsProgram(
        mode, RigExecEvaluationModeSource::Explicit);
    std::vector<std::string> errors;

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(mode);
    CHECK(rig.Compile(&errors));
    RigExecRigEvaluator reference(referenceStage, rigPath);
    reference.SetEvaluationMode(RigExecEvaluationMode::ExecReference);
    CHECK(reference.Compile(&errors));

    // Whole, the program answers.
    const std::string name = what;
    CheckIsTheOracle(name + " whole", reference.Evaluate(UsdTimeCode(1.0)),
                     rig.Evaluate(UsdTimeCode(1.0)));
    CHECK(rig.GetBakedGenerationCount() == (runs ? 1u : 0u));

    // Mid-edit: the recompile builds one program, which bails; the walk
    // answers that frame and every one after it, and nothing is built again.
    SetChainControls(stage, false);
    SetChainControls(referenceStage, false);
    const size_t attempts = rig.GetBakedProgramBuildAttemptCount();
    const size_t generations = rig.GetBakedGenerationCount();
    for (int frame = 1; frame <= 6; ++frame) {
        const UsdTimeCode time(frame);
        const RigExecRigPose expected = reference.Evaluate(time);
        const RigExecRigPose pose = rig.Evaluate(time);
        const std::string where =
            name + " mid-edit frame " + std::to_string(frame);
        CHECK(pose.valid);
        CHECK(Mentions(pose, "Blend published no element 2"));
        if (pose.diagnostics != expected.diagnostics) {
            ++failures;
            std::printf("FAIL %s: diagnostics differ from the oracle's\n",
                        where.c_str());
            for (const std::string &line : expected.diagnostics) {
                std::printf("    [reference] %s\n", line.c_str());
            }
            for (const std::string &line : pose.diagnostics) {
                std::printf("    [%s] %s\n", what, line.c_str());
            }
        }
        CheckIsTheOracle(where, expected, pose);
    }
    const size_t built = rig.GetBakedProgramBuildAttemptCount() - attempts;
    if (built != (runs ? 1u : 0u)) {
        ++failures;
        std::printf("FAIL %s: %zu program build(s) over six mid-edit "
                    "frames, expected %d\n",
                    what, built, runs ? 1 : 0);
    }
    CHECK(rig.GetBakedGenerationCount() == generations);

    // Finishing the edit is a stage edit, and the memo does not outlive one:
    // the next frame builds again and the program answers from then on.
    SetChainControls(stage, true);
    SetChainControls(referenceStage, true);
    for (int frame = 1; frame <= 3; ++frame) {
        const UsdTimeCode time(frame);
        CheckIsTheOracle(name + " finished frame " + std::to_string(frame),
                         reference.Evaluate(time), rig.Evaluate(time));
    }
    CHECK(rig.GetBakedGenerationCount() == generations + (runs ? 3u : 0u));
}

// An unresolvable constraint target, on the program itself. The live path
// does not reach it -- the settle recompiles for any edit that could cause
// one, and the compile refuses a target that is not a transform -- so the
// run is asked directly, between the edit and the settle, the way the
// frozen sampler asks the seed hook.
//
// The ribbon spine, with an aim constraint added that moves a plain Xform:
// the Xform is the target the stage frames read, and the ribbon's animated
// driver curve is per-run state the prologue consumes before it reaches
// them -- the points it swaps are what the next run compares against.
static void
TestAnUnresolvableTargetIsTheProgramsAnswer(const std::string &examplesDir)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/05_TwistRibbonSpine.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    const SdfPath target("/SpineAsset/Geom/Probe");
    const UsdPrim targetPrim = stage->DefinePrim(target, TfToken("Xform"));
    const UsdPrim aim = stage->DefinePrim(
        rigPath.AppendPath(SdfPath("Movers/ProbeAim")),
        TfToken("RigExecAimConstraint"));
    CHECK(targetPrim && aim);
    if (!targetPrim || !aim) return;
    CHECK(aim.CreateAttribute(TfToken("rigExec:aimAxis"),
                              SdfValueTypeNames->Token, false,
                              SdfVariabilityUniform)
              .Set(TfToken("z")));
    CHECK(aim.CreateRelationship(TfToken("rigExec:aimTarget"))
              .SetTargets({rigPath.AppendPath(SdfPath("Joints/Root/Chest"))}));
    CHECK(aim.CreateRelationship(TfToken("rigExec:moves"))
              .SetTargets({target}));
    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    // Two programs of one epoch: `kept` meets the bail, `twin` does not.
    const std::unique_ptr<RigExecBakedProgram> kept =
        RigExecBakedProgram::Build(&rig, nullptr);
    const std::unique_ptr<RigExecBakedProgram> twin =
        RigExecBakedProgram::Build(&rig, nullptr);
    CHECK(kept && twin);
    if (!kept || !twin) return;
    RigExecRigPose first, twinFirst;
    CHECK(kept->Run(UsdTimeCode(1001.0), &first));
    CHECK(kept->GetLastBail() == RigExecBakedBail::None);
    CHECK(twin->Run(UsdTimeCode(1001.0), &twinFirst));

    // The target stops being a transform. The run gives the generation back
    // where the walk does, and what it leaves is what the walk returns
    // there: the target's line last, nothing published, no mover graph.
    CHECK(targetPrim.SetTypeName(TfToken("Scope")));
    const UsdTimeCode bent(1024.0);
    RigExecRigPose given;
    CHECK(!kept->Run(bent, &given));
    CHECK(kept->GetLastBail() == RigExecBakedBail::StageFrames);
    CHECK(!given.valid);
    CHECK(!given.diagnostics.empty() &&
          given.diagnostics.back() ==
              "could not resolve constraint target " + target.GetString() +
                  " relative to the asset root");
    CHECK(!Mentions(given, "mover graph:"));
    CHECK(given.jointMatricesFinal.empty());
    CHECK(given.moverGraphRevisionsCreated == 0);
    CHECK(given.moverGraphSchedulesBuilt == 0);

    // Restored, at the same time: the program that bailed answers exactly
    // as the one that never saw the bail. The ribbon points the bailed
    // prologue swapped in are already this time's, so only a run of
    // everything -- the one a bail leaves owed -- re-samples the ribbon.
    CHECK(targetPrim.SetTypeName(TfToken("Xform")));
    RigExecRigPose after, twinAfter;
    CHECK(kept->Run(bent, &after));
    CHECK(kept->GetLastBail() == RigExecBakedBail::None);
    CHECK(twin->Run(bent, &twinAfter));
    CHECK(after.valid && twinAfter.valid);
    rigExecTest::CompareEveryMap(&failures, "ribbon spine after the bail",
                                 twinAfter, after);
}

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
        std::printf("usage: testRigExecDynamicDispatch <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    // Every assertion is about the mode a rig is in when nobody but the
    // stage chose it; a session-wide mode would make them about that instead.
    if (!TfGetenv("RIGEXEC_EVALUATION_MODE", "").empty()) {
        std::printf("FATAL: run without RIGEXEC_EVALUATION_MODE\n");
        return 2;
    }
    std::printf("RIGEXEC_DYNAMIC_RUNS_PROGRAM: %s\n",
                RigExecDynamicRunsProgram() ? "on" : "off");

    TestTheMapping();
    for (const char *stageName : {"ArmShotAnim.usda",
                                  "05_TwistRibbonSpine.usda",
                                  "11_VolumeWeights.usda",
                                  "components/spider_leg_ik.usd"}) {
        TestADefaultRigRunsTheProgramOnlyUnderTheFlag(examplesDir, stageName);
    }
    TestEverySourceOfDynamicFollowsTheFlag(examplesDir);
    TestAPersistentBailBuildsOnce(RigExecEvaluationMode::Baked, "baked");
    TestAPersistentBailBuildsOnce(RigExecEvaluationMode::BakedWithParityCheck,
                                  "parity");
    TestAPersistentBailBuildsOnce(RigExecEvaluationMode::Dynamic, "dynamic");
    TestAnUnresolvableTargetIsTheProgramsAnswer(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecDynamicDispatch: all passed\n");
    return 0;
}
