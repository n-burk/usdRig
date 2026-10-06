// The baked program's op trace: every step the last run executed, with the
// order in which its body finished. The trace is observability only, so what
// this suite holds it to is that it tells the truth about the run:
//   * it is a valid completion order over the step graph -- each executed
//     predecessor finished before its executed successor, under whichever
//     executor RIGEXEC_BAKED_SCHEDULE picked;
//   * it lists exactly the steps the run executed: the sources plus the
//     closed steps;
//   * a first run reaches every evaluation domain the rig has;
//   * a repeated time executes nothing, in the sense the cone suite uses;
//   * the op graph it is checked against is acyclic and mirrors itself;
//   * the profiler replay labels each step event with its kind, domain and
//     sequence number.
// argv[1] = path to the examples directory.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/parallel.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include "rigExecOpTrace.h"

#include <cstdio>
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

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

struct LiveRig {
    UsdStageRefPtr stage;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
};

LiveRig
OpenRig(const std::string &stagePath, RigExecEvaluationMode mode)
{
    LiveRig rig;
    rig.stage = UsdStage::Open(stagePath);
    if (!rig.stage) {
        return rig;
    }
    const SdfPath rigPath = FindRig(rig.stage);
    if (rigPath.IsEmpty()) {
        return rig;
    }
    rig.evaluator = std::make_unique<RigExecRigEvaluator>(rig.stage, rigPath);
    rig.evaluator->SetEvaluationMode(mode);
    std::vector<std::string> errors;
    if (!rig.evaluator->Compile(&errors)) {
        rig.evaluator.reset();
    }
    return rig;
}

bool
Report(const std::string &what, const std::vector<std::string> &violations)
{
    if (violations.empty()) {
        return true;
    }
    ++failures;
    std::printf("FAIL %s: %zu violation(s)\n", what.c_str(),
                violations.size());
    for (size_t i = 0; i < violations.size() && i < 10; ++i) {
        std::printf("    %s\n", violations[i].c_str());
    }
    return false;
}

/// The trace lists exactly the steps the run executed: every source, and
/// every closed step. Read off the program's own closure, so it holds under
/// either executor.
void
CheckTraceMatchesClosure(const RigExecRigEvaluator &E,
                         const std::vector<RigExecOpTraceEntry> &trace,
                         const std::string &what)
{
    const RigExecBakedProgram *program = E.GetBakedProgram();
    if (!program) {
        ++failures;
        std::printf("FAIL %s: no program\n", what.c_str());
        return;
    }
    // Under RIGEXEC_BAKED_VERIFY_CONES the forced second pass leaves
    // `closedSteps` describing itself while the trace is restored to the
    // cone run, so there is no set to compare against.
    if (RigExecBakedVerifyConesRequested()) {
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    std::set<size_t> traced;
    for (const RigExecOpTraceEntry &entry : trace) {
        CHECK(entry.step < B.steps.size());
        if (entry.step >= B.steps.size()) continue;
        CHECK(B.steps[entry.step].runSeq == entry.seq);
        traced.insert(entry.step);
    }
    size_t mismatches = 0;
    for (size_t k = 0; k < B.steps.size(); ++k) {
        if (B.steps[k].isHead) {
            CHECK(!B.steps[k].isSource);
            CHECK(!B.closedSteps.Test(int(k)));
            CHECK(B.steps[k].startUs == 0 && B.steps[k].endUs == 0);
        }
        const bool ran =
            B.steps[k].isHead ? B.steps[k].runSeq != 0
                              : B.steps[k].isSource || B.closedSteps.Test(int(k));
        if (ran != (traced.count(k) != 0)) {
            if (mismatches < 5) {
                std::printf("    %s: step %zu (%s) %s\n", what.c_str(), k,
                            B.steps[k].label.c_str(),
                            ran ? "ran but is not traced"
                                : "is traced but did not run");
            }
            ++mismatches;
        }
    }
    if (mismatches) {
        ++failures;
        std::printf("FAIL %s: %zu step(s) disagree with the closure\n",
                    what.c_str(), mismatches);
    }
}

/// Under RIGEXEC_TRACE_ALL_STEPS every timed step that ran is replayed, so
/// the step events' seq arguments are exactly the ordinary non-source
/// seqs. Memo heads share body dispatch and stamps but retain untimed profiling.
void
CheckProfilerReplay(const RigExecRigEvaluator &E,
                    const std::vector<RigExecOpTraceEntry> &trace,
                    const std::string &what)
{
    const RigExecBakedProgramImpl &B = E.GetBakedProgram()->GetStepGraph();
    std::multiset<std::string> expected, recorded;
    for (const RigExecOpTraceEntry &entry : trace) {
        if (!B.steps[entry.step].isHead && !B.steps[entry.step].isSource) {
            expected.insert(std::to_string(entry.seq));
        }
    }
    size_t missingArgs = 0;
    for (const RigExecProfileEvent &event : E.GetProfiler().GetEvents()) {
        if (event.category != "step") {
            continue;
        }
        const auto kind = event.args.find("kind");
        const auto domain = event.args.find("domain");
        const auto seq = event.args.find("seq");
        if (kind == event.args.end() || domain == event.args.end() ||
            seq == event.args.end()) {
            ++missingArgs;
            continue;
        }
        recorded.insert(seq->second);
    }
    CHECK(missingArgs == 0);
    if (expected != recorded) {
        ++failures;
        std::printf("FAIL %s: %zu step event(s) replayed, %zu expected\n",
                    what.c_str(), recorded.size(), expected.size());
    }
}

void
TestStage(const std::string &examplesDir, const std::string &file,
          double t0, double t1)
{
    const std::string path = examplesDir + "/" + file;
    LiveRig rig = OpenRig(path, RigExecEvaluationMode::Baked);
    if (!rig.evaluator) {
        ++failures;
        std::printf("FAIL %s: does not compile\n", file.c_str());
        return;
    }
    RigExecRigEvaluator &E = *rig.evaluator;
    E.SetProfilingEnabled(true);

    // (a) Two different times, each a valid completion order.
    E.ClearProfile();
    const RigExecRigPose first = E.Evaluate(UsdTimeCode(t0));
    if (!first.valid || E.GetBakedGenerationCount() != 1) {
        ++failures;
        std::printf("FAIL %s: the first generation did not come from the "
                    "program\n", file.c_str());
        return;
    }
    const std::vector<RigExecOpGraphNode> graph = E.GetOpGraph();
    const std::vector<RigExecOpTraceEntry> trace0 = E.GetLastOpTrace();
    CHECK(!graph.empty());
    CHECK(!trace0.empty());
    const auto &B = E.GetBakedProgram()->GetStepGraph();
    size_t headCount = 0;
    while (headCount < B.steps.size() && B.steps[headCount].isHead) ++headCount;
    CHECK(headCount > 0);
    for (size_t i = 0; i < graph.size(); ++i) {
        CHECK(B.steps[i].isHead == (i < headCount));
        CHECK((graph[i].domain == "head") == B.steps[i].isHead);
        if (B.steps[i].isHead) CHECK(!B.steps[i].isSource);
    }
    Report(file + " op graph", rigExecTest::CheckOpGraphIsAcyclic(graph));
    Report(file + " first trace",
           rigExecTest::CheckTraceRespectsEdges(trace0, graph));
    CheckTraceMatchesClosure(E, trace0, file + " first trace");
    CheckProfilerReplay(E, trace0, file + " first trace");

    // (b) The first run reaches every domain the graph holds.
    for (const char *domain : {"pose", "weight", "geometry"}) {
        size_t inGraph = 0;
        for (const RigExecOpGraphNode &node : graph) {
            inGraph += node.domain == domain ? 1 : 0;
        }
        const size_t inTrace = rigExecTest::CountTraceDomain(trace0, domain);
        if (inGraph > 0 && inTrace == 0) {
            ++failures;
            std::printf("FAIL %s: the graph holds %zu %s step(s) and the "
                        "first run executed none\n",
                        file.c_str(), inGraph, domain);
        }
    }
    const size_t poseSteps = rigExecTest::CountTraceDomain(trace0, "pose");
    const size_t geometrySteps =
        rigExecTest::CountTraceDomain(trace0, "geometry");

    E.ClearProfile();
    const RigExecRigPose second = E.Evaluate(UsdTimeCode(t1));
    CHECK(second.valid);
    CHECK(E.GetBakedGenerationCount() == 2);
    const std::vector<RigExecOpTraceEntry> trace1 = E.GetLastOpTrace();
    CHECK(!trace1.empty());
    Report(file + " second trace",
           rigExecTest::CheckTraceRespectsEdges(trace1, graph));
    CheckTraceMatchesClosure(E, trace1, file + " second trace");
    CheckProfilerReplay(E, trace1, file + " second trace");

    // (c) The same time again: nothing moved, so no revision executed or
    // was created -- what the cone suite asserts of a repeated time -- and
    // the trace is still exactly the run's closure.
    E.ClearProfile();
    const RigExecRigPose third = E.Evaluate(UsdTimeCode(t1));
    CHECK(third.valid);
    CHECK(E.GetBakedGenerationCount() == 3);
    CHECK(third.moverGraphRevisionsExecuted == 0);
    CHECK(third.moverGraphRevisionsCreated == 0);
    const std::vector<RigExecOpTraceEntry> trace2 = E.GetLastOpTrace();
    Report(file + " repeated trace",
           rigExecTest::CheckTraceRespectsEdges(trace2, graph));
    CheckTraceMatchesClosure(E, trace2, file + " repeated trace");
    CHECK(trace2.size() <= trace1.size());
    // The step-grain form of the cone suite's "ran fewer clusters than the
    // program holds", phased reads included.
    CHECK(trace2.size() < trace0.size());

    std::printf("  %s: %zu step(s); first run %zu (%zu pose, %zu weight, "
                "%zu geometry), t=%g ran %zu, repeat ran %zu (%zu geometry), "
                "%zu of %zu cluster(s)\n",
                file.c_str(), graph.size(), trace0.size(), poseSteps,
                rigExecTest::CountTraceDomain(trace0, "weight"),
                geometrySteps, t1, trace1.size(), trace2.size(),
                rigExecTest::CountTraceDomain(trace2, "geometry"),
                E.GetBakedClustersRunLastGeneration(),
                E.GetBakedClusterCount());
}

/// The accessors answer for the program only: a generation the walk
/// answered has no op trace and no op graph.
void
TestAWalkGenerationHasNoTrace(const std::string &examplesDir)
{
    LiveRig rig = OpenRig(examplesDir + "/ArmRig.usda",
                          RigExecEvaluationMode::ExecReference);
    if (!rig.evaluator) {
        ++failures;
        std::printf("FAIL ArmRig (reference): does not compile\n");
        return;
    }
    const RigExecRigPose pose = rig.evaluator->Evaluate(UsdTimeCode(1001));
    CHECK(pose.valid);
    CHECK(rig.evaluator->GetBakedGenerationCount() == 0);
    CHECK(rig.evaluator->GetLastOpTrace().empty());
    CHECK(rig.evaluator->GetOpGraph().empty());
}

/// A program the evaluator holds but that did not answer the last
/// generation has no trace and no graph to report: switching a clean epoch
/// to the parity mode rebuilds the program, and the rebuilt one has run
/// nothing until the next generation.
void
TestARebuiltProgramHasNoTraceUntilItRuns(const std::string &examplesDir)
{
    LiveRig rig = OpenRig(examplesDir + "/ArmRig.usda",
                          RigExecEvaluationMode::Baked);
    if (!rig.evaluator) {
        ++failures;
        std::printf("FAIL ArmRig (rebuild): does not compile\n");
        return;
    }
    RigExecRigEvaluator &E = *rig.evaluator;
    CHECK(E.Evaluate(UsdTimeCode(1001)).valid);
    CHECK(E.GetBakedGenerationCount() == 1);
    CHECK(!E.GetLastOpTrace().empty());
    CHECK(!E.GetOpGraph().empty());

    E.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    CHECK(E.GetBakedProgram() != nullptr);
    CHECK(E.GetLastOpTrace().empty());
    CHECK(E.GetOpGraph().empty());

    CHECK(E.Evaluate(UsdTimeCode(1024)).valid);
    CHECK(!E.GetLastOpTrace().empty());
    CHECK(!E.GetOpGraph().empty());
}

/// A run that gives the generation back at the stage frames executes no
/// step, so the program's own trace is empty rather than the previous
/// run's. The rig is the dispatch suite's: the ribbon spine with an aim
/// constraint moving a plain Xform whose type then stops being a transform.
/// The live path recompiles before it can reach this bail, so the program
/// is run directly between the edit and the settle.
void
TestAStageFramesBailLeavesNoTrace(const std::string &examplesDir)
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
    const std::unique_ptr<RigExecBakedProgram> program =
        RigExecBakedProgram::Build(&rig, nullptr);
    CHECK(program);
    if (!program) return;
    RigExecRigPose first;
    CHECK(program->Run(UsdTimeCode(1001.0), &first));
    CHECK(!program->GetLastOpTrace().empty());

    CHECK(targetPrim.SetTypeName(TfToken("Scope")));
    RigExecRigPose given;
    CHECK(!program->Run(UsdTimeCode(1024.0), &given));
    CHECK(program->GetLastBail() == RigExecBakedBail::StageFrames);
    CHECK(program->GetLastOpTrace().empty());
    CHECK(!program->GetOpGraph().empty());
}

/// Precedes and the finders agree with the seq numbers they read.
void
TestTheHelpersReadTheTrace()
{
    std::vector<RigExecOpTraceEntry> trace(3);
    trace[0].step = 4; trace[0].seq = 1; trace[0].kind = "Solve";
    trace[0].domain = "pose"; trace[0].label = "Solve /A";
    trace[1].step = 2; trace[1].seq = 2; trace[1].kind = "RevisionFuse";
    trace[1].domain = "geometry"; trace[1].label = "RevisionFuse /B";
    trace[2].step = 7; trace[2].seq = 3; trace[2].kind = "Solve";
    trace[2].domain = "pose"; trace[2].label = "Solve /C";
    CHECK(rigExecTest::Precedes(trace, 4, 2));
    CHECK(!rigExecTest::Precedes(trace, 2, 4));
    CHECK(!rigExecTest::Precedes(trace, 4, 99));
    CHECK(rigExecTest::FindTraceEntries(trace, "Solve").size() == 2);
    CHECK(rigExecTest::FindTraceEntries(trace, "", "geometry").size() == 1);
    CHECK(rigExecTest::FindTraceEntries(trace, "", "", "/C").size() == 1);

    // A graph 4 -> 2 and 2 -> 7, then the same trace with 2 and 7 swapped.
    std::vector<RigExecOpGraphNode> graph(8);
    for (size_t i = 0; i < graph.size(); ++i) {
        graph[i].step = i;
    }
    graph[4].succs = {2};
    graph[2].preds = {4};
    graph[2].succs = {7};
    graph[7].preds = {2};
    // Not program-ordered, but a DAG: the checker does not assume
    // program order.
    CHECK(rigExecTest::CheckOpGraphIsAcyclic(graph).empty());
    CHECK(rigExecTest::CheckTraceRespectsEdges(trace, graph).empty());
    std::vector<RigExecOpTraceEntry> swapped = trace;
    swapped[1].seq = 3;
    swapped[2].seq = 2;
    CHECK(rigExecTest::CheckTraceRespectsEdges(swapped, graph).size() == 1);
    std::vector<RigExecOpTraceEntry> repeated = trace;
    repeated[2].seq = 2;
    CHECK(!rigExecTest::CheckTraceRespectsEdges(repeated, graph).empty());
    graph[4].preds = {7};
    graph[7].succs = {4};
    CHECK(!rigExecTest::CheckOpGraphIsAcyclic(graph).empty());
    graph[7].succs.clear();
    CHECK(!rigExecTest::CheckOpGraphIsAcyclic(graph).empty());
}

std::string
SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecOpTrace <examplesDir>\n");
        return 2;
    }
    // Before the first replay reads it: every timed step is then an event,
    // which is what CheckProfilerReplay counts against.
    TfSetenv("RIGEXEC_TRACE_ALL_STEPS", "1");
    const std::string examplesDir = argv[1];
    const std::string resources = SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    std::printf("testRigExecOpTrace: %s executor\n",
                RigExecBakedScheduleModeFromEnvironment() ==
                        RigExecBakedScheduleMode::Parallel
                    ? "parallel"
                    : "serial");
    TestTheHelpersReadTheTrace();
    TestStage(examplesDir, "ArmRig.usda", 1001, 1024);
    TestStage(examplesDir, "ArmShotAnim.usda", 1001, 1024);
    TestStage(examplesDir, "13_ReadPhases.usda", 1001, 1024);
    TestStage(examplesDir, "11_VolumeWeights.usda", 1001, 1024);
    TestStage(examplesDir, "04_BlendShapeFace.usda", 1001, 1024);
    TestStage(examplesDir, "biped/Biped_anim.usda", 1, 5);
    TestAWalkGenerationHasNoTrace(examplesDir);
    TestARebuiltProgramHasNoTraceUntilItRuns(examplesDir);
    TestAStageFramesBailLeavesNoTrace(examplesDir);
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecOpTrace: all tests passed\n");
    return 0;
}
