// The baked program's op trace: every step the last run executed, with the
// order in which its body finished. The trace is observability only, so what
// this suite holds it to is that it tells the truth about the run:
//   * it is a valid completion order over the step graph -- each executed
//     predecessor finished before its executed successor, under whichever
//     executor RIGEXEC_BAKED_SCHEDULE picked;
//   * it lists exactly the steps the run executed: the sources plus the
//     executed common graph operations;
//   * a first run reaches every evaluation domain the rig has;
//   * a repeated time executes nothing, in the sense the cone suite uses;
//   * the op graph it is checked against is acyclic and mirrors itself;
//   * the profiler replay labels each step event with its kind, domain and
//     sequence number;
//   * each timed op's memo and publication stamps bracket its body, whether
//     the profiler or op timing alone asked for them, and a measurement
//     (step timing or calibration alone) folds all three.
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

#include <algorithm>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
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
OpenRig(const std::string &stagePath, bool referenceChecks = false)
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
    rig.evaluator->cpuReference = referenceChecks;
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

/// The trace lists exactly the common engine's executed operations.
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
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    std::set<size_t> traced;
    for (const RigExecOpTraceEntry &entry : trace) {
        CHECK(entry.step < B.steps.size());
        if (entry.step >= B.steps.size()) continue;
        CHECK(B.opExecution.completion[entry.step] == entry.seq);
        traced.insert(entry.step);
    }
    size_t mismatches = 0;
    for (size_t k = 0; k < B.steps.size(); ++k) {
        const bool ran = k < B.opExecution.ran.size() && B.opExecution.ran[k];
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

/// Profiler rows report every timed body with its common completion sequence.
void
CheckProfilerReplay(const RigExecRigEvaluator &E,
                    const std::vector<RigExecOpTraceEntry> &trace,
                    const std::string &what)
{
    std::multiset<std::string> expected, recorded;
    for (const RigExecOpTraceEntry &entry : trace) {
        expected.insert(std::to_string(entry.seq));
    }
    size_t missingArgs = 0;
    for (const RigExecProfileEvent &event : E.GetProfiler().GetEvents()) {
        if (event.category != "op") {
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

/// Every timed op's memo and publication stamps bracket its body on the
/// body's clock, the trace reports exactly those phases, and under the
/// serial executor no two ops of one thread overlap from memo start to
/// publication end.
void
CheckOpPhases(const RigExecRigEvaluator &E,
              const std::vector<RigExecOpTraceEntry> &trace,
              const std::string &what)
{
    const RigExecBakedProgramImpl &B = E.GetBakedProgram()->GetStepGraph();
    std::map<std::string, std::vector<std::pair<uint64_t, uint64_t>>> lanes;
    size_t unbracketed = 0;
    for (const RigExecOpTraceEntry &entry : trace) {
        if (entry.step >= B.steps.size()) {
            continue;
        }
        const RigExecBakedStep &step = B.steps[entry.step];
        const uint64_t endUs = entry.startUs + entry.durationUs;
        const bool bracketed = entry.startUs != 0 && step.memoStartNs != 0 &&
            step.publishEndNs != 0 && step.memoStartNs / 1000 <= entry.startUs &&
            step.publishEndNs / 1000 >= endUs &&
            entry.memoUs == entry.startUs - step.memoStartNs / 1000 &&
            entry.publishUs == step.publishEndNs / 1000 - endUs;
        if (!bracketed) {
            if (unbracketed < 5) {
                std::printf("    %s: %s memo %llu ns, body %llu+%llu us, "
                            "publication end %llu ns\n", what.c_str(),
                            entry.label.c_str(),
                            (unsigned long long)step.memoStartNs,
                            (unsigned long long)entry.startUs,
                            (unsigned long long)entry.durationUs,
                            (unsigned long long)step.publishEndNs);
            }
            ++unbracketed;
            continue;
        }
        lanes[entry.thread].emplace_back(step.memoStartNs, step.publishEndNs);
    }
    if (unbracketed) {
        ++failures;
        std::printf("FAIL %s: %zu op(s) whose phases do not bracket the "
                    "body\n", what.c_str(), unbracketed);
    }
    if (B.opAdapter.parallel) {
        return;
    }
    size_t overlaps = 0;
    for (auto &lane : lanes) {
        std::sort(lane.second.begin(), lane.second.end());
        for (size_t i = 1; i < lane.second.size(); ++i) {
            overlaps += lane.second[i - 1].second > lane.second[i].first;
        }
    }
    if (overlaps) {
        ++failures;
        std::printf("FAIL %s: %zu serial op(s) overlap the previous op on "
                    "their thread\n", what.c_str(), overlaps);
    }
}

void
TestStage(const std::string &examplesDir, const std::string &file,
          double t0, double t1, bool expectTimeChange = true)
{
    const std::string path = examplesDir + "/" + file;
    LiveRig rig = OpenRig(path);
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
    CHECK(graph.size() == B.opGraph.ops.size());
    for (size_t i = 0; i < graph.size(); ++i) {
        CHECK(B.opGraph.ops[i].originalIndex == i);
        CHECK(B.opGraph.canonicalIndex[i] == int32_t(i));
        CHECK(graph[i].step == i);
    }
    Report(file + " op graph", rigExecTest::CheckOpGraphIsAcyclic(graph));
    Report(file + " first trace",
           rigExecTest::CheckTraceRespectsEdges(trace0, graph));
    CheckTraceMatchesClosure(E, trace0, file + " first trace");
    CheckProfilerReplay(E, trace0, file + " first trace");
    CheckOpPhases(E, trace0, file + " first trace");

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
    if (expectTimeChange) {
        CHECK(!trace1.empty());
    } else {
        // ArmRig is the static rig beneath ArmShotAnim. Moving time alone
        // changes no typed source; its cold first run above remains the
        // meaningful body/ordering/profiler proof, followed by literal idle.
        CHECK(trace1.empty());
        CHECK(second.executedOpCount == 0);
    }
    Report(file + " second trace",
           rigExecTest::CheckTraceRespectsEdges(trace1, graph));
    CheckTraceMatchesClosure(E, trace1, file + " second trace");
    CheckProfilerReplay(E, trace1, file + " second trace");
    CheckOpPhases(E, trace1, file + " second trace");

    // (c) The same time again: nothing moved, so no revision executed or
    // was created -- what the cone suite asserts of a repeated time -- and
    // the trace is still exactly the run's closure.
    E.ClearProfile();
    const RigExecRigPose third = E.Evaluate(UsdTimeCode(t1));
    CHECK(third.valid);
    CHECK(E.GetBakedGenerationCount() == 3);
    const std::vector<RigExecOpTraceEntry> trace2 = E.GetLastOpTrace();
    CHECK(third.executedOpCount == trace2.size());
    CHECK(trace2.empty());
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

/// Scalar reference checks validate the production graph without replacing it.
void
TestReferenceChecksUseProductionGraph(const std::string &examplesDir)
{
    LiveRig rig = OpenRig(examplesDir + "/ArmRig.usda", true);
    CHECK(rig.evaluator != nullptr);
    if (!rig.evaluator) return;
    const RigExecRigPose pose = rig.evaluator->Evaluate(UsdTimeCode(1001));
    CHECK(pose.valid);
    CHECK(pose.referenceMismatches == 0);
    CHECK(pose.referenceAgreements > 0);
    CHECK(!rig.evaluator->GetOpGraph().empty());
    CHECK(rig.evaluator->GetLastOpTrace().size() == pose.executedOpCount);
    CheckTraceMatchesClosure(*rig.evaluator, rig.evaluator->GetLastOpTrace(),
                             "reference checks");
}

/// A newly compiled artifact is observable before its first execution.
void
TestARebuiltProgramHasNoTraceUntilItRuns(const std::string &examplesDir)
{
    LiveRig rig = OpenRig(examplesDir + "/ArmRig.usda");
    CHECK(rig.evaluator != nullptr);
    if (!rig.evaluator) return;
    RigExecRigEvaluator &E = *rig.evaluator;
    CHECK(E.GetBakedProgram() != nullptr);
    CHECK(E.GetLastOpTrace().empty());
    CHECK(!E.GetOpGraph().empty());
    CHECK(E.Evaluate(UsdTimeCode(1001)).valid);
    CHECK(!E.GetLastOpTrace().empty());
    std::vector<std::string> errors;
    CHECK(E.Compile(&errors));
    CHECK(E.GetLastOpTrace().empty());
    CHECK(!E.GetOpGraph().empty());
    CHECK(E.Evaluate(UsdTimeCode(1024)).valid);
    CHECK(!E.GetLastOpTrace().empty());
}

/// Op timing alone, with the profiler off -- what a live inspection panel
/// turns on -- stamps every executed op's memo start and publication end,
/// and the trace reports them. Without a measurement it takes no other
/// stamp.
void
TestOpTimingAloneStampsOperationPhases(const std::string &examplesDir,
                                       const std::string &file, double t0,
                                       double t1)
{
    LiveRig rig = OpenRig(examplesDir + "/" + file);
    CHECK(rig.evaluator != nullptr);
    if (!rig.evaluator) return;
    RigExecRigEvaluator &E = *rig.evaluator;
    E.SetOpTimingEnabled(true);
    CHECK(!E.GetProfilingEnabled());
    const bool measuring = RigExecBakedStepTimingRequested() ||
                           RigExecBakedScheduleCalibrationRequested();
    for (const double t : {t0, t1}) {
        CHECK(E.Evaluate(UsdTimeCode(t)).valid);
        const RigExecBakedProgramImpl &B =
            E.GetBakedProgram()->GetStepGraph();
        CHECK(B.recordOpTimings);
        CHECK(!B.profiler || !B.profiler->IsEnabled());
        const std::vector<RigExecOpTraceEntry> trace = E.GetLastOpTrace();
        CHECK(!trace.empty());
        CheckOpPhases(E, trace, file + " op timing at " + std::to_string(t));
        if (!measuring) {
            for (const RigExecBakedStep &step : B.steps) {
                CHECK(step.memoEndNs == 0 && step.bodyEndNs == 0);
            }
        }
    }
}

/// A measurement -- RIGEXEC_BAKED_STEP_TIMING, or RIGEXEC_BAKED_SCHEDULE_
/// CALIBRATE on its own -- folds each op's memo, body and publication into
/// the op's accumulators after the join. Without one, and without op timing
/// or the profiler, no op takes a stamp and nothing accumulates.
void
TestMeasurementFoldsOperationPhases(const std::string &examplesDir)
{
    LiveRig rig = OpenRig(examplesDir + "/ArmShotAnim.usda");
    CHECK(rig.evaluator != nullptr);
    if (!rig.evaluator) return;
    RigExecRigEvaluator &E = *rig.evaluator;
    CHECK(!E.GetOpTimingEnabled());
    CHECK(E.Evaluate(UsdTimeCode(1001)).valid);
    CHECK(E.Evaluate(UsdTimeCode(1024)).valid);
    const RigExecBakedProgramImpl &B = E.GetBakedProgram()->GetStepGraph();
    const bool requested = RigExecBakedStepTimingRequested() ||
                           RigExecBakedScheduleCalibrationRequested();
    CHECK(B.opAdapter.measuring == requested);
    size_t bodies = 0, memos = 0, ranLast = 0;
    double publications = 0;
    for (size_t c = 0; c < B.steps.size(); ++c) {
        const RigExecBakedStep &step = B.steps[c];
        // Every body run followed a memo of the same op.
        CHECK(step.measuredMemoRuns >= step.measuredRuns);
        bodies += step.measuredRuns;
        memos += step.measuredMemoRuns;
        publications += step.measuredPublishUs;
        const bool ran = c < B.opExecution.ran.size() && B.opExecution.ran[c];
        ranLast += ran;
        if (requested && ran) {
            CHECK(step.measuredRuns > 0 && step.measuredMemoRuns > 0);
            CHECK(step.memoStartNs != 0 && step.memoEndNs >= step.memoStartNs);
            CHECK(step.bodyEndNs >= step.memoEndNs &&
                  step.publishEndNs >= step.bodyEndNs);
        }
        if (!requested) {
            CHECK(step.memoStartNs == 0 && step.memoEndNs == 0 &&
                  step.bodyEndNs == 0 && step.publishEndNs == 0);
            CHECK(step.measuredUs == 0 && step.measuredMemoUs == 0 &&
                  step.measuredPublishUs == 0);
        }
    }
    CHECK(ranLast > 0);
    if (requested) {
        CHECK(bodies > 0 && memos >= bodies && publications > 0);
    } else {
        CHECK(bodies == 0 && memos == 0);
    }
    std::printf("  measurement %s: %zu body run(s), %zu memo run(s), "
                "%.1f us of publication\n", requested ? "on" : "off",
                bodies, memos, publications);
}

/// An unresolved plain constraint target publishes invalid source state.
/// The graph records ordinary pure callback outcomes while publication is withheld.
void
TestInvalidTargetStillExecutesUnrelatedOperations(const std::string &examplesDir)
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
    const size_t callerExecutedOpCount = 91;
    given.executedOpCount = callerExecutedOpCount;
    CHECK(!program->Run(UsdTimeCode(1024.0), &given));
    CHECK(program->GetLastBail() == RigExecBakedBail::StageFrames);
    CHECK(!given.valid && given.jointMatricesFinal.empty());
    CHECK(given.providerXforms.empty() && given.providerBaseXforms.empty());
    const auto trace=program->GetLastOpTrace();
    CHECK(!trace.empty());
    CHECK(std::any_of(trace.begin(),trace.end(),[](const auto &entry) {
        return entry.kind=="Solve";
    }));
    CHECK(!given.diagnostics.empty() && given.diagnostics.back() ==
        "could not resolve constraint target " + target.GetString() +
        " relative to the asset root");
    const auto &state = program->GetStepGraph();
    // Refusal preserves caller metadata; the trace describes actual callbacks.
    CHECK(given.executedOpCount == callerExecutedOpCount);
    CHECK(trace.size() == size_t(std::count_if(
        state.opExecution.ran.begin(), state.opExecution.ran.end(),
        [](const auto ran) { return bool(ran); })));
    for (const auto &entry : trace) {
        CHECK(entry.step < state.opExecution.ran.size());
        if (entry.step < state.opExecution.ran.size())
            CHECK(state.opExecution.ran[entry.step]);
    }
    CHECK(targetPrim.SetTypeName(TfToken("Xform")));
    RigExecRigPose recovered;
    CHECK(program->Run(UsdTimeCode(1024.0), &recovered));
    CHECK(recovered.valid && program->GetLastBail() == RigExecBakedBail::None);
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
    TestStage(examplesDir, "ArmRig.usda", 1001, 1024, false);
    TestStage(examplesDir, "ArmShotAnim.usda", 1001, 1024);
    TestStage(examplesDir, "13_ReadPhases.usda", 1001, 1024);
    TestStage(examplesDir, "11_VolumeWeights.usda", 1001, 1024);
    TestStage(examplesDir, "04_BlendShapeFace.usda", 1001, 1024);
    TestStage(examplesDir, "biped/Biped_anim.usda", 1, 5);
    TestReferenceChecksUseProductionGraph(examplesDir);
    TestARebuiltProgramHasNoTraceUntilItRuns(examplesDir);
    TestOpTimingAloneStampsOperationPhases(examplesDir, "ArmShotAnim.usda",
                                           1001, 1024);
    TestOpTimingAloneStampsOperationPhases(examplesDir,
                                           "biped/Biped_anim.usda", 1, 5);
    TestMeasurementFoldsOperationPhases(examplesDir);
    TestInvalidTargetStillExecutesUnrelatedOperations(examplesDir);
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecOpTrace: all tests passed\n");
    return 0;
}
