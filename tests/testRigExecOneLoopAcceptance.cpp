// Acceptance criteria for one interleaved evaluation loop, measured on the
// baked program's op trace and op graph (rigExec/bakedTrace.h) as it stands.
// Each criterion asserts what holds today. Where the criterion cannot hold
// yet, the test asserts the CURRENT behaviour instead, with a comment naming
// the migration stage that changes it, so that stage has to update the
// assertion on purpose rather than drift past it.
//
// Fixtures (tests/fixtures):
//   oneloop_cross_domain.usda  property clamp -> IK/FK blend -> constraint ->
//                              FK solver -> skin -> preceding sphere -> smooth
//   oneloop_two_limbs.usda     two limbs, one solver set and one skinned mesh
//                              each, sharing nothing
//   oneloop_cycle.usda         IK reads its effector at `final`; a constraint
//                              moves that effector from a volume weight riding
//                              the IK's end joint (an intra-pose loop through
//                              the volume's frame)
//
// Criteria:
//   (a) the trace crosses domains along the fixture's chain;
//   (b) a limb's geometry does not depend on the other limb's pose;
//   (c) the preceding weight is computed by an ordinary step after the
//       revision it measures;
//   (d) a property drag executes only the forward cone of its readers;
//   (e) a cycle is rejected: fixture C's intra-pose loop, and fixture A's
//       volume weight reading its target at `final`, a cross-domain loop.
// Fixtures A and B are also held to bit-for-bit dynamic/baked parity here,
// because tests/exampleFixtures.cmake covers examples/ only.
// argv[1] = path to tests/fixtures.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include "rigExecOpTrace.h"
#include "rigExecPoseCompare.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
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

const char *const kCrossDomain = "oneloop_cross_domain.usda";
const char *const kTwoLimbs = "oneloop_two_limbs.usda";
const char *const kCycle = "oneloop_cycle.usda";

const SdfPath kCrossRig("/CrossAsset/Rig");
const SdfPath kLimbsRig("/LimbsAsset/Rig");
const SdfPath kCycleRig("/CycleAsset/Rig");

bool
SerialExecutor()
{
    return RigExecBakedScheduleModeFromEnvironment() ==
           RigExecBakedScheduleMode::Serial;
}

struct LiveRig {
    UsdStageRefPtr stage;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
    std::vector<std::string> errors;
};

LiveRig
OpenRig(const std::string &stagePath, const SdfPath &rigPath,
        RigExecEvaluationMode mode)
{
    LiveRig rig;
    rig.stage = UsdStage::Open(stagePath);
    if (!rig.stage) {
        return rig;
    }
    rig.evaluator = std::make_unique<RigExecRigEvaluator>(rig.stage, rigPath);
    rig.evaluator->SetEvaluationMode(mode);
    if (!rig.evaluator->Compile(&rig.errors)) {
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

/// The one step of \p kind whose label contains \p label, or SIZE_MAX with
/// a failure when there is not exactly one.
size_t
OneStep(const std::vector<RigExecOpGraphNode> &graph, const std::string &kind,
        const std::string &label)
{
    const std::vector<size_t> found =
        rigExecTest::FindOpGraphSteps(graph, kind, label);
    if (found.size() != 1) {
        ++failures;
        std::printf("FAIL: expected one %s step labelled *%s*, found %zu\n",
                    kind.c_str(), label.c_str(), found.size());
        return SIZE_MAX;
    }
    return found.front();
}

void
PrintTrace(const std::string &what,
           const std::vector<RigExecOpTraceEntry> &trace)
{
    std::printf("  %s: %zu step(s)\n", what.c_str(), trace.size());
    for (const RigExecOpTraceEntry &entry : trace) {
        std::printf("    seq %3u step %3zu %-8s %-16s %s\n", entry.seq,
                    entry.step, entry.domain.c_str(), entry.kind.c_str(),
                    entry.label.c_str());
    }
}

// --- parity -----------------------------------------------------------------

/// Dynamic and baked answer every frame identically, forwards, backwards
/// and forwards again, and every baked generation came from the program.
/// With \p drag set, the drag is held for a second sweep and released for a
/// third.
///
/// Three evaluators over three stages. `checked` runs BakedWithParityCheck:
/// it executes the program and the dynamic path each generation, counts
/// their disagreements in bakedParityMismatches, and publishes the DYNAMIC
/// generation. `baked` runs Baked and publishes the program's own answer,
/// so ComparePose(reference, baked) is the bit-for-bit dynamic/baked check
/// on the published pose; ComparePose(reference, checked) checks that two
/// dynamic evaluations of one rig agree.
void
CheckParity(const std::string &fixturesDir, const char *file,
            const SdfPath &rigPath, const std::vector<double> &frames,
            const std::vector<RigExecValueOverride> &drag = {})
{
    const std::string path = fixturesDir + "/" + file;
    const UsdStageRefPtr referenceStage = UsdStage::Open(path);
    const UsdStageRefPtr checkedStage = UsdStage::Open(path);
    const UsdStageRefPtr bakedStage = UsdStage::Open(path);
    CHECK(referenceStage && checkedStage && bakedStage);
    if (!referenceStage || !checkedStage || !bakedStage) return;
    RigExecRigEvaluator reference(referenceStage, rigPath);
    RigExecRigEvaluator checked(checkedStage, rigPath);
    RigExecRigEvaluator baked(bakedStage, rigPath);
    reference.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    checked.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    if (!reference.Compile(&errors) || !checked.Compile(&errors) ||
        !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: does not compile\n", file);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    CHECK(reference.GetSkippedOperations().empty());
    for (RigExecRigEvaluator *program : {&checked, &baked}) {
        std::vector<std::string> reasons;
        if (!program->IsBakeable(&reasons)) {
            ++failures;
            std::printf("FAIL %s: does not bake\n", file);
            for (const std::string &reason : reasons) {
                std::printf("    %s\n", reason.c_str());
            }
            return;
        }
    }

    std::vector<double> sweep = frames;
    sweep.insert(sweep.end(), frames.rbegin(), frames.rend());
    sweep.insert(sweep.end(), frames.begin(), frames.end());
    size_t generations = 0;
    const auto compareSweep = [&](const char *phase) {
        for (const double frame : sweep) {
            const std::string where = std::string(file) + " " + phase +
                                      " t=" + TfStringify(frame);
            const RigExecRigPose a = reference.Evaluate(UsdTimeCode(frame));
            const RigExecRigPose b = checked.Evaluate(UsdTimeCode(frame));
            const RigExecRigPose c = baked.Evaluate(UsdTimeCode(frame));
            ++generations;
            CHECK(a.valid && b.valid && c.valid);
            if (b.bakedParityMismatches) {
                ++failures;
                std::printf("FAIL %s: %zu baked parity mismatch(es)\n",
                            where.c_str(), b.bakedParityMismatches);
                for (const std::string &line : b.diagnostics) {
                    std::printf("    %s\n", line.c_str());
                }
            }
            rigExecTest::ComparePose(&failures, where + " (checked)", a, b);
            rigExecTest::ComparePose(&failures, where + " (baked)", a, c);
        }
    };
    const auto setDrag = [&](const std::vector<RigExecValueOverride> &o) {
        for (RigExecRigEvaluator *e : {&reference, &checked, &baked}) {
            if (o.empty()) {
                e->ClearInteractiveOverrides();
            } else {
                e->SetInteractiveOverrides(o);
            }
        }
    };
    compareSweep("authored");
    if (!drag.empty()) {
        setDrag(drag);
        compareSweep("dragged");
        setDrag({});
        compareSweep("released");
    }
    for (const RigExecRigEvaluator *program : {&checked, &baked}) {
        if (program->GetBakedGenerationCount() != generations) {
            ++failures;
            std::printf("FAIL %s: %zu of %zu generation(s) came from the "
                        "program\n", file, program->GetBakedGenerationCount(),
                        generations);
        }
    }
    std::printf("  %s: parity over %zu generation(s)\n", file, generations);
}

std::vector<RigExecValueOverride>
ClampMaxDrag(float value)
{
    return {RigExecValueOverride{
        kCrossRig.AppendPath(SdfPath("PropertyMovers/ClampBlendWeight")),
        TfToken(), TfToken("inputs:max"), VtValue(value)}};
}

// --- (a) cross-domain trace --------------------------------------------------

/// The fixture's chain, as program steps.
struct CrossDomainChain {
    size_t blendSolve = SIZE_MAX, blendCommit = SIZE_MAX;
    size_t constraint = SIZE_MAX;
    size_t fingerSolve = SIZE_MAX, fingerCommit = SIZE_MAX;
    size_t skinFuse = SIZE_MAX;
    size_t volumePlacements = SIZE_MAX, volumePacket = SIZE_MAX;
    size_t smoothStatic = SIZE_MAX, smoothFuse = SIZE_MAX;
    size_t armFkSolve = SIZE_MAX, armIkSolve = SIZE_MAX;
};

CrossDomainChain
FindChain(const std::vector<RigExecOpGraphNode> &graph)
{
    CrossDomainChain c;
    c.blendSolve = OneStep(graph, "Solve", "/IKFKBlend");
    c.constraint = OneStep(graph, "Constraint", "/FingerToWrist");
    c.fingerSolve = OneStep(graph, "Solve", "/FingerFK");
    c.armFkSolve = OneStep(graph, "Solve", "/ArmFK");
    c.armIkSolve = OneStep(graph, "Solve", "/ArmIK");
    c.skinFuse = OneStep(graph, "RevisionFuse", "/ArmSmooth/ArmSkin");
    c.volumePlacements = OneStep(graph, "VolumePlacements", "/FingerVolume");
    c.volumePacket = OneStep(graph, "WeightPacket", "/FingerVolume");
    // The smooth's own steps: the label is a prefix of the skin's, so take
    // the one that is not the skin's.
    for (const size_t step :
         rigExecTest::FindOpGraphSteps(graph, "RevisionStatic", "/ArmSmooth")) {
        if (graph[step].label.find("/ArmSkin") == std::string::npos) {
            c.smoothStatic = step;
        }
    }
    for (const size_t step :
         rigExecTest::FindOpGraphSteps(graph, "RevisionFuse", "/ArmSmooth")) {
        if (graph[step].label.find("/ArmSkin") == std::string::npos) {
            c.smoothFuse = step;
        }
    }
    CHECK(c.smoothStatic != SIZE_MAX && c.smoothFuse != SIZE_MAX);
    // The commit that lands each solver's frames: the first SolverCommit
    // successor of the solve.
    const auto commitOf = [&graph](size_t solve) {
        if (solve == SIZE_MAX) return SIZE_MAX;
        for (const size_t succ : graph[solve].succs) {
            if (graph[succ].kind == "SolverCommit") return succ;
        }
        return SIZE_MAX;
    };
    c.blendCommit = commitOf(c.blendSolve);
    c.fingerCommit = commitOf(c.fingerSolve);
    CHECK(c.blendCommit != SIZE_MAX && c.fingerCommit != SIZE_MAX);
    return c;
}

bool
ChainFound(const CrossDomainChain &c)
{
    for (const size_t step :
         {c.blendSolve, c.blendCommit, c.constraint, c.fingerSolve,
          c.fingerCommit, c.skinFuse, c.volumePlacements, c.volumePacket,
          c.smoothStatic, c.smoothFuse, c.armFkSolve, c.armIkSolve}) {
        if (step == SIZE_MAX) return false;
    }
    return true;
}

void
TestCrossDomainTrace(const std::string &fixturesDir)
{
    LiveRig rig = OpenRig(fixturesDir + "/" + kCrossDomain, kCrossRig,
                          RigExecEvaluationMode::Baked);
    CHECK(rig.evaluator);
    if (!rig.evaluator) return;
    RigExecRigEvaluator &E = *rig.evaluator;
    const RigExecRigPose pose = E.Evaluate(UsdTimeCode(12.0));
    CHECK(pose.valid);
    CHECK(E.GetBakedGenerationCount() == 1);
    const std::vector<RigExecOpGraphNode> graph = E.GetOpGraph();
    const std::vector<RigExecOpTraceEntry> trace = E.GetLastOpTrace();
    Report("(a) op graph", rigExecTest::CheckOpGraphIsAcyclic(graph));
    Report("(a) trace", rigExecTest::CheckTraceRespectsEdges(trace, graph));
    PrintTrace("(a) cross-domain first run", trace);
    const CrossDomainChain c = FindChain(graph);
    if (!ChainFound(c)) return;

    // One path along the chain: every hop is a graph edge path, so its
    // completion order holds under either executor.
    const std::vector<size_t> path = {c.blendSolve,  c.blendCommit,
                                      c.constraint,  c.fingerSolve,
                                      c.fingerCommit, c.skinFuse,
                                      c.smoothStatic, c.smoothFuse};
    for (size_t i = 0; i < path.size(); ++i) {
        CHECK(rigExecTest::TraceSeqOf(trace, path[i]) != 0);
        if (i > 0) {
            CHECK(rigExecTest::OpGraphForwardCone(graph, {path[i - 1]})
                      [path[i]]);
            CHECK(rigExecTest::Precedes(trace, path[i - 1], path[i]));
        }
    }
    // The whole chain: every step on some path from the blend to the
    // smooth's fuse, so a step later inserted anywhere between them is in
    // it. Its domains, collapsed in completion order.
    const std::vector<char> afterBlend =
        rigExecTest::OpGraphForwardCone(graph, {c.blendSolve});
    const std::vector<char> beforeSmooth =
        rigExecTest::OpGraphBackwardCone(graph, {c.smoothFuse});
    std::vector<char> onChain(graph.size(), 0);
    size_t chainSize = 0;
    for (size_t k = 0; k < graph.size(); ++k) {
        onChain[k] = afterBlend[k] && beforeSmooth[k];
        if (onChain[k]) {
            ++chainSize;
            CHECK(rigExecTest::TraceSeqOf(trace, k) != 0);
        }
    }
    for (const size_t step : path) CHECK(onChain[step]);
    CHECK(onChain[c.volumePlacements] && onChain[c.volumePacket]);
    std::vector<std::string> domains;
    std::set<std::string> domainSet;
    for (const RigExecOpTraceEntry &entry : trace) {
        if (entry.step >= onChain.size() || !onChain[entry.step]) continue;
        domainSet.insert(entry.domain);
        if (domains.empty() || domains.back() != entry.domain) {
            domains.push_back(entry.domain);
        }
    }
    // Today the chain reads pose -> weight -> geometry: the property
    // revision runs in the prologue outside the graph (until S3); the weight
    // steps are the volume's placements and packet, which read pose only;
    // and the preceding field is measured inside the smooth's
    // RevisionStatic, a geometry step (until S4 makes it a WeightField op
    // between the two revisions). The target adds the property domain
    // ahead and a weight step between the skin's and the smooth's geometry:
    // property, pose, ..., geometry, weight, geometry. Completion order
    // within the domain set is fixed only by program order, so serial only;
    // the parallel executor is held to the set and the edge orders above.
    CHECK((domainSet ==
           std::set<std::string>{"pose", "weight", "geometry"}));
    if (SerialExecutor()) {
        CHECK((domains ==
               std::vector<std::string>{"pose", "weight", "geometry"}));
    }
    for (const RigExecOpGraphNode &node : graph) {
        CHECK(node.domain == "pose" || node.domain == "weight" ||
              node.domain == "geometry");
    }
    CHECK(rigExecTest::CountTraceDomain(trace, "property") == 0);

    // The fixture's weight-domain steps read pose only: neither waits on the
    // skin revision whose points the preceding field measures (S4). Their
    // order after the finger commit is an edge; their order before the skin
    // fuse is program order, so serial only.
    CHECK(rigExecTest::Precedes(trace, c.fingerCommit, c.volumePacket));
    CHECK(!rigExecTest::OpGraphForwardCone(graph, {c.skinFuse})
               [c.volumePacket]);
    CHECK(!rigExecTest::OpGraphForwardCone(graph, {c.skinFuse})
               [c.volumePlacements]);
    if (SerialExecutor()) {
        // Serial program order: the whole pose half, then the weights, then
        // the geometry half (barriers B9/B12, retired in S8).
        CHECK(rigExecTest::Precedes(trace, c.volumePacket, c.skinFuse));
        size_t lastPose = 0, firstGeometryAfterPose = SIZE_MAX;
        for (const RigExecOpTraceEntry &entry : trace) {
            if (entry.domain == "pose") lastPose = entry.seq;
        }
        for (const RigExecOpTraceEntry &entry : trace) {
            const RigExecBakedStep &step =
                E.GetBakedProgram()->GetStepGraph().steps[entry.step];
            // A source step runs in the source pass ahead of everything.
            if (entry.domain == "geometry" && !step.isSource) {
                firstGeometryAfterPose =
                    std::min<size_t>(firstGeometryAfterPose, entry.seq);
            }
        }
        CHECK(firstGeometryAfterPose > lastPose);
    }
    std::string sequence;
    for (const std::string &domain : domains) {
        sequence += (sequence.empty() ? "" : " -> ") + domain;
    }
    std::printf("  (a) chain of %zu step(s), domains today: %s\n",
                chainSize, sequence.c_str());
}

// --- (b) geometry starts when its own pose is done ---------------------------

void
TestTwoLimbs(const std::string &fixturesDir)
{
    LiveRig rig = OpenRig(fixturesDir + "/" + kTwoLimbs, kLimbsRig,
                          RigExecEvaluationMode::Baked);
    CHECK(rig.evaluator);
    if (!rig.evaluator) return;
    RigExecRigEvaluator &E = *rig.evaluator;
    CHECK(E.Evaluate(UsdTimeCode(6.0)).valid);
    CHECK(E.GetBakedGenerationCount() == 1);
    const std::vector<RigExecOpGraphNode> graph = E.GetOpGraph();
    const std::vector<RigExecOpTraceEntry> trace = E.GetLastOpTrace();
    Report("(b) op graph", rigExecTest::CheckOpGraphIsAcyclic(graph));
    Report("(b) trace", rigExecTest::CheckTraceRespectsEdges(trace, graph));
    PrintTrace("(b) two limbs first run", trace);

    // Each limb's pose seeds: its solver and the compose steps of its own
    // controls and joints.
    const auto poseSeeds = [&graph](const std::vector<std::string> &labels) {
        std::vector<size_t> seeds;
        for (const RigExecOpGraphNode &node : graph) {
            if (node.domain != "pose") continue;
            for (const std::string &label : labels) {
                if (node.label.find(label) != std::string::npos) {
                    seeds.push_back(node.step);
                    break;
                }
            }
        }
        return seeds;
    };
    const std::vector<size_t> limbAPose =
        poseSeeds({"/LimbAFK", "/Controls/A0", "/Joints/LimbA0"});
    const std::vector<size_t> limbBPose = poseSeeds(
        {"/LimbBIK", "/Controls/BRoot", "/Controls/BEffector",
         "/Controls/BPole", "/Joints/LimbB0"});
    const std::vector<size_t> limbAGeometry = [&graph] {
        std::vector<size_t> out;
        for (const RigExecOpGraphNode &node : graph) {
            if (node.domain == "geometry" &&
                node.label.find("MeshA") != std::string::npos) {
                out.push_back(node.step);
            }
        }
        return out;
    }();
    const std::vector<size_t> limbBGeometry = [&graph] {
        std::vector<size_t> out;
        for (const RigExecOpGraphNode &node : graph) {
            if (node.domain == "geometry" &&
                node.label.find("MeshB") != std::string::npos) {
                out.push_back(node.step);
            }
        }
        return out;
    }();
    CHECK(!limbAPose.empty() && !limbBPose.empty());
    CHECK(!limbAGeometry.empty() && !limbBGeometry.empty());

    // Structural, at step grain: holds today. No path from limb B's pose to
    // limb A's geometry, nor the other way round.
    const std::vector<char> fromB =
        rigExecTest::OpGraphForwardCone(graph, limbBPose);
    const std::vector<char> fromA =
        rigExecTest::OpGraphForwardCone(graph, limbAPose);
    for (const size_t step : limbAGeometry) CHECK(!fromB[step]);
    for (const size_t step : limbBGeometry) CHECK(!fromA[step]);
    // And each limb's geometry does wait on its own pose.
    const size_t meshAFuse = OneStep(graph, "RevisionFuse", "/MeshASkin");
    const size_t meshBFuse = OneStep(graph, "RevisionFuse", "/MeshBSkin");
    const size_t limbASolve = OneStep(graph, "Solve", "/LimbAFK");
    const size_t limbBSolve = OneStep(graph, "Solve", "/LimbBIK");
    if (meshAFuse == SIZE_MAX || meshBFuse == SIZE_MAX ||
        limbASolve == SIZE_MAX || limbBSolve == SIZE_MAX) {
        return;
    }
    CHECK(fromA[meshAFuse]);
    CHECK(fromB[meshBFuse]);

    // Structural, at the production cluster grain. The packing bins by
    // longest-path level, and every pose level of this rig costs less than
    // the 5 us grain floor, so each level is one bin holding both limbs:
    // limb A's skin fuse waits on limb B's pose at cluster grain today. S3.5
    // lowering decides clusters from the op graph, and S8's loop is held to
    // "no path" at this grain. Asserted only at the default grain. The count
    // printed is of limb A geometry steps that wait (source steps and
    // members ahead of the seeds in their own cluster do not).
    const RigExecBakedProgramImpl &B = E.GetBakedProgram()->GetStepGraph();
    std::vector<char> sources(B.steps.size(), 0);
    for (size_t k = 0; k < B.steps.size(); ++k) {
        sources[k] = B.steps[k].isSource ? 1 : 0;
    }
    const std::vector<char> fromBClusters =
        rigExecTest::OpGraphClusterForwardCone(graph, limbBPose, sources);
    size_t limbAGeometryWaitingOnB = 0;
    for (const size_t step : limbAGeometry) {
        limbAGeometryWaitingOnB += fromBClusters[step] ? 1 : 0;
    }
    std::printf("  (b) cluster grain %.2f us, %zu cluster(s): %zu of %zu "
                "limb A geometry step(s) wait on limb B's pose\n",
                B.clustering.grainUs, B.clustering.clusters.size(),
                limbAGeometryWaitingOnB, limbAGeometry.size());
    // A source step never waits, whatever cluster holds it.
    for (const size_t step : limbAGeometry) {
        if (sources[step]) CHECK(!fromBClusters[step]);
    }
    if (TfGetenv("RIGEXEC_BAKED_GRAIN_US").empty()) {
        CHECK(B.clustering.grainUs > 0);
        CHECK(fromBClusters[meshAFuse]);
    }

    // Behavioural: under the serial executor limb A's skin finishes after
    // limb B's solver, because program order runs the whole pose half first
    // (B9/B12). S8 replaces this with a gated limb-B leaf and asserts limb
    // A's skin still completes.
    std::printf("  (b) limb A skin fuse seq %u, limb B solve seq %u\n",
                rigExecTest::TraceSeqOf(trace, meshAFuse),
                rigExecTest::TraceSeqOf(trace, limbBSolve));
    if (SerialExecutor()) {
        CHECK(rigExecTest::Precedes(trace, limbBSolve, meshAFuse));
        CHECK(rigExecTest::Precedes(trace, limbASolve, meshBFuse));
    }
}

// --- (c) preceding weights as ordinary dependencies --------------------------

void
TestPrecedingWeight(const std::string &fixturesDir)
{
    LiveRig rig = OpenRig(fixturesDir + "/" + kCrossDomain, kCrossRig,
                          RigExecEvaluationMode::Baked);
    CHECK(rig.evaluator);
    if (!rig.evaluator) return;
    RigExecRigEvaluator &E = *rig.evaluator;
    CHECK(E.Evaluate(UsdTimeCode(6.0)).valid);
    const std::vector<RigExecOpGraphNode> graph = E.GetOpGraph();
    const std::vector<RigExecOpTraceEntry> trace = E.GetLastOpTrace();
    const CrossDomainChain c = FindChain(graph);
    if (!ChainFound(c)) return;
    const RigExecBakedProgramImpl &B = E.GetBakedProgram()->GetStepGraph();
    const SdfPath volume =
        kCrossRig.AppendPath(SdfPath("Joints/Finger/FingerVolume"));

    // The program knows the volume reads `preceding`.
    CHECK(B.currentPhaseWeights.count(volume) == 1);

    // Today the field is measured inside the consuming revision's
    // RevisionStatic (a geometry step) through the evaluator's weight
    // oracle, against the points the skin left. Its predecessors are the
    // skin's fuse, the volume placements and the volume's packet -- the
    // last two pose-only. S4 splits it into a WeightField op that reads
    // RevisionFuse(k-1) and feeds RevisionStatic(k).
    const RigExecOpGraphNode &smooth = graph[c.smoothStatic];
    CHECK(smooth.domain == "geometry");
    const std::set<size_t> preds(smooth.preds.begin(), smooth.preds.end());
    CHECK(preds.count(c.skinFuse) == 1);
    CHECK(preds.count(c.volumePlacements) == 1);
    CHECK(preds.count(c.volumePacket) == 1);
    // It reads the one placement its field's volume has: the slot the
    // FingerVolume placement step writes, and no other.
    size_t weightFramesReads = 0;
    bool readsPacket = false, readsOwnPlacement = false;
    for (const RigExecOpSlotRange &range : smooth.reads) {
        if (range.domain == "WeightFrames") {
            ++weightFramesReads;
            for (const RigExecOpSlotRange &write :
                     graph[c.volumePlacements].writes) {
                readsOwnPlacement |= write.domain == "WeightFrames" &&
                                     write.first == range.first &&
                                     write.last == range.last;
            }
        }
        readsPacket |= range.domain == "WeightPacket";
    }
    CHECK(weightFramesReads == 1 && readsOwnPlacement && readsPacket);
    // No weight-domain step depends on the skin revision: nothing in the
    // weight domain is downstream of geometry today (S4).
    const std::vector<char> afterSkin =
        rigExecTest::OpGraphForwardCone(graph, {c.skinFuse});
    for (const RigExecOpGraphNode &node : graph) {
        if (node.domain == "weight") CHECK(!afterSkin[node.step]);
    }
    // The volume's packet and the placements read pose only, so the
    // cone machinery marks the packet as reading outside the program and
    // runs it every generation (until S4 makes placements per-volume ops).
    CHECK(B.steps[c.volumePacket].externalReads);
    // Each revision runs once per evaluation.
    CHECK(rigExecTest::FindTraceEntries(trace, "RevisionFuse", "",
                                        "/ArmSmooth")
              .size() == 2);
    std::printf("  (c) preceding field computed by step %zu (%s %s), preds "
                "skin fuse %zu, placements %zu, packet %zu\n",
                c.smoothStatic, smooth.kind.c_str(), smooth.label.c_str(),
                c.skinFuse, c.volumePlacements, c.volumePacket);

    // The field follows the skin revision it measures: points 8 and 18
    // carry part Wrist inside the sphere riding Finger, so FingerCtl over
    // time and the clamp drag both move them relative to the sphere. A
    // field left at an earlier generation's or the rest points fails here.
    const auto fieldAt = [&E, &volume](double frame) {
        const RigExecRigPose pose = E.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid);
        const auto it = pose.weightFields.find(volume);
        CHECK(it != pose.weightFields.end());
        return it != pose.weightFields.end() ? it->second.weights
                                             : std::vector<float>{};
    };
    const std::vector<float> atStart = fieldAt(1.0);
    const std::vector<float> settled = fieldAt(12.0);
    E.SetInteractiveOverrides(ClampMaxDrag(0.6f));
    const std::vector<float> dragged = fieldAt(12.0);
    E.ClearInteractiveOverrides();
    CHECK(!atStart.empty() && atStart.size() == settled.size() &&
          settled.size() == dragged.size());
    // Not bitwise: points rigid to Finger already move the field by
    // rounding as the joints turn. The skinned points move it by tenths.
    const auto largestChange = [](const std::vector<float> &a,
                                  const std::vector<float> &b) {
        float largest = 0.0f;
        for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
            largest = std::max(largest, std::abs(a[i] - b[i]));
        }
        return largest;
    };
    const float overTime = largestChange(atStart, settled);
    const float underDrag = largestChange(settled, dragged);
    CHECK(overTime > 0.1f);
    CHECK(underDrag > 0.1f);
    std::printf("  (c) field moves by %.4f from t=1 to t=12 and by %.4f "
                "under the drag\n", overTime, underDrag);
}

// --- (d) invalidation across domains -----------------------------------------

void
TestPropertyDragCone(const std::string &fixturesDir)
{
    LiveRig rig = OpenRig(fixturesDir + "/" + kCrossDomain, kCrossRig,
                          RigExecEvaluationMode::Baked);
    CHECK(rig.evaluator);
    if (!rig.evaluator) return;
    RigExecRigEvaluator &E = *rig.evaluator;
    // At t = 12 the authored blend weight is 1.3 and the clamp holds it at
    // 1; dragging the clamp's max to 0.6 moves the blend.
    const UsdTimeCode time(12.0);
    const SdfPath weight =
        kCrossRig.AppendPath(SdfPath("ArmSolvers/IKFKBlend"))
            .AppendProperty(TfToken("inputs:weight"));
    const RigExecRigPose settled = E.Evaluate(time);
    CHECK(settled.valid);
    const RigExecRigPose repeat = E.Evaluate(time);
    CHECK(repeat.valid);
    const std::vector<RigExecOpTraceEntry> repeatTrace = E.GetLastOpTrace();
    E.SetInteractiveOverrides(ClampMaxDrag(0.6f));
    const RigExecRigPose dragged = E.Evaluate(time);
    CHECK(dragged.valid);
    CHECK(E.GetBakedGenerationCount() == 3);
    const std::vector<RigExecOpTraceEntry> trace = E.GetLastOpTrace();
    const std::vector<RigExecOpGraphNode> graph = E.GetOpGraph();
    Report("(d) trace", rigExecTest::CheckTraceRespectsEdges(trace, graph));
    PrintTrace("(d) clamp max dragged to 0.6", trace);
    E.ClearInteractiveOverrides();

    // The drag reached the property chain and moved the published pose.
    const auto weightOf = [&weight](const RigExecRigPose &pose) {
        const auto it = pose.movedProperties.find(weight);
        return it != pose.movedProperties.end() && it->second.IsHolding<float>()
                   ? it->second.UncheckedGet<float>()
                   : -1.0f;
    };
    CHECK(weightOf(settled) == 1.0f);
    CHECK(weightOf(dragged) == 0.6f);
    const SdfPath finger = kCrossRig.AppendPath(SdfPath("Joints/Finger"));
    CHECK(settled.jointMatricesFinal.count(finger) &&
          dragged.jointMatricesFinal.count(finger) &&
          settled.jointMatricesFinal.at(finger) !=
              dragged.jointMatricesFinal.at(finger));

    const CrossDomainChain c = FindChain(graph);
    if (!ChainFound(c)) return;
    const RigExecBakedProgramImpl &B = E.GetBakedProgram()->GetStepGraph();

    // The steps that may read the clamp's result. No slot names it: the
    // property chain runs in the prologue and its readers carry a per-step
    // "reads resolved inputs" flag instead -- the blend's solve for its
    // weight, and the volume's packet for the mesh points it measures (until
    // S3 adds PropertyRevision ops and PropertyResult reads).
    std::vector<size_t> readers;
    for (size_t k = 0; k < B.steps.size(); ++k) {
        if (B.steps[k].resolvedInputReads) readers.push_back(k);
    }
    CHECK((readers == std::vector<size_t>{c.blendSolve, c.volumePacket}));
    // The packet is downstream of the blend anyway, so the cone below is
    // the blend's.
    CHECK(rigExecTest::OpGraphForwardCone(graph, {c.blendSolve})
              [c.volumePacket]);
    for (const RigExecOpSlotRange &range : graph[c.blendSolve].reads) {
        CHECK(range.domain != "PropertyResult");
    }

    // Executed set within the forward cone of the readers, except for the
    // steps that run every generation: the source pass (isSource) and the
    // externally-read steps with their cones -- the same set a repeated time
    // with nothing changed executes. S3/S8 fold sources into the loop and
    // leave nothing outside the cone.
    const std::vector<char> cone =
        rigExecTest::OpGraphForwardCone(graph, readers);
    std::set<size_t> alwaysRun;
    for (const RigExecOpTraceEntry &entry : repeatTrace) {
        alwaysRun.insert(entry.step);
    }
    std::set<size_t> executed;
    std::set<std::string> domainsRun;
    for (const RigExecOpTraceEntry &entry : trace) {
        executed.insert(entry.step);
        domainsRun.insert(entry.domain);
        if (!cone[entry.step]) {
            CHECK(alwaysRun.count(entry.step) == 1);
            CHECK(B.steps[entry.step].isSource);
        }
    }
    // Every cone step ran: there is no unchanged-output cutoff yet (S8's
    // loop stops a wave at an unchanged output).
    size_t coneSize = 0;
    for (size_t k = 0; k < cone.size(); ++k) {
        if (!cone[k]) continue;
        ++coneSize;
        if (!executed.count(k)) {
            ++failures;
            std::printf("FAIL (d): cone step %zu (%s) did not run\n", k,
                        graph[k].label.c_str());
        }
    }
    // The re-run crossed pose, weight and geometry; the two arm solvers
    // feeding the blend are outside the cone and stayed clean.
    CHECK((domainsRun ==
           std::set<std::string>{"pose", "weight", "geometry"}));
    CHECK(!executed.count(c.armFkSolve));
    CHECK(!executed.count(c.armIkSolve));
    for (const size_t step :
         {c.blendSolve, c.constraint, c.fingerSolve, c.skinFuse,
          c.volumePacket, c.smoothStatic, c.smoothFuse}) {
        CHECK(executed.count(step) == 1);
    }
    // The repeated time ran only the always-run set: the source pass and
    // the externally-read volume packet with the revision that reads it.
    for (const size_t step : alwaysRun) {
        CHECK(B.steps[step].isSource ||
              rigExecTest::OpGraphForwardCone(graph, {c.volumePacket})[step]);
    }
    std::printf("  (d) drag ran %zu step(s): cone %zu, outside it %zu "
                "(sources); a repeated time runs %zu\n",
                trace.size(), coneSize, executed.size() - coneSize,
                repeatTrace.size());
}

// --- (e) cycles rejected -----------------------------------------------------

void
TestCycle(const std::string &fixturesDir)
{
    LiveRig rig = OpenRig(fixturesDir + "/" + kCycle, kCycleRig,
                          RigExecEvaluationMode::Baked);
    // Today's policy (D3, kept): the compile succeeds, the loop's members
    // are set aside with a diagnostic, and the rest of the rig evaluates.
    CHECK(rig.evaluator);
    if (!rig.evaluator) {
        for (const std::string &error : rig.errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    RigExecRigEvaluator &E = *rig.evaluator;
    const SdfPath constraint =
        kCycleRig.AppendPath(SdfPath("Constraints/EffectorFollowsVolume"));
    const SdfPath solver = kCycleRig.AppendPath(SdfPath("Solvers/LimbIK"));
    const std::string loop = "pose dependency cycle among: " +
                             constraint.GetString() + " -> " +
                             solver.GetString() + " -> " +
                             constraint.GetString();
    const std::map<SdfPath, std::string> &skipped = E.GetSkippedOperations();
    CHECK(skipped.size() == 2);
    for (const SdfPath &member : {constraint, solver}) {
        const auto it = skipped.find(member);
        CHECK(it != skipped.end() &&
              TfStringStartsWith(it->second, loop));
    }
    bool reported = false;
    for (const std::string &error : rig.errors) {
        reported |= error.find(loop) != std::string::npos;
        std::printf("  (e) compile reported: %s\n", error.c_str());
    }
    CHECK(reported);
    // The loop is reported by operator only: the volume weight it runs
    // through and the joint and control it crosses are not named (S6's one
    // reporter names the loop by op and prim, VolumePlacement included).
    for (const auto &[member, message] : skipped) {
        CHECK(message.find("EndVolume") == std::string::npos);
    }

    // What remains bakes and evaluates, without the loop's steps.
    CHECK(E.IsBakeable());
    const RigExecRigPose pose = E.Evaluate(UsdTimeCode(2.0));
    CHECK(pose.valid);
    CHECK(E.GetBakedGenerationCount() == 1);
    const std::vector<RigExecOpGraphNode> graph = E.GetOpGraph();
    Report("(e) op graph", rigExecTest::CheckOpGraphIsAcyclic(graph));
    CHECK(rigExecTest::FindOpGraphSteps(graph, "Solve").empty());
    CHECK(rigExecTest::FindOpGraphSteps(graph, "Constraint").empty());
    // One placement step per volume, and the rig has one: EndVolume.
    CHECK(rigExecTest::FindOpGraphSteps(graph, "VolumePlacements").size() ==
          1);
    CHECK(rigExecTest::FindOpGraphSteps(graph, "VolumePlacements",
                                        "/EndVolume")
              .size() == 1);
    CHECK(rigExecTest::FindOpGraphSteps(graph, "RevisionFuse", "/EndCarry")
              .size() == 1);

    // The cross-domain loop that can be authored today: fixture A's volume
    // weight reading its target at `final`, after the smooth that consumes
    // the field. Compile refuses the phase itself, in either mode, before
    // any ordering is attempted (S4/S6 turn this into a reported cycle).
    const SdfPath volume =
        kCrossRig.AppendPath(SdfPath("Joints/Finger/FingerVolume"));
    const std::string refusal =
        volume.GetString() +
        ": rigExecReadPhase 'final' on rigExec:weightTarget is not "
        "supported; a volume weight measures its source at base or the "
        "points in flight at preceding";
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        const UsdStageRefPtr stage =
            UsdStage::Open(fixturesDir + "/" + kCrossDomain);
        CHECK(stage);
        if (!stage) continue;
        stage->SetEditTarget(stage->GetSessionLayer());
        const UsdPrim prim = stage->GetPrimAtPath(volume);
        const UsdRelationship target =
            prim ? prim.GetRelationship(TfToken("rigExec:weightTarget"))
                 : UsdRelationship();
        CHECK(target);
        if (!target) continue;
        CHECK(target.SetMetadata(TfToken("rigExecReadPhase"),
                                 std::string("final")));
        RigExecRigEvaluator evaluator(stage, kCrossRig);
        evaluator.SetEvaluationMode(mode);
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        bool refused = false;
        for (const std::string &error : errors) {
            refused |= error.find(refusal) != std::string::npos;
        }
        CHECK(refused);
        if (!refused) {
            for (const std::string &error : errors) {
                std::printf("    %s\n", error.c_str());
            }
        }
    }
}

std::string
SchemaResourceDir(const std::string &fixturesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)fixturesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(fixturesDir + "/../../plugin/rigExecSchema/resources");
#endif
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecOneLoopAcceptance <fixturesDir>\n");
        return 2;
    }
    const std::string fixturesDir = argv[1];
    const std::string resources = SchemaResourceDir(fixturesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    std::printf("testRigExecOneLoopAcceptance: %s executor\n",
                SerialExecutor() ? "serial" : "parallel");
    CheckParity(fixturesDir, kCrossDomain, kCrossRig,
                {1, 3, 4.5, 6, 8, 10, 12}, ClampMaxDrag(0.6f));
    CheckParity(fixturesDir, kTwoLimbs, kLimbsRig, {1, 3, 4.5, 6, 8, 10, 12});
    TestCrossDomainTrace(fixturesDir);
    TestTwoLimbs(fixturesDir);
    TestPrecedingWeight(fixturesDir);
    TestPropertyDragCone(fixturesDir);
    TestCycle(fixturesDir);
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecOneLoopAcceptance: all tests passed\n");
    return 0;
}
