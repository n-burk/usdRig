// Canonical one-loop acceptance over actual native operations, production
// lowering and backend runners. Fixtures are the supported oneloop_* scenes.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/frozenContext.h"
#include "rigExecBake/bake.h"
#include "rigExecBinary/format.h"
#include "rigExecRuntime/runtime.h"
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
#include "pxr/usd/sdf/types.h"

#include "rigExecOpTrace.h"
#include "rigExecPoseCompare.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <atomic>
#include <thread>
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

#include "rigExecFileEdit.h"
#include "rigExecRuntimeDrive.h"

namespace {

const char *const kCrossDomain = "oneloop_cross_domain.usda";
const char *const kTwoLimbs = "oneloop_two_limbs.usda";

const SdfPath kCrossRig("/CrossAsset/Rig");
const SdfPath kLimbsRig("/LimbsAsset/Rig");

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
OpenRig(const std::string &stagePath, const SdfPath &rigPath)
{
    LiveRig rig;
    rig.stage = UsdStage::Open(stagePath);
    if (!rig.stage) {
        return rig;
    }
    rig.evaluator = std::make_unique<RigExecRigEvaluator>(rig.stage, rigPath);
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
    size_t property = SIZE_MAX, precedingField = SIZE_MAX;
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
    c.property = OneStep(graph,"PropertyRevision","/ClampBlendWeight");
    c.precedingField = OneStep(graph,"WeightField","/FingerVolume");
    c.blendSolve = OneStep(graph, "Solve", "/IKFKBlend");
    c.constraint = OneStep(graph, "Constraint", "/FingerToWrist");
    c.fingerSolve = OneStep(graph, "Solve", "/FingerFK");
    c.armFkSolve = OneStep(graph, "Solve", "/ArmFK");
    c.armIkSolve = OneStep(graph, "Solve", "/ArmIK");
    c.skinFuse = OneStep(graph, "RevisionFuse", "/ArmSmooth/ArmSkin");
    size_t basePlacements=0;
    for(const auto &node:graph)if(node.kind=="VolumePlacements" && node.label.find("/FingerVolume")!=std::string::npos)
        if(std::any_of(node.writes.begin(),node.writes.end(),[](const auto &range){return range.domain=="WeightFramesBase";})) {
            c.volumePlacements=node.step;++basePlacements;
        }
    CHECK(basePlacements==1);
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
         {c.property, c.precedingField, c.blendSolve, c.blendCommit, c.constraint, c.fingerSolve,
          c.fingerCommit, c.skinFuse, c.volumePlacements, c.volumePacket,
          c.smoothStatic, c.smoothFuse, c.armFkSolve, c.armIkSolve}) {
        if (step == SIZE_MAX) return false;
    }
    return true;
}

void
TestCrossDomainTrace(const std::string &fixturesDir)
{
    auto rig=OpenRig(fixturesDir+"/"+kCrossDomain,kCrossRig);
    CHECK(rig.evaluator); if(!rig.evaluator)return;
    auto &E=*rig.evaluator;
    CHECK(E.Evaluate(UsdTimeCode(12)).valid);
    const auto graph=E.GetOpGraph(); const auto trace=E.GetLastOpTrace();
    Report("(a) native graph",rigExecTest::CheckOpGraphIsAcyclic(graph));
    Report("(a) completion trace",rigExecTest::CheckTraceRespectsEdges(trace,graph));
    const auto c=FindChain(graph); if(!ChainFound(c))return;
    const std::vector<size_t> path={c.property,c.blendSolve,c.blendCommit,c.constraint,
        c.fingerSolve,c.fingerCommit,c.skinFuse,c.precedingField,c.smoothStatic,c.smoothFuse};
    std::vector<std::string> domains;
    for(size_t i=0;i<path.size();++i) {
        CHECK(rigExecTest::TraceSeqOf(trace,path[i])!=0);
        if(i) {
            CHECK(rigExecTest::OpGraphForwardCone(graph,{path[i-1]})[path[i]]);
            CHECK(rigExecTest::Precedes(trace,path[i-1],path[i]));
        }
        const auto &domain=graph[path[i]].domain;
        if(domains.empty() || domains.back()!=domain)domains.push_back(domain);
    }
    CHECK((domains==std::vector<std::string>{"property","pose","geometry","weight","geometry"}));
    const auto &B=E.GetBakedProgram()->GetStepGraph();
    CHECK(B.opGraph.ops.size()==B.steps.size());
    for(size_t i=0;i<B.steps.size();++i) {
        CHECK(B.opGraph.ops[i].originalIndex==i);
        CHECK(B.steps[i].preds.size()==B.opGraph.ops[i].predecessors.size());
    }
    PrintTrace("(a) property -> pose -> geometry -> weight -> geometry",trace);
}

// --- (b) geometry starts when its own pose is done ---------------------------

void
TestTwoLimbs(const std::string &fixturesDir)
{
    auto rig=OpenRig(fixturesDir+"/"+kTwoLimbs,kLimbsRig);
    CHECK(rig.evaluator); if(!rig.evaluator)return;
    auto &E=*rig.evaluator;
    CHECK(E.Evaluate(UsdTimeCode(6)).valid);
    const auto graph=E.GetOpGraph();
    const size_t aSolve=OneStep(graph,"Solve","/LimbAFK");
    const size_t bSolve=OneStep(graph,"Solve","/LimbBIK");
    const size_t aSkin=OneStep(graph,"RevisionFuse","/MeshASkin");
    const size_t bSkin=OneStep(graph,"RevisionFuse","/MeshBSkin");
    if(aSolve==SIZE_MAX || bSolve==SIZE_MAX || aSkin==SIZE_MAX || bSkin==SIZE_MAX)return;
    CHECK(rigExecTest::OpGraphForwardCone(graph,{aSolve})[aSkin]);
    CHECK(rigExecTest::OpGraphForwardCone(graph,{bSolve})[bSkin]);
    CHECK(!rigExecTest::OpGraphForwardCone(graph,{bSolve})[aSkin]);
    CHECK(!rigExecTest::OpGraphForwardCone(graph,{aSolve})[bSkin]);
    // This is the compiled production artifact, including real kernel chunks.
    const auto &B=E.GetBakedProgram()->GetStepGraph();
    CHECK(B.opGraph.ops.size()==B.steps.size());
    std::string clusterError;
    CHECK(RigExecValidateOpClusters(B.opGraph,&clusterError));
    CHECK(!B.opGraph.clusters.empty());
    const auto clusterCone=[&](size_t source) {
        std::vector<char> reached(B.opGraph.clusters.size(),0);
        std::vector<uint32_t> todo={B.opGraph.opClusters[source]};
        while(!todo.empty()) {
            const auto cluster=todo.back(); todo.pop_back();
            if(reached[cluster])continue;
            reached[cluster]=1;
            for(auto successor:B.opGraph.clusters[cluster].successors)todo.push_back(successor);
        }
        return reached;
    };
    CHECK(clusterCone(aSolve)[B.opGraph.opClusters[aSkin]]);
    CHECK(clusterCone(bSolve)[B.opGraph.opClusters[bSkin]]);
    CHECK(!clusterCone(bSolve)[B.opGraph.opClusters[aSkin]]);
    CHECK(!clusterCone(aSolve)[B.opGraph.opClusters[bSkin]]);
    if(SerialExecutor())return;
    struct Gate {
        std::atomic<bool> blocked{false},release{false},aFinished{false},bFinished{false};
    } gate;
    RigExecBakedProgramTesting::SetOpObservers(*E.GetBakedProgram(),
        [&](uint32_t step) {
            if(step!=bSolve)return;
            gate.blocked.store(true,std::memory_order_release);
            while(!gate.release.load(std::memory_order_acquire))std::this_thread::yield();
        },[&](uint32_t step) {
            if(step==aSkin)gate.aFinished.store(true,std::memory_order_release);
            if(step==bSolve)gate.bFinished.store(true,std::memory_order_release);
        });
    bool early=false;
    bool bWasStillBlocked=false;
    std::thread monitor([&] {
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
        while(std::chrono::steady_clock::now()<deadline) {
            if(gate.blocked.load(std::memory_order_acquire) && gate.aFinished.load(std::memory_order_acquire)) {
                early=true;break;
            }
            std::this_thread::yield();
        }
        bWasStillBlocked=!gate.bFinished.load(std::memory_order_acquire);
        gate.release.store(true,std::memory_order_release);
    });
    const auto pose=E.Evaluate(UsdTimeCode(12));
    monitor.join(); CHECK(pose.valid); CHECK(early && bWasStillBlocked);
    RigExecBakedProgramTesting::SetOpObservers(*E.GetBakedProgram(),{},{});
    CHECK(gate.bFinished.load(std::memory_order_acquire));
    std::printf("  (b) independent mesh finished while unrelated solver was gated: %s\n",early?"yes":"no");
}

// --- (c) preceding weights as ordinary dependencies --------------------------

void
TestPrecedingWeight(const std::string &fixturesDir)
{
    LiveRig rig = OpenRig(fixturesDir + "/" + kCrossDomain, kCrossRig);
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

    const auto &field=graph[c.precedingField];
    CHECK(field.domain=="weight");
    CHECK(std::find(field.preds.begin(),field.preds.end(),c.skinFuse)!=field.preds.end());
    CHECK(std::find(graph[c.smoothStatic].preds.begin(),graph[c.smoothStatic].preds.end(),c.precedingField)!=graph[c.smoothStatic].preds.end());
    CHECK(rigExecTest::Precedes(trace,c.skinFuse,c.precedingField));
    CHECK(rigExecTest::Precedes(trace,c.precedingField,c.smoothStatic));
    const auto &bound=B.weightFields[size_t(B.steps[c.precedingField].object)];
    CHECK(bound.form==RigExecBakedProgramImpl::WeightField::Form::Revision);
    CHECK(bound.placementPhase==RigExecBakedProgramImpl::WeightField::PlacementPhase::Base);
    const auto [chain,revision]=B.revisionIndex[size_t(bound.consumer)];
    CHECK(revision>0);
    const auto readsSlot=[&](const char *domain,int slot) {
        return std::any_of(field.reads.begin(),field.reads.end(),[&](const auto &range) {
            return range.domain==domain && slot>=0 && uint32_t(slot)>=range.first && uint32_t(slot)<=range.last;
        });
    };
    // RevisionDone publishes the completed currentSource-resolved points;
    // RevisionOut names a chunk's staging range. Every earlier completion
    // and the base are required because a revision can pass through its input.
    CHECK(readsSlot("ChainBase",chain));
    for(int earlier=0;earlier<revision;++earlier)
        CHECK(readsSlot("RevisionDone",B.chainRevisionBegin[size_t(chain)]+earlier));
    CHECK(readsSlot("ChainDirty",bound.consumer-1));
    CHECK(!bound.volumes.empty());
    for(int slot:bound.volumes)CHECK(readsSlot("WeightFramesBase",slot));
    CHECK(std::any_of(graph[c.skinFuse].writes.begin(),graph[c.skinFuse].writes.end(),[&](const auto &range) {
        return range.domain=="RevisionDone" && uint32_t(bound.consumer-1)>=range.first && uint32_t(bound.consumer-1)<=range.last;
    }));
    const auto &preceding=B.chains[size_t(chain)].revisions[size_t(revision-1)];
    CHECK(!preceding.output.empty());
    CHECK(bound.count==preceding.output.size());
    std::set<size_t> completed;
    for(const auto &entry:trace)CHECK(completed.insert(entry.step).second);
    CHECK(completed.count(c.precedingField)==1);

    // The field follows the skin revision it measures: points 8 and 18
    // carry part Wrist inside the sphere riding Finger, so FingerCtl over
    // time and the clamp drag both move them relative to the sphere. A
    // field left at an earlier generation's or the rest points fails here.
    const auto fieldAt = [&E, &volume](double frame) {
        const RigExecRigPose pose = E.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid);
        const auto it = pose.weightFields.find(volume);
        CHECK(it != pose.weightFields.end());
        return it != pose.weightFields.end()
            ? std::vector<float>(it->second.weights.cbegin(),
                                 it->second.weights.cend())
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
    auto rig=OpenRig(fixturesDir+"/"+kCrossDomain,kCrossRig);
    CHECK(rig.evaluator); if(!rig.evaluator)return;
    auto &E=*rig.evaluator; const UsdTimeCode time(12);
    CHECK(E.Evaluate(time).valid); CHECK(E.Evaluate(time).valid);
    CHECK(E.GetLastOpTrace().empty());
    const auto graph=E.GetOpGraph(); const auto c=FindChain(graph); if(!ChainFound(c))return;
    const auto cone=rigExecTest::OpGraphForwardCone(graph,{c.property});
    E.SetInteractiveOverrides(ClampMaxDrag(0.6f));
    CHECK(E.Evaluate(time).valid);
    const auto trace=E.GetLastOpTrace(); const auto &B=E.GetBakedProgram()->GetStepGraph();
    std::set<size_t> ran;
    for(const auto &entry:trace) {CHECK(cone[entry.step]);CHECK(ran.insert(entry.step).second);}
    CHECK(ran.count(c.property) && ran.count(c.blendSolve) && ran.count(c.constraint));
    CHECK(ran.count(c.fingerSolve) && ran.count(c.skinFuse) && ran.count(c.precedingField) && ran.count(c.smoothFuse));
    CHECK(!ran.count(c.armFkSolve) && !ran.count(c.armIkSolve));
    // Independently derive the stopped wave from compiled value-reader edges
    // and the exact current-generation change flags of their stored outputs.
    std::vector<char> expected(B.opGraph.ops.size(),0); expected[c.property]=1;
    for(size_t op=0;op<B.opGraph.ops.size();++op) {
        if(!expected[op])continue;
        for(const auto value:B.opGraph.ops[op].descriptor.writes) {
            if(!B.opAdapter.values[size_t(value)].changed)continue;
            const auto found=B.opGraph.readers.find(value);
            if(found!=B.opGraph.readers.end())for(const auto reader:found->second)expected[reader]=1;
        }
    }
    for(size_t op=0;op<expected.size();++op)CHECK(bool(expected[op])==bool(ran.count(op)));
    // Different input, identical clamp output: the property op executes,
    // and its unchanged version prevents every successor body from running.
    E.SetInteractiveOverrides(ClampMaxDrag(2.0f)); CHECK(E.Evaluate(time).valid);
    E.SetInteractiveOverrides(ClampMaxDrag(3.0f)); CHECK(E.Evaluate(time).valid);
    const auto cutoff=E.GetLastOpTrace();
    CHECK(cutoff.size()==1 && cutoff.front().step==c.property);
    CHECK(!B.opExecution.ran[c.blendSolve]);
    E.ClearInteractiveOverrides();
    std::printf("  (d) exact cross-domain wave and unchanged-property cutoff\n");
}

// Skip is told only the ops whose skip can change state. After a first run,
// in which every body ran, no other op holds a counter MarkSkipped zeroes.
void
TestSkipEffects(const std::string &fixturesDir)
{
    auto rig=OpenRig(fixturesDir+"/"+kCrossDomain,kCrossRig);
    CHECK(rig.evaluator); if(!rig.evaluator)return;
    auto &E=*rig.evaluator;
    CHECK(E.Evaluate(UsdTimeCode(12)).valid);
    const auto &B=E.GetBakedProgram()->GetStepGraph();
    const auto &effects=B.opAdapter.skipEffects;
    CHECK(std::is_sorted(effects.begin(),effects.end()));
    size_t listed=0;
    for(uint32_t c=0;c<B.opGraph.ops.size();++c) {
        CHECK(B.opExecution.ran[c]);
        const auto &step=B.steps[B.opGraph.ops[c].originalIndex];
        const bool effect=RigExecBakedIsGeometryStep(step.kind) ||
            step.kind==RigExecBakedStepKind::PropertyRevision;
        const bool found=std::binary_search(effects.begin(),effects.end(),c);
        CHECK(found==effect);
        listed+=found;
        if(!found) CHECK(!step.counters.revisionsExecuted && !step.counters.revisionsCreated &&
                         !step.counters.schedulesBuilt);
    }
    CHECK(listed==effects.size() && listed>0 && listed<B.opGraph.ops.size());
    std::printf("  (d) skip is told %zu of %zu operation(s)\n",listed,B.opGraph.ops.size());
}

// Every backend runs each authored geometry kernel once, through its actual
// compiled chunk operations. A fused assembly is a separate operation.
void
TestBackendKernelsAndBinaryCycle(const std::string &fixturesDir)
{
    auto rig=OpenRig(fixturesDir+"/"+kCrossDomain,kCrossRig);
    CHECK(rig.evaluator); if(!rig.evaluator)return;
    auto &E=*rig.evaluator;
    const auto live=E.Evaluate(UsdTimeCode(12)); CHECK(live.valid);
    const auto &B=E.GetBakedProgram()->GetStepGraph();
    // Partition metadata names natural typed producers. Scheduling levels
    // cannot stand in for the matrix-domain/provider identities the body reads.
    for (const auto &chain : B.chains) for (const auto &revision : chain.revisions) {
        if (revision.partitionProducerSets.empty()) continue;
        const auto domain = uint8_t(RigExecBakedOwnMatrixDomain(revision));
        std::set<std::vector<std::pair<uint8_t,int>>> distinct;
        size_t minimum = SIZE_MAX, maximum = 0;
        for (const auto &producers : revision.partitionProducerSets) {
            CHECK(std::is_sorted(producers.begin(),producers.end()));
            CHECK(std::adjacent_find(producers.begin(),producers.end()) == producers.end());
            for (const auto &producer : producers) {
                CHECK(producer.first == domain);
                CHECK(std::find(revision.influenceSlots.begin(),revision.influenceSlots.end(),producer.second) != revision.influenceSlots.end());
            }
            distinct.insert(producers);
            minimum = std::min(minimum,producers.size());
            maximum = std::max(maximum,producers.size());
        }
        CHECK(distinct.size() == revision.partitionDistinctReads);
        CHECK(minimum == size_t(revision.partitionProducerMin));
        CHECK(maximum == size_t(revision.partitionProducerMax));
    }
    const auto checkNative=[&](const std::vector<RigExecOpTraceEntry> &trace) {
        // Baking recompiles the evaluator, so each trace belongs to the
        // program currently held by the evaluator.
        const auto &B=E.GetBakedProgram()->GetStepGraph();
        std::vector<size_t> counts(B.steps.size(),0);
        for(const auto &entry:trace) {CHECK(entry.step<counts.size());if(entry.step<counts.size())++counts[entry.step];}
        size_t kernels=0;
        for(size_t i=0;i<B.steps.size();++i) {
            CHECK(counts[i]<=1);
            if(B.steps[i].kind==RigExecBakedStepKind::RevisionChunk) {
                ++kernels;CHECK(counts[i]==1);
                const auto identity = B.revisionIndex[size_t(B.steps[i].object)];
                const auto &revision = B.chains[size_t(identity.first)].revisions[size_t(identity.second)];
                if (revision.chunked && !revision.partitionProducerSets.empty()) {
                    std::vector<std::pair<uint8_t,int>> reads;
                    const auto own = RigExecBakedOwnMatrixDomain(revision);
                    for (const auto &range : B.steps[i].reads) if (range.domain == own)
                        for (int slot = range.begin; slot < range.end; ++slot) reads.emplace_back(uint8_t(own),slot);
                    std::sort(reads.begin(),reads.end());
                    reads.erase(std::unique(reads.begin(),reads.end()),reads.end());
                    CHECK(size_t(B.steps[i].part) < revision.partitionProducerSets.size());
                    if (size_t(B.steps[i].part) < revision.partitionProducerSets.size())
                        CHECK(reads == revision.partitionProducerSets[size_t(B.steps[i].part)]);
                }
            }
        }
        CHECK(kernels>=2);
    };
    checkNative(E.GetLastOpTrace());
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(E,&frozen,&error));
    RigExecFrameInputs inputs;
    CHECK(RigExecSampleFrameInputs(E,UsdTimeCode(6),{},&inputs,&error));
    if(frozen) {
        RigExecFrozenEvalContext context;
        context.epochDigest=E.GetBindingEpochDigest();
        context.slotCount=E.GetBakedProgram()->GetProviderCount();
        context.varyingInputCount=inputs.values.size();
        context.flags=(E.GetPublishWeightFields()?kRigExecFrozenPublishWeightFields:0)
            |(E.GetSolverGuidesEnabled()?kRigExecFrozenSolverGuidesEnabled:0);
        context.frozen=frozen.get();
        RigExecFrozenRunReport report;
        const auto pose=RigExecEvaluateFrozen(context,inputs,RigExecMakeProductionStepRunner(),nullptr,kCrossRig,&report);
        CHECK(pose.valid && report.ran); checkNative(report.region);
        const auto liveAt6=E.Evaluate(UsdTimeCode(6));
        rigExecTest::ComparePose(&failures,"frozen actual one-loop kernels",liveAt6,pose);
    }
    RigExecBakeOpts opts;opts.time=6;
    RigExecBakeResult baked;
    CHECK(RigExecBakeToBinary(E,opts,&baked,&error));
    if(baked.bytes.empty())return;
    checkNative(E.GetLastOpTrace());
    auto reader=RigExecRuntimeReader::Open(baked.bytes.data(),baked.bytes.size(),&error);
    CHECK(reader); if(!reader)return;
    CHECK(reader->Execute(&error));
    const auto file=RigExecTestUnpack(baked.bytes); if(!file)return;
    std::vector<size_t> runtimeCounts(file->steps.size(),0);
    for(auto index:reader->GetLastRunTraceForTesting()) {
        CHECK(index>=0 && size_t(index)<runtimeCounts.size());
        if(index>=0 && size_t(index)<runtimeCounts.size())++runtimeCounts[size_t(index)];
    }
    size_t runtimeKernels=0;
    for(size_t i=0;i<file->steps.size();++i) {
        CHECK(runtimeCounts[i]<=1);
        if(file->steps[i].kind==fb::StepKind::RevisionChunk) {++runtimeKernels;CHECK(runtimeCounts[i]==1);}
    }
    CHECK(runtimeKernels>=2);
    std::vector<std::string> differences;
    const bool runtimeMatches=RigExecCompareRuntimeRun(E.Evaluate(UsdTimeCode(6)),*reader,&differences,true);
    if(!runtimeMatches)for(const auto &difference:differences)
        std::printf("  runtime acceptance mismatch: %s\n",difference.c_str());
    CHECK(runtimeMatches);
    CHECK(!file->steps.empty());
    if(!file->steps.empty()) {
        const auto cyclic=RigExecTestEdited(baked.bytes,[](fb::RigExecWireFile *edited) {
            const auto self=uint32_t(edited->steps.size()-1);
            edited->steps.back().preds.push_back(int32_t(self));
            edited->steps.back().succs.push_back(int32_t(self));
            // Keep the body and its common descriptor in agreement: the
            // defect is a genuine self-cycle, rather than mismatched tables.
            CHECK(edited->commonGraph && edited->commonGraph->ops.size()==edited->steps.size());
            if(edited->commonGraph && edited->commonGraph->ops.size()==edited->steps.size()) {
                auto &op=edited->commonGraph->ops.back();
                CHECK(op.originalIndex==self);
                op.descriptorPredecessors.push_back(self);
                op.predecessors.push_back(self);
                op.successors.push_back(self);
            }
        });
        error.clear();
        CHECK(!RigExecRuntimeReader::Open(cyclic.data(),cyclic.size(),&error));
        CHECK(error=="invalid .rigexec: step "+std::to_string(file->steps.size()-1)+" depends on itself");
    }
    std::printf("  (c,f) live, frozen and runtime kernels once; cyclic binary refused\n");
}

// --- (e) live cycles fail closed --------------------------------------------

void
TestCycle(const std::string &fixturesDir)
{
    const SdfPath rigPath("/S9Asset/Rig");
    auto rig=OpenRig(fixturesDir+"/oneloop_cross_cycle.usda",rigPath);
    CHECK(rig.evaluator); if(!rig.evaluator)return;
    auto &E=*rig.evaluator;
    const auto pose=E.Evaluate(UsdTimeCode(2)); CHECK(pose.valid);
    const auto &B=E.GetBakedProgram()->GetStepGraph();
    // D3 preserves unrelated work while every output of the complete cycle
    // is cleared. No member may run against a prior-generation loop value.
    CHECK(!B.opGraph.cycles.empty()); CHECK(!B.excludedSteps.empty());
    bool solveNamed=false,geometryNamed=false,commitNamed=false;
    const SdfPath ribbon("/S9Asset/Rig/Solvers/Ribbon");
    const SdfPath lift("/S9Asset/Rig/Geometry/Lift");
    for(const auto &cycle:B.opGraph.cycles) {
        CHECK(cycle.size()>1 && cycle.front()==cycle.back());
        for(const auto &member:cycle) {
            const auto found=std::find_if(B.excludedSteps.begin(),B.excludedSteps.end(),[&](const auto &step) {
                return step.descriptorKey==member;
            });
            const bool removed=std::find(B.cycleExclusionProof.removedKeys.begin(),
                B.cycleExclusionProof.removedKeys.end(),member)!=B.cycleExclusionProof.removedKeys.end();
            CHECK((found!=B.excludedSteps.end())!=removed);
            if(removed) {
                CHECK(std::none_of(B.steps.begin(),B.steps.end(),[&](const auto &step){return step.descriptorKey==member;}));
                const auto commitPrefix=ribbon.GetString()+"/category:"+
                    std::to_string(int(RigExecBakedStepKind::SolverCommit))+"/stack:";
                commitNamed|=member.compare(0,commitPrefix.size(),commitPrefix)==0;
                continue;
            }
            if(found==B.excludedSteps.end())continue;
            if(found->kind==RigExecBakedStepKind::Solve)
                solveNamed|=B.solvers[size_t(found->object)].path==ribbon;
            if(found->kind==RigExecBakedStepKind::RevisionFuse) {
                const auto index=B.revisionIndex[size_t(found->object)];
                geometryNamed|=B.chains[size_t(index.first)].revisions[size_t(index.second)].moverPath==lift;
            }
            if(found->kind==RigExecBakedStepKind::SolverCommit)
                commitNamed|=found->label==ribbon.GetString();
        }
    }
    CHECK(solveNamed && geometryNamed && commitNamed);
    for(const auto &step:B.excludedSteps) {
        CHECK(std::none_of(B.steps.begin(),B.steps.end(),[&](const auto &live) {
            return live.kind==step.kind && live.object==step.object && live.part==step.part;
        }));
        if(step.kind==RigExecBakedStepKind::Solve)CHECK(B.aggregates[size_t(step.object)].frames.empty());
        if(step.kind==RigExecBakedStepKind::RevisionFuse) {
            const auto index=B.revisionIndex[size_t(step.object)];
            const auto &revision=B.chains[size_t(index.first)].revisions[size_t(index.second)];
            CHECK(revision.output.empty());
        }
    }
    Report("(f) acyclic remainder",rigExecTest::CheckOpGraphIsAcyclic(E.GetOpGraph()));
    CHECK(rigExecTest::FindOpGraphSteps(E.GetOpGraph(),"Solve","/Solvers/Lift").size()==1);
    CHECK(rigExecTest::FindOpGraphSteps(E.GetOpGraph(),"PropertyRevision","/RawSelected").size()==1);
    bool diagnosed=false;
    for(const auto &line:pose.diagnostics)diagnosed|=line.find("cycle")!=std::string::npos;
    for(const auto &line:rig.errors)diagnosed|=line.find("cycle")!=std::string::npos;
    CHECK(diagnosed);
    CHECK(E.Evaluate(UsdTimeCode(3)).valid);
    for(const auto &step:B.excludedSteps)
        if(step.kind==RigExecBakedStepKind::Solve)CHECK(B.aggregates[size_t(step.object)].frames.empty());
    std::printf("  (f) complete cross-domain SCC named and excluded; independent work runs\n");
    {
    // An earlier optional-driver revision shares the SCC geometry chain.
    // Withdrawing it must not renumber the later retained exclusion identity.
    LiveRig rig;
    rig.stage=UsdStage::Open(fixturesDir+"/oneloop_cross_cycle.usda");
    CHECK(rig.stage); if(!rig.stage)return;
    const auto oldTarget=rig.stage->GetEditTarget();
    rig.stage->SetEditTarget(rig.stage->GetSessionLayer());
    const SdfPath withdrawn("/S9Asset/Rig/Geometry/BeforeLift");
    auto earlier=rig.stage->DefinePrim(withdrawn,TfToken("RigExecCurveMover"));
    CHECK(earlier.AddAppliedSchema(TfToken("RigExecMoverAPI")));
    CHECK(earlier.CreateRelationship(TfToken("rigExec:moves")).SetTargets({SdfPath("/S9Asset/Geom/Curve.points")}));
    CHECK(earlier.CreateRelationship(TfToken("rigExec:driverCurve")).SetTargets({SdfPath("/S9Asset/Geom/Curve")}));
    CHECK(earlier.CreateRelationship(TfToken("rigExec:driverFrames")).SetTargets({SdfPath("/S9Asset/Rig/Solvers/Ribbon")}));
    CHECK(earlier.CreateAttribute(TfToken("rigExec:mode"),SdfValueTypeNames->Token,false,SdfVariabilityUniform).Set(TfToken("emitGuidePoints")));
    // The mover compiler visits siblings in reverse authored order.
    const std::vector<TfToken> revisionOrder{TfToken("Lift"),TfToken("BeforeLift")};
    const auto geometry=rig.stage->GetPrimAtPath(SdfPath("/S9Asset/Rig/Geometry"));
    geometry.SetChildrenReorder(revisionOrder);
    CHECK(geometry.GetChildrenReorder()==revisionOrder);
    rig.stage->SetEditTarget(oldTarget);
    rig.evaluator=std::make_unique<RigExecRigEvaluator>(rig.stage,rigPath);
    if(!rig.evaluator->Compile(&rig.errors))rig.evaluator.reset();
    CHECK(rig.evaluator); if(!rig.evaluator)return;
    auto &E=*rig.evaluator;
    CHECK(E.Evaluate(UsdTimeCode(2)).valid);
    const auto &B=E.GetBakedProgram()->GetStepGraph();
    const SdfPath lift("/S9Asset/Rig/Geometry/Lift");
    CHECK(E.GetSkippedOperations().count(withdrawn)==1);
    bool laterIdentity=false;
    for(const auto &step:B.excludedSteps)if(step.kind==RigExecBakedStepKind::RevisionFuse) {
        const auto index=B.revisionIndex[size_t(step.object)];
        const auto &revision=B.chains[size_t(index.first)].revisions[size_t(index.second)];
        if(revision.moverPath==lift) {
            CHECK(index.second==0); // withdrawn predecessor is no longer a revision
            CHECK(step.descriptorKey.find("/revision:1/")!=std::string::npos);
            laterIdentity=true;
        }
    }
    CHECK(laterIdentity);
    for(const auto &chain:B.chains)for(const auto &revision:chain.revisions)
        CHECK(revision.moverPath!=withdrawn);
    CHECK(!B.opGraph.cycles.empty());
    CHECK(rigExecTest::FindOpGraphSteps(E.GetOpGraph(),"Solve","/Solvers/Lift").size()==1);
    CHECK(E.Evaluate(UsdTimeCode(3)).valid);
    for(const auto &step:B.excludedSteps)if(step.kind==RigExecBakedStepKind::Solve)
        CHECK(B.aggregates[size_t(step.object)].frames.empty());
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
    TestCrossDomainTrace(fixturesDir);
    TestTwoLimbs(fixturesDir);
    TestPrecedingWeight(fixturesDir);
    TestPropertyDragCone(fixturesDir);
    TestSkipEffects(fixturesDir);
    TestBackendKernelsAndBinaryCycle(fixturesDir);
    TestCycle(fixturesDir);
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecOneLoopAcceptance: all tests passed\n");
    return 0;
}
