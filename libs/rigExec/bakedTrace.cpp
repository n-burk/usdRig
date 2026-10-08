// The baked program's op trace and op graph. See bakedTrace.h.
#include "bakedTrace.h"

#include "bakedProgram.h"
#include "bakedProgramImpl.h"
#include "rigEvaluator.h"

#include <algorithm>
#include <map>
#include <set>

namespace rigExec {

const char *
RigExecBakedStepDomainName(RigExecBakedStepKind kind)
{
    switch (kind) {
    case RigExecBakedStepKind::PropertyRevision:
        return "property";
    case RigExecBakedStepKind::RestCompose:
    case RigExecBakedStepKind::LadderCompose:
    case RigExecBakedStepKind::ComposeSubtree:
    case RigExecBakedStepKind::Solve:
    case RigExecBakedStepKind::SolverCommit:
    case RigExecBakedStepKind::Constraint:
    case RigExecBakedStepKind::CommitDelta:
    case RigExecBakedStepKind::PropagateChunk:
    case RigExecBakedStepKind::CommitApply:
    case RigExecBakedStepKind::ProviderMatrix:
    case RigExecBakedStepKind::SnapshotFinals:
    case RigExecBakedStepKind::PoseInterpolator:
    case RigExecBakedStepKind::FrameMatrix:
    case RigExecBakedStepKind::AvarInputs:
    case RigExecBakedStepKind::SpaceExpression:
    case RigExecBakedStepKind::SpaceCheckpoint:
    case RigExecBakedStepKind::ProviderRefresh:
        return "pose";
    case RigExecBakedStepKind::VolumePlacements:
    case RigExecBakedStepKind::WeightField:
    case RigExecBakedStepKind::WeightPacket:
        return "weight";
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::SkinTopology:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
    case RigExecBakedStepKind::ChainStatus:
    case RigExecBakedStepKind::Derived:
    case RigExecBakedStepKind::ChainInputs:
        return "geometry";
    }
    return "unknown";
}

std::vector<RigExecOpTraceEntry>
RigExecBakedLastRunTrace(const RigExecBakedProgramImpl &B)
{
    std::vector<RigExecOpTraceEntry> trace;
    for(size_t canonical=0;canonical<B.opGraph.ops.size();++canonical) {
        if(canonical>=B.opExecution.ran.size() || !B.opExecution.ran[canonical] ||
            canonical>=B.opExecution.completion.size()) continue;
        const auto original=B.opGraph.ops[canonical].originalIndex;
        if(original>=B.steps.size()) continue;
        const auto &step=B.steps[original]; RigExecOpTraceEntry entry;
        entry.step=canonical; entry.kind=RigExecBakedStepKindName(step.kind);
        entry.domain=RigExecBakedStepDomainName(step.kind); entry.label=step.label;
        entry.seq=uint32_t(B.opExecution.completion[canonical]);
        entry.cluster=canonical<B.opGraph.opClusters.size() ? int(B.opGraph.opClusters[canonical]) : -1;
        trace.push_back(std::move(entry));
    }
    std::sort(trace.begin(), trace.end(),
              [](const RigExecOpTraceEntry &a, const RigExecOpTraceEntry &b) {
                  return a.seq < b.seq;
              });
    return trace;
}

namespace {

std::vector<RigExecOpSlotRange>
SlotRanges(const std::vector<RigExecBakedSlotRange> &ranges)
{
    std::vector<RigExecOpSlotRange> out;
    out.reserve(ranges.size());
    for (const RigExecBakedSlotRange &range : ranges) {
        if (range.IsEmpty()) {
            continue;
        }
        out.push_back({RigExecBakedSlotDomainName(range.domain), range.begin,
                       range.end - 1});
    }
    return out;
}

}  // namespace

std::vector<RigExecOpGraphNode>
RigExecBakedOpGraph(const RigExecBakedProgramImpl &B)
{
    std::vector<RigExecOpGraphNode> graph(B.opGraph.ops.size());
    for(size_t canonical=0;canonical<B.opGraph.ops.size();++canonical) {
        const auto &op=B.opGraph.ops[canonical];
        if(op.originalIndex>=B.steps.size()) continue;
        const auto &step=B.steps[op.originalIndex]; auto &node=graph[canonical];
        node.step=canonical; node.kind=RigExecBakedStepKindName(step.kind);
        node.domain=RigExecBakedStepDomainName(step.kind); node.label=step.label;
        node.preds.assign(op.predecessors.begin(),op.predecessors.end());
        node.succs.assign(op.successors.begin(),op.successors.end());
        node.cluster=canonical<B.opGraph.opClusters.size() ? int(B.opGraph.opClusters[canonical]) : -1;
        node.reads=SlotRanges(step.reads); node.writes=SlotRanges(step.writes);
    }
    return graph;
}

std::vector<RigExecOpTraceEntry>
RigExecBakedProgram::GetLastOpTrace() const
{
    return RigExecBakedLastRunTrace(*_impl);
}

std::vector<RigExecOpGraphNode>
RigExecBakedProgram::GetOpGraph() const
{
    return RigExecBakedOpGraph(*_impl);
}

std::map<SdfPath, std::string>
RigExecBakedProgram::_GetCycleSkips(
    const std::map<SdfPath, SdfPath> &switchOwners) const
{
    const auto &B = *_impl;
    using K = RigExecBakedStepKind;
    struct Member { SdfPath owner; bool solver = false, pose = false;
        const RigExecBakedStep *step = nullptr; };
    std::map<std::string, Member> members;
    // Excluded steps are exactly the original SCC members, retained before
    // canonicalization. A dependent that remains runnable cannot enter this map.
    for (const auto &step : B.excludedSteps) {
        Member member; member.step = &step;
        if (step.object < 0) continue;
        const size_t object = size_t(step.object);
        switch (step.kind) {
        case K::Solve:
            member.owner = B.solvers[object].path;
            member.solver = member.pose = true;
            break;
        case K::SolverCommit: case K::Constraint: case K::CommitDelta:
        case K::PropagateChunk: case K::CommitApply: {
            if (object >= B.commits.size() || object >= B.walkSteps.size()) break;
            const auto &commit = B.commits[object];
            member.owner = commit.moverPath;
            if (member.owner.IsEmpty() && object < B.walkSteps.size()) {
                const auto &walk = B.walkSteps[object];
                if (!walk.batchSolvers.empty())
                    member.owner = B.solvers[size_t(walk.batchSolvers.front())].path;
            }
            member.solver = B.walkSteps[object].solverBatch;
            member.pose = true;
            break;
        }
        case K::ComposeSubtree: case K::RestCompose: case K::LadderCompose: {
            const auto &group = B.composeGroups[object];
            const auto found = switchOwners.find(B.paths[size_t(group.begin)]);
            if (found != switchOwners.end()) member.owner = found->second;
            member.pose = true;
            break;
        }
        case K::PropertyRevision:
            if (step.part > 0)
                member.owner = B.propertyChains[object].revisions[size_t(step.part - 1)].mover;
            break;
        case K::SkinTopology:
            if (const auto *revision = RigExecBakedLayoutRevision(B, step.object))
                member.owner = revision->moverPath;
            break;
        case K::InfluenceFold: case K::RevisionStatic:
        case K::RevisionChunk: case K::RevisionFuse: {
            const auto &entry = B.revisionIndex[object];
            member.owner = B.chains[size_t(entry.first)].revisions[size_t(entry.second)].moverPath;
            break;
        }
        case K::Derived: {
            const auto &entry = B.derivedIndex[object];
            member.owner = B.chains[size_t(entry.first)].derived[size_t(entry.second)].revision.moverPath;
            break;
        }
        case K::PoseInterpolator:
            member.owner = B.poseInterpolators[object].path;
            member.pose = true;
            break;
        case K::FrameMatrix:
            member.owner = B.frameRecords[object].mover;
            member.pose = true;
            break;
        case K::WeightPacket:
            member.owner = B.weightObjects[object].path;
            break;
        case K::WeightField:
            member.owner = B.weightObjects[size_t(B.weightFields[object].object)].path;
            break;
        default:
            // Provider seeds/bridges and chain bookkeeping are not authored
            // operations. They may be SCC members without public skip owners.
            break;
        }
        members.emplace(step.descriptorKey, std::move(member));
    }
    std::map<SdfPath, std::string> result = B.cycleSkipReasons;
    for (size_t component = 0; component < B.opGraph.cycleMembers.size(); ++component) {
        // Historical SCC reasons were captured against their own revision
        // arrays before normalization; never interpret old object indices
        // against the replacement tables.
        if (std::find(B.cycleExclusionProof.cycleMembers.begin(),
                      B.cycleExclusionProof.cycleMembers.end(),
                      B.opGraph.cycleMembers[component]) !=
            B.cycleExclusionProof.cycleMembers.end()) continue;
        std::set<SdfPath> owners;
        bool pose = false, onlySolvers = true;
        for (const auto &key : B.opGraph.cycleMembers[component]) {
            const auto found = members.find(key);
            if (found == members.end()) continue;
            pose = pose || found->second.pose;
            if (!found->second.owner.IsEmpty()) {
                owners.insert(found->second.owner);
                onlySolvers = onlySolvers && found->second.solver;
            }
        }
        if (owners.empty()) continue;
        // A Final read of the reader's own chain closes a real dependency
        // through that chain's status writer. Report the captured authored
        // phase only when both descriptors belong to this same SCC.
        std::set<int> finalChains;
        for (const auto &key : B.opGraph.cycleMembers[component]) {
            const auto found = members.find(key);
            if (found != members.end() && found->second.step &&
                found->second.step->kind == K::ChainStatus)
                finalChains.insert(found->second.step->object);
        }
        std::map<SdfPath, std::set<std::string>> authoredReads;
        for (const auto &key : B.opGraph.cycleMembers[component]) {
            const auto found = members.find(key);
            if (found == members.end() || !found->second.step) continue;
            const auto &step = *found->second.step;
            if (step.kind != K::InfluenceFold && step.kind != K::RevisionStatic &&
                step.kind != K::RevisionChunk && step.kind != K::RevisionFuse) continue;
            const auto &entry = B.revisionIndex[size_t(step.object)];
            if (!finalChains.count(entry.first)) continue;
            const auto &chain = B.chains[size_t(entry.first)];
            const auto &revision = chain.revisions[size_t(entry.second)];
            const auto inspect = [&](const RigExecBakedPointsBinding &binding) {
                if (binding.phase.kind != RigExecReadPhaseKind::Final ||
                    !binding.finalRead || binding.input != chain.target) return;
                const bool closesOwnChain = std::any_of(binding.candidates.begin(),
                    binding.candidates.end(), [&](const auto &candidate) {
                        return candidate.chain == entry.first &&
                            candidate.version == int(chain.revisions.size());
                    });
                if (closesOwnChain)
                    authoredReads[revision.moverPath].insert(revision.moverPath.GetString() +
                        ": read phase 'final' on " + binding.input.GetString() +
                        " creates a self dependency on the final output");
            };
            for (const auto &binding : revision.pointBindings) inspect(binding);
            for (const auto &channel : revision.blendChannels)
                for (const auto &sample : channel.samples) inspect(sample.pointBinding);
        }
        std::string reason = "operation cycle";
        if (pose) reason += onlySolvers ? " (solver dependency cycle)" : " (pose dependency cycle)";
        reason += ": ";
        const auto &loop = B.opGraph.cycles[component];
        // Keep the actual edge-following descriptor loop even when a short
        // loop omits another SCC owner. The full owner set is separately named.
        for (size_t i = 0; i < loop.size(); ++i) {
            if (i) reason += " -> ";
            const auto found = members.find(loop[i]);
            reason += found != members.end() && !found->second.owner.IsEmpty()
                ? found->second.owner.GetString() : loop[i];
        }
        reason += "; cyclic operations: ";
        bool first = true;
        for (const auto &owner : owners) {
            if (!first) reason += ", ";
            first = false;
            reason += owner.GetString();
        }
        for (const auto &owner : owners) {
            std::string ownerReason = reason;
            const auto provenance = authoredReads.find(owner);
            if (provenance != authoredReads.end())
                for (const auto &line : provenance->second) ownerReason += "; " + line;
            const auto added = result.emplace(owner, ownerReason);
            if (!added.second && added.first->second != ownerReason)
                added.first->second += "\n" + ownerReason;
        }
    }
    return result;
}

std::vector<RigExecOpTraceEntry>
RigExecRigEvaluator::GetLastOpTrace() const
{
    if (!_lastGenerationRanProgram || !_bakedProgram) {
        return {};
    }
    return _bakedProgram->GetLastOpTrace();
}

std::vector<RigExecOpGraphNode>
RigExecRigEvaluator::GetOpGraph() const
{
    if (!_bakedProgram) {
        return {};
    }
    return _bakedProgram->GetOpGraph();
}

}  // namespace rigExec
