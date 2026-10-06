// The baked program's op trace and op graph. See bakedTrace.h.
#include "bakedTrace.h"

#include "bakedProgram.h"
#include "bakedProgramImpl.h"
#include "rigEvaluator.h"

#include <algorithm>

namespace rigExec {

const char *
RigExecBakedStepDomainName(RigExecBakedStepKind kind)
{
    switch (kind) {
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
        return "pose";
    case RigExecBakedStepKind::VolumePlacements:
    case RigExecBakedStepKind::WeightPacket:
        return "weight";
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
    case RigExecBakedStepKind::ChainStatus:
    case RigExecBakedStepKind::Derived:
        return "geometry";
    }
    return "unknown";
}

std::vector<RigExecOpTraceEntry>
RigExecBakedLastRunTrace(const RigExecBakedProgramImpl &B)
{
    std::vector<RigExecOpTraceEntry> trace;
    for (size_t index = 0; index < B.steps.size(); ++index) {
        const RigExecBakedStep &step = B.steps[index];
        if (step.runSeq == 0) {
            continue;
        }
        RigExecOpTraceEntry entry;
        entry.step = index;
        entry.kind = RigExecBakedStepKindName(step.kind);
        entry.domain = RigExecBakedStepDomainName(step.kind);
        entry.label = step.label;
        entry.seq = step.runSeq;
        entry.cluster = step.cluster;
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
    std::vector<RigExecOpGraphNode> graph(B.steps.size());
    for (size_t index = 0; index < B.steps.size(); ++index) {
        const RigExecBakedStep &step = B.steps[index];
        RigExecOpGraphNode &node = graph[index];
        node.step = index;
        node.kind = RigExecBakedStepKindName(step.kind);
        node.domain = RigExecBakedStepDomainName(step.kind);
        node.label = step.label;
        node.preds.assign(step.preds.begin(), step.preds.end());
        node.succs.assign(step.succs.begin(), step.succs.end());
        node.cluster = step.cluster;
        node.level = step.level;
        node.reads = SlotRanges(step.reads);
        node.writes = SlotRanges(step.writes);
    }
    return graph;
}

std::vector<RigExecOpTraceEntry>
RigExecBakedLastHeadTrace(const RigExecBakedProgramImpl &B)
{
    std::vector<RigExecOpTraceEntry> trace;
    for (size_t index = 0; index < B.headSteps.size(); ++index) {
        const RigExecBakedHeadStep &step = B.headSteps[index];
        if (step.runSeq == 0) {
            continue;
        }
        RigExecOpTraceEntry entry;
        entry.step = index;
        entry.kind = RigExecBakedHeadKindName(step.kind);
        entry.domain = "head";
        entry.label = step.label;
        entry.seq = step.runSeq;
        trace.push_back(std::move(entry));
    }
    std::sort(trace.begin(), trace.end(),
              [](const RigExecOpTraceEntry &a, const RigExecOpTraceEntry &b) {
                  return a.seq < b.seq;
              });
    return trace;
}

std::vector<RigExecOpGraphNode>
RigExecBakedHeadGraph(const RigExecBakedProgramImpl &B)
{
    const auto ranges = [](const std::vector<RigExecBakedHeadRange> &list) {
        std::vector<RigExecOpSlotRange> out;
        for (const RigExecBakedHeadRange &range : list) {
            if (!range.IsEmpty()) {
                out.push_back({RigExecBakedHeadDomainName(range.domain),
                               range.begin, range.end - 1});
            }
        }
        return out;
    };
    std::vector<int> level(B.headSteps.size(), 0);
    std::vector<RigExecOpGraphNode> graph;
    for (const uint32_t index : B.headOrder) {
        const RigExecBakedHeadStep &step = B.headSteps[index];
        for (const uint32_t pred : step.preds) {
            level[index] = std::max(level[index], level[pred] + 1);
        }
        RigExecOpGraphNode node;
        node.step = index;
        node.kind = RigExecBakedHeadKindName(step.kind);
        node.domain = "head";
        node.label = step.label;
        node.preds.assign(step.preds.begin(), step.preds.end());
        node.succs.assign(step.succs.begin(), step.succs.end());
        node.level = level[index];
        node.reads = ranges(step.reads);
        node.writes = ranges(step.writes);
        graph.push_back(std::move(node));
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
    if (!_lastGenerationRanProgram || !_bakedProgram) {
        return {};
    }
    return _bakedProgram->GetOpGraph();
}

}  // namespace rigExec
