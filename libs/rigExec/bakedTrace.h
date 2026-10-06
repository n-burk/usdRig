// The baked program's op trace and op graph: what the last run executed, in
// completion order, and the step graph it ran over, as plain records.
// Observability only. Nothing here is read by evaluation; both views are
// built on the calling thread from step-owned storage after a run.
#ifndef RIGEXEC_BAKED_TRACE_H
#define RIGEXEC_BAKED_TRACE_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecBakedProgramImpl;
enum class RigExecBakedStepKind;

/// The evaluation domain a step kind belongs to: "pose", "weight" or
/// "geometry".
const char *RigExecBakedStepDomainName(RigExecBakedStepKind kind);

/// One step the last run executed.
struct RigExecOpTraceEntry {
    /// Program index of the step.
    size_t step = 0;
    /// RigExecBakedStepKindName of the step.
    std::string kind;
    /// RigExecBakedStepDomainName of the step.
    std::string domain;
    /// The step's report label.
    std::string label;
    /// 1-based completion order within the run.
    uint32_t seq = 0;
    /// The cluster the step belongs to, -1 when unclustered.
    int cluster = -1;
};

/// One declared slot range of a step. `first` and `last` are inclusive.
struct RigExecOpSlotRange {
    std::string domain;
    uint32_t first = 0;
    uint32_t last = 0;
};

/// One step of the program's graph.
struct RigExecOpGraphNode {
    size_t step = 0;
    std::string kind;
    std::string domain;
    std::string label;
    /// Program indices; every pred is smaller than `step`.
    std::vector<size_t> preds, succs;
    int cluster = -1;
    /// Longest-path level from the clustering pass.
    int level = 0;
    /// Declared reads and writes, empty ranges omitted.
    std::vector<RigExecOpSlotRange> reads, writes;
};

/// The steps \p program's last run executed, sorted by completion order.
/// Empty before the first run.
std::vector<RigExecOpTraceEntry>
RigExecBakedLastRunTrace(const RigExecBakedProgramImpl &program);

/// Every step of \p program, in program order.
std::vector<RigExecOpGraphNode>
RigExecBakedOpGraph(const RigExecBakedProgramImpl &program);

/// The head-tier ops \p program's last run executed, in execution order,
/// as the same records: `step` is the head step's index, `kind`
/// RigExecBakedHeadKindName, `domain` "head", `cluster` -1. Empty before
/// the first run. Separate while the head tier is outside the step
/// graph.
std::vector<RigExecOpTraceEntry>
RigExecBakedLastHeadTrace(const RigExecBakedProgramImpl &program);

/// Every head step of \p program, in head order, with its declared reads
/// and writes by RigExecBakedHeadDomainName; `level` is the longest path
/// over its predecessors.
std::vector<RigExecOpGraphNode>
RigExecBakedHeadGraph(const RigExecBakedProgramImpl &program);

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_TRACE_H
