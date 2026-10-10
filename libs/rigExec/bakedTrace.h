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
    /// Canonical operation index.
    size_t step = 0;
    /// RigExecBakedStepKindName of the step.
    std::string kind;
    /// RigExecBakedStepDomainName of the step.
    std::string domain;
    /// The step's report label.
    std::string label;
    /// 1-based completion order; skipped operations can leave gaps.
    uint32_t seq = 0;
    /// The cluster the step belongs to, -1 when unclustered.
    int cluster = -1;
    /// Last-run body interval and actual runner; empty when timing is off.
    uint64_t startUs = 0, durationUs = 0;
    std::string thread;
    /// The memo before the body and the value publication after it, in
    /// microseconds on the same clock: the op held its thread from
    /// startUs - memoUs to startUs + durationUs + publishUs. Zero when
    /// timing is off.
    uint64_t memoUs = 0, publishUs = 0;
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
    /// Canonical predecessor/successor operation indices.
    std::vector<size_t> preds, succs;
    int cluster = -1;
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

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_TRACE_H
