//
// The baked program's scheduler: edges, executors and the report.
//
// See bakedSchedule.h for what belongs here and what belongs with a domain.
//
#include "bakedSchedule.h"

#include "parallel.h"
#include "profiler.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

const char *
RigExecBakedSlotDomainName(RigExecBakedSlotDomain domain)
{
    switch (domain) {
    case RigExecBakedSlotDomain::Avars: return "Avars";
    case RigExecBakedSlotDomain::PoseBase: return "PoseBase";
    case RigExecBakedSlotDomain::PoseFin: return "PoseFin";
    case RigExecBakedSlotDomain::PosedM: return "PosedM";
    case RigExecBakedSlotDomain::FinalMatrix: return "FinalMatrix";
    case RigExecBakedSlotDomain::BaseMatrix: return "BaseMatrix";
    case RigExecBakedSlotDomain::Aggregate: return "Aggregate";
    case RigExecBakedSlotDomain::Candidates: return "Candidates";
    case RigExecBakedSlotDomain::CommitTable: return "CommitTable";
    case RigExecBakedSlotDomain::CommitDelta: return "CommitDelta";
    case RigExecBakedSlotDomain::CommitStaging: return "CommitStaging";
    case RigExecBakedSlotDomain::PropertyResult: return "PropertyResult";
    case RigExecBakedSlotDomain::ChainBase: return "ChainBase";
    case RigExecBakedSlotDomain::RevisionPacket: return "RevisionPacket";
    case RigExecBakedSlotDomain::RevisionTransforms:
        return "RevisionTransforms";
    case RigExecBakedSlotDomain::RevisionOut: return "RevisionOut";
    case RigExecBakedSlotDomain::RevisionDone: return "RevisionDone";
    case RigExecBakedSlotDomain::ChainDirty: return "ChainDirty";
    case RigExecBakedSlotDomain::ChainPoints: return "ChainPoints";
    case RigExecBakedSlotDomain::DerivedOut: return "DerivedOut";
    case RigExecBakedSlotDomain::Snapshots: return "Snapshots";
    }
    return "unknown";
}

const char *
RigExecBakedStepKindName(RigExecBakedStepKind kind)
{
    switch (kind) {
    case RigExecBakedStepKind::ComposeSubtree: return "ComposeSubtree";
    case RigExecBakedStepKind::Solve: return "Solve";
    case RigExecBakedStepKind::SolverCommit: return "SolverCommit";
    case RigExecBakedStepKind::Constraint: return "Constraint";
    case RigExecBakedStepKind::CommitDelta: return "CommitDelta";
    case RigExecBakedStepKind::PropagateChunk: return "PropagateChunk";
    case RigExecBakedStepKind::CommitApply: return "CommitApply";
    case RigExecBakedStepKind::ProviderMatrix: return "ProviderMatrix";
    case RigExecBakedStepKind::SnapshotFinals: return "SnapshotFinals";
    case RigExecBakedStepKind::InfluenceFold: return "InfluenceFold";
    case RigExecBakedStepKind::RevisionStatic: return "RevisionStatic";
    case RigExecBakedStepKind::RevisionChunk: return "RevisionChunk";
    case RigExecBakedStepKind::RevisionFuse: return "RevisionFuse";
    case RigExecBakedStepKind::ChainStatus: return "ChainStatus";
    case RigExecBakedStepKind::Derived: return "Derived";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// Execution mode.
// ---------------------------------------------------------------------------

RigExecBakedScheduleMode
RigExecBakedScheduleModeFromEnvironment()
{
    // Read once. A mode that could change between two frames of one session
    // would make "the same program produced two different traces" a question
    // about when the variable was read rather than about the schedule.
    static const RigExecBakedScheduleMode mode = [] {
        if (!RigExecParallelEvaluationEnabled()) {
            return RigExecBakedScheduleMode::Serial;
        }
        return TfGetenv("RIGEXEC_BAKED_SCHEDULE", "serial") == "parallel"
                   ? RigExecBakedScheduleMode::Parallel
                   : RigExecBakedScheduleMode::Serial;
    }();
    return mode;
}

bool
RigExecBakedScheduleReportRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_BAKED_SCHEDULE_REPORT", false);
    return requested;
}

// ---------------------------------------------------------------------------
// Edges.
// ---------------------------------------------------------------------------

namespace {

/// A half-open run of one domain's slots, and the step it belongs to.
struct SlotInterval {
    uint32_t begin = 0, end = 0;
    int step = 0;
};

/// Removes [\p begin, \p end) from every interval of \p intervals, splitting
/// one that straddles it. This is what "update lastWriter for the range"
/// means when the bookkeeping is per interval rather than per slot: the
/// parts of an older writer's range the new one did not cover are still that
/// writer's.
void
SubtractRange(std::vector<SlotInterval> *intervals, uint32_t begin,
              uint32_t end)
{
    std::vector<SlotInterval> kept;
    kept.reserve(intervals->size() + 1);
    for (const SlotInterval &interval : *intervals) {
        if (interval.end <= begin || end <= interval.begin) {
            kept.push_back(interval);
            continue;
        }
        if (interval.begin < begin) {
            kept.push_back({interval.begin, begin, interval.step});
        }
        if (end < interval.end) {
            kept.push_back({end, interval.end, interval.step});
        }
    }
    intervals->swap(kept);
}

void
SortRanges(std::vector<RigExecBakedSlotRange> *ranges)
{
    std::sort(ranges->begin(), ranges->end());
    ranges->erase(std::unique(ranges->begin(), ranges->end()), ranges->end());
}

}  // namespace

namespace {
std::string StepLabel(const RigExecBakedProgramImpl &B,
                      const RigExecBakedStep &step);
}  // namespace

void
RigExecBakedBuildSchedule(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    std::array<std::vector<SlotInterval>, RigExecBakedSlotDomainCount>
        writers, readers;
    for (int index = 0; index < int(B.steps.size()); ++index) {
        RigExecBakedStep &step = B.steps[size_t(index)];
        SortRanges(&step.reads);
        SortRanges(&step.writes);
        step.preds.clear();
        const auto overlapping = [&](const std::vector<SlotInterval> &table,
                                     const RigExecBakedSlotRange &range) {
            for (const SlotInterval &interval : table) {
                if (interval.begin < range.end && range.begin < interval.end &&
                    interval.step != index) {
                    step.preds.push_back(interval.step);
                }
            }
        };
        for (const RigExecBakedSlotRange &range : step.reads) {
            overlapping(writers[size_t(range.domain)], range);
        }
        // Registered before the write pass, so that a read-modify-write step
        // -- every constraint is one -- does not raise a write-after-read
        // edge against itself; `overlapping` skips its own index.
        for (const RigExecBakedSlotRange &range : step.reads) {
            readers[size_t(range.domain)].push_back(
                {range.begin, range.end, index});
        }
        for (const RigExecBakedSlotRange &range : step.writes) {
            overlapping(writers[size_t(range.domain)], range);
            overlapping(readers[size_t(range.domain)], range);
        }
        for (const RigExecBakedSlotRange &range : step.writes) {
            std::vector<SlotInterval> &writerTable =
                writers[size_t(range.domain)];
            SubtractRange(&writerTable, range.begin, range.end);
            SubtractRange(&readers[size_t(range.domain)], range.begin,
                          range.end);
            writerTable.push_back({range.begin, range.end, index});
        }
        std::sort(step.preds.begin(), step.preds.end());
        step.preds.erase(std::unique(step.preds.begin(), step.preds.end()),
                         step.preds.end());
        // The invariant the whole design rests on: an edge only ever leaves a
        // step the sweep has already passed, so a cluster running its members
        // in increasing program index is always in topological order.
        for (const int pred : step.preds) {
            TF_VERIFY(pred < index,
                      "rigExec: baked step %d depends on later step %d",
                      index, pred);
        }
    }
    for (RigExecBakedStep &step : B.steps) {
        step.succs.clear();
        step.label = std::string(RigExecBakedStepKindName(step.kind)) + " " +
                     StepLabel(B, step);
    }
    for (int index = 0; index < int(B.steps.size()); ++index) {
        for (const int pred : B.steps[size_t(index)].preds) {
            B.steps[size_t(pred)].succs.push_back(index);
        }
    }
}

// ---------------------------------------------------------------------------
// The serial executor.
// ---------------------------------------------------------------------------

bool
RigExecBakedRunSteps(RigExecBakedProgramImpl *program, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    // The mode is read for the report and for the shape of this loop; the
    // parallel executor lands with the clustering pass, and until it does an
    // asked-for parallel run is a correct serial one rather than no run.
    const bool profiling = B.profiler && B.profiler->IsEnabled();
    uint64_t mark = profiling ? RigExecProfiler::NowUs() : 0;
    for (RigExecBakedStep &step : B.steps) {
        // A run's output is cleared HERE rather than in the body, so that the
        // clearing is the executor's promise and not something fifteen bodies
        // each have to remember.
        step.BeginRun();
        if (RigExecBakedIsGeometryStep(step.kind)) {
            RigExecBakedRunGeometryStep(&B, &step, time);
        } else {
            RigExecBakedRunPoseStep(&B, &step, time);
        }
        if (profiling) {
            // ONE clock read per step boundary, not two per step: a biped's
            // graph is several hundred steps and a thousand reads of a
            // vDSO clock is a measurable part of the frame being measured.
            // What each interval then covers is the step plus the few
            // instructions of bookkeeping below it, which is where the time
            // went. The parallel executor cannot share a boundary this way
            // and will take its own pair per cluster, which it can afford.
            const uint64_t now = RigExecProfiler::NowUs();
            step.startUs = mark;
            step.endUs = now;
            mark = now;
        }
        // The phased-read records this step made, folded into the run's store
        // in step order -- which is what lets a step record into a store
        // nobody else can see and still leave the walk's order deciding what
        // the store ends up holding.
        if (!step.snapshots.IsEmpty()) {
            B.runSnapshots.Merge(std::move(step.snapshots));
        }
        // Bodies size their own lists at Build; a body that grew past what it
        // declared is a step allocating inside the region, which is the thing
        // the declaration exists to prevent.
        TF_VERIFY(step.diagnostics.size() <= step.maxDiagnostics,
                  "rigExec: baked step emitted %zu diagnostics, at most %zu "
                  "declared", step.diagnostics.size(), step.maxDiagnostics);
        if (step.bail) {
            // The generation is going back to the dynamic path, so nothing
            // after this step is worth running -- and nothing it would have
            // written is ever read: the caller drops the program.
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// The report.
// ---------------------------------------------------------------------------

namespace {

std::string
StepLabel(const RigExecBakedProgramImpl &B, const RigExecBakedStep &step)
{
    const auto revisionOf = [&B](int id) -> const
        RigExecBakedProgramImpl::GeomRevision & {
        const auto &[chain, revision] = B.revisionIndex[size_t(id)];
        return B.chains[size_t(chain)].revisions[size_t(revision)];
    };
    switch (step.kind) {
    case RigExecBakedStepKind::ComposeSubtree:
        return B.paths[size_t(B.composeGroups[size_t(step.object)].begin)]
            .GetString();
    case RigExecBakedStepKind::Solve:
        return B.solvers[size_t(step.object)].path.GetString();
    case RigExecBakedStepKind::SolverCommit:
    case RigExecBakedStepKind::Constraint:
    case RigExecBakedStepKind::CommitDelta:
    case RigExecBakedStepKind::PropagateChunk:
    case RigExecBakedStepKind::CommitApply: {
        const RigExecBakedCommit &commit = B.commits[size_t(step.object)];
        return commit.moverPath.IsEmpty()
                   ? "batch " + std::to_string(step.object)
                   : commit.moverPath.GetString();
    }
    case RigExecBakedStepKind::ProviderMatrix:
        return B.paths[size_t(step.object)].GetString() +
               (step.part ? " final" : " base");
    case RigExecBakedStepKind::SnapshotFinals:
        return "every provider";
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
        return revisionOf(step.object).moverPath.GetString();
    case RigExecBakedStepKind::ChainStatus:
        return B.chains[size_t(step.object)].target.GetString();
    case RigExecBakedStepKind::Derived: {
        const auto &[chain, derived] = B.derivedIndex[size_t(step.object)];
        return B.chains[size_t(chain)]
            .derived[size_t(derived)]
            .target.GetString();
    }
    }
    return std::string();
}

void
AppendRanges(std::string *out, const char *what,
              const std::vector<RigExecBakedSlotRange> &ranges)
{
    *out += "        ";
    *out += what;
    if (ranges.empty()) {
        *out += " -\n";
        return;
    }
    for (const RigExecBakedSlotRange &range : ranges) {
        *out += " ";
        *out += RigExecBakedSlotDomainName(range.domain);
        *out += "[" + std::to_string(range.begin) + "," +
                std::to_string(range.end) + ")";
    }
    *out += "\n";
}

}  // namespace

std::string
RigExecBakedScheduleReport(const RigExecBakedProgramImpl &B)
{
    size_t edges = 0;
    std::map<std::string, size_t> byKind;
    for (const RigExecBakedStep &step : B.steps) {
        edges += step.preds.size();
        ++byKind[RigExecBakedStepKindName(step.kind)];
    }
    std::string out = "rigExec baked schedule: " +
                      std::to_string(B.steps.size()) + " step(s), " +
                      std::to_string(edges) + " edge(s), " +
                      std::to_string(B.paths.size()) + " provider slot(s)\n";
    out += "  mode=";
    out += RigExecBakedScheduleModeFromEnvironment() ==
                   RigExecBakedScheduleMode::Parallel
               ? "parallel"
               : "serial";
    out += "\n";
    for (const auto &[kind, count] : byKind) {
        out += "  " + kind + " " + std::to_string(count) + "\n";
    }
    for (int index = 0; index < int(B.steps.size()); ++index) {
        const RigExecBakedStep &step = B.steps[size_t(index)];
        out += "  [" + std::to_string(index) + "] " + step.label;
        if (step.part >= 0) {
            out += " #" + std::to_string(step.part);
        }
        out += "\n";
        AppendRanges(&out, "reads ", step.reads);
        AppendRanges(&out, "writes", step.writes);
        out += "        preds ";
        if (step.preds.empty()) {
            out += "-";
        }
        for (const int pred : step.preds) {
            out += " " + std::to_string(pred);
        }
        out += "\n";
    }
    return out;
}


// ---------------------------------------------------------------------------
// Profiling.
// ---------------------------------------------------------------------------

void
RigExecBakedReplayStepTimings(const RigExecBakedProgramImpl &B)
{
    if (!B.profiler || !B.profiler->IsEnabled()) {
        return;
    }
    for (const RigExecBakedStep &step : B.steps) {
        // A step that did not take a whole microsecond is on nobody's
        // critical path, and a biped's graph holds several hundred of them
        // -- so recording each would cost more than the steps did and would
        // bury the events that matter under zero-length ones. The interval
        // is still measured; what is dropped is the report of it.
        if (step.endUs <= step.startUs) {
            continue;
        }
        B.profiler->Record(step.label, "step", step.startUs, step.endUs);
    }
}

}  // namespace rigExec
