// Native body dispatch, semantic declaration checks, costs, and graph reports.
// Canonical dependency compilation and execution live in bakedOpGraph.
#ifndef RIGEXEC_BAKED_SCHEDULE_H
#define RIGEXEC_BAKED_SCHEDULE_H

#include "bakedProgramImpl.h"

#include "pxr/usd/usd/timeCode.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

/// Which executor a run uses.
enum class RigExecBakedScheduleMode {
    /// Steps in program order on one thread. The reference, and what
    /// RIGEXEC_BAKED_SCHEDULE=serial asks for.
    Serial,
    /// Clusters spread across the work arena, one task per cluster, with a
    /// padded atomic remaining-predecessor counter deciding what becomes
    /// runnable. What RIGEXEC_BAKED_SCHEDULE=parallel asks for.
    Parallel,
};

/// The mode RIGEXEC_BAKED_SCHEDULE asks for, read once.
///
/// Forced to Serial when RigExecParallelEvaluationEnabled() is false: the
/// one switch every parallel region in this library is behind answers for
/// this one too.
RigExecBakedScheduleMode RigExecBakedScheduleModeFromEnvironment();

/// Whether \p program's step graph is one every executor may trust, which
/// Build asks before it hands the program out and refuses it when not.
///
/// Three checks. The edges: `preds` and `succs` sorted, unique, pointing
/// backward and forward respectively, and each the inverse of the other.
/// The producers: every slot a step reads in a domain the prologue does not
/// fill has a writer at a strictly lower index, and no step names the retired
/// Snapshots domain; every pose version a commit, a solve or a last-version
/// reader is bound to was written by an earlier step; and every reader of a
/// chain's points declares the version it reads, whose only producer is one
/// revision's fuse and is among the reader's preds. The clusters: a partition of the steps with
/// members in program order, cluster edges that cover the step edges, and a
/// topological order naming every cluster once.
///
/// On failure \p error receives the first violation, naming the steps, the
/// domain and the slot, and how many more there were.
bool RigExecBakedValidateStepGraph(const RigExecBakedProgramImpl &program,
                                   std::string *error);

/// Validates property, rest, ladder, layout, and weight-field declarations.
/// Head classification does not constrain canonical order or dependencies.
bool RigExecBakedValidateHeadTier(const RigExecBakedProgramImpl &program,
                                  std::string *error);


/// The clusters a value edit on override index \p index re-runs, when the
/// index is a ladder channel's (kEditRouteHead): the rest and ladder ops
/// whose binding leaves hold it, their forward closure within the head
/// tier (a parent's RestCompose reaches its children's through
/// `Rest[parent]`), and the cluster of every region step that declares a
/// head output of an op in that closure. Sorted and unique; empty for an
/// index no head op reads.
std::vector<int> RigExecBakedHeadSeeds(const RigExecBakedProgramImpl &program,
                                       int index);

/// RigExecBakedHeadSeeds in parts, for a caller asking for many indices:
/// per head step, the clusters its closure's outputs reach; the head steps
/// whose binding leaves hold \p index's leaf; and the clusters a set of
/// head steps reaches, sorted.
std::vector<RigExecBakedClusterSet> RigExecBakedHeadOpSeeds(
    const RigExecBakedProgramImpl &program);
std::vector<uint32_t> RigExecBakedHeadOpsReading(
    const RigExecBakedProgramImpl &program, int index);
std::vector<int> RigExecBakedHeadSeedsFrom(
    const RigExecBakedProgramImpl &program,
    const std::vector<RigExecBakedClusterSet> &opSeeds,
    const std::vector<uint32_t> &ops);

/// Assigns every step from \p firstStep on its size, its cost and its
/// longest-path level.
///
/// The cost model is `a[kind] + b[kind] x size`, with the constants in the
/// table at the head of bakedSchedule.cpp and the sizes §5.1 names. It is
/// evaluated at Build from the program's own shape and never from a
/// measurement -- see RigExecBakedScheduleCalibrationRequested for how the
/// table is replaced when the shape stops predicting the machine.
///
/// A step's cost reads nothing a later step adds, and its level only its
/// predecessors', which are earlier -- so the steps before \p firstStep keep
/// what an earlier call gave them, and Build costs each half once.
void RigExecBakedAssignStepCosts(RigExecBakedProgramImpl *program,
                                 size_t firstStep = 0);

/// The grain Build uses: RIGEXEC_BAKED_GRAIN_US when it is set, and
/// otherwise `clamp(total cost / (4 x concurrency), 5us, 50us)`.
double RigExecBakedScheduleGrainUs(double totalCost);

/// The clusters of \p clustering, each after all of its predecessors.
///
/// Cluster ids come out of the level packing, which numbers bins and not
/// dependencies -- cluster 6 can perfectly well have cluster 10 among its
/// predecessors -- so anything that walks the cluster graph one cluster at a
/// time needs this order and not increasing id. A cycle is a coding error
/// and leaves the clusters on it out of the answer. Build caches the result
/// as `RigExecBakedClustering::topologicalOrder`.
std::vector<int> RigExecBakedClusterTopologicalOrder(
    const RigExecBakedClustering &clustering);

/// Computes cluster reachability and source-to-operation lookup tables.
///
/// Once at Build, `cone[c]` records downstream clusters for output-affected
/// queries. The common executor closes candidate operations over its actual
/// successor adjacency; no dense all-pairs step table is retained. Cluster
/// closure walks cached topological order. These tables depend only on the
/// compiled graph, not on timing or a particular execution.
///
/// There is no second closure. A cluster never has to run so that another
/// can read what it wrote LAST run -- versioned pose storage (§3.1) leaves
/// every version where its writer left it.
void RigExecBakedBuildCones(RigExecBakedProgramImpl *program);

/// Dispatches one body with optional timing and immutable check-row capture.
void RigExecBakedRunStepBody(RigExecBakedProgramImpl *program,
    RigExecBakedStep *step, UsdTimeCode time);

/// The clock RigExecProfiler::NowUs reads, in nanoseconds: a plain clock
/// read, so an op may stamp itself with it.
uint64_t RigExecBakedNowNs();

void RigExecBakedPrepareHeadOps(RigExecBakedProgramImpl *);

std::vector<char> RigExecBakedExpectedStageFramesAdmissionReads(const RigExecBakedProgramImpl &);
void RigExecBakedDeclareStageFramesAdmission(RigExecBakedProgramImpl *);
bool RigExecBakedRequiresStageFramesAdmission(const RigExecBakedStep &);
bool RigExecBakedPublishStageFramesRefusal(RigExecBakedProgramImpl *, RigExecRigPose *);
void RigExecBakedFinishHeadOp(RigExecBakedProgramImpl *, const RigExecBakedStep &);
bool RigExecBakedHeadValueChanged(const RigExecBakedProgramImpl &,
    RigExecBakedSlotDomain, uint32_t slot);

void RigExecBakedClearRunStamps(RigExecBakedProgramImpl *program);

/// Runs every step of \p program, returning false when one of them gave the
/// generation back.
///
/// Nothing here touches the pose: a step writes its diagnostics and its
/// counter deltas into itself, and the epilogue replays them.
/// \p force asks for every cluster, which is what a first run, a bumped
/// program stamp and the verifier's second pass all need.
bool RigExecBakedRunSteps(RigExecBakedProgramImpl *program, UsdTimeCode time,
                          bool force = false);

/// Replays this run's per-step intervals into the profiler, in step order.
///
/// A step body opens no profile scope: RIGEXEC_PROFILE_SCOPE takes three
/// mutexes even when recording is off, and a step is not allowed to take a
/// lock. Each step stores two timestamps instead and the epilogue records
/// them here -- which also makes the trace deterministic, because the events
/// arrive in program order rather than in whatever order the threads
/// finished.
void RigExecBakedReplayStepTimings(const RigExecBakedProgramImpl &program);

/// Whether RIGEXEC_BAKED_SCHEDULE_CALIBRATE asks for a measured cost table.
///
/// Opt-in, and serial: the mode runs the program the reference way, times
/// every op's memo, body and value publication with plain clock reads into
/// that op's own fields -- no lock, no shared counter -- and after the
/// requested number of frames fits the two constants of every step kind to
/// the bodies by least squares and prints a table ready to paste over the
/// one in bakedSchedule.cpp. Build itself never measures anything.
bool RigExecBakedScheduleCalibrationRequested();

/// Folds this run's per-step intervals into the calibration accumulators and,
/// once enough frames have been seen, prints the replacement table.
void RigExecBakedScheduleCalibrate(RigExecBakedProgramImpl *program);

/// A line-per-step dump of the graph: kinds, declared ranges, edges and
/// totals.
///
/// Deterministic -- it names nothing that depends on a run, an address or a
/// map iteration order -- so two builds of one stage produce the same text,
/// which is what makes it usable as a golden.
std::string RigExecBakedScheduleReport(const RigExecBakedProgramImpl &program);

/// The last run's per-cluster wait and run times, one line per cluster.
///
/// Separate from the structural report because it is the only part of the
/// schedule that is an observation: the structure can be printed at Build,
/// this can only be printed after a frame.
std::string RigExecBakedScheduleRunReport(
    const RigExecBakedProgramImpl &program);

/// Whether RIGEXEC_BAKED_SCHEDULE_REPORT asks for that dump on stderr.
bool RigExecBakedScheduleReportRequested();

/// Whether RIGEXEC_BAKED_STEP_TIMING asks what a frame spends where.
///
/// Opt-in and OFF by default, in both schedule modes, because the answer
/// costs a few clock reads per step and a frame has several hundred of them
/// -- enough to move the number being asked about. With it off no executor
/// reads a clock unless the profiler is recording, op timing is on
/// (RigExecRigEvaluator::SetOpTimingEnabled), calibration is measuring, or
/// the schedule report asked for cluster times, which is what makes the
/// default frame the frame a caller actually gets.
bool RigExecBakedStepTimingRequested();

/// Folds one frame's phase and step times into the accumulators and, once
/// enough frames have been seen, prints the table on stderr.
///
/// The phases are the three a frame divides into -- the serial prologue, the
/// region, the serial epilogue -- and the steps are grouped by kind, so the
/// table says both which third of the frame to attack and which step kind
/// within it. The body table comes first; after it, the ops' memos and
/// value publications by kind. Averaged over the frames watched rather than
/// printed per frame: a single frame of a few hundred microseconds is
/// mostly noise.
///
/// The frame count and the phases come only from a frame that published a
/// pose, and the cone verifier's second pass is excluded from everything.
/// The per-op sums are not gated on publication: a body is summed whenever
/// it ran, and a memo or a publication whenever its run's op graph
/// completed, so frames whose generation a step or the publication gave
/// back overstate the per-kind lines against the divisor.
void RigExecBakedStepTimingReport(RigExecBakedProgramImpl *program);

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_SCHEDULE_H
