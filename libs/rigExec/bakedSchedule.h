//
// The baked program's scheduler: the edges between its steps, the executors
// that run them, and the report that shows both.
//
// The step bodies live with the domain they belong to (bakedPose.cpp,
// bakedGeometry.cpp). What lives here is everything that is true of a step
// whatever it computes: that its reads and writes are declared as slot
// ranges, that the edges between steps follow mechanically from those
// declarations, that every edge points forward in program order, and that
// running the steps in program order on one thread reproduces the straight
// line this graph was derived from -- which is the reference every other
// order is measured against.
//
#ifndef RIGEXEC_BAKED_SCHEDULE_H
#define RIGEXEC_BAKED_SCHEDULE_H

#include "bakedProgramImpl.h"

#include "pxr/usd/usd/timeCode.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecBakedEdgeSweep;

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

/// Computes the edges between \p program's steps from their declared slot
/// ranges, and checks that every one of them points forward.
///
/// Once, at Build, in program order: for each read range, an edge from every
/// step still holding the last write of any slot in it; for each write
/// range, an edge from those writers (write-after-write) and from every step
/// that has read any slot of it since the program started (write-after-read,
/// which is why the reader lists are seeded from program start and not from
/// the previous write -- a step reading a slot no earlier step wrote is
/// reading last run's value, and the step that overwrites it must still
/// follow).
///
/// \p sweep carries the edge sweep Build started over the pose half; this
/// extends it over the steps appended since (see RigExecBakedEdgeSweep).
void RigExecBakedBuildSchedule(RigExecBakedProgramImpl *program,
                               RigExecBakedEdgeSweep *sweep);

/// One half-open run of a domain's slots, and the step it belongs to: the
/// unit the edge sweep's writer and reader tables are kept in.
struct RigExecBakedSlotInterval {
    uint32_t begin = 0, end = 0;
    int step = 0;
};

/// The edge sweep's running state, which is everything a step's edges
/// depend on besides its own declarations: per domain, which step still
/// holds the last write of each slot, and which steps have read each slot
/// since that write.
///
/// Kept between the two halves of Build rather than rebuilt, because the
/// sweep is a fold over the steps in program order and a step appended
/// later can change nothing an earlier step's edges were derived from. So
/// extending the tables over the geometry steps produces exactly the edges a
/// single sweep over the finished program would have -- the pose steps' own
/// edges, and with them the levels the vertex partition was cut from, are
/// never revisited and cannot move.
///
/// Sound only while the program GROWS BY APPENDING: a step at or below
/// `swept` must not have its declared ranges changed, nor a step be inserted
/// before it. Build's two halves obey that (the geometry half appends, and
/// reads the pose half's levels without writing them).
struct RigExecBakedEdgeSweep {
    std::array<std::vector<RigExecBakedSlotInterval>,
               RigExecBakedSlotDomainCount>
        writers, readers;
    /// Steps [0, swept) carry their final predecessors and labels, and
    /// successors among themselves; a later sweep only appends to those.
    int swept = 0;
};

/// The edge sweep over the steps appended since \p sweep last ran, and
/// checks that every new edge points forward.
///
/// Run twice by Build, over one \p sweep: once over the pose half by
/// itself, so that the vertex partition can ask what LEVEL a chunk's joints
/// land at before it decides whether cutting the revision buys anything, and
/// again from RigExecBakedBuildSchedule over the geometry steps appended
/// since. See RigExecBakedEdgeSweep for why the two passes together are the
/// one sweep.
void RigExecBakedBuildStepEdges(RigExecBakedProgramImpl *program,
                                RigExecBakedEdgeSweep *sweep);

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

/// Partitions \p program's steps into clusters at \p grainUs microseconds.
///
/// Pure: it reads the program and returns a partition, so a caller may ask
/// the same program for the schedule at several grains and compare them --
/// which is exactly what the byte-identity-across-grains argument needs.
/// A grain of zero means one step per cluster.
///
/// Level-pack (§5.1): steps are grouped by longest-path level, each level is
/// cut into contiguous bins of about one grain, single-successor chains are
/// fused to a fixpoint and a cluster too small to be worth a task joins its
/// one predecessor. Every one of those transformations preserves acyclicity
/// of the quotient graph, which is what makes any cluster order that
/// respects the cluster edges a valid execution order.
RigExecBakedClustering RigExecBakedBuildClusters(
    const RigExecBakedProgramImpl &program, double grainUs);

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

/// Computes the cone closure of \p program's steps and clusters (§7).
///
/// Once, at Build, from the edges and the clustering: `stepCone` holds every
/// step that has to run when a given step does, `cone[c]` every cluster that
/// has to run when c does, and the lookup tables beside each are what a
/// frame maps a changed source onto, at that grain. Nothing here measures
/// anything and nothing depends on a run. The cluster closure walks the
/// clusters in topological order, which it caches on `program->clustering`;
/// the step closure walks program order backwards.
///
/// There is no second closure. A cluster never has to run so that another
/// can read what it wrote LAST run -- versioned pose storage (§3.1) leaves
/// every version where its writer left it.
void RigExecBakedBuildCones(RigExecBakedProgramImpl *program);

/// Decides which clusters this run executes, and updates the source state
/// the next run compares against.
///
/// Sources -- the avar table, each chain's base points, each skin
/// revision's static packet, the property-chain results -- have already been
/// evaluated when this is called, and are compared by VALUE. The result is
/// left in `program->closedSteps`, the forward step closure of the steps a
/// moved source reaches, and in `program->closed`, the clusters holding one
/// of them; `force` asks for the whole program, which is what the first run
/// of an epoch, a bumped program stamp and the verifier's second pass all
/// want.
void RigExecBakedComputeClosure(RigExecBakedProgramImpl *program,
                               UsdTimeCode time, bool force);

/// Runs every step of \p program, returning false when one of them gave the
/// generation back.
///
/// Nothing here touches the pose: a step writes its diagnostics, its counter
/// deltas and its phased-read records into itself, and this merges the
/// records into the run's store in step order. The epilogue replays the
/// rest.
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
/// every step with two clock reads into that step's own accumulator -- no
/// lock, no shared counter -- and after the requested number of frames fits
/// the two constants of every step kind by least squares and prints a table
/// ready to paste over the one in bakedSchedule.cpp. Build itself never
/// measures anything.
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
/// costs two clock reads per step and a frame has several hundred of them --
/// enough to move the number being asked about. With it off no executor
/// reads a clock unless the profiler is recording or the schedule report
/// asked for cluster times, which is what makes the default frame the frame
/// a caller actually gets.
bool RigExecBakedStepTimingRequested();

/// Folds one frame's phase and step times into the accumulators and, once
/// enough frames have been seen, prints the table on stderr.
///
/// The phases are the three a frame divides into -- the serial prologue, the
/// region, the serial epilogue -- and the steps are grouped by kind, so the
/// table says both which third of the frame to attack and which step kind
/// within it. Averaged over the frames watched rather than printed per
/// frame: a single frame of a few hundred microseconds is mostly noise.
///
/// Only a frame that published a pose is watched, numerator and divisor
/// together, and the cone verifier's second pass is excluded from both --
/// so a table is always the cost of one frame of the kind a caller gets.
void RigExecBakedStepTimingReport(RigExecBakedProgramImpl *program);

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_SCHEDULE_H
