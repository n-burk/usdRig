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

#include <string>

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
void RigExecBakedBuildSchedule(RigExecBakedProgramImpl *program);

/// Assigns every step its size, its cost and its longest-path level.
///
/// The cost model is `a[kind] + b[kind] x size`, with the constants in the
/// table at the head of bakedSchedule.cpp and the sizes §5.1 names. It is
/// evaluated at Build from the program's own shape and never from a
/// measurement -- see RigExecBakedScheduleCalibrationRequested for how the
/// table is replaced when the shape stops predicting the machine.
void RigExecBakedAssignStepCosts(RigExecBakedProgramImpl *program);

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

/// Runs every step of \p program, returning false when one of them gave the
/// generation back.
///
/// Nothing here touches the pose: a step writes its diagnostics, its counter
/// deltas and its phased-read records into itself, and this merges the
/// records into the run's store in step order. The epilogue replays the
/// rest.
bool RigExecBakedRunSteps(RigExecBakedProgramImpl *program, UsdTimeCode time);

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

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_SCHEDULE_H
