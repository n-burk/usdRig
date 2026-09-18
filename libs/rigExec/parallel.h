//
// The one switch every parallel region in rigExec is behind.
//
#ifndef RIGEXEC_PARALLEL_H
#define RIGEXEC_PARALLEL_H

#include <cstddef>

namespace rigExec {

/// Whether rigExec may spread its own work across threads.
///
/// Backed by the TfEnvSetting RIGEXEC_ENABLE_PARALLEL_EVAL, default true.
/// Every parallel region this library opens -- the compile-time structure
/// digest, the compile-time exec warm-up, the per-point geometry kernels and
/// the per-chain tasks of a parallel-safe chain level -- asks here first, so
/// a threading regression can be bisected to rigExec
/// by one setting. PXR_WORK_THREAD_LIMIT cannot answer that question: it also
/// serialises exec's own compilation and scheduling, so a run with it set
/// differs from the one being diagnosed in more ways than the one under test.
bool RigExecParallelEvaluationEnabled();

/// Points per task in the per-point geometry kernels.
///
/// Large enough that a task is worth its dispatch, small enough that the
/// tail task does not decide the wall time on a wide machine.
inline constexpr size_t RigExecGeometryGrainSize = 512;

/// Below this many points a per-point geometry kernel stays serial: the
/// dispatch and the join cost more than the work they would spread, and a
/// rig is far more likely to hold many small meshes than one large one.
inline constexpr size_t RigExecGeometryParallelThreshold = 4096;

}  // namespace rigExec

#endif  // RIGEXEC_PARALLEL_H
