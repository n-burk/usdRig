// RigExec parallelism switch. See parallel.h.
#include "parallel.h"

#include "pxr/base/tf/envSetting.h"

PXR_NAMESPACE_USING_DIRECTIVE

PXR_NAMESPACE_OPEN_SCOPE

TF_DEFINE_ENV_SETTING(
    RIGEXEC_ENABLE_PARALLEL_EVAL, true,
    "Whether rigExec may run its own work in parallel. Set to false to make "
    "every rigExec parallel region -- the compile-time digest and prefetch "
    "tasks, the op-graph clusters and the per-point geometry kernels -- run "
    "on the calling thread, and background warming stay off, without "
    "disturbing exec's own threading the way "
    "PXR_WORK_THREAD_LIMIT would.");

PXR_NAMESPACE_CLOSE_SCOPE

namespace rigExec {

bool
RigExecParallelEvaluationEnabled()
{
    return TfGetEnvSetting(RIGEXEC_ENABLE_PARALLEL_EVAL);
}

}  // namespace rigExec
