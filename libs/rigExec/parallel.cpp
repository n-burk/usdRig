// RigExec parallelism switch. See parallel.h.
#include "parallel.h"

#include "pxr/base/tf/envSetting.h"
#include "pxr/base/tf/getenv.h"

PXR_NAMESPACE_USING_DIRECTIVE

PXR_NAMESPACE_OPEN_SCOPE

TF_DEFINE_ENV_SETTING(
    RIGEXEC_ENABLE_PARALLEL_EVAL, true,
    "Whether rigExec may run its own work in parallel. Set to false to make "
    "every rigExec parallel region -- the compile-time digest and exec "
    "warm-up tasks, the per-point geometry kernels and the per-chain tasks "
    "of a chain level -- run on the calling thread, without disturbing "
    "exec's own threading the way "
    "PXR_WORK_THREAD_LIMIT would.");

PXR_NAMESPACE_CLOSE_SCOPE

namespace rigExec {

bool
RigExecParallelEvaluationEnabled()
{
    // Live read. TfGetEnvSetting caches the first value for the process,
    // which would freeze a suite that runs the serial executor after the
    // parallel one. The env setting above still warns when the variable is
    // set at startup.
    return TfGetenvBool("RIGEXEC_ENABLE_PARALLEL_EVAL", true);
}

}  // namespace rigExec
