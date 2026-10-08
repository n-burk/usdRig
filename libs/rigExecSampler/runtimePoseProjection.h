#ifndef RIGEXEC_SAMPLER_RUNTIME_POSE_PROJECTION_H
#define RIGEXEC_SAMPLER_RUNTIME_POSE_PROJECTION_H

#include "rigExec/rigEvaluator.h"
#include "rigExecRuntime/runtime.h"

namespace rigExec {
/// Copy the runtime's actual published values into the common pose encoding.
/// This adapter performs no evaluation, stage read, or transform arithmetic.
/// The caller supplies the sampled time and Execute's success state.
bool RigExecProjectRuntimePose(const RigExecRuntimeReader &reader,
    UsdTimeCode time, bool valid, RigExecRigPose *pose,
    std::string *error = nullptr);
}
#endif
