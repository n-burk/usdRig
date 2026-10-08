#ifndef RIGEXEC_OBSERVER_HOST_H
#define RIGEXEC_OBSERVER_HOST_H

namespace rigExec {
/// Executable-owned completion after all observed owners close, before teardown.
/// Both functions are idempotent and do nothing when observation is disabled.
void RigExecFinalizeGoldenSuite();
void RigExecFinalizeInputReplay();
}
#endif
