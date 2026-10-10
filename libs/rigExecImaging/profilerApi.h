// Native profiler reports for hosts without the Python rigexec binding.
#ifndef RIGEXEC_IMAGING_PROFILER_API_H
#define RIGEXEC_IMAGING_PROFILER_API_H
#include "registry.h"
extern "C" {
// Owner-thread inspection of the already-active imaging evaluator. These
// calls neither open a private stage nor evaluate it. The JSON reply uses
// the same two-call buffer convention as the reports below.
RIGEXEC_IMAGING_C_API int RigExecImaging_LiveDebugJsonForStage(
    long long stageCacheId, const char *rigPath, const char *knownGraph,
    const char *knownGeneration, char *out, int cap);
RIGEXEC_IMAGING_C_API int RigExecImaging_SetLiveOpTimingForStage(
    long long stageCacheId, const char *rigPath, int enabled);
// Capture on the stage's owning thread before measuring on a worker. The
// returned stage is independent of the source's layers, session and notices.
RIGEXEC_IMAGING_C_API long long RigExecIntrospect_CreateSnapshot(long long stageCacheId);
RIGEXEC_IMAGING_C_API void RigExecIntrospect_ReleaseSnapshot(long long stageCacheId);
// JSON calls return required bytes excluding NUL, or -1 for an invalid ID.
// A short buffer may be retried on the same thread without re-measuring.
RIGEXEC_IMAGING_C_API int RigExecIntrospect_ScheduleReportJson(
    long long stageCacheId, const char *rigPath, int samples,
    const char *controlPath, const char *avar, char *out, int cap);
RIGEXEC_IMAGING_C_API int RigExecIntrospect_MoverOrderJson(
    long long stageCacheId, const char *rigPath, char *out, int cap);
RIGEXEC_IMAGING_C_API int RigExecIntrospect_WriteProfileTrace(
    long long stageCacheId, const char *rigPath, const char *controlPath,
    const char *avar, const char *path);
}
#endif
