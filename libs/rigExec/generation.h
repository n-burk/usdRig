//
// RigExec generation tokens: the edit fence in front of the per-frame cache.
//
// A generation is a rig's edit serial. The UI thread takes the current token
// at enqueue time, samples the job's input vector under it, and the worker
// rechecks the token twice: before it runs and again before it publishes.
// Any mismatch drops the job's result, and the frame it would have warmed
// simply evaluates live when asked. The token source itself lives in the
// background scheduler (backgroundScheduler.h); what lives here is the
// checking discipline -- which call sites ask, and what each answer means --
// plus the D7 admission policy, so that every producer and consumer of a
// token answers those questions the same way.
//
// The bump contract (Stream E wires it; the calls already exist): every
// rig-affecting edit calls RigExecBackgroundScheduler::CancelGeneration for
// the rig, and the edit-commit trigger enqueues fresh jobs under the new
// token. Entering a rig at generation 0 and bumping on every edit is what
// makes "stale" decidable from the token alone, with no frame-level
// invalidation bookkeeping.
//
// Header-only on purpose: the checks are two comparisons, and every caller
// -- the frozen worker, the scheduler, the imaging publish fence -- inlines
// them rather than paying for a call across the library boundary per frame.
//

#ifndef RIGEXEC_GENERATION_H
#define RIGEXEC_GENERATION_H

#include "backgroundScheduler.h"

#include "pxr/usd/sdf/path.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Whether the job enqueued under \p generation may run: true when the token
/// is still the rig's current one. Asked by the worker before it evaluates.
///
/// A stale token means an edit landed between enqueue and run. The sampled
/// input vector is still a faithful record of the older edit, but the epoch
/// it was sampled for no longer stands, so running would warm a frame nobody
/// will ever ask for under a digest nobody will ever compute. The worker
/// frees its arena and exits; the caller evaluates live instead.
inline bool
RigExecGenerationCheckAtStart(const RigExecBackgroundScheduler &scheduler,
                             const SdfPath &rig,
                             RigExecFrameGeneration generation)
{
    return scheduler.IsGenerationCurrent(rig, generation);
}

/// Whether the result computed under \p generation may be published: true
/// when the token survived the run. Asked by the worker after it evaluates
/// and before the result reaches the cache.
///
/// The same predicate as CheckAtStart, and deliberately a second call
/// rather than the first call's answer carried forward: an edit that lands
/// DURING the run must drop a result that was current when the run began.
/// Publishing it would store a pose of the older edit under a digest the
/// newer edit's lookup recomputes -- a plausible wrong answer, which the
/// cache's "drop, don't guess" rule exists to forbid.
inline bool
RigExecGenerationCheckBeforePublish(
    const RigExecBackgroundScheduler &scheduler, const SdfPath &rig,
    RigExecFrameGeneration generation)
{
    return scheduler.IsGenerationCurrent(rig, generation);
}

/// Whether a background warming job may be created for a rig whose bake
/// state is \p bakeRefused (plan D7).
///
/// False for a refusal rig: the dynamic path drives OpenExec against the
/// live stage and cannot run on a worker, so there is nothing a background
/// job could evaluate. The refusal is not an error -- the rig still incidentally
/// memoizes the UI thread's own live evaluations through the same publish
/// path -- but no job is ever enqueued for it, and the frozen worker declines
/// a context carrying the refusal flag even if one somehow arrives.
inline bool
RigExecShouldEnqueueBackgroundJob(bool bakeRefused)
{
    return !bakeRefused;
}

/// Whether the UI thread's own live result for a rig may be memoized into
/// the frame cache. Always true, including for bake refusals: D7 memoizes
/// UI-thread evaluations on the same key with no background fill, so a
/// refusal rig still hits on a repeated frame -- it just never warms one it
/// has not visited.
inline bool
RigExecShouldMemoizeUiThreadResult(bool bakeRefused)
{
    (void)bakeRefused;
    return true;
}

}  // namespace rigExec

#endif  // RIGEXEC_GENERATION_H
