//
// RigExec background scheduler: the frame-job queue and the generation fence
// in front of it.
//
// Frame jobs warm the per-frame cache on a small pool of below-normal OS
// priority workers, each job running the baked serial executor end to end
// under a frozen serial scope. Workers never call Work*/TBB: a kernel-level
// parallel call would run its tasks on the shared arena at normal priority,
// leaking the work back past the priority boundary at exactly the moment it
// should hold. The pool only warms: the playhead always evaluates live on the
// calling thread, so the playhead frame is never enqueued here. Background
// order is scrub neighbors (+-1..N of the playhead) first, then the
// Premonition-style sweep of the surrounding range.
//
// Warming is triggered by edit-commit (drag release / value commit), not by
// idle detection, with an idle signal as a secondary trigger. There are no
// timers inside the scheduler: both triggers are explicit calls from the
// imaging/registry layer. Any new edit bumps the rig's generation and cancels
// its in-flight jobs: queued jobs are purged promptly, a job checks its
// generation before it runs, and the frozen worker rechecks again before it
// publishes -- a mismatch at any fence drops the result. A dropped job frees
// its inputs promptly; the frame it would have warmed simply evaluates live
// when asked.
//
// Switches (plan D6): RIGEXEC_FRAME_CACHE={on,off,warm-off}, default on,
// where off restores exact current behavior and warm-off keeps cache reads
// while disabling background fill; RIGEXEC_ENABLE_PARALLEL_EVAL=0 likewise
// disables background fill while reads are still served. When fill is off
// every Enqueue declines and every trigger enqueues nothing, so the caller
// evaluates live -- the fail-closed answer -- while the generation fence
// keeps working: edits still bump, tokens still compare.
//
#ifndef RIGEXEC_BACKGROUND_SCHEDULER_H
#define RIGEXEC_BACKGROUND_SCHEDULER_H

#include "profiler.h"

#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/timeCode.h"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Default background worker count, from
/// reports/frame-cache-measurements.md: two below-normal workers drain the
/// biped +-8 neighborhood in ~28 ms without contending with the UI thread.
constexpr int kRigExecBackgroundSchedulerDefaultWorkers = 2;

/// Most workers a scheduler ever runs. Past four the pool contends with the
/// UI thread it exists to stay out of the way of; the constructor clamps to
/// this.
constexpr int kRigExecBackgroundSchedulerMaxWorkers = 4;

/// Default scrub-neighbor radius, from reports/frame-cache-measurements.md:
/// +-8 (16 neighbors) costs one 1.2 ms sampling burst at enqueue and ~25 ms
/// of background drain on two workers.
constexpr int kRigExecFrameCacheDefaultNeighborRadius = 8;

/// Default per-rig in-flight cap: queued plus running jobs for one rig. One
/// full default commit burst is 16 neighbors plus a 64-frame sweep; 96 holds
/// that burst plus one more neighbor set while the first drains, and bounds
/// the queued input vectors (~30 MB worst case on the biped) however fast
/// commits arrive. Enqueue past the cap declines and the caller evaluates
/// live.
constexpr size_t kRigExecBackgroundSchedulerDefaultPerRigJobs = 96;

/// A rig's edit generation. Bumped by CancelGeneration on every rig-affecting
/// edit; a job enqueued under an older one is stale and its result is
/// dropped. Starts at zero for a rig the scheduler has never seen.
/// A rig's edit generation. Bumped by CancelGeneration on every rig-affecting
/// edit; a job enqueued under an older one is stale and its result is
/// dropped. Starts at zero for a rig the scheduler has never seen.
using RigExecFrameGeneration = uint64_t;

/// A per-time publish fence token (plan 2.1). Scoped cancellation
/// (CancelGenerationTimes) purges queued jobs for affected times WITHOUT
/// bumping the generation -- old and requeued jobs would then share one
/// generation token -- so the purge assigns each affected time a fresh
/// token from the rig's monotonic counter. A warm request carries the
/// token current at enqueue time, and the publish fence requires a token
/// match beside the generation check: an old job for a requeued time
/// drops on mismatch and can never overwrite the new result, while
/// unaffected times keep their tokens and their running jobs publish
/// normally. Zero is the never-purged token.
using RigExecWarmFenceToken = uint64_t;

/// Warming order. Lower runs first; the playhead itself is never enqueued.
enum class RigExecWarmPriority {
    /// A scrub neighbor of the playhead (+-1..N).
    Neighbor = 0,
    /// The edit-commit sweep of the surrounding range.
    Sweep = 1,
};

/// The RIGEXEC_FRAME_CACHE switch (plan D6).
enum class RigExecFrameCacheMode {
    /// Warming on, reads served. The default.
    On,
    /// Exact current behavior: no warming, no reads. The scheduler's half of
    /// off is declining every Enqueue and enqueueing nothing from triggers;
    /// reads are Stream E's half.
    Off,
    /// Reads served, background fill off. The scheduler's half is the same
    /// decline as Off.
    WarmOff,
};

/// Parses a RIGEXEC_FRAME_CACHE value: "on", "off", "warm-off". Anything else
/// -- including empty -- answers On, the default.
RigExecFrameCacheMode RigExecParseFrameCacheMode(const std::string &value);

/// Reads RIGEXEC_FRAME_CACHE from the environment, live on every call, so a
/// host can flip warming without restarting. Unset answers On. An
/// unrecognized value warns once per process and answers On.
RigExecFrameCacheMode RigExecFrameCacheModeFromEnvironment();

/// The background-fill gate as a pure function of its two inputs: fill runs
/// only when the frame-cache mode is On AND parallel evaluation is enabled.
/// RIGEXEC_ENABLE_PARALLEL_EVAL=0 therefore disables background fill while
/// cache reads are still served, exactly like warm-off.
bool RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode mode,
                                     bool parallelEvalEnabled);

/// The background-fill gate from the environment: the RIGEXEC_FRAME_CACHE
/// mode (live) and RigExecParallelEvaluationEnabled (the library's cached
/// switch). Enqueue and the triggers consult this on every call.
bool RigExecBackgroundWarmingEnabled();

/// One warming request: evaluate \p time for \p rig, publishing into the
/// frame cache behind the generation fence.
struct RigExecWarmRequest {
    SdfPath rig;
    UsdTimeCode time = UsdTimeCode::Default();
    RigExecWarmPriority priority = RigExecWarmPriority::Neighbor;
    RigExecFrameGeneration generation = 0;
    /// The per-time fence token current at enqueue time, stamped by the
    /// scheduler on insert. The publish fence requires a match (see
    /// IsWarmRequestCurrent); callers never set this.
    RigExecWarmFenceToken fenceToken = 0;
};

/// One ran job's publish disposition, reported by the work instead of void
/// so completion accounting splits on what actually happened: a "ran" job
/// that declined inside used to count completed exactly like a publish.
enum class RigExecWarmOutcome {
    /// Evaluated and published into the frame cache.
    Published,
    /// Declined to publish: the evaluation declined, the pose was invalid,
    /// the cache refused the entry -- or the work threw (the scheduler
    /// warns and lands this, fail-closed). The frame evaluates live when
    /// asked.
    DeclinedInvalid,
    /// Dropped by the publish fence: an edit landed mid-run. The frame
    /// requeues under the live generation.
    DeclinedGeneration,
};

/// The per-job evaluation a worker runs: serial, from frozen inputs only,
/// never Work/TBB. The scheduler invokes it under a frozen serial scope --
/// every kernel variant the job reaches takes its serial form on the worker
/// thread, whatever the process-wide switch says -- after a generation check
/// at start; the work itself (production: RigExecEvaluateFrozen plus publish)
/// rechecks before publish and reports the disposition. Sampled inputs travel
/// inside the closure, built on the UI thread at enqueue time: the worker
/// receives values, never live state, stage handles, or queries.
using RigExecWarmWork =
    std::function<RigExecWarmOutcome(const RigExecWarmRequest &)>;

/// Why a factory built no work for a frame. Reasoned skips count the
/// aggregate declined plus their per-reason breakout; unreasoned skips
/// (trigger shape, not warmability) skip silently, as before.
enum class RigExecWarmSkipReason {
    /// Work was built, or the skip carries no warmability verdict.
    None,
    /// The rig refused the freeze: no snapshot, no job. The detail is the
    /// session's named freeze cause.
    FreezeRefused,
    /// The frame could not be sampled faithfully. The detail names the
    /// sampler gate.
    Unsampleable,
    /// D7: no baked program, so the rig evaluates dynamically and never
    /// warms.
    D7Exempt,
};

/// One factory call's answer: the work (empty when the frame skips) plus
/// the skip reason. Implicitly constructible from bare work so test and
/// bench factories -- which carry no verdict -- keep compiling unchanged;
/// an unreasoned empty build skips silently.
struct RigExecWarmFactoryResult {
    RigExecWarmWork work;
    RigExecWarmSkipReason skip = RigExecWarmSkipReason::None;
    std::string detail;
    RigExecWarmFactoryResult() = default;
    RigExecWarmFactoryResult(RigExecWarmWork built)
        : work(std::move(built))
    {
    }
};

/// Builds one frame's work on the calling (UI) thread at enqueue time: sample
/// the frame's input vector, close over it, and return the closure the worker
/// runs. An empty build skips the frame -- it could not be sampled
/// faithfully -- and the frame evaluates live when asked. Called only while
/// the generation is current and the fill gate is open, and skipped without
/// sampling for a time already queued (best-effort under concurrent
/// triggers: a time that queues between the check and the sample merges
/// after the sample, freeing it).
using RigExecWarmJobFactory =
    std::function<RigExecWarmFactoryResult(UsdTimeCode)>;

/// Per-trigger sampling budget: bounds the UI-thread factory sampling one
/// trigger performs (the in-flight cap does not bound one tick's sampling).
/// Already-queued merges consume none -- they sample nothing -- so a tick
/// over warm ground is nearly free however large the sweep vector is.
struct RigExecWarmSamplingBudget {
    /// Max factory invocations per trigger. Unlimited by default.
    size_t maxFactoryInvocations = std::numeric_limits<size_t>::max();
    /// Millisecond stop, measured from \p startUs. Unlimited by default.
    double maxMs = std::numeric_limits<double>::infinity();
    /// Stamped by the caller (the registry stamps after burst prep, so the
    /// stop bounds factory sampling, not prep). Zero measures from the
    /// trigger's entry.
    uint64_t startUs = 0;
};

/// One per-frame queue transition, reported to the transition hook: the
/// registry records it into the shared warm-frame index, which is what the
/// per-frame query API reads. Delivered under the scheduler lock, in the
/// same critical section as the counter it explains, so the hook must be
/// fast, leaf-only, and never re-enter the scheduler or the registry.
enum class RigExecWarmTransitionKind {
    /// Enqueued (direct or trigger): the frame is warming-from-queue.
    Queued,
    /// Popped by a worker (or the manual drain): warming-from-running.
    Running,
    /// Ran to completion: \p outcome says what happened.
    Finished,
    /// Purged by CancelGeneration.
    Canceled,
    /// Purged by NotePlaybackAdvanced.
    Shed,
    /// Popped, then an edit landed before the pre-run check.
    DroppedStale,
    /// Still queued at shutdown.
    DroppedAtShutdown,
};

/// \p outcome is meaningful for Finished only.
struct RigExecWarmTransition {
    SdfPath rig;
    double timeValue = 0.0;
    RigExecWarmTransitionKind kind = RigExecWarmTransitionKind::Queued;
    RigExecWarmOutcome outcome = RigExecWarmOutcome::Published;
};

/// Per-frame queue visibility for the warm-frame index. Null by default
/// (no reporting cost beyond one branch); the registry installs one that
/// records into shared state. Set before the first trigger -- workers
/// report through it -- and never from inside it.
using RigExecWarmTransitionHook =
    std::function<void(const RigExecWarmTransition &)>;

/// Sets the calling thread to below-normal OS priority for background
/// warming. Returns whether the platform honored it; when it cannot -- no API
/// on the platform, or the call failed -- it warns once per process and the
/// worker runs at normal priority (the logged no-op fallback). Idempotent.
bool RigExecSetBackgroundThreadPriority();

/// Names the RigExecSetBackgroundThreadPriority backend on this platform:
/// "SetThreadPriority" (Windows), "sched-batch" (Linux), "qos-utility"
/// (macOS), or "none" (logged no-op fallback). Never null. For the Stream F
/// bench report and for diagnosing whether priority applied; gate tests never
/// assert OS priority values.
const char *RigExecBackgroundPriorityBackend();

/// Lifetime counters. Every counter only moves forward except across
/// destruction. Jobs that enter the queue balance exactly: queued-ever ==
/// published + declinedInvalid + declinedGeneration + droppedStale +
/// canceled + shed + droppedAtShutdown + queuedDepth + running, where
/// queued-ever is accepts minus merges -- coalesced and upgraded merges
/// accept without queueing -- and declined counts every Enqueue refusal.
/// completed ("ran", kept for compat) == published + declinedInvalid +
/// declinedGeneration; a throwing job warns and lands declinedInvalid.
struct RigExecBackgroundSchedulerStats {
    /// Jobs queued and waiting for a worker.
    size_t queuedDepth = 0;
    /// Jobs popped and executing right now.
    size_t running = 0;
    /// Jobs whose work ran to completion. (Whether the work published is the
    /// work's own publish fence to decide; completion means "ran".)
    size_t completed = 0;
    /// Ran jobs that evaluated and published into the frame cache.
    size_t published = 0;
    /// Ran jobs that declined to publish: the evaluation declined, the pose
    /// was invalid, the cache refused the entry -- or the work threw. The
    /// frame evaluates live when asked.
    size_t declinedInvalid = 0;
    /// Ran jobs dropped by the publish fence: an edit landed mid-run. The
    /// frame requeues under the live generation.
    size_t declinedGeneration = 0;
    /// Jobs dropped by the pre-run generation check: popped, then an edit
    /// landed before the check. Rare by construction -- CancelGeneration
    /// purges queued jobs first -- but the race is real, so the fence stands.
    size_t droppedStale = 0;
    /// Queued jobs purged by CancelGeneration.
    size_t canceled = 0;
    /// Queued sweep jobs purged by NotePlaybackAdvanced.
    size_t shed = 0;
    /// Enqueues merged into an already-queued job at the same priority.
    size_t coalesced = 0;
    /// Enqueues merged into an already-queued sweep job by promoting it to
    /// neighbor. Disjoint from coalesced.
    size_t upgraded = 0;
    /// Enqueue refusals: fill gate closed, stale generation, unusable time,
    /// empty work, per-rig cap, or shutdown -- plus reasoned factory skips,
    /// which never entered the queue either (broken out below).
    size_t declined = 0;
    /// Reasoned factory skips by cause, each also counted in declined: the
    /// rig refused the freeze, the frame was unsampleable, the rig is
    /// D7-exempt.
    size_t declinedFreezeRefused = 0;
    size_t declinedUnsampleable = 0;
    size_t declinedD7Exempt = 0;
    /// Queued jobs dropped by the destructor. Running jobs finish; pending
    /// jobs are dropped and their frames evaluate live when asked.
    size_t droppedAtShutdown = 0;
    /// Factory invocations across triggers: sampling attempts, whether they
    /// enqueued, merged, or skipped. Not part of the balance -- it counts
    /// trigger work, not queue jobs -- but the per-trigger delta proves a
    /// trigger held its sampling budget.
    size_t factoryInvocations = 0;
};

/// The frame-job queue. Thread-safe; the lock is held only across token and
/// queue edits, never across input sampling or evaluation.
///
/// One scheduler serves every rig: generations and in-flight counts are per
/// rig inside it, but the queue itself is one global priority map
/// (neighbors before sweep, FIFO within a priority), so one rig's neighbors
/// preempt another rig's sweep. Harmless at one session. The destructor
/// stops the pool -- running jobs finish, pending jobs are dropped -- and
/// joins every worker, so no Enqueue, trigger, or ForTesting call may run
/// concurrently with destruction.
class RigExecBackgroundScheduler {
public:
    /// Starts \p workerCount below-normal workers (clamped to
    /// [0, kRigExecBackgroundSchedulerMaxWorkers]). Zero runs no threads:
    /// the manual mode, where queued jobs wait for RunNextQueuedForTesting /
    /// DrainQueueForTesting on the calling thread. Production uses the
    /// default.
    explicit RigExecBackgroundScheduler(
        int workerCount = kRigExecBackgroundSchedulerDefaultWorkers);
    ~RigExecBackgroundScheduler();

    RigExecBackgroundScheduler(const RigExecBackgroundScheduler &) = delete;
    RigExecBackgroundScheduler &operator=(
        const RigExecBackgroundScheduler &) = delete;

    /// The rig's current generation: the token its next job is enqueued
    /// under. Zero for a rig the scheduler has never seen.
    RigExecFrameGeneration CurrentGeneration(const SdfPath &rig) const;

    /// Queued plus running jobs for the rig (0 for a rig with none in
    /// flight). The registry's activation consults this to skip its commit
    /// cancel when idle -- an idle bump retires nothing and would orphan
    /// the initial evaluation's memos, which stamped the pre-commit
    /// generation.
    size_t InFlightCount(const SdfPath &rig) const;

    /// Cancels the rig's in-flight jobs by bumping its generation and purging
    /// its queued jobs, whose inputs free promptly. Every rig-affecting edit
    /// calls this; the edit-commit trigger then enqueues fresh jobs under the
    /// new token. Running jobs are not preempted: the pre-run and publish
    /// fences drop their results instead.
    /// Cancels the rig's in-flight jobs by bumping its generation and purging
    /// its queued jobs, whose inputs free promptly. Every rig-affecting edit
    /// calls this; the edit-commit trigger then enqueues fresh jobs under the
    /// new token. Running jobs are not preempted: the pre-run and publish
    /// fences drop their results instead.
    void CancelGeneration(const SdfPath &rig);
    /// Drops the rig's fence tokens and queued jobs -- but not its generation,
    /// which the warm index keys completions by -- as on deactivation, so a
    /// reactivated rig never inherits stale tokens. Global stats stand.
    void ResetRig(const SdfPath &rig);

    /// Cancels the rig's queued jobs for \p times only, WITHOUT bumping
    /// the generation (plan 2.1 scoped cancellation): the outcome-driven
    /// notice path calls this for patch/stamp-bump edits, where the epoch
    /// stands and only affected times requeue. Each affected time gets a
    /// fresh per-time fence token from the rig's monotonic counter, so an
    /// old job for a requeued time drops at the publish fence on token
    /// mismatch; unaffected times keep their tokens and their running
    /// jobs publish normally. Queued jobs for other times and other rigs
    /// are untouched. Answers how many jobs were purged. Global
    /// CancelGeneration stays for epoch moves only.
    size_t CancelGenerationTimes(const SdfPath &rig,
                                 const std::vector<UsdTimeCode> &times);

    /// The rig's current per-time fence token for \p time: zero when the
    /// time was never scoped-purged, otherwise the token the purge
    /// assigned. A requeued job is stamped with this at enqueue.
    RigExecWarmFenceToken CurrentFenceToken(const SdfPath &rig,
                                            UsdTimeCode time) const;

    /// Whether a warm request is still current: \p generation is the
    /// rig's current one AND \p fenceToken matches the time's current
    /// token (zero matches only a never-purged time). The publish fence
    /// asks this; the pre-run fence asks IsGenerationCurrent alone (a
    /// popped job's token can only have been superseded by a purge whose
    /// requeue has not run yet -- still droppable at publish, never
    /// runnable twice).
    bool IsWarmRequestCurrent(const SdfPath &rig,
                              RigExecFrameGeneration generation,
                              UsdTimeCode time,
                              RigExecWarmFenceToken fenceToken) const;

    /// The rig's currently queued times, in index order. The scoped-cancel
    /// path unions these with the warm index's completed times to find
    /// the affected set.
    std::vector<UsdTimeCode> QueuedTimes(const SdfPath &rig) const;

    /// The publish fence mutex (plan 3.3): worker publish and cancel/clear
    /// share it. The background completion path holds it across the
    /// fence-check (generation + per-time token) and the cache insert, and
    /// the fenced clear holds it across generation-bump + queue-purge +
    /// cache-clear + index-reset -- so a running job can never pass its
    /// check before the bump and insert after the clear. Lock order: fence
    /// outermost, then the scheduler mutex, then cache shards, then the
    /// warm index -- never inverted. Acquire it holding neither the
    /// scheduler mutex nor the registry mutex.
    std::mutex &FenceMutex() const { return _fenceMutex; }

    /// Whether \p generation is still the rig's current one. A worker asks
    /// before it runs and the frozen entry point asks again before publish.
    bool IsGenerationCurrent(
        const SdfPath &rig, RigExecFrameGeneration generation) const;

    /// Enqueues \p time for warming at \p priority, running \p work on a
    /// worker. Returns false -- without enqueueing, counting declined -- when
    /// the fill gate is closed, \p generation is stale, \p time is default or
    /// non-finite, \p work is empty, the rig is at its in-flight cap, or the
    /// pool is stopping: either way the caller evaluates live instead.
    ///
    /// A time already queued for the rig coalesces instead of queueing twice:
    /// returns true -- warming is coming -- counting coalesced, or promoting
    /// a queued sweep to neighbor and counting upgraded. First-wins: the
    /// queued work stands and the new closure is destroyed promptly.
    ///
    /// The playhead frame must never be enqueued; the pool only warms. The
    /// triggers enforce this (they know the playhead); this primitive trusts
    /// its caller.
    bool Enqueue(const SdfPath &rig, UsdTimeCode time,
                 RigExecWarmPriority priority,
                 RigExecFrameGeneration generation, RigExecWarmWork work);

    /// The primary warming trigger: call on drag release / value commit, on
    /// the UI thread, after CancelGeneration for the edit. Enqueues the
    /// scrub neighbors +-1..\p neighborRadius of \p playhead closest-first
    /// (-1, +1, -2, +2, ...) as Neighbor, then \p sweepTimes in order as
    /// Sweep, skipping the playhead itself wherever it appears, skipping
    /// default sweep entries, and building each job's work through \p factory
    /// on this thread. Returns how many jobs were enqueued.
    ///
    /// Stops early -- returning the count so far -- when \p generation goes
    /// stale mid-trigger (an edit landed; its own commit follows), when the
    /// rig hits its in-flight cap, when the sampling \p budget runs out
    /// (invocations or millisecond stop), or when a frame's factory returns
    /// empty (that frame is skipped, the trigger continues). A stale
    /// generation, a closed fill gate, or a default playhead enqueues
    /// nothing and never calls the factory. The gate is read once per
    /// burst: a flip mid-burst lands on the next trigger. \p neighborRadius
    /// <= 0 enqueues no neighbors.
    size_t OnEditCommitted(const SdfPath &rig, UsdTimeCode playhead,
                           RigExecFrameGeneration generation,
                           const std::vector<UsdTimeCode> &sweepTimes,
                           RigExecWarmJobFactory factory,
                           int neighborRadius =
                               kRigExecFrameCacheDefaultNeighborRadius,
                           RigExecWarmSamplingBudget budget =
                               RigExecWarmSamplingBudget());

    /// The secondary warming trigger: call from the frame loop when the UI is
    /// idle. Enqueues \p sweepTimes in order as Sweep -- neighbors belong to
    /// the commit trigger -- with the same skipping, factory, cap, staleness,
    /// budget, and gate rules as OnEditCommitted. Returns how many jobs were
    /// enqueued.
    size_t OnIdle(const SdfPath &rig, UsdTimeCode playhead,
                  RigExecFrameGeneration generation,
                  const std::vector<UsdTimeCode> &sweepTimes,
                  RigExecWarmJobFactory factory,
                  RigExecWarmSamplingBudget budget =
                      RigExecWarmSamplingBudget());

    /// Sheds queued sweep work the playhead already passed during active
    /// (forward) playback: purges the rig's queued Sweep jobs at or before \p
    /// playhead, whose frames the live path serves now, and returns how many
    /// were purged. Neighbor jobs are untouched -- the frames ahead of the
    /// playhead still warm the imminent scrub path -- and the standing
    /// neighbor-before-sweep order is what deprioritizes the sweep that
    /// remains. A default playhead purges nothing.
    size_t NotePlaybackAdvanced(const SdfPath &rig, UsdTimeCode playhead);

    /// The per-rig in-flight cap (queued plus running). Defaults to
    /// kRigExecBackgroundSchedulerDefaultPerRigJobs. Lowering it declines new
    /// jobs until the rig drains below it; it never purges.
    void SetPerRigJobCap(size_t cap);
    size_t GetPerRigJobCap() const;

    /// Installs the per-frame transition hook (null uninstalls). The
    /// registry installs one recording into the shared warm-frame index;
    /// see RigExecWarmTransition for the delivery contract.
    void SetTransitionHook(RigExecWarmTransitionHook hook);

    /// The worker count this scheduler runs (after clamping). Zero is the
    /// manual mode.
    int GetWorkerCount() const { return _workerCount; }

    RigExecBackgroundSchedulerStats Stats() const;

    /// The scheduler's profiler: queue-depth samples and purge points record
    /// here. Off by default; when off, each sample site costs one branch.
    const RigExecProfiler &GetProfiler() const { return _profiler; }
    RigExecProfiler *MutableProfiler() { return &_profiler; }

    /// Blocks until no job is queued or running. Event-driven, not timed:
    /// returns when the last completion, drop, purge, or shed lands.
    void WaitUntilIdle();

    /// Pops the highest-priority queued job -- neighbors before sweep, FIFO
    /// within a priority -- and runs it on the calling thread under the same
    /// serial scope and pre-run fence a worker applies. Returns false when
    /// the queue is empty. For deterministic tests with zero workers;
    /// production warms on the pool. A stale pop counts droppedStale and
    /// returns true: the queue moved.
    bool RunNextQueuedForTesting();

    /// Runs queued jobs on the calling thread until the queue is empty.
    /// Returns how many jobs' work ran (stale pops are dropped, not run).
    /// For deterministic tests with zero workers.
    size_t DrainQueueForTesting();

private:
    // One queued job: the request plus the UI-sampled work a worker runs.
    struct _QueuedJob {
        RigExecWarmRequest request;
        RigExecWarmWork work;
    };
    // Queue order: priority first, then enqueue sequence (FIFO within a
    // priority). The commit trigger enqueues neighbors closest-first, so FIFO
    // within Neighbor serves the closest frame next.
    struct _QueueKey {
        int priority = 0;
        uint64_t sequence = 0;
        bool operator<(const _QueueKey &other) const
        {
            if (priority != other.priority) {
                return priority < other.priority;
            }
            return sequence < other.sequence;
        }
    };

    void _WorkerMain();
    // Pops the next queued job under the lock; the caller runs it outside the
    // lock. When \p wait, blocks until a job arrives or the pool stops,
    // otherwise answers at once on an empty queue. Answers whether *job owns
    // a job; a false from the waiting form means "stopping, exit".
    bool _PopJob(_QueuedJob *job, bool wait);
    // Runs a popped job outside the lock: the pre-run generation fence,
    // then the work under a frozen serial scope, capturing its outcome
    // (threw maps to declinedInvalid, warn preserved), then the terminal
    // state. Answers whether the work ran. Shared by the workers and the
    // ForTesting drain so both apply exactly the same fence and scope.
    bool _ExecutePoppedJob(_QueuedJob *job);
    // Reports one transition to the hook, if any. Call with _mutex held:
    // the hook runs inside the scheduler lock, so it must be leaf-only.
    void _ReportTransitionLocked(const RigExecWarmRequest &request,
                                 RigExecWarmTransitionKind kind,
                                 RigExecWarmOutcome outcome =
                                     RigExecWarmOutcome::Published);
    // Records a popped job's terminal state under the lock: \p ran true
    // counts a completion (the work already ran, outside the lock) and
    // splits it on \p outcome; false counts a stale drop (the work never
    // ran, \p outcome ignored). Either way the rig's in-flight count
    // falls and idleness is re-signalled.
    void _FinishJob(const RigExecWarmRequest &request, bool ran,
                    RigExecWarmOutcome outcome);
    // The time's current fence token: the sidecar's entry, or zero when the
    // time was never scoped-purged. Call with _mutex held.
    RigExecWarmFenceToken _CurrentFenceTokenLocked(
        const SdfPath &rig, double timeValue) const;
    // Merges a request for an already-queued (rig, time): coalesces, or
    // promotes a queued sweep to neighbor keeping its sequence. Answers
    // whether the time was queued (and merged). Call with _mutex held.
    bool _MergeIfQueuedLocked(const std::pair<SdfPath, double> &coalesceKey,
                              RigExecWarmPriority priority);
    // One trigger time's outcome: enqueued (count it), skipped (already
    // queued or unsampleable -- the trigger continues), or stop (stale
    // generation, per-rig cap, or sampling budget -- the trigger ends).
    enum class _TriggerOutcome { Enqueued, Skipped, Stop };
    // One trigger's sampling-budget ledger, threaded through the per-frame
    // path (re-entrancy-safe: a factory that triggers again gets its own).
    struct _BudgetLedger {
        const RigExecWarmSamplingBudget *budget = nullptr;
        size_t invocations = 0;
        uint64_t startUs = 0;
    };
    // Enqueues one trigger time at \p priority, sampling its work through
    // \p factory outside the lock (sampling is slow; the lock never spans
    // it) and deciding staleness, coalescing, and cap under it. \p ledger
    // counts the invocation against the trigger's budget.
    _TriggerOutcome _EnqueueTriggerTime(
        const SdfPath &rig, UsdTimeCode time, RigExecWarmPriority priority,
        RigExecFrameGeneration generation,
        const RigExecWarmJobFactory &factory, _BudgetLedger *ledger);
    // The sweep core shared by OnEditCommitted and OnIdle: \p sweepTimes in
    // order as Sweep, skipping the playhead and default entries.
    size_t _EnqueueSweepTimes(const SdfPath &rig, UsdTimeCode playhead,
                              RigExecFrameGeneration generation,
                              const std::vector<UsdTimeCode> &sweepTimes,
                              const RigExecWarmJobFactory &factory,
                              _BudgetLedger *ledger);

    mutable std::mutex _mutex;
    // The publish fence mutex (see FenceMutex): outermost in the lock
    // order, ahead of _mutex.
    mutable std::mutex _fenceMutex;
    std::condition_variable _cv;
    std::map<SdfPath, RigExecFrameGeneration> _generations;
    std::map<_QueueKey, _QueuedJob> _queue;
    // Coalescing index: (rig, time value) -> queue key. Queued jobs only;
    // a popped job leaves the index, so a re-enqueue while it runs queues
    // honestly beside it (idempotent warming, not a duplicate run).
    // Coalescing index: (rig, time value) -> queue key. Queued jobs only;
    // a popped job leaves the index, so a re-enqueue while it runs queues
    // honestly beside it (idempotent warming, not a duplicate run).
    std::map<std::pair<SdfPath, double>, _QueueKey> _queuedByRigAndTime;
    // Per-time publish fence tokens (plan 2.1): (rig, time value) ->
    // current token, assigned by CancelGenerationTimes from the rig's
    // monotonic counter below. Absent means never-purged (token zero).
    std::map<std::pair<SdfPath, double>, RigExecWarmFenceToken> _fenceTokens;
    std::map<SdfPath, RigExecWarmFenceToken> _fenceCounters;
    // In-flight per rig: queued plus popped-and-executing. Drives the cap.
    std::map<SdfPath, size_t> _inFlight;
    std::vector<std::thread> _workers;
    uint64_t _sequence = 0;
    size_t _running = 0;
    size_t _completed = 0;
    size_t _published = 0;
    size_t _declinedInvalid = 0;
    size_t _declinedGeneration = 0;
    size_t _droppedStale = 0;
    size_t _canceled = 0;
    size_t _shed = 0;
    size_t _coalesced = 0;
    size_t _upgraded = 0;
    size_t _declined = 0;
    size_t _declinedFreezeRefused = 0;
    size_t _declinedUnsampleable = 0;
    size_t _declinedD7Exempt = 0;
    size_t _factoryInvocations = 0;
    size_t _droppedAtShutdown = 0;
    RigExecWarmTransitionHook _transitionHook;
    size_t _perRigCap = kRigExecBackgroundSchedulerDefaultPerRigJobs;
    int _workerCount = 0;
    bool _stopping = false;
    // The profiler's lock is a leaf: recording sites run under _mutex, and
    // the profiler never calls back into the scheduler.
    RigExecProfiler _profiler;
};

}  // namespace rigExec

#endif  // RIGEXEC_BACKGROUND_SCHEDULER_H
