//
// RigExec background scheduler. See backgroundScheduler.h.
//
// The pool is std::thread only: no Work, no TBB anywhere in this file. A
// kernel-level parallel call on a worker would run its tasks on the shared
// arena at normal priority, past the below-normal boundary the pool exists to
// hold -- so each job runs under a RigExecFrozenSerialScope instead, and the
// OS priority call sits behind the one function the plan's risk table asks
// for, with a logged no-op fallback.
//
#include "backgroundScheduler.h"

#include "frozenContext.h"
#include "parallel.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"

#include <atomic>
#include <cmath>

#if defined(_WIN32)
// NOMINMAX and WIN32_LEAN_AND_MEAN arrive on the command line; windows.h
// stays in this translation unit only.
#include <windows.h>
#elif defined(__linux__)
#include <pthread.h>
#include <sched.h>
#elif defined(__APPLE__)
#include <pthread.h>
#include <sys/qos.h>
#endif

namespace rigExec {

namespace {

// Warns once per process per call site: the priority fallback, the thread
// shortfall, and an unrecognized switch value are host configuration, worth
// one line each, never a per-frame line.
void
_WarnPriorityOnce(const char *message)
{
    static std::atomic<bool> warned{false};
    if (!warned.exchange(true)) {
        TF_WARN("rigExec: %s", message);
    }
}

void
_WarnSwitchOnce(const char *message)
{
    static std::atomic<bool> warned{false};
    if (!warned.exchange(true)) {
        TF_WARN("rigExec: %s", message);
    }
}

}  // namespace

RigExecFrameCacheMode
RigExecParseFrameCacheMode(const std::string &value)
{
    if (value == "on") {
        return RigExecFrameCacheMode::On;
    }
    if (value == "off") {
        return RigExecFrameCacheMode::Off;
    }
    if (value == "warm-off") {
        return RigExecFrameCacheMode::WarmOff;
    }
    return RigExecFrameCacheMode::On;
}

RigExecFrameCacheMode
RigExecFrameCacheModeFromEnvironment()
{
    const std::string value = TfGetenv("RIGEXEC_FRAME_CACHE", "on");
    const RigExecFrameCacheMode mode = RigExecParseFrameCacheMode(value);
    if (mode == RigExecFrameCacheMode::On && value != "on") {
        _WarnSwitchOnce("RIGEXEC_FRAME_CACHE has an unrecognized value; "
                  "using on (want one of: on, off, warm-off)");
    }
    return mode;
}

bool
RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode mode,
                                bool parallelEvalEnabled)
{
    return mode == RigExecFrameCacheMode::On && parallelEvalEnabled;
}

bool
RigExecBackgroundWarmingEnabled()
{
    return RigExecBackgroundWarmingEnabled(
        RigExecFrameCacheModeFromEnvironment(),
        RigExecParallelEvaluationEnabled());
}

bool
RigExecSetBackgroundThreadPriority()
{
#if defined(_WIN32)
    // Below-normal, not background mode: the workers should run hot when the
    // UI is idle and yield when it is not, which is THREAD_PRIORITY_-
    // BELOW_NORMAL -- not the I/O-throttled background mode -- for CPU work.
    if (SetThreadPriority(GetCurrentThread(),
                          THREAD_PRIORITY_BELOW_NORMAL)) {
        return true;
    }
    _WarnPriorityOnce("background workers run at normal thread priority: "
                      "SetThreadPriority failed");
    return false;
#elif defined(__linux__)
    // SCHED_BATCH is the closest unprivileged per-thread below-normal: the
    // worker stays on the fair runqueue but never preempts a normal task on
    // wakeup. (A nice value would be process-wide -- it would deprioritize
    // the UI thread too -- and SCHED_IDLE would starve warming outright on a
    // loaded machine.) Failure degrades to the logged no-op: warming still
    // runs, just without the priority boundary.
    struct sched_param param;
    param.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &param) == 0) {
        return true;
    }
    _WarnPriorityOnce("background workers run at normal thread priority: "
                      "pthread_setschedparam(SCHED_BATCH) failed");
    return false;
#elif defined(__APPLE__)
    // Utility QoS: below default, above background -- the band for work the
    // user did not ask to wait for but still wants done.
    if (pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0) == 0) {
        return true;
    }
    _WarnPriorityOnce("background workers run at default QoS: "
                      "pthread_set_qos_class_self_np failed");
    return false;
#else
    _WarnPriorityOnce("background workers run at normal thread priority: no "
                      "below-normal API on this platform");
    return false;
#endif
}

const char *
RigExecBackgroundPriorityBackend()
{
#if defined(_WIN32)
    return "SetThreadPriority";
#elif defined(__linux__)
    return "sched-batch";
#elif defined(__APPLE__)
    return "qos-utility";
#else
    return "none";
#endif
}

RigExecBackgroundScheduler::RigExecBackgroundScheduler(int workerCount)
{
    if (workerCount < 0) {
        workerCount = 0;
    }
    if (workerCount > kRigExecBackgroundSchedulerMaxWorkers) {
        workerCount = kRigExecBackgroundSchedulerMaxWorkers;
    }
    for (int i = 0; i < workerCount; ++i) {
        try {
            _workers.emplace_back(
                &RigExecBackgroundScheduler::_WorkerMain, this);
        } catch (const std::exception &) {
            // A pool that cannot spawn is a smaller pool, not a failed
            // scheduler: the queued jobs wait for the workers that did start
            // (or for the manual drain at zero), and the caller always keeps
            // its live fallback.
            _WarnPriorityOnce("background scheduler started fewer workers "
                              "than asked: thread creation failed");
            break;
        }
    }
    _workerCount = int(_workers.size());
}

RigExecBackgroundScheduler::~RigExecBackgroundScheduler()
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _stopping = true;
    }
    _cv.notify_all();
    for (std::thread &worker : _workers) {
        worker.join();
    }
    // Running jobs finished above (join waits); what is still queued is
    // dropped, fail-closed -- each frame evaluates live when asked -- and
    // counted so the books balance.
    std::lock_guard<std::mutex> lock(_mutex);
    for (const auto &entry : _queue) {
        const SdfPath &rig = entry.second.request.rig;
        const auto inFlight = _inFlight.find(rig);
        if (inFlight != _inFlight.end() && inFlight->second > 0) {
            if (--(inFlight->second) == 0) {
                _inFlight.erase(inFlight);
            }
        }
        _ReportTransitionLocked(entry.second.request,
                                  RigExecWarmTransitionKind::DroppedAtShutdown);
        ++_droppedAtShutdown;
    }
    if (!_queue.empty()) {
        _profiler.RecordSchedulerCancel(_queue.size(), "shutdown");
    }
    _queue.clear();
    _queuedByRigAndTime.clear();
}

void
RigExecBackgroundScheduler::SetTransitionHook(
    RigExecWarmTransitionHook hook)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _transitionHook = std::move(hook);
}

void
RigExecBackgroundScheduler::_ReportTransitionLocked(
    const RigExecWarmRequest &request, RigExecWarmTransitionKind kind,
    RigExecWarmOutcome outcome)
{
    if (!_transitionHook) {
        return;
    }
    RigExecWarmTransition transition;
    transition.rig = request.rig;
    transition.timeValue = request.time.GetValue();
    transition.kind = kind;
    transition.outcome = outcome;
    _transitionHook(transition);
}

RigExecFrameGeneration
RigExecBackgroundScheduler::CurrentGeneration(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _generations.find(rig);
    if (found == _generations.end()) {
        return 0;
    }
    return found->second;
}

size_t
RigExecBackgroundScheduler::InFlightCount(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _inFlight.find(rig);
    return found == _inFlight.end() ? 0 : found->second;
}

void
RigExecBackgroundScheduler::CancelGeneration(const SdfPath &rig)
{
    std::unique_lock<std::mutex> lock(_mutex);
    // A rig never seen before enters at 1: generation 0 means "no edit yet",
    // so the first edit must move it.
    _generations[rig] += 1;
    // Purge the rig's queued jobs under the same lock as the bump: nothing
    // queued can be stale afterwards, so the pre-run check below only ever
    // fires for the pop race (a worker that popped just ahead of this bump).
    // The purged closures -- and the input vectors sampled for the retired
    // edit -- are moved out and freed promptly, but AFTER the lock closes:
    // up to a rig's in-flight cap of multi-megabyte vectors is not a "map
    // edit", and a worker's pop must not wait on their teardown.
    std::vector<RigExecWarmWork> retired;
    for (auto it = _queue.begin(); it != _queue.end();) {
        if (it->second.request.rig != rig) {
            ++it;
            continue;
        }
        _queuedByRigAndTime.erase(std::make_pair(
            rig, it->second.request.time.GetValue()));
        _ReportTransitionLocked(it->second.request,
                                  RigExecWarmTransitionKind::Canceled);
        retired.push_back(std::move(it->second.work));
        it = _queue.erase(it);
        const auto inFlight = _inFlight.find(rig);
        if (inFlight != _inFlight.end() && inFlight->second > 0) {
            if (--(inFlight->second) == 0) {
                _inFlight.erase(inFlight);
            }
        }
        ++_canceled;
    }
    if (!retired.empty()) {
        _profiler.RecordSchedulerCancel(retired.size(), "edit");
    }
    _profiler.RecordSchedulerQueue(_queue.size(), _running, _canceled);
    lock.unlock();
    _cv.notify_all();
    // retired dies here, outside the lock.
}

void
RigExecBackgroundScheduler::ResetRig(const SdfPath &rig)
{
    std::unique_lock<std::mutex> lock(_mutex);
    // Generations stand: the warm index keys its completions by them
    // and erasing here would desync the memo/lookup generation
    // comparison. Only the per-time fence tokens (which would fail
    // a reactivated rig's publishes) and queued jobs reset.
    _fenceCounters.erase(rig);
    for (auto it = _fenceTokens.begin(); it != _fenceTokens.end();) {
        if (it->first.first == rig) {
            it = _fenceTokens.erase(it);
        } else {
            ++it;
        }
    }
    std::vector<RigExecWarmWork> retired;
    for (auto it = _queue.begin(); it != _queue.end();) {
        if (it->second.request.rig != rig) {
            ++it;
            continue;
        }
        _queuedByRigAndTime.erase(std::make_pair(
            rig, it->second.request.time.GetValue()));
        _ReportTransitionLocked(it->second.request,
                                  RigExecWarmTransitionKind::Canceled);
        retired.push_back(std::move(it->second.work));
        it = _queue.erase(it);
        const auto inFlight = _inFlight.find(rig);
        if (inFlight != _inFlight.end() && inFlight->second > 0) {
            if (--(inFlight->second) == 0) {
                _inFlight.erase(inFlight);
            }
        }
        ++_canceled;
    }
    _inFlight.erase(rig);
    if (!retired.empty()) {
        _profiler.RecordSchedulerCancel(retired.size(), "reset");
    }
    _profiler.RecordSchedulerQueue(_queue.size(), _running, _canceled);
    lock.unlock();
    _cv.notify_all();
}

RigExecWarmFenceToken
RigExecBackgroundScheduler::_CurrentFenceTokenLocked(
    const SdfPath &rig, double timeValue) const
{
    const auto found =
        _fenceTokens.find(std::make_pair(rig, timeValue));
    return found == _fenceTokens.end() ? 0 : found->second;
}

size_t
RigExecBackgroundScheduler::CancelGenerationTimes(
    const SdfPath &rig, const std::vector<UsdTimeCode> &times)
{
    std::unique_lock<std::mutex> lock(_mutex);
    // No generation bump: unaffected times keep their tokens and their
    // running jobs publish normally. Every affected time gets a fresh
    // per-time fence token from the rig's monotonic counter instead, so
    // an old job for a requeued time drops at the publish fence on token
    // mismatch and can never overwrite the new result.
    std::map<double, bool> affected;
    for (const UsdTimeCode &time : times) {
        if (time.IsNumeric() && std::isfinite(time.GetValue())) {
            affected[time.GetValue()] = true;
        }
    }
    for (const auto &entry : affected) {
        _fenceTokens[std::make_pair(rig, entry.first)] =
            ++_fenceCounters[rig];
    }
    // Purge the rig's queued jobs at those times, same loop shape as
    // CancelGeneration: the purged closures free AFTER the lock closes.
    std::vector<RigExecWarmWork> retired;
    for (auto it = _queue.begin(); it != _queue.end();) {
        if (it->second.request.rig != rig ||
            affected.find(it->second.request.time.GetValue()) ==
                affected.end()) {
            ++it;
            continue;
        }
        _queuedByRigAndTime.erase(std::make_pair(
            rig, it->second.request.time.GetValue()));
        _ReportTransitionLocked(it->second.request,
                                RigExecWarmTransitionKind::Canceled);
        retired.push_back(std::move(it->second.work));
        it = _queue.erase(it);
        const auto inFlight = _inFlight.find(rig);
        if (inFlight != _inFlight.end() && inFlight->second > 0) {
            if (--(inFlight->second) == 0) {
                _inFlight.erase(inFlight);
            }
        }
        ++_canceled;
    }
    if (!retired.empty()) {
        _profiler.RecordSchedulerCancel(retired.size(), "edit-times");
    }
    _profiler.RecordSchedulerQueue(_queue.size(), _running, _canceled);
    lock.unlock();
    _cv.notify_all();
    // retired dies here, outside the lock.
    return retired.size();
}

RigExecWarmFenceToken
RigExecBackgroundScheduler::CurrentFenceToken(
    const SdfPath &rig, UsdTimeCode time) const
{
    if (!time.IsNumeric() || !std::isfinite(time.GetValue())) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _CurrentFenceTokenLocked(rig, time.GetValue());
}

bool
RigExecBackgroundScheduler::IsWarmRequestCurrent(
    const SdfPath &rig, RigExecFrameGeneration generation, UsdTimeCode time,
    RigExecWarmFenceToken fenceToken) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto current = _generations.find(rig);
    const RigExecFrameGeneration currentGeneration =
        current == _generations.end() ? 0 : current->second;
    if (currentGeneration != generation) {
        return false;
    }
    if (!time.IsNumeric() || !std::isfinite(time.GetValue())) {
        return fenceToken == 0;
    }
    return _CurrentFenceTokenLocked(rig, time.GetValue()) == fenceToken;
}

std::vector<UsdTimeCode>
RigExecBackgroundScheduler::QueuedTimes(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<UsdTimeCode> times;
    for (const auto &entry : _queuedByRigAndTime) {
        if (entry.first.first == rig) {
            times.emplace_back(entry.first.second);
        }
    }
    return times;
}

bool
RigExecBackgroundScheduler::IsGenerationCurrent(
    const SdfPath &rig, RigExecFrameGeneration generation) const
{
    return CurrentGeneration(rig) == generation;
}

bool
RigExecBackgroundScheduler::Enqueue(const SdfPath &rig, UsdTimeCode time,
                                    RigExecWarmPriority priority,
                                    RigExecFrameGeneration generation,
                                    RigExecWarmWork work)
{
    // Declines first, without touching the lock: the gate is the library's
    // live answer, and an unusable time or an empty closure can never become
    // a job. (A non-finite time would corrupt the coalescing index -- NaN
    // compares equivalent to everything -- so it declines like a default.)
    if (!RigExecBackgroundWarmingEnabled()) {
        std::lock_guard<std::mutex> lock(_mutex);
        ++_declined;
        return false;
    }
    if (!time.IsNumeric() || !std::isfinite(time.GetValue()) || !work) {
        std::lock_guard<std::mutex> lock(_mutex);
        ++_declined;
        return false;
    }
    const double timeValue = time.GetValue();
    const std::pair<SdfPath, double> coalesceKey =
        std::make_pair(rig, timeValue);
    std::unique_lock<std::mutex> lock(_mutex);
    if (_stopping) {
        ++_declined;
        return false;
    }
    // Inline, not via IsGenerationCurrent: the lock is already held and
    // std::mutex does not nest.
    const auto current = _generations.find(rig);
    const RigExecFrameGeneration currentGeneration =
        current == _generations.end() ? 0 : current->second;
    if (currentGeneration != generation) {
        ++_declined;
        return false;
    }
    // Coalesce: warming for this frame is already coming. First-wins on the
    // work: the queued closure stands and the new one is destroyed on
    // return, freeing its inputs promptly.
    if (_MergeIfQueuedLocked(coalesceKey, priority)) {
        return true;
    }
    const auto inFlight = _inFlight.find(rig);
    const size_t flying =
        inFlight == _inFlight.end() ? 0 : inFlight->second;
    if (flying >= _perRigCap) {
        ++_declined;
        return false;
    }
    RigExecWarmRequest request;
    request.rig = rig;
    request.time = time;
    request.priority = priority;
    request.generation = generation;
    request.fenceToken = _CurrentFenceTokenLocked(rig, timeValue);
    const _QueueKey key{int(priority), _sequence++};
    _QueuedJob queuedJob;
    queuedJob.request = request;
    queuedJob.work = std::move(work);
    _queue.emplace(key, std::move(queuedJob));
    _queuedByRigAndTime.emplace(coalesceKey, key);
    _inFlight[rig] = flying + 1;
    _ReportTransitionLocked(request, RigExecWarmTransitionKind::Queued);
    _profiler.RecordSchedulerQueue(_queue.size(), _running, _canceled);
    lock.unlock();
    _cv.notify_all();
    return true;
}

size_t
RigExecBackgroundScheduler::OnEditCommitted(
    const SdfPath &rig, UsdTimeCode playhead,
    RigExecFrameGeneration generation,
    const std::vector<UsdTimeCode> &sweepTimes,
    RigExecWarmJobFactory factory, int neighborRadius,
    RigExecWarmSamplingBudget budget)
{
    // Nothing to sample from without a playhead: no neighborhood to compute
    // and no playhead frame to protect. Fail-closed, before any factory call.
    if (!playhead.IsNumeric() || !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // The fill gate, once per burst: the per-frame path below does not
    // re-read it, so a flip mid-burst takes effect at the next trigger
    // rather than stopping this one. (Memoizing the mode across bursts
    // would need a reset hook -- tests toggle it mid-run -- while one live
    // read per burst keeps the host-flippable contract.)
    if (!RigExecBackgroundWarmingEnabled()) {
        return 0;
    }
    _BudgetLedger ledger;
    ledger.budget = &budget;
    ledger.startUs =
        budget.startUs != 0 ? budget.startUs : RigExecProfiler::NowUs();
    size_t enqueued = 0;
    if (neighborRadius > 0 && factory) {
        // Closest first: the +-1 pair, then +-2, so FIFO within Neighbor
        // serves the frame beside the playhead next.
        const double center = playhead.GetValue();
        for (int step = 1; step <= neighborRadius; ++step) {
            for (int side = 0; side < 2; ++side) {
                const double offset =
                    side == 0 ? -double(step) : double(step);
                const UsdTimeCode time(center + offset);
                // Center +- step can round back to center at extreme
                // magnitudes; the playhead is never queued, however it
                // arises.
                if (time == playhead) {
                    continue;
                }
                const _TriggerOutcome outcome = _EnqueueTriggerTime(
                    rig, time, RigExecWarmPriority::Neighbor, generation,
                    factory, &ledger);
                if (outcome == _TriggerOutcome::Stop) {
                    return enqueued;
                }
                if (outcome == _TriggerOutcome::Enqueued) {
                    ++enqueued;
                }
            }
        }
    }
    enqueued += _EnqueueSweepTimes(rig, playhead, generation, sweepTimes,
                                   factory, &ledger);
    return enqueued;
}

size_t
RigExecBackgroundScheduler::OnIdle(
    const SdfPath &rig, UsdTimeCode playhead,
    RigExecFrameGeneration generation,
    const std::vector<UsdTimeCode> &sweepTimes,
    RigExecWarmJobFactory factory, RigExecWarmSamplingBudget budget)
{
    if (!playhead.IsNumeric() || !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // Once per burst, as in OnEditCommitted: the per-frame path assumes it.
    if (!RigExecBackgroundWarmingEnabled()) {
        return 0;
    }
    _BudgetLedger ledger;
    ledger.budget = &budget;
    ledger.startUs =
        budget.startUs != 0 ? budget.startUs : RigExecProfiler::NowUs();
    return _EnqueueSweepTimes(rig, playhead, generation, sweepTimes, factory,
                              &ledger);
}

size_t
RigExecBackgroundScheduler::NotePlaybackAdvanced(const SdfPath &rig,
                                                 UsdTimeCode playhead)
{
    // NaN is numeric but orders against nothing: every `request.time >
    // playhead` test below is false, so without the finiteness check a NaN
    // playhead would shed the whole sweep.
    if (!playhead.IsNumeric() || !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    size_t shed = 0;
    for (auto it = _queue.begin(); it != _queue.end();) {
        const RigExecWarmRequest &request = it->second.request;
        if (request.rig != rig ||
            request.priority != RigExecWarmPriority::Sweep ||
            request.time > playhead) {
            ++it;
            continue;
        }
        // A sweep job at or behind the playhead: the live path serves that
        // frame now, so warming it is pointless. Neighbors stand -- the
        // frames ahead still warm the imminent scrub path -- and the standing
        // neighbor-before-sweep order deprioritizes the sweep that remains.
        _queuedByRigAndTime.erase(
            std::make_pair(rig, request.time.GetValue()));
        _ReportTransitionLocked(request, RigExecWarmTransitionKind::Shed);
        it = _queue.erase(it);
        const auto inFlight = _inFlight.find(rig);
        if (inFlight != _inFlight.end() && inFlight->second > 0) {
            if (--(inFlight->second) == 0) {
                _inFlight.erase(inFlight);
            }
        }
        ++_shed;
        ++shed;
    }
    if (shed > 0) {
        _profiler.RecordSchedulerCancel(shed, "playback");
        _cv.notify_all();
    }
    _profiler.RecordSchedulerQueue(_queue.size(), _running, _canceled);
    return shed;
}

void
RigExecBackgroundScheduler::SetPerRigJobCap(size_t cap)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _perRigCap = cap;
}

size_t
RigExecBackgroundScheduler::GetPerRigJobCap() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _perRigCap;
}

RigExecBackgroundSchedulerStats
RigExecBackgroundScheduler::Stats() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    RigExecBackgroundSchedulerStats stats;
    stats.queuedDepth = _queue.size();
    stats.running = _running;
    stats.completed = _completed;
    stats.published = _published;
    stats.declinedInvalid = _declinedInvalid;
    stats.declinedGeneration = _declinedGeneration;
    stats.droppedStale = _droppedStale;
    stats.canceled = _canceled;
    stats.shed = _shed;
    stats.coalesced = _coalesced;
    stats.upgraded = _upgraded;
    stats.declined = _declined;
    stats.declinedFreezeRefused = _declinedFreezeRefused;
    stats.declinedUnsampleable = _declinedUnsampleable;
    stats.declinedD7Exempt = _declinedD7Exempt;
    stats.droppedAtShutdown = _droppedAtShutdown;
    stats.factoryInvocations = _factoryInvocations;
    return stats;
}

void
RigExecBackgroundScheduler::WaitUntilIdle()
{
    std::unique_lock<std::mutex> lock(_mutex);
    // A stopping pool never drains its queue, so waiting on emptiness alone
    // would hang against a concurrent shutdown (or a zero-worker pool).
    _cv.wait(lock, [this] {
        return _stopping || (_queue.empty() && _running == 0);
    });
}

bool
RigExecBackgroundScheduler::RunNextQueuedForTesting()
{
    _QueuedJob job;
    if (!_PopJob(&job, /*wait=*/false)) {
        return false;
    }
    _ExecutePoppedJob(&job);
    return true;
}

size_t
RigExecBackgroundScheduler::DrainQueueForTesting()
{
    size_t ran = 0;
    _QueuedJob job;
    while (_PopJob(&job, /*wait=*/false)) {
        if (_ExecutePoppedJob(&job)) {
            ++ran;
        }
    }
    return ran;
}

void
RigExecBackgroundScheduler::_WorkerMain()
{
    RigExecSetBackgroundThreadPriority();
    _QueuedJob job;
    while (_PopJob(&job, /*wait=*/true)) {
        _ExecutePoppedJob(&job);
    }
}

bool
RigExecBackgroundScheduler::_PopJob(_QueuedJob *job, bool wait)
{
    std::unique_lock<std::mutex> lock(_mutex);
    if (wait) {
        _cv.wait(lock,
                 [this] { return _stopping || !_queue.empty(); });
        if (_stopping) {
            return false;
        }
    } else if (_stopping || _queue.empty()) {
        return false;
    }
    auto next = _queue.begin();
    *job = std::move(next->second);
    _queuedByRigAndTime.erase(std::make_pair(job->request.rig,
                                             job->request.time.GetValue()));
    _queue.erase(next);
    // Still in-flight -- the cap counts popped-and-executing -- until
    // _FinishJob lands.
    ++_running;
    _ReportTransitionLocked(job->request,
                            RigExecWarmTransitionKind::Running);
    return true;
}

bool
RigExecBackgroundScheduler::_ExecutePoppedJob(_QueuedJob *job)
{
    // The pre-run fence: a bump that landed between the pop and this check
    // -- CancelGeneration purges queued jobs, but this one already popped --
    // drops the work unrun and frees its sampled inputs promptly.
    const RigExecWarmRequest request = job->request;
    if (!IsGenerationCurrent(request.rig, request.generation)) {
        *job = _QueuedJob();
        _FinishJob(request, /*ran=*/false,
                   RigExecWarmOutcome::DeclinedInvalid);
        return false;
    }
    // The D4 boundary: the serial scope forces every kernel variant this run
    // reaches into its serial form on this thread, whatever the
    // process-wide switch says, so no work leaks back onto the shared arena
    // at normal priority. The work must also never hop threads: the scope
    // constrains this thread and no other.
    // A throwing job must not take the worker -- or the calling thread on
    // the manual drain -- down with it: fail-closed, warn, and land the
    // terminal state the books require. The frame evaluates live when asked.
    RigExecWarmOutcome outcome = RigExecWarmOutcome::DeclinedInvalid;
    try {
        RigExecFrozenSerialScope serial;
        outcome = job->work(request);
    } catch (const std::exception &error) {
        TF_WARN("rigExec: background warming job threw: %s", error.what());
    } catch (...) {
        TF_WARN("rigExec: background warming job threw an unknown exception");
    }
    *job = _QueuedJob();
    _FinishJob(request, /*ran=*/true, outcome);
    return true;
}

void
RigExecBackgroundScheduler::_FinishJob(const RigExecWarmRequest &request,
                                       bool ran,
                                       RigExecWarmOutcome outcome)
{
    std::unique_lock<std::mutex> lock(_mutex);
    if (_running > 0) {
        --_running;
    }
    const auto inFlight = _inFlight.find(request.rig);
    if (inFlight != _inFlight.end() && inFlight->second > 0) {
        if (--(inFlight->second) == 0) {
            _inFlight.erase(inFlight);
        }
    }
    if (ran) {
        ++_completed;
        switch (outcome) {
        case RigExecWarmOutcome::Published:
            ++_published;
            break;
        case RigExecWarmOutcome::DeclinedGeneration:
            ++_declinedGeneration;
            break;
        case RigExecWarmOutcome::DeclinedInvalid:
            ++_declinedInvalid;
            break;
        }
        _ReportTransitionLocked(request,
                                RigExecWarmTransitionKind::Finished, outcome);
    } else {
        ++_droppedStale;
        _ReportTransitionLocked(request,
                                RigExecWarmTransitionKind::DroppedStale);
    }
    _profiler.RecordSchedulerQueue(_queue.size(), _running, _canceled);
    lock.unlock();
    _cv.notify_all();
}

bool
RigExecBackgroundScheduler::_MergeIfQueuedLocked(
    const std::pair<SdfPath, double> &coalesceKey,
    RigExecWarmPriority priority)
{
    const auto queued = _queuedByRigAndTime.find(coalesceKey);
    if (queued == _queuedByRigAndTime.end()) {
        return false;
    }
    // A neighbor request promotes a queued sweep -- the frame matters more
    // than the trigger thought -- keeping its enqueue sequence: FIFO within
    // neighbor still serves the earliest-sampled frame next.
    const _QueueKey oldKey = queued->second;
    const auto job = _queue.find(oldKey);
    if (job != _queue.end() &&
        priority == RigExecWarmPriority::Neighbor &&
        job->second.request.priority == RigExecWarmPriority::Sweep) {
        _QueuedJob promoted = std::move(job->second);
        promoted.request.priority = RigExecWarmPriority::Neighbor;
        _queue.erase(job);
        const _QueueKey newKey{int(RigExecWarmPriority::Neighbor),
                               oldKey.sequence};
        _queue.emplace(newKey, std::move(promoted));
        queued->second = newKey;
        ++_upgraded;
    } else {
        ++_coalesced;
    }
    return true;
}

RigExecBackgroundScheduler::_TriggerOutcome
RigExecBackgroundScheduler::_EnqueueTriggerTime(
    const SdfPath &rig, UsdTimeCode time, RigExecWarmPriority priority,
    RigExecFrameGeneration generation, const RigExecWarmJobFactory &factory,
    _BudgetLedger *ledger)
{
    if (!time.IsNumeric() || !std::isfinite(time.GetValue())) {
        return _TriggerOutcome::Skipped;
    }
    // No gate check here: the triggers check once per burst, and re-reading
    // per frame cost an env lookup per warmed frame. Mid-burst staleness is
    // still decided per frame below (stopping, generation, queued set, cap).
    const double timeValue = time.GetValue();
    const std::pair<SdfPath, double> coalesceKey =
        std::make_pair(rig, timeValue);
    {
        std::lock_guard<std::mutex> lock(_mutex);
        const auto current = _generations.find(rig);
        const RigExecFrameGeneration currentGeneration =
            current == _generations.end() ? 0 : current->second;
        if (_stopping || currentGeneration != generation) {
            return _TriggerOutcome::Stop;
        }
        // Already queued: merge without sampling, since the queued closure
        // stands either way.
        if (_MergeIfQueuedLocked(coalesceKey, priority)) {
            return _TriggerOutcome::Skipped;
        }
        const auto inFlight = _inFlight.find(rig);
        const size_t flying =
            inFlight == _inFlight.end() ? 0 : inFlight->second;
        if (flying >= _perRigCap) {
            return _TriggerOutcome::Stop;
        }
    }
    // The sampling budget: already-queued merges above consumed none (no
    // factory call); this frame would sample, so it counts. The stop ends
    // the trigger (Stop), not the frame: the budget is per-trigger.
    if (ledger->invocations >= ledger->budget->maxFactoryInvocations) {
        return _TriggerOutcome::Stop;
    }
    if (ledger->budget->maxMs < std::numeric_limits<double>::infinity() &&
        double(RigExecProfiler::NowUs() - ledger->startUs) / 1000.0 >=
            ledger->budget->maxMs) {
        return _TriggerOutcome::Stop;
    }
    // Sampling runs on the calling (UI) thread, outside the lock: it reads
    // attribute queries and resolved inputs, and the lock never spans it.
    // An empty factory -- or an empty build -- skips the frame, and the
    // trigger continues with the next.
    RigExecWarmFactoryResult built;
    if (factory) {
        RigExecProfileScope scope(MutableProfiler(),
                                  "Scheduler.WarmFactorySample", "scheduler");
        built = factory(time);
        ++ledger->invocations;
        std::lock_guard<std::mutex> lock(_mutex);
        ++_factoryInvocations;
    }
    if (!built.work) {
        // A reasoned skip counts declined (it never entered the queue)
        // plus its per-reason breakout; an unreasoned one skips silently,
        // as before.
        if (built.skip != RigExecWarmSkipReason::None) {
            std::lock_guard<std::mutex> lock(_mutex);
            ++_declined;
            switch (built.skip) {
            case RigExecWarmSkipReason::FreezeRefused:
                ++_declinedFreezeRefused;
                break;
            case RigExecWarmSkipReason::Unsampleable:
                ++_declinedUnsampleable;
                break;
            case RigExecWarmSkipReason::D7Exempt:
                ++_declinedD7Exempt;
                break;
            case RigExecWarmSkipReason::None:
                break;
            }
        }
        return _TriggerOutcome::Skipped;
    }
    RigExecWarmWork work = std::move(built.work);
    std::unique_lock<std::mutex> lock(_mutex);
    // Recheck under the lock: the factory above runs arbitrary caller code,
    // and an edit may have landed mid-sample -- the generation, the queued
    // set, and the cap are all re-decided, never trusted across the call.
    // (A worker racing this trigger can only have popped jobs -- never
    // queued -- so a miss here still inserts honestly beside a running twin.)
    const auto current = _generations.find(rig);
    const RigExecFrameGeneration currentGeneration =
        current == _generations.end() ? 0 : current->second;
    if (_stopping || currentGeneration != generation) {
        return _TriggerOutcome::Stop;
    }
    if (_MergeIfQueuedLocked(coalesceKey, priority)) {
        return _TriggerOutcome::Skipped;
    }
    const auto inFlight = _inFlight.find(rig);
    const size_t flying =
        inFlight == _inFlight.end() ? 0 : inFlight->second;
    if (flying >= _perRigCap) {
        return _TriggerOutcome::Stop;
    }
    RigExecWarmRequest request;
    request.rig = rig;
    request.time = time;
    request.priority = priority;
    request.generation = generation;
    request.fenceToken = _CurrentFenceTokenLocked(rig, timeValue);
    const _QueueKey key{int(priority), _sequence++};
    _QueuedJob queuedJob;
    queuedJob.request = request;
    queuedJob.work = std::move(work);
    _queue.emplace(key, std::move(queuedJob));
    _queuedByRigAndTime.emplace(coalesceKey, key);
    _inFlight[rig] = flying + 1;
    _ReportTransitionLocked(request, RigExecWarmTransitionKind::Queued);
    _profiler.RecordSchedulerQueue(_queue.size(), _running, _canceled);
    lock.unlock();
    _cv.notify_all();
    return _TriggerOutcome::Enqueued;
}

size_t
RigExecBackgroundScheduler::_EnqueueSweepTimes(
    const SdfPath &rig, UsdTimeCode playhead,
    RigExecFrameGeneration generation,
    const std::vector<UsdTimeCode> &sweepTimes,
    const RigExecWarmJobFactory &factory, _BudgetLedger *ledger)
{
    if (!factory) {
        return 0;
    }
    size_t enqueued = 0;
    for (const UsdTimeCode &time : sweepTimes) {
        // The playhead is never queued, wherever the range puts it.
        if (time == playhead) {
            continue;
        }
        const _TriggerOutcome outcome = _EnqueueTriggerTime(
            rig, time, RigExecWarmPriority::Sweep, generation, factory,
            ledger);
        if (outcome == _TriggerOutcome::Stop) {
            return enqueued;
        }
        if (outcome == _TriggerOutcome::Enqueued) {
            ++enqueued;
        }
    }
    return enqueued;
}

}  // namespace rigExec