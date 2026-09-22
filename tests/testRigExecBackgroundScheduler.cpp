//
// RigExecBackgroundScheduler, Stream C: the frame-job queue behind the
// generation fence.
//
// The fence tests from the Stream 0 skeleton stand unchanged -- per-rig
// tokens, monotonic bumps, independence, staleness -- and the stub's "a
// current job declines" test is replaced by the pool's own contract, one rule
// per test in the testRigExecStaticInputCache style:
//
//   * FENCE. Tokens, bumps, independence, stale refusal (kept from Stream 0).
//   * SWITCH. RIGEXEC_FRAME_CACHE parses to on/off/warm-off, the fill gate is
//     the mode AND parallel-eval, and a closed gate declines warming while the
//     fence keeps working.
//   * ORDER. Neighbors before sweep, FIFO within a priority, commit neighbors
//     closest-first -- all drained on the calling thread, so the sequences
//     are exact.
//   * COALESCE. Duplicate times merge (sweep promotes to neighbor), factory
//     sampling is skipped for queued times, merges never consume the cap.
//   * CAP. The per-rig in-flight cap declines past its limit, frees as jobs
//     run or purge, and never crosses rigs.
//   * CANCEL. CancelGeneration purges queued jobs promptly; a bump inside a
//     trigger stops it; running jobs keep their publish fence (Stream B).
//   * TRIGGER. OnEditCommitted enqueues neighbors then sweep and never the
//     playhead; OnIdle enqueues sweep only; both stop at staleness and cap.
//   * SHED. NotePlaybackAdvanced purges passed sweep jobs, keeps neighbors
//     and ahead frames, and frees the cap.
//   * POOL. Workers run every job exactly once under the frozen serial scope,
//     shutdown joins running jobs and drops pending ones, and the priority
//     backend reports honestly. Waits are event-driven (queue-empty, started,
//     released) -- no sleeps, no timing assertions, no wall clock.
//   * OUTCOME. Ran jobs split completed by disposition (published,
//     declined-invalid, declined-generation); threw lands
//     declined-invalid; completed still equals the three.
//   * HOOK. The transition hook reports every per-frame queue move --
//     queued, running, finished, canceled, shed, dropped-at-shutdown --
//     in order, on the manual drain.
//   * FAIRNESS. One worker preserves the pop order exactly (neighbors
//     closest-first, then sweep); under a two-worker flood both rigs
//     still complete every job and the books balance.
//   * BUDGET. The sampling budget bounds factory invocations per trigger;
//     the millisecond stop binds sampling; already-queued merges consume
//     nothing; the unlimited default preserves the full burst shape.
//
// Two process modes: most tests force RIGEXEC_FRAME_CACHE=on (live-read) and
// run the warm path, skipping with a note when the ambient
// RIGEXEC_ENABLE_PARALLEL_EVAL=0 closes the gate -- that cached switch cannot
// be flipped in-process, so its off mode is covered by the fallback test plus
// the validation-plan runs of this binary under =0. The fallback test forces
// the gate closed and passes in every ambient mode.
//
#include "rigExec/backgroundScheduler.h"
#include "rigExec/frozenContext.h"
#include "rigExec/parallel.h"

#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/setenv.h"

#include <algorithm>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

// Sets RIGEXEC_FRAME_CACHE for the test's duration, restoring the ambient
// value -- or unsetting when it was unset -- on destruction.
class ScopedFrameCacheMode {
public:
    explicit ScopedFrameCacheMode(const char *value)
    {
        // Presence, not value: an unset variable restores to unset, and a
        // set-but-empty one restores to empty. (_dupenv_s on MSVC, where
        // getenv is deprecated.)
#if defined(_WIN32)
        char *ambient = nullptr;
        size_t length = 0;
        _had = (_dupenv_s(&ambient, &length, "RIGEXEC_FRAME_CACHE") == 0 &&
                ambient != nullptr);
        if (_had) {
            _saved = ambient;
            free(ambient);
        }
#else
        const char *ambient = std::getenv("RIGEXEC_FRAME_CACHE");
        _had = (ambient != nullptr);
        if (_had) {
            _saved = ambient;
        }
#endif
        TfSetenv("RIGEXEC_FRAME_CACHE", value);
    }
    ~ScopedFrameCacheMode()
    {
        if (_had) {
            TfSetenv("RIGEXEC_FRAME_CACHE", _saved);
        } else {
            TfUnsetenv("RIGEXEC_FRAME_CACHE");
        }
    }

private:
    bool _had = false;
    std::string _saved;
};

// One executed job, as the recording work observed it.
struct RunRecord {
    SdfPath rig;
    double time = 0.0;
    RigExecWarmPriority priority = RigExecWarmPriority::Sweep;
    RigExecFrameGeneration generation = 0;
    bool serialActive = false;
};

// Builds recording work and counting factories. Thread-safe: workers record
// while the test asserts afterwards on its own thread.
class Recorder {
public:
    RigExecWarmWork Work()
    {
        return [this](const RigExecWarmRequest &request) {
            RunRecord record;
            record.rig = request.rig;
            record.time = request.time.GetValue();
            record.priority = request.priority;
            record.generation = request.generation;
            record.serialActive = RigExecFrozenSerialActive();
            std::lock_guard<std::mutex> lock(_mutex);
            _runs.push_back(record);
            return RigExecWarmOutcome::Published;
        };
    }

    // A factory returning recording work and counting its own invocations.
    // Invocations happen on the triggering thread only, so the count needs
    // no lock -- but the member lives here for convenience.
    RigExecWarmJobFactory Factory()
    {
        return [this](UsdTimeCode time) -> RigExecWarmWork {
            ++_factoryCalls;
            (void)time;
            return Work();
        };
    }

    std::vector<RunRecord> Runs() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _runs;
    }

    size_t FactoryCalls() const { return _factoryCalls; }

private:
    mutable std::mutex _mutex;
    std::vector<RunRecord> _runs;
    size_t _factoryCalls = 0;
};

// The times of \p runs, in run order.
std::vector<double>
RunTimes(const std::vector<RunRecord> &runs)
{
    std::vector<double> times;
    for (const RunRecord &run : runs) {
        times.push_back(run.time);
    }
    return times;
}

bool
TimesEqual(const std::vector<double> &a, const std::vector<double> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// An unseen rig sits at generation zero: "no edit yet" is a token like any
// other, and the first edit must move it.
void
TestAnUnseenRigSitsAtGenerationZero()
{
    RigExecBackgroundScheduler scheduler;
    CHECK(scheduler.CurrentGeneration(SdfPath("/Asset/Rig")) == 0);
    CHECK(scheduler.IsGenerationCurrent(SdfPath("/Asset/Rig"), 0));
    CHECK(!scheduler.IsGenerationCurrent(SdfPath("/Asset/Rig"), 1));
}

// CancelGeneration bumps monotonically: every edit retires every job sampled
// before it, and no token is ever reused.
void
TestCancelBumpsMonotonically()
{
    RigExecBackgroundScheduler scheduler;
    const SdfPath rig("/Asset/Rig");
    CHECK(scheduler.CurrentGeneration(rig) == 0);
    scheduler.CancelGeneration(rig);
    CHECK(scheduler.CurrentGeneration(rig) == 1);
    scheduler.CancelGeneration(rig);
    CHECK(scheduler.CurrentGeneration(rig) == 2);
    CHECK(scheduler.IsGenerationCurrent(rig, 2));
    CHECK(!scheduler.IsGenerationCurrent(rig, 1));
    CHECK(!scheduler.IsGenerationCurrent(rig, 0));
}

// Generations are per rig: an edit to one rig cancels nothing of another's.
void
TestRigsAreIndependent()
{
    RigExecBackgroundScheduler scheduler;
    const SdfPath first("/Asset/RigA");
    const SdfPath second("/Asset/RigB");
    scheduler.CancelGeneration(first);
    CHECK(scheduler.CurrentGeneration(first) == 1);
    CHECK(scheduler.CurrentGeneration(second) == 0);
    CHECK(scheduler.IsGenerationCurrent(second, 0));
}

// A job enqueued under a superseded token is refused for staleness: the edit
// it was sampled for is gone, and warming the old inputs would publish a
// pose no frame asks for anymore.
void
TestAStaleJobIsRefused()
{
    ScopedFrameCacheMode on("on");
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration sampled = scheduler.CurrentGeneration(rig);
    scheduler.CancelGeneration(rig);
    CHECK(!scheduler.IsGenerationCurrent(rig, sampled));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                             RigExecWarmPriority::Neighbor, sampled,
                             recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                             RigExecWarmPriority::Sweep, sampled,
                             recorder.Work()));
    CHECK(scheduler.Stats().declined == 2);
    CHECK(scheduler.Stats().queuedDepth == 0);
}

// The switch parses to its three modes; anything else is the default.
void
TestFrameCacheModeParses()
{
    CHECK(RigExecParseFrameCacheMode("on") == RigExecFrameCacheMode::On);
    CHECK(RigExecParseFrameCacheMode("off") == RigExecFrameCacheMode::Off);
    CHECK(RigExecParseFrameCacheMode("warm-off") ==
          RigExecFrameCacheMode::WarmOff);
    CHECK(RigExecParseFrameCacheMode("") == RigExecFrameCacheMode::On);
    CHECK(RigExecParseFrameCacheMode("ON") == RigExecFrameCacheMode::On);
    CHECK(RigExecParseFrameCacheMode("bogus") == RigExecFrameCacheMode::On);
}

// The fill gate is the mode AND parallel-eval: warm-off and parallel-off both
// mean "reads served, fill off", and only on+enabled warms.
void
TestWarmingGateTruthTable()
{
    CHECK(RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::On, true));
    CHECK(!RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::On, false));
    CHECK(!RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::Off, true));
    CHECK(!RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::Off, false));
    CHECK(!RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::WarmOff,
                                           true));
    CHECK(!RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::WarmOff,
                                           false));
}

// The mode reads live from the environment: a host flips warming without
// restarting. (The bogus value warns once; the parse still answers On.)
void
TestFrameCacheModeReadsLiveFromTheEnvironment()
{
    {
        ScopedFrameCacheMode on("on");
        CHECK(RigExecFrameCacheModeFromEnvironment() ==
              RigExecFrameCacheMode::On);
    }
    {
        ScopedFrameCacheMode off("off");
        CHECK(RigExecFrameCacheModeFromEnvironment() ==
              RigExecFrameCacheMode::Off);
    }
    {
        ScopedFrameCacheMode warmOff("warm-off");
        CHECK(RigExecFrameCacheModeFromEnvironment() ==
              RigExecFrameCacheMode::WarmOff);
    }
    {
        ScopedFrameCacheMode bogus("bogus");
        CHECK(RigExecFrameCacheModeFromEnvironment() ==
              RigExecFrameCacheMode::On);
    }
    // The environment-ambient gate agrees with the pure gate fed the live
    // mode and the library's cached parallel switch.
    ScopedFrameCacheMode on("on");
    CHECK(RigExecBackgroundWarmingEnabled() ==
          RigExecBackgroundWarmingEnabled(
              RigExecFrameCacheModeFromEnvironment(),
              RigExecParallelEvaluationEnabled()));
}

// A closed gate declines warming while the fence keeps working: every
// Enqueue refuses, every trigger enqueues nothing without sampling, and edits
// still bump. Passes in every ambient mode -- it forces the gate closed.
void
TestClosedGateDeclinesWarmingButKeepsTheFence()
{
    for (const char *mode : {"off", "warm-off"}) {
        ScopedFrameCacheMode closed(mode);
        CHECK(!RigExecBackgroundWarmingEnabled());
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                 RigExecWarmPriority::Neighbor, current,
                                 recorder.Work()));
        CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                 RigExecWarmPriority::Sweep, current,
                                 recorder.Work()));
        const std::vector<UsdTimeCode> sweep{UsdTimeCode(4.0)};
        CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current,
                                        sweep, recorder.Factory()) == 0);
        CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                               recorder.Factory()) == 0);
        CHECK(recorder.FactoryCalls() == 0);
        CHECK(scheduler.Stats().declined == 2);
        CHECK(scheduler.Stats().queuedDepth == 0);
        // The fence is unaffected by the gate: edits still bump.
        scheduler.CancelGeneration(rig);
        CHECK(scheduler.CurrentGeneration(rig) == current + 1);
        CHECK(!scheduler.IsGenerationCurrent(rig, current));
    }
}

// A current job is accepted and runs: the ordering is neighbors before sweep
// whatever order they enqueue in.
void
TestACurrentJobIsAcceptedNeighborBeforeSweep()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestACurrentJobIsAcceptedNeighborBeforeSweep: "
                    "warming unavailable in this process\n");
        return;
    }
    CHECK(int(RigExecWarmPriority::Neighbor) <
          int(RigExecWarmPriority::Sweep));
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    // Enqueued backwards on purpose: the sweep must still run second.
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    CHECK(scheduler.Stats().queuedDepth == 2);
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(!scheduler.RunNextQueuedForTesting());
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(runs.size() == 2);
    CHECK(runs[0].time == 2.0);
    CHECK(runs[0].priority == RigExecWarmPriority::Neighbor);
    CHECK(runs[1].time == 1.0);
    CHECK(runs[1].priority == RigExecWarmPriority::Sweep);
    CHECK(scheduler.Stats().completed == 2);
    CHECK(scheduler.Stats().queuedDepth == 0);
    // The accepted requests moved no token.
    CHECK(scheduler.CurrentGeneration(rig) == current);
}

// Within one priority the queue is FIFO: enqueue order, not time order.
void
TestFifoWithinAPriority()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestFifoWithinAPriority: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.DrainQueueForTesting() == 3);
    CHECK(TimesEqual(RunTimes(recorder.Runs()), {3.0, 1.0, 2.0}));
}

// A duplicate time coalesces: the second Enqueue accepts -- warming is coming
// -- without queueing twice, and the job runs once.
void
TestDuplicateTimesCoalesce()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestDuplicateTimesCoalesce: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Stats().queuedDepth == 1);
    CHECK(scheduler.Stats().coalesced == 1);
    CHECK(scheduler.Stats().upgraded == 0);
    CHECK(scheduler.DrainQueueForTesting() == 1);
    CHECK(recorder.Runs().size() == 1);
}

// A neighbor request for a queued sweep time promotes it: the frame matters
// more than the trigger thought. Re-enqueueing a sweep for a queued neighbor
// changes nothing.
void
TestSweepPromotesToNeighborOnReenqueue()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestSweepPromotesToNeighborOnReenqueue: "
                    "warming unavailable in this process\n");
        return;
    }
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                RigExecWarmPriority::Sweep, current,
                                recorder.Work()));
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                RigExecWarmPriority::Neighbor, current,
                                recorder.Work()));
        CHECK(scheduler.Stats().queuedDepth == 1);
        CHECK(scheduler.Stats().upgraded == 1);
        CHECK(scheduler.Stats().coalesced == 0);
        CHECK(scheduler.DrainQueueForTesting() == 1);
        CHECK(recorder.Runs().size() == 1);
        CHECK(recorder.Runs()[0].priority == RigExecWarmPriority::Neighbor);
    }
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(4.0),
                                RigExecWarmPriority::Neighbor, current,
                                recorder.Work()));
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(4.0),
                                RigExecWarmPriority::Sweep, current,
                                recorder.Work()));
        CHECK(scheduler.Stats().coalesced == 1);
        CHECK(scheduler.Stats().upgraded == 0);
        CHECK(scheduler.DrainQueueForTesting() == 1);
        CHECK(recorder.Runs()[0].priority == RigExecWarmPriority::Neighbor);
    }
}

// A default, non-finite, or work-less Enqueue declines: none of them can ever
// become a job.
void
TestUnusableEnqueuesDecline()
{
    ScopedFrameCacheMode on("on");
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode::Default(),
                             RigExecWarmPriority::Neighbor, current,
                             recorder.Work()));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(nan),
                             RigExecWarmPriority::Sweep, current,
                             recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(inf),
                             RigExecWarmPriority::Sweep, current,
                             recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                             RigExecWarmPriority::Neighbor, current,
                             RigExecWarmWork()));
    CHECK(scheduler.Stats().declined == 4);
    CHECK(scheduler.Stats().queuedDepth == 0);
}

// The per-rig in-flight cap declines past its limit, frees as jobs run, and
// never crosses rigs. A coalesced duplicate does not consume the cap.
void
TestPerRigCapDeclinesPastItsLimit()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPerRigCapDeclinesPastItsLimit: "
                    "warming unavailable in this process\n");
        return;
    }
    CHECK(RigExecBackgroundScheduler(/*workerCount=*/0).GetPerRigJobCap() ==
          kRigExecBackgroundSchedulerDefaultPerRigJobs);
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    scheduler.SetPerRigJobCap(2);
    CHECK(scheduler.GetPerRigJobCap() == 2);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const SdfPath other("/Asset/Other");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                             RigExecWarmPriority::Sweep, current,
                             recorder.Work()));
    // A duplicate of a queued time merges instead of declining: it asks for
    // no new warming.
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Stats().coalesced == 1);
    // Another rig has its own budget.
    const RigExecFrameGeneration otherCurrent =
        scheduler.CurrentGeneration(other);
    CHECK(scheduler.Enqueue(other, UsdTimeCode(1.0),
                            RigExecWarmPriority::Sweep, otherCurrent,
                            recorder.Work()));
    // Running one frees one.
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Stats().declined == 1);
}

// CancelGeneration purges the rig's queued jobs -- inputs freed, cap space
// back -- and touches no other rig. Fresh jobs enqueue under the new token.
void
TestCancelPurgesQueuedJobsPromptly()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestCancelPurgesQueuedJobsPromptly: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    scheduler.SetPerRigJobCap(1);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const SdfPath other("/Asset/Other");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const RigExecFrameGeneration otherCurrent =
        scheduler.CurrentGeneration(other);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(other, UsdTimeCode(1.0),
                            RigExecWarmPriority::Sweep, otherCurrent,
                            recorder.Work()));
    scheduler.CancelGeneration(rig);
    CHECK(scheduler.Stats().canceled == 1);
    CHECK(scheduler.Stats().queuedDepth == 1);
    // The other rig's job survived the purge.
    CHECK(scheduler.DrainQueueForTesting() == 1);
    CHECK(recorder.Runs().size() == 1);
    CHECK(recorder.Runs()[0].rig == other);
    // The purge freed the cap: a fresh job under the new token enqueues.
    const RigExecFrameGeneration fresh = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Sweep, fresh,
                            recorder.Work()));
    CHECK(scheduler.DrainQueueForTesting() == 1);
    CHECK(recorder.Runs().size() == 2);
    CHECK(scheduler.Stats().completed == 2);
}

// The commit trigger enqueues neighbors closest-first, then the sweep in
// order, and never the playhead -- even when the sweep names it.
void
TestCommitTriggerEnqueuesNeighborsThenSweepNeverPlayhead()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestCommitTriggerEnqueuesNeighborsThenSweepNeverPlayhead: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(10.0), UsdTimeCode(5.0),
                                         UsdTimeCode(6.0)};
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                    recorder.Factory(),
                                    /*neighborRadius=*/2) == 6);
    CHECK(recorder.FactoryCalls() == 6);
    CHECK(scheduler.DrainQueueForTesting() == 6);
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(TimesEqual(RunTimes(runs), {9.0, 11.0, 8.0, 12.0, 5.0, 6.0}));
    for (size_t i = 0; i < 4; ++i) {
        CHECK(runs[i].priority == RigExecWarmPriority::Neighbor);
    }
    for (size_t i = 4; i < 6; ++i) {
        CHECK(runs[i].priority == RigExecWarmPriority::Sweep);
    }
    for (const RunRecord &run : runs) {
        CHECK(run.time != 10.0);
    }
}

// The trigger skips default sweep entries without sampling them, merges
// duplicates without sampling them twice, and stops at the cap.
void
TestTriggerSkipsUnsampleableAndStopsAtCap()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestTriggerSkipsUnsampleableAndStopsAtCap: "
                    "warming unavailable in this process\n");
        return;
    }
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        const std::vector<UsdTimeCode> sweep{
            UsdTimeCode::Default(), UsdTimeCode(5.0), UsdTimeCode(5.0)};
        CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current,
                                        sweep, recorder.Factory(),
                                        /*neighborRadius=*/0) == 1);
        CHECK(recorder.FactoryCalls() == 1);
        CHECK(scheduler.Stats().coalesced == 1);
        CHECK(scheduler.DrainQueueForTesting() == 1);
    }
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        scheduler.SetPerRigJobCap(2);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        const std::vector<UsdTimeCode> sweep{UsdTimeCode(5.0)};
        CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current,
                                        sweep, recorder.Factory(),
                                        /*neighborRadius=*/8) == 2);
        CHECK(recorder.FactoryCalls() == 2);
        CHECK(scheduler.DrainQueueForTesting() == 2);
        CHECK(TimesEqual(RunTimes(recorder.Runs()), {9.0, 11.0}));
    }
    // A null factory builds nothing and crashes nothing.
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        const std::vector<UsdTimeCode> sweep{UsdTimeCode(5.0)};
        CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current,
                                        sweep, RigExecWarmJobFactory(),
                                        /*neighborRadius=*/2) == 0);
        CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                               RigExecWarmJobFactory()) == 0);
        CHECK(scheduler.Stats().queuedDepth == 0);
    }
}

// A stale generation, or a default playhead, enqueues nothing and never calls
// the factory: there is nothing faithful to sample.
void
TestTriggerWithStaleGenerationOrDefaultPlayheadEnqueuesNothing()
{
    ScopedFrameCacheMode on("on");
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration sampled = scheduler.CurrentGeneration(rig);
    scheduler.CancelGeneration(rig);
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(5.0)};
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), sampled, sweep,
                                    recorder.Factory()) == 0);
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), sampled, sweep,
                           recorder.Factory()) == 0);
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode::Default(), current,
                                    sweep, recorder.Factory()) == 0);
    CHECK(scheduler.OnIdle(rig, UsdTimeCode::Default(), current, sweep,
                           recorder.Factory()) == 0);
    CHECK(recorder.FactoryCalls() == 0);
    CHECK(scheduler.Stats().queuedDepth == 0);
}

// An edit landing mid-trigger stops it: the trigger rechecks the generation
// around every sample, so the bump inside the factory ends the commit and the
// bump's own purge retires what the trigger had queued. Deterministic -- the
// factory itself bumps -- so no wall clock and no sleep.
void
TestTriggerStopsWhenAnEditLandsMidTrigger()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestTriggerStopsWhenAnEditLandsMidTrigger: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    size_t calls = 0;
    RigExecWarmJobFactory bumping =
        [&](UsdTimeCode time) -> RigExecWarmFactoryResult {
        ++calls;
        if (calls == 2) {
            scheduler.CancelGeneration(rig);
        }
        return recorder.Factory()(time);
    };
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(5.0), UsdTimeCode(6.0)};
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                    bumping,
                                    /*neighborRadius=*/2) == 1);
    CHECK(calls == 2);
    // The bump purged the one job the trigger had queued.
    CHECK(scheduler.Stats().canceled == 1);
    CHECK(scheduler.Stats().queuedDepth == 0);
    CHECK(scheduler.DrainQueueForTesting() == 0);
    CHECK(recorder.Runs().empty());
}

// The idle trigger enqueues sweep only -- neighbors belong to the commit --
// and skips the playhead like every trigger.
void
TestIdleTriggerEnqueuesSweepOnly()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestIdleTriggerEnqueuesSweepOnly: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(10.0), UsdTimeCode(5.0),
                                         UsdTimeCode(6.0)};
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory()) == 2);
    CHECK(recorder.FactoryCalls() == 2);
    CHECK(scheduler.DrainQueueForTesting() == 2);
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(TimesEqual(RunTimes(runs), {5.0, 6.0}));
    for (const RunRecord &run : runs) {
        CHECK(run.priority == RigExecWarmPriority::Sweep);
        CHECK(run.time != 10.0);
    }
}

// Playback advance sheds queued sweep at or behind the playhead -- the live
// path serves those frames now -- keeps neighbors and ahead frames, and frees
// the cap for what is still worth warming.
void
TestPlaybackAdvanceShedsPassedSweep()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPlaybackAdvanceShedsPassedSweep: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    scheduler.SetPerRigJobCap(4);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(8.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(9.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(10.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(11.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(10.0)) == 2);
    CHECK(scheduler.Stats().shed == 2);
    CHECK(scheduler.Stats().queuedDepth == 2);
    // The shed freed the cap: an ahead frame enqueues.
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(12.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.DrainQueueForTesting() == 3);
    // Neighbor first, then the ahead sweep in FIFO order.
    CHECK(TimesEqual(RunTimes(recorder.Runs()), {8.0, 11.0, 12.0}));
    CHECK(recorder.Runs()[0].priority == RigExecWarmPriority::Neighbor);
    // Nothing to shed: default playhead, unknown rig, empty queue.
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode::Default()) == 0);
    CHECK(scheduler.NotePlaybackAdvanced(SdfPath("/Asset/Other"),
                                         UsdTimeCode(10.0)) == 0);
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(10.0)) == 0);
    CHECK(scheduler.Stats().shed == 2);
}

// A NaN playhead sheds nothing: NaN is numeric but orders against nothing,
// so every at-or-behind test is false and -- without the finiteness guard --
// the call would purge the whole sweep.
void
TestPlaybackAdvanceWithNaNShedsNothing()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPlaybackAdvanceWithNaNShedsNothing: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(9.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(10.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(nan)) == 0);
    CHECK(scheduler.Stats().shed == 0);
    CHECK(scheduler.Stats().queuedDepth == 2);
    // The queue is intact: a real advance still sheds what it should.
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(10.0)) == 2);
    CHECK(scheduler.Stats().queuedDepth == 0);
}

// The scheduler lane records in production: queue-depth samples on every
// queue mutation and purge points with causes -- off by default, exact when
// enabled.
void
TestProfilerLaneRecordsQueueAndCancels()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestProfilerLaneRecordsQueueAndCancels: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(9.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(10.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    // Disabled by default: queue mutations record nothing.
    CHECK(scheduler.GetProfiler().GetEventCount() == 0);

    scheduler.MutableProfiler()->SetEnabled(true);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(8.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    scheduler.CancelGeneration(rig);
    const RigExecFrameGeneration next = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(11.0),
                            RigExecWarmPriority::Sweep, next,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(12.0),
                            RigExecWarmPriority::Sweep, next,
                            recorder.Work()));
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(12.0)) == 2);
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(13.0)) == 0);

    std::vector<std::string> names;
    std::vector<std::map<std::string, double>> queues;
    std::vector<std::pair<std::string, std::string>> cancels;
    for (const RigExecProfileEvent &event :
         scheduler.GetProfiler().GetEvents()) {
        names.push_back(event.name);
        if (event.kind == RigExecProfileEventKind::Counter &&
            event.name == "warmQueue") {
            queues.push_back(event.counters);
        }
        if (event.kind == RigExecProfileEventKind::Instant &&
            event.name == "warmCancel") {
            cancels.push_back({event.args.at("purged"),
                               event.args.at("cause")});
        }
    }
    const std::vector<std::string> expectedNames(
        {"warmQueue", "warmCancel", "warmQueue", "warmQueue", "warmQueue",
         "warmCancel", "warmQueue", "warmQueue"});
    CHECK(names == expectedNames);
    CHECK(queues.size() == 6);
    const double depths[6] = {3, 0, 1, 2, 0, 0};
    const double canceled[6] = {0, 3, 3, 3, 3, 3};
    for (size_t i = 0; i < queues.size(); ++i) {
        CHECK(queues[i].at("queuedDepth") == depths[i]);
        CHECK(queues[i].at("running") == 0);
        CHECK(queues[i].at("canceled") == canceled[i]);
    }
    const std::vector<std::pair<std::string, std::string>> expectedCancels(
        {{"3", "edit"}, {"2", "playback"}});
    CHECK(cancels == expectedCancels);
}

// The counters balance exactly over a mixed scenario: every job that entered
// the queue ends completed, canceled, or shed, and merges never enter it.
void
TestStatsBalanceOverAMixedScenario()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestStatsBalanceOverAMixedScenario: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(recorder.Runs().size() == 1);
    CHECK(recorder.Runs()[0].time == 2.0);
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(1.5)) == 1);
    scheduler.CancelGeneration(rig);
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.completed == 1);
    CHECK(stats.coalesced == 1);
    CHECK(stats.upgraded == 1);
    CHECK(stats.shed == 1);
    CHECK(stats.canceled == 1);
    CHECK(stats.droppedStale == 0);
    CHECK(stats.declined == 0);
    CHECK(stats.queuedDepth == 0);
    CHECK(stats.running == 0);
    // The one ran job published; completed still equals the split.
    CHECK(stats.published == 1);
    CHECK(stats.declinedInvalid == 0);
    CHECK(stats.declinedGeneration == 0);
    CHECK(stats.completed ==
          stats.published + stats.declinedInvalid + stats.declinedGeneration);
    // Three jobs entered the queue (five accepts minus two merges); all three
    // reached a terminal state, under the split equation.
    CHECK(stats.published + stats.declinedInvalid + stats.declinedGeneration +
              stats.canceled + stats.shed + stats.droppedStale +
              stats.droppedAtShutdown + stats.queuedDepth + stats.running ==
          3);
}

// The pool runs every job exactly once: enqueue eight, wait for idle, and all
// eight ran. The wait is event-driven -- queue empty and none running -- not
// timed.
void
TestPoolRunsEveryJobExactlyOnce()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPoolRunsEveryJobExactlyOnce: "
                    "warming unavailable in this process\n");
        return;
    }
    CHECK(!RigExecFrozenSerialActive());
    RigExecBackgroundScheduler scheduler;
    CHECK(scheduler.GetWorkerCount() ==
          kRigExecBackgroundSchedulerDefaultWorkers);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    for (int i = 0; i < 8; ++i) {
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(double(i)),
                                i % 2 == 0 ? RigExecWarmPriority::Neighbor
                                           : RigExecWarmPriority::Sweep,
                                current, recorder.Work()));
    }
    scheduler.WaitUntilIdle();
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(runs.size() == 8);
    CHECK(scheduler.Stats().completed == 8);
    CHECK(scheduler.Stats().queuedDepth == 0);
    CHECK(scheduler.Stats().running == 0);
    // Every enqueued time ran exactly once (order across two workers is
    // unscheduled, so the comparison is a set, not a sequence).
    std::vector<double> times = RunTimes(runs);
    std::sort(times.begin(), times.end());
    CHECK(TimesEqual(times, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}));
    // Every job ran under the frozen serial scope: the per-job serial
    // executor boundary, whatever the process-wide switch says. The scope is
    // thread-local to the workers -- this thread never entered one.
    for (const RunRecord &run : runs) {
        CHECK(run.serialActive);
    }
    CHECK(!RigExecFrozenSerialActive());
    // Idleness composes: a second wait on an idle pool returns.
    scheduler.WaitUntilIdle();
}

// A cancel racing the pool still balances the books: every job lands in
// exactly one terminal state, whatever the interleaving.
void
TestThreadedCancelBalancesTheBooks()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestThreadedCancelBalancesTheBooks: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler;
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    size_t accepted = 0;
    for (int i = 0; i < 32; ++i) {
        accepted += scheduler.Enqueue(rig, UsdTimeCode(double(i)),
                                      RigExecWarmPriority::Sweep, current,
                                      recorder.Work())
                        ? 1
                        : 0;
    }
    scheduler.CancelGeneration(rig);
    scheduler.WaitUntilIdle();
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.queuedDepth == 0);
    CHECK(stats.running == 0);
    CHECK(stats.completed + stats.droppedStale + stats.canceled == accepted);
    CHECK(recorder.Runs().size() == stats.completed);
}

// Destruction with pending jobs drops them without running them: the frames
// evaluate live when asked.
void
TestShutdownDropsPendingJobs()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestShutdownDropsPendingJobs: "
                    "warming unavailable in this process\n");
        return;
    }
    Recorder recorder;
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                                RigExecWarmPriority::Sweep, current,
                                recorder.Work()));
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                                RigExecWarmPriority::Neighbor, current,
                                recorder.Work()));
        CHECK(scheduler.Stats().queuedDepth == 2);
    }
    CHECK(recorder.Runs().empty());
}

// Destruction with a job running joins it: the release arrives on a second
// thread once the job starts, so both the worker and the joining destructor
// move on events, and the running job finishes instead of stranding the join.
void
TestShutdownJoinsARunningJob()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestShutdownJoinsARunningJob: "
                    "warming unavailable in this process\n");
        return;
    }
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool started = false;
    bool released = false;
    size_t ran = 0;
    {
        RigExecBackgroundScheduler scheduler;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        RigExecWarmWork blocking = [&](const RigExecWarmRequest &) {
            {
                std::lock_guard<std::mutex> lock(gateMutex);
                started = true;
            }
            gateCv.notify_all();
            std::unique_lock<std::mutex> lock(gateMutex);
            gateCv.wait(lock, [&] { return released; });
            ++ran;
            return RigExecWarmOutcome::Published;
        };
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                                RigExecWarmPriority::Neighbor, current,
                                blocking));
        // The release arrives from a second thread once the job starts, so
        // the destructor below -- which joins the worker running the job --
        // cannot deadlock however the threads interleave.
        std::thread releaser([&] {
            std::unique_lock<std::mutex> lock(gateMutex);
            gateCv.wait(lock, [&] { return started; });
            released = true;
            lock.unlock();
            gateCv.notify_all();
        });
        releaser.join();
    }
    CHECK(ran == 1);
}

// A cancel purging while a job runs drops the queued jobs behind it: one
// worker holds the running job, so the sweep behind is still queued when the
// purge lands, and it never runs.
void
TestCancelPurgesWhileAJobRuns()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestCancelPurgesWhileAJobRuns: "
                    "warming unavailable in this process\n");
        return;
    }
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool started = false;
    bool released = false;
    RigExecBackgroundScheduler scheduler(/*workerCount=*/1);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    RigExecWarmWork blocking = [&](const RigExecWarmRequest &) {
        {
            std::lock_guard<std::mutex> lock(gateMutex);
            started = true;
        }
        gateCv.notify_all();
        std::unique_lock<std::mutex> lock(gateMutex);
        gateCv.wait(lock, [&] { return released; });
        return RigExecWarmOutcome::Published;
    };
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Neighbor, current, blocking));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Sweep, current,
                            recorder.Work()));
    {
        std::unique_lock<std::mutex> lock(gateMutex);
        gateCv.wait(lock, [&] { return started; });
    }
    // The worker holds the neighbor; the sweep is still queued, so the purge
    // retires exactly it.
    scheduler.CancelGeneration(rig);
    CHECK(scheduler.Stats().canceled == 1);
    {
        std::lock_guard<std::mutex> lock(gateMutex);
        released = true;
    }
    gateCv.notify_all();
    scheduler.WaitUntilIdle();
    CHECK(recorder.Runs().empty());
    CHECK(scheduler.Stats().completed == 1);
    CHECK(scheduler.Stats().queuedDepth == 0);
}

// Worker counts clamp to the documented range: production's 2-4, and zero
// for the manual mode.
void
TestWorkerCountClampsToRange()
{
    CHECK(RigExecBackgroundScheduler().GetWorkerCount() ==
          kRigExecBackgroundSchedulerDefaultWorkers);
    CHECK(RigExecBackgroundScheduler(0).GetWorkerCount() == 0);
    CHECK(RigExecBackgroundScheduler(4).GetWorkerCount() == 4);
    CHECK(RigExecBackgroundScheduler(17).GetWorkerCount() ==
          kRigExecBackgroundSchedulerMaxWorkers);
    CHECK(RigExecBackgroundScheduler(-3).GetWorkerCount() == 0);
    CHECK(kRigExecBackgroundSchedulerDefaultWorkers >= 2);
    CHECK(kRigExecBackgroundSchedulerDefaultWorkers <=
          kRigExecBackgroundSchedulerMaxWorkers);
}

// The priority backend reports honestly: a known name, and a callable setter
// whose answer the test never interprets -- gate tests assert progress, not
// OS priority values.
void
TestPriorityBackendReportsHonestly()
{
    const char *backend = RigExecBackgroundPriorityBackend();
    CHECK(backend != nullptr);
    const std::string name = backend ? backend : "";
    CHECK(name == "SetThreadPriority" || name == "sched-batch" ||
          name == "qos-utility" || name == "none");
    (void)RigExecSetBackgroundThreadPriority();
}

// OUTCOME. Ran jobs split completed by disposition: published,
// declined-invalid, declined-generation; a throwing job warns and lands
// declined-invalid; completed still equals the three.
void
TestOutcomesSplitCompletion()
{
    std::printf("progress: TestOutcomesSplitCompletion\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestOutcomesSplitCompletion: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const auto outcome = [](RigExecWarmOutcome o) {
        return [o](const RigExecWarmRequest &) { return o; };
    };
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Neighbor, current,
                            outcome(RigExecWarmOutcome::Published)));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Neighbor, current,
                            outcome(RigExecWarmOutcome::Published)));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                            RigExecWarmPriority::Neighbor, current,
                            outcome(RigExecWarmOutcome::DeclinedInvalid)));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(4.0),
                            RigExecWarmPriority::Neighbor, current,
                            outcome(RigExecWarmOutcome::DeclinedGeneration)));
    CHECK(scheduler.Enqueue(
        rig, UsdTimeCode(5.0), RigExecWarmPriority::Neighbor, current,
        [](const RigExecWarmRequest &) -> RigExecWarmOutcome {
            throw std::runtime_error("test throw");
        }));
    CHECK(scheduler.DrainQueueForTesting() == 5);
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.completed == 5);
    CHECK(stats.published == 2);
    CHECK(stats.declinedInvalid == 2);
    CHECK(stats.declinedGeneration == 1);
    CHECK(stats.completed == stats.published + stats.declinedInvalid +
          stats.declinedGeneration);
    // Five accepts, no merges, all five terminal under the new equation.
    CHECK(stats.published + stats.declinedInvalid + stats.declinedGeneration +
              stats.droppedStale + stats.canceled + stats.shed +
              stats.droppedAtShutdown + stats.queuedDepth + stats.running ==
          5);
}

// Records transitions for the hook test. Thread-safe (workers report while
// the test asserts afterwards), like Recorder.
class TransitionRecorder {
public:
    RigExecWarmTransitionHook Hook()
    {
        return [this](const RigExecWarmTransition &transition) {
            std::lock_guard<std::mutex> lock(_mutex);
            _transitions.push_back(transition);
        };
    }

    std::vector<RigExecWarmTransition> Transitions() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _transitions;
    }

private:
    mutable std::mutex _mutex;
    std::vector<RigExecWarmTransition> _transitions;
};

// HOOK. The transition hook reports every per-frame queue move -- queued,
// running, finished, canceled, shed, dropped-at-shutdown -- in order, on
// the manual drain.
void
TestTransitionHookReportsQueueVisibility()
{
    std::printf("progress: TestTransitionHookReportsQueueVisibility\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestTransitionHookReportsQueueVisibility: "
                    "warming unavailable in this process\n");
        return;
    }
    TransitionRecorder recorder;
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        scheduler.SetTransitionHook(recorder.Hook());
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        RigExecWarmWork published = [](const RigExecWarmRequest &) {
            return RigExecWarmOutcome::Published;
        };
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                                RigExecWarmPriority::Neighbor, current,
                                published));
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                                RigExecWarmPriority::Sweep, current,
                                published));
        CHECK(scheduler.RunNextQueuedForTesting());
        CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(2.0)) == 1);
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                RigExecWarmPriority::Sweep, current,
                                published));
        scheduler.CancelGeneration(rig);
    }
    const std::vector<RigExecWarmTransition> got = recorder.Transitions();
    CHECK(got.size() == 7);
    const auto expect = [&got](size_t i, double time,
                               RigExecWarmTransitionKind kind,
                               RigExecWarmOutcome outcome =
                                   RigExecWarmOutcome::Published) {
        return i < got.size() && got[i].timeValue == time &&
            got[i].kind == kind && got[i].outcome == outcome;
    };
    CHECK(expect(0, 1.0, RigExecWarmTransitionKind::Queued));
    CHECK(expect(1, 2.0, RigExecWarmTransitionKind::Queued));
    CHECK(expect(2, 1.0, RigExecWarmTransitionKind::Running));
    CHECK(expect(3, 1.0, RigExecWarmTransitionKind::Finished,
                 RigExecWarmOutcome::Published));
    CHECK(expect(4, 2.0, RigExecWarmTransitionKind::Shed));
    CHECK(expect(5, 3.0, RigExecWarmTransitionKind::Queued));
    CHECK(expect(6, 3.0, RigExecWarmTransitionKind::Canceled));

    // A scheduler destroyed with queued jobs reports the shutdown drops.
    TransitionRecorder shutdown;
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        scheduler.SetTransitionHook(shutdown.Hook());
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current =
            scheduler.CurrentGeneration(rig);
        RigExecWarmWork published = [](const RigExecWarmRequest &) {
            return RigExecWarmOutcome::Published;
        };
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(5.0),
                                RigExecWarmPriority::Sweep, current,
                                published));
    }
    const std::vector<RigExecWarmTransition> lost = shutdown.Transitions();
    CHECK(lost.size() == 2);
    if (lost.size() == 2) {
        CHECK(lost[0].timeValue == 5.0 &&
              lost[0].kind == RigExecWarmTransitionKind::Queued);
        CHECK(lost[1].timeValue == 5.0 &&
              lost[1].kind == RigExecWarmTransitionKind::DroppedAtShutdown);
    }
}

// FAIRNESS. One worker preserves the pop order exactly: neighbors
// closest-first, then the sweep in vector order.
void
TestPoolPreservesPriorityOrder()
{
    std::printf("progress: TestPoolPreservesPriorityOrder\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPoolPreservesPriorityOrder: "
                    "warming unavailable in this process\n");
        return;
    }
    // One worker: pops serialize with runs, so the execution order is the
    // pop order, exactly.
    RigExecBackgroundScheduler scheduler(/*workerCount=*/1);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(20.0), UsdTimeCode(21.0)};
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                    recorder.Factory(),
                                    /*neighborRadius=*/2) == 6);
    scheduler.WaitUntilIdle();
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(runs.size() == 6);
    const double expected[] = {9.0, 11.0, 8.0, 12.0, 20.0, 21.0};
    for (size_t i = 0; i < runs.size() && i < 6; ++i) {
        CHECK(runs[i].time == expected[i]);
        CHECK(runs[i].priority == (i < 4 ? RigExecWarmPriority::Neighbor
                                         : RigExecWarmPriority::Sweep));
    }
}

// FAIRNESS. Under a two-worker flood no rig starves: both rigs complete
// every job and the books balance.
void
TestNoRigStarvesUnderFlood()
{
    std::printf("progress: TestNoRigStarvesUnderFlood\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestNoRigStarvesUnderFlood: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/2);
    Recorder recorder;
    const SdfPath rigA("/Asset/RigA");
    const SdfPath rigB("/Asset/RigB");
    size_t accepts = 0;
    for (int t = 1; t <= 60; ++t) {
        accepts += scheduler.Enqueue(rigA, UsdTimeCode(double(t)),
                                     RigExecWarmPriority::Sweep,
                                     scheduler.CurrentGeneration(rigA),
                                     recorder.Work())
            ? 1
            : 0;
    }
    for (int t = 1; t <= 60; ++t) {
        accepts += scheduler.Enqueue(rigB, UsdTimeCode(double(t)),
                                     RigExecWarmPriority::Sweep,
                                     scheduler.CurrentGeneration(rigB),
                                     recorder.Work())
            ? 1
            : 0;
    }
    CHECK(accepts == 120);
    scheduler.WaitUntilIdle();
    size_t ranA = 0;
    size_t ranB = 0;
    for (const RunRecord &run : recorder.Runs()) {
        if (run.rig == rigA) {
            ++ranA;
        } else if (run.rig == rigB) {
            ++ranB;
        }
    }
    CHECK(ranA == 60);
    CHECK(ranB == 60);
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.published == 120);
    CHECK(stats.published + stats.declinedInvalid + stats.declinedGeneration +
              stats.droppedStale + stats.canceled + stats.shed +
              stats.droppedAtShutdown + stats.queuedDepth + stats.running ==
          120);
}

// BUDGET. The sampling budget bounds factory invocations per trigger; the
// millisecond stop binds sampling; already-queued merges consume nothing;
// the unlimited default preserves the full burst shape.
void
TestSamplingBudgetBoundsFactoryInvocations()
{
    std::printf("progress: TestSamplingBudgetBoundsFactoryInvocations\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestSamplingBudgetBoundsFactoryInvocations: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    std::vector<UsdTimeCode> sweep;
    for (int t = 20; t < 40; ++t) {
        sweep.push_back(UsdTimeCode(double(t)));
    }
    RigExecWarmSamplingBudget budget;
    budget.maxFactoryInvocations = 5;
    // Five invocations, five jobs; the trigger stops at the budget.
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory(), budget) == 5);
    CHECK(scheduler.Stats().factoryInvocations == 5);
    // The same sweep again: the five queued times merge without sampling
    // and the rest sample up to the budget.
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory(), budget) == 5);
    CHECK(scheduler.Stats().factoryInvocations == 10);
    CHECK(scheduler.DrainQueueForTesting() == 10);
    // Drained, the next trigger samples the same first five again: the
    // scheduler keeps no visited-memory (the registry cursor skips
    // visited frames; the scheduler re-samples whatever it is given).
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory(), budget) == 5);
    CHECK(scheduler.Stats().factoryInvocations == 15);
    // A zero millisecond stop samples nothing (deterministic: elapsed 0
    // meets the stop before the first sample).
    RigExecWarmSamplingBudget stopped;
    stopped.maxMs = 0.0;
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                    recorder.Factory(), 2, stopped) == 0);
    CHECK(scheduler.Stats().factoryInvocations == 15);
    CHECK(scheduler.DrainQueueForTesting() == 5);
    // The unlimited default preserves the full burst shape.
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory()) == 20);
    CHECK(scheduler.Stats().factoryInvocations == 35);
    // The factory-side count agrees with the scheduler's.
    CHECK(recorder.FactoryCalls() == 35);
}

}  // namespace

int
main()
{
    TestAnUnseenRigSitsAtGenerationZero();
    TestCancelBumpsMonotonically();
    TestRigsAreIndependent();
    TestAStaleJobIsRefused();
    TestFrameCacheModeParses();
    TestWarmingGateTruthTable();
    TestFrameCacheModeReadsLiveFromTheEnvironment();
    TestClosedGateDeclinesWarmingButKeepsTheFence();
    TestACurrentJobIsAcceptedNeighborBeforeSweep();
    TestFifoWithinAPriority();
    TestDuplicateTimesCoalesce();
    TestSweepPromotesToNeighborOnReenqueue();
    TestUnusableEnqueuesDecline();
    TestPerRigCapDeclinesPastItsLimit();
    TestCancelPurgesQueuedJobsPromptly();
    TestCommitTriggerEnqueuesNeighborsThenSweepNeverPlayhead();
    TestTriggerSkipsUnsampleableAndStopsAtCap();
    TestTriggerWithStaleGenerationOrDefaultPlayheadEnqueuesNothing();
    TestTriggerStopsWhenAnEditLandsMidTrigger();
    TestIdleTriggerEnqueuesSweepOnly();
    TestPlaybackAdvanceShedsPassedSweep();
    TestPlaybackAdvanceWithNaNShedsNothing();
    TestProfilerLaneRecordsQueueAndCancels();
    TestStatsBalanceOverAMixedScenario();
    TestOutcomesSplitCompletion();
    TestTransitionHookReportsQueueVisibility();
    TestPoolPreservesPriorityOrder();
    TestNoRigStarvesUnderFlood();
    TestSamplingBudgetBoundsFactoryInvocations();
    TestPoolRunsEveryJobExactlyOnce();
    TestThreadedCancelBalancesTheBooks();
    TestShutdownDropsPendingJobs();
    TestShutdownJoinsARunningJob();
    TestCancelPurgesWhileAJobRuns();
    TestWorkerCountClampsToRange();
    TestPriorityBackendReportsHonestly();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBackgroundScheduler: all tests passed\n");
    return 0;
}