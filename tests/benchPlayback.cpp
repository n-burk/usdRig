//
// benchPlayback -- what pressing play costs, tick by tick, with warming on.
//
// Simulates exactly what the usdview plugin does on every playback tick
// (_OnFrameChanged with no pending edit): SetTime(frame) then OnIdle().
// Two passes: a cold first play paced at 24 fps (the playhead a user
// watches), then an unpaced SetTime-only replay of the same frames (what
// memoization alone serves on a second visit). Reports per-tick SetTime /
// OnIdle costs, evaluator pulls, frame-cache hits/misses, and scheduler
// completions, so a "playback feels slow" report can be attributed to the
// tick path, the warming race, or the cache rather than guessed at.
// Prints human-readable numbers; asserts nothing, so it is built but
// deliberately NOT registered with ctest (like benchCommitLag).
//
//   benchPlayback [examplesDir] [frameCount] [paceMs] [stage] [profile]
//
// paceMs paces tick starts apart (41.7 ~= 24 fps); 0 runs unpaced, which
// is the mechanics run (the playhead outruns the workers on purpose).
// stage is the stage path under examplesDir (default
// "biped/Biped_anim.usda"); "biped/Biped_stack_anim.usda" is the heavy
// lane. profile (default 0) enables the scheduler and bridge profilers
// and prints their summaries after the passes; profiling perturbs tick
// timings, so it stays off unless asked.
//
// Environment, read and reported, never assumed: RIGEXEC_FRAME_CACHE (unset
// is set to "on" in-process for the run; an explicit off/warm-off is
// honored) and RIGEXEC_ENABLE_PARALLEL_EVAL (the process default).

#include "rigExecImaging/registry.h"
#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"

#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

const char *
GetEnv(const char *name)
{
#ifdef _WIN32
    static char buffer[256];
    size_t needed = 0;
    if (getenv_s(&needed, buffer, sizeof(buffer), name) != 0 ||
        needed == 0) {
        return nullptr;
    }
    return buffer;
#else
    return getenv(name);
#endif
}

void
SetEnv(const char *name, const char *value)
{
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

double
NowUs()
{
    return double(std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count());
}

double
Median(std::vector<double> samples)
{
    if (samples.empty()) {
        return 0.0;
    }
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
}

double
MaxOf(const std::vector<double> &samples)
{
    if (samples.empty()) {
        return 0.0;
    }
    return *std::max_element(samples.begin(), samples.end());
}

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

}  // namespace

int
main(int argc, char **argv)
{
    const std::string examplesDir =
        argc > 1 ? argv[1] : std::string("examples");
    const int frameCount = argc > 2 ? std::atoi(argv[2]) : 48;
    const double paceMs = argc > 3 ? std::atof(argv[3]) : 41.7;
    const std::string stageFile =
        argc > 4 ? argv[4] : std::string("biped/Biped_anim.usda");
    const bool profile = argc > 5 ? std::atoi(argv[5]) != 0 : false;
    if (!GetEnv("RIGEXEC_FRAME_CACHE")) {
        SetEnv("RIGEXEC_FRAME_CACHE", "on");
    }
    std::printf("RIGEXEC_FRAME_CACHE=%s\n",
                GetEnv("RIGEXEC_FRAME_CACHE"));
    std::printf("RIGEXEC_ENABLE_PARALLEL_EVAL=%s\n",
                GetEnv("RIGEXEC_ENABLE_PARALLEL_EVAL") ?
                    GetEnv("RIGEXEC_ENABLE_PARALLEL_EVAL") : "(unset)");
    std::printf("background warming enabled: %s\n",
                RigExecBackgroundWarmingEnabled() ? "yes" : "NO");

    const std::string stagePath = examplesDir + "/" + stageFile;
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    if (!stage) {
        std::printf("FATAL: cannot open %s\n", stagePath.c_str());
        return 1;
    }
    const SdfPath rig = FindRig(stage);
    if (rig.IsEmpty()) {
        std::printf("FATAL: no RigExecRoot on %s\n", stagePath.c_str());
        return 1;
    }
    double startValue = 1.0;
    const double stageStart = stage->GetStartTimeCode();
    if (std::isfinite(stageStart)) {
        startValue = stageStart;
    }
    const std::string::size_type slash = stageFile.rfind('/');
    const std::string label =
        slash == std::string::npos ? stageFile : stageFile.substr(slash + 1);
    std::printf("[%s] %s rig=%s start=%g frames=%d pace=%g ms\n",
                label.c_str(), stagePath.c_str(), rig.GetString().c_str(),
                startValue, frameCount, paceMs);

    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    {
        std::vector<std::string> errors;
        if (!registry.Activate(stage, rig, UsdTimeCode(startValue),
                               &errors)) {
            std::printf("FATAL: activation failed\n");
            for (const std::string &error : errors) {
                std::printf("  %s\n", error.c_str());
            }
            return 1;
        }
    }
    // Cold first play: no priming commit. A real session warms the held
    // playhead only after an edit (the release flush) or a scrub, so a
    // user who opens the stage and immediately presses play starts here.
    if (profile) {
        if (RigExecProfiler *schedulerProfiler =
                registry.MutableSchedulerProfiler()) {
            schedulerProfiler->SetEnabled(true);
        }
        if (RigExecProfiler *bridgeProfiler =
                registry.MutableBridgeProfiler(rig)) {
            bridgeProfiler->SetEnabled(true);
        }
    }
    const size_t evalsAtOpen = registry.GetSessionEvaluationCount(rig);
    std::printf("pass 1: cold first play, %d ticks (SetTime + OnIdle)\n",
                frameCount);
    std::vector<double> setTimes;
    std::vector<double> idles;
    const auto playStart = std::chrono::steady_clock::now();
    for (int tick = 0; tick < frameCount; ++tick) {
        const auto tickStart = std::chrono::steady_clock::now();
        const UsdTimeCode time(startValue + double(tick));
        const double setStart = NowUs();
        if (!registry.SetTime(time)) {
            std::printf("FATAL: SetTime(%g) failed\n", time.GetValue());
            return 1;
        }
        const double setUs = NowUs() - setStart;
        const double idleStart = NowUs();
        const size_t enqueued = registry.OnIdle();
        const double idleUs = NowUs() - idleStart;
        setTimes.push_back(setUs);
        idles.push_back(idleUs);
        if (tick < 12 || tick % 8 == 7) {
            const RigExecBackgroundSchedulerStats progress =
                registry.GetBackgroundStats();
            std::printf("  tick %3d f=%6g set=%8.1f us idle=%8.1f us "
                        "(enq %zu) completed=%zu queued=%zu\n",
                        tick, time.GetValue(), setUs, idleUs, enqueued,
                        progress.completed, progress.queuedDepth);
        }
        if (paceMs > 0.0) {
            const auto nextStart =
                tickStart +
                std::chrono::microseconds(int64_t(paceMs * 1000.0));
            std::this_thread::sleep_until(nextStart);
        }
    }
    const double playMs =
        double(std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - playStart)
                   .count()) /
        1000.0;
    const size_t evalsAfterPlay = registry.GetSessionEvaluationCount(rig);
    const RigExecFrameCacheStats cacheAfterPlay =
        registry.GetFrameCacheStats(rig);
    const RigExecBackgroundSchedulerStats schedAfterPlay =
        registry.GetBackgroundStats();
    std::printf("  SetTime: median %8.1f us, max %8.1f us\n",
                Median(setTimes), MaxOf(setTimes));
    std::printf("  OnIdle:  median %8.1f us, max %8.1f us\n",
                Median(idles), MaxOf(idles));
    std::printf("  wall: %.1f ms for %d ticks (%.1f fps equivalent)\n",
                playMs, frameCount,
                playMs > 0.0 ? 1000.0 * double(frameCount) / playMs : 0.0);
    std::printf("  pulls: %zu over the pass (%zu at open)\n",
                evalsAfterPlay - evalsAtOpen, evalsAtOpen);
    std::printf("  cache: hits=%zu misses=%zu published=%zu evictions=%zu "
                "entries=%zu\n",
                cacheAfterPlay.hits, cacheAfterPlay.misses,
                cacheAfterPlay.published, cacheAfterPlay.evictions,
                cacheAfterPlay.entryCount);
    std::printf("  scheduler: queued=%zu running=%zu completed=%zu "
                "droppedStale=%zu canceled=%zu shed=%zu coalesced=%zu "
                "upgraded=%zu declined=%zu\n",
                schedAfterPlay.queuedDepth, schedAfterPlay.running,
                schedAfterPlay.completed, schedAfterPlay.droppedStale,
                schedAfterPlay.canceled, schedAfterPlay.shed,
                schedAfterPlay.coalesced, schedAfterPlay.upgraded,
                schedAfterPlay.declined);

    // Second visit: the same frames again, SetTime only, unpaced. Whatever
    // the first pass memoized (or the workers finished since) serves here
    // without evaluating.
    registry.WaitUntilBackgroundIdle();
    const RigExecBackgroundSchedulerStats schedDrained =
        registry.GetBackgroundStats();
    std::printf("pass 2: unpaced replay, SetTime only (drained: "
                "completed=%zu droppedStale=%zu shed=%zu declined=%zu)\n",
                schedDrained.completed, schedDrained.droppedStale,
                schedDrained.shed, schedDrained.declined);
    std::vector<double> replays;
    for (int tick = 0; tick < frameCount; ++tick) {
        const double start = NowUs();
        if (!registry.SetTime(UsdTimeCode(startValue + double(tick)))) {
            std::printf("FATAL: replay SetTime failed\n");
            return 1;
        }
        replays.push_back(NowUs() - start);
    }
    const size_t evalsAfterReplay = registry.GetSessionEvaluationCount(rig);
    const RigExecFrameCacheStats cacheAfterReplay =
        registry.GetFrameCacheStats(rig);
    std::printf("  SetTime: median %8.1f us, max %8.1f us\n",
                Median(replays), MaxOf(replays));
    std::printf("  pulls: %zu over the replay\n",
                evalsAfterReplay - evalsAfterPlay);
    std::printf("  cache: hits=%zu (+%zu) misses=%zu (+%zu) entries=%zu\n",
                cacheAfterReplay.hits,
                cacheAfterReplay.hits - cacheAfterPlay.hits,
                cacheAfterReplay.misses,
                cacheAfterReplay.misses - cacheAfterPlay.misses,
                cacheAfterReplay.entryCount);
    if (profile) {
        const auto printSummary = [](const char *whose,
                                     RigExecProfiler *profiler) {
            if (!profiler) {
                return;
            }
            std::printf("profile: %s\n", whose);
            for (const RigExecProfileSummaryRow &row :
                 profiler->Summarize()) {
                std::printf("  %-40s %6zu x %10llu us total %8llu us max\n",
                            (row.category + "." + row.name).c_str(), row.count,
                            (unsigned long long)row.totalUs,
                            (unsigned long long)row.maxUs);
            }
        };
        printSummary("scheduler", registry.MutableSchedulerProfiler());
        printSummary("bridge", registry.MutableBridgeProfiler(rig));
    }
    registry.Deactivate();
    return 0;
}
