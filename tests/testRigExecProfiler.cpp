//
// Tests for the evaluation profiler (libs/rigExec/profiler.h).
//
// The profiler is deliberately free of USD dependencies, so this test needs
// no stage: it covers the enabled/disabled gate, nested scopes, summary
// aggregation, and Chrome Trace output.
//

#include "rigExec/profiler.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

static int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("FAILED: %s (line %d)\n", #cond, __LINE__);            \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

static int nameEvaluations = 0;

static const char *
NameWithSideEffect()
{
    ++nameEvaluations;
    return "evaluated";
}

static std::string
ReadFile(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

int
main()
{
    // Disabled by default: nothing is recorded.
    {
        rigExec::RigExecProfiler profiler;
        CHECK(!profiler.IsEnabled());
        {
            RIGEXEC_PROFILE_SCOPE(profiler, "never");
        }
        profiler.Record("never", "rig", 0, 10);
        CHECK(profiler.GetEventCount() == 0);
    }

    // The macro does not evaluate its name expression when disabled, so
    // unprofiled evaluation pays no string building.
    {
        rigExec::RigExecProfiler profiler;
        nameEvaluations = 0;
        {
            RIGEXEC_PROFILE_SCOPE(profiler, NameWithSideEffect());
            RIGEXEC_PROFILE_SCOPE_CAT(profiler, NameWithSideEffect(), "pose");
        }
        CHECK(nameEvaluations == 0);
        CHECK(profiler.GetEventCount() == 0);
        profiler.SetEnabled(true);
        {
            RIGEXEC_PROFILE_SCOPE(profiler, NameWithSideEffect());
        }
        CHECK(nameEvaluations == 1);
        CHECK(profiler.GetEventCount() == 1);
    }

    // Nested scopes record names, categories, and non-decreasing spans.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);
        {
            RIGEXEC_PROFILE_SCOPE_CAT(profiler, "Outer", "evaluate");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            {
                RIGEXEC_PROFILE_SCOPE_CAT(profiler, "Inner", "pose");
            }
        }
        const std::vector<rigExec::RigExecProfileEvent> events =
            profiler.GetEvents();
        CHECK(events.size() == 2);
        // Completion order: the inner scope closes first.
        CHECK(events[0].name == "Inner");
        CHECK(events[0].category == "pose");
        CHECK(events[1].name == "Outer");
        CHECK(events[1].category == "evaluate");
        CHECK(events[1].startUs <= events[0].startUs);
        CHECK(events[1].durationUs >= events[0].durationUs);
        CHECK(events[1].durationUs >= 2000);
    }

    // Scopes can carry args; recording from two threads yields two lanes.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);
        {
            rigExec::RigExecProfileScope scope(&profiler, "WithArgs", "exec");
            scope.AddArg("solvers", "4");
        }
        std::thread worker([&profiler]() {
            RIGEXEC_PROFILE_SCOPE(profiler, "Worker");
        });
        worker.join();
        const std::vector<rigExec::RigExecProfileEvent> events =
            profiler.GetEvents();
        CHECK(events.size() == 2);
        CHECK(events[0].args.count("solvers") == 1);
        CHECK(events[0].args.at("solvers") == "4");
        CHECK(events[0].threadIndex != events[1].threadIndex);
    }

    // Summaries aggregate per (category, name), sorted by total descending.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);
        profiler.Record("b", "rig", 0, 10);
        profiler.Record("a", "rig", 10, 40);
        profiler.Record("b", "rig", 40, 55);
        const std::vector<rigExec::RigExecProfileSummaryRow> summary =
            profiler.Summarize();
        CHECK(summary.size() == 2);
        CHECK(summary[0].name == "a");
        CHECK(summary[0].count == 1);
        CHECK(summary[0].totalUs == 30);
        CHECK(summary[0].maxUs == 30);
        CHECK(summary[1].name == "b");
        CHECK(summary[1].count == 2);
        CHECK(summary[1].totalUs == 25);
        CHECK(summary[1].maxUs == 15);
    }

    // Chrome Trace output: complete events with normalized timestamps.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);
        profiler.Record("First", "pose", 1000000, 1001000,
                        {{"frame", "1"}});
        profiler.Record("Second", "geometry", 1000500, 1000700);
        const std::string path = "testRigExecProfiler.trace";
        std::string error;
        CHECK(profiler.WriteChromeTrace(path, &error));
        CHECK(error.empty());
        const std::string text = ReadFile(path);
        CHECK(text.find("\"traceEvents\"") != std::string::npos);
        CHECK(text.find("\"ph\":\"X\"") != std::string::npos);
        CHECK(text.find("\"name\":\"First\"") != std::string::npos);
        CHECK(text.find("\"cat\":\"pose\"") != std::string::npos);
        // Origin-normalized: the first event starts at ts 0.
        CHECK(text.find("\"ts\":0,") != std::string::npos);
        CHECK(text.find("\"ts\":500,") != std::string::npos);
        CHECK(text.find("\"dur\":1000") != std::string::npos);
        CHECK(text.find("\"frame\":\"1\"") != std::string::npos);
        CHECK(text.find("\"displayTimeUnit\":\"ms\"") != std::string::npos);
        std::remove(path.c_str());
    }

    // JSON escaping keeps names with quotes parseable.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);
        profiler.Record("a\"b\\c", "r\"g", 0, 1);
        const std::string path = "testRigExecProfilerEscape.trace";
        CHECK(profiler.WriteChromeTrace(path, nullptr));
        const std::string text = ReadFile(path);
        CHECK(text.find("a\\\"b\\\\c") != std::string::npos);
        std::remove(path.c_str());
    }

    // An unwritable path fails with a message instead of crashing.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);
        std::string error;
        CHECK(!profiler.WriteChromeTrace(
            "no-such-directory/out.trace", &error));
        CHECK(!error.empty());
    }

    // Clear drops events but keeps the enabled state.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);
        profiler.Record("x", "rig", 0, 1);
        CHECK(profiler.GetEventCount() == 1);
        profiler.Clear();
        CHECK(profiler.GetEventCount() == 0);
        CHECK(profiler.IsEnabled());
        profiler.Record("y", "rig", 0, 1);
        CHECK(profiler.GetEventCount() == 1);
    }

    // The enabling thread owns row 0, whoever records first. Without this
    // the main row of a trace moves between runs, because which thread
    // finishes a scope first is a race.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);
        std::thread worker([&profiler] { profiler.Record("w", "rig", 0, 1); });
        worker.join();
        profiler.Record("m", "rig", 0, 1);
        const auto events = profiler.GetEvents();
        CHECK(events.size() == 2);
        CHECK(events[0].name == "w");
        CHECK(events[0].threadIndex != 0);  // the worker, recorded first
        CHECK(events[1].name == "m");
        CHECK(events[1].threadIndex == 0);  // the thread that enabled it
        CHECK(profiler.GetThreadCount() == 2);
    }

    // RecordOn attributes an interval to the thread that RAN it, not the one
    // reporting it. This is what lets the baked program's parallel executor
    // time a step where it runs -- lock-free, no scope -- and replay it from
    // the epilogue without collapsing every step onto the epilogue's row.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);

        std::thread::id ranOn;
        std::thread worker([&ranOn] { ranOn = std::this_thread::get_id(); });
        worker.join();

        // Replayed from THIS thread, on behalf of one that has already gone.
        profiler.RecordOn(ranOn, "step", "step", 0, 10);
        profiler.Record("epilogue", "rig", 0, 10);

        const auto events = profiler.GetEvents();
        CHECK(events.size() == 2);
        CHECK(events[0].threadIndex != events[1].threadIndex);
        CHECK(events[1].threadIndex == 0);
        CHECK(profiler.GetThreadCount() == 2);

        // A default id means "nobody said": fall back to the caller rather
        // than invent a row.
        profiler.RecordOn(std::thread::id(), "unattributed", "rig", 0, 10);
        const auto after = profiler.GetEvents();
        CHECK(after.back().threadIndex == 0);
        CHECK(profiler.GetThreadCount() == 2);
    }

    // Every row is named in the trace, so a reader does not have to guess
    // which number is the main thread.
    {
        rigExec::RigExecProfiler profiler;
        profiler.SetEnabled(true);
        std::thread::id ranOn;
        std::thread worker([&ranOn] { ranOn = std::this_thread::get_id(); });
        worker.join();
        profiler.RecordOn(ranOn, "step", "step", 0, 10);
        profiler.Record("here", "rig", 0, 10);

        const std::string path = "testRigExecProfilerThreads.trace";
        CHECK(profiler.WriteChromeTrace(path, nullptr));
        const std::string text = ReadFile(path);
        CHECK(text.find("\"thread_name\"") != std::string::npos);
        CHECK(text.find("main (Evaluate)") != std::string::npos);
        CHECK(text.find("rigExec worker 1") != std::string::npos);
        // Two rows, and the step is not on the main one.
        CHECK(text.find("\"tid\":1") != std::string::npos);
        std::remove(path.c_str());
    }

    if (failures == 0) {
        std::printf("testRigExecProfiler: all checks passed\n");
    }
    return failures == 0 ? 0 : 1;
}
