//
// RigExec profiler: a lightweight scoped phase timer for rig evaluation.
//
// The evaluator already counts work (revisions created/executed, schedules
// built, solver evaluations); this answers the complementary question of
// where wall time goes. Recording is off by default and costs one branch per
// scope when disabled. When enabled, scopes nest: Evaluate() opens the
// outermost span and each phase, solver batch, constraint, and geometry
// chain opens a child span, so the Chrome trace shows the critical path
// directly.
//
// The output is the Chrome Trace Event format ("X" complete scopes, plus "I"
// instant points and "C" counters for the per-frame cache lanes), which
// Perfetto (ui.perfetto.dev) and chrome://tracing both open. Timestamps are
// microseconds, normalized so the first recorded event starts at zero.
//
// This header depends only on the standard library, so the timing harness
// itself is unit-testable without a stage or a USD build.
//

#ifndef RIGEXEC_PROFILER_H
#define RIGEXEC_PROFILER_H

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace rigExec {

/// The Chrome Trace Event kinds the profiler writes.
enum class RigExecProfileEventKind {
    /// A named interval ("X"): the cost rows Summarize aggregates.
    Complete,
    /// A point in time ("I"): a cache hit, a miss, a cancel.
    Instant,
    /// A sampled value ("C"): queue depth, running jobs, cancel counts.
    Counter,
};

/// One recorded event: a named interval, instant point, or counter sample
/// with a category and string args.
struct RigExecProfileEvent {
    std::string name;
    std::string category;
    uint64_t startUs = 0;
    uint64_t durationUs = 0;
    uint64_t threadIndex = 0;
    std::map<std::string, std::string> args;
    RigExecProfileEventKind kind = RigExecProfileEventKind::Complete;
    /// Counter samples, for Counter events only; empty otherwise. Written
    /// as JSON numbers, not strings.
    std::map<std::string, double> counters;
};

/// Aggregated cost of one (category, name) pair across a run.
struct RigExecProfileSummaryRow {
    std::string name;
    std::string category;
    size_t count = 0;
    uint64_t totalUs = 0;
    uint64_t maxUs = 0;
};

/// Trace categories for the per-frame cache lanes (Stream F). Lookup hits
/// and misses record on "frameCache"; scheduler queue depth, running jobs,
/// and cancels on "scheduler". Perfetto groups by category, so each renders
/// as its own lane.
constexpr const char *kRigExecProfileCategoryFrameCache = "frameCache";
constexpr const char *kRigExecProfileCategoryScheduler = "scheduler";

/// Collects RigExecProfileEvents. Not copyable; Record/Summarize/Write are
/// safe to call from any thread.
class RigExecProfiler {
public:
    RigExecProfiler() = default;
    RigExecProfiler(const RigExecProfiler &) = delete;
    RigExecProfiler &operator=(const RigExecProfiler &) = delete;

    void SetEnabled(bool enabled)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _enabled = enabled;
        if (enabled) {
            // Claim index 0 for whoever turns recording on. Without this the
            // indices fall out of the order threads happen to FIRST record,
            // and a compile-time worker task that finishes its scope before
            // the main thread finishes its own outer one would take tid 0 --
            // making the trace's main row a different thread from run to
            // run. The enabling thread is the one that opens Evaluate, so
            // this is the row a reader looks at first.
            _threadIds.clear();
            _ThreadIndexLocked(std::this_thread::get_id());
        }
    }

    bool IsEnabled() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _enabled;
    }

    /// Drops every recorded event. Does not change the enabled state.
    void Clear()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _events.clear();
        _threadIds.clear();
    }

    /// Records one completed interval, attributed to the calling thread.
    /// No-op unless enabled. Const because profiling is observability: it
    /// never changes evaluated values, so const evaluation paths can record
    /// into it.
    void Record(std::string name, std::string category, uint64_t startUs,
                uint64_t endUs,
                std::map<std::string, std::string> args = {}) const
    {
        RecordOn(std::this_thread::get_id(), std::move(name),
                 std::move(category), startUs, endUs, std::move(args));
    }

    /// Records one completed interval that ran on \p runner, which need not
    /// be the calling thread.
    ///
    /// This is the entry point for work that is timed where it runs and
    /// reported somewhere else. The baked program's parallel executor is the
    /// case it exists for: a step cannot open a profile scope, because
    /// Record takes this mutex and several hundred steps a frame contending
    /// on it would time the lock rather than the rig -- so a step stamps two
    /// integers and the thread that ran it into storage it alone owns, and
    /// the epilogue replays the lot in step order. Without a runner the
    /// replay would put every step on the epilogue's row and the trace would
    /// claim a parallel frame ran on one thread.
    ///
    /// A default-constructed \p runner means "no thread said", and lands on
    /// the calling thread's row, so a caller with nothing to pass gets the
    /// old behaviour rather than a bogus row of its own.
    void RecordOn(std::thread::id runner, std::string name,
                  std::string category, uint64_t startUs, uint64_t endUs,
                  std::map<std::string, std::string> args = {}) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_enabled) {
            return;
        }
        RigExecProfileEvent event;
        event.name = std::move(name);
        event.category = std::move(category);
        event.startUs = startUs;
        event.durationUs = endUs >= startUs ? endUs - startUs : 0;
        event.threadIndex = _ThreadIndexLocked(
            runner == std::thread::id() ? std::this_thread::get_id()
                                        : runner);
        event.args = std::move(args);
        _events.push_back(std::move(event));
    }

    /// Records one instant point (a hit, a miss, a cancel) at \p timeUs,
    /// attributed to the calling thread. No-op unless enabled.
    void RecordInstant(std::string name, std::string category,
                       uint64_t timeUs,
                       std::map<std::string, std::string> args = {}) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_enabled) {
            return;
        }
        RigExecProfileEvent event;
        event.name = std::move(name);
        event.category = std::move(category);
        event.startUs = timeUs;
        event.durationUs = 0;
        event.kind = RigExecProfileEventKind::Instant;
        event.threadIndex = _ThreadIndexLocked(std::this_thread::get_id());
        event.args = std::move(args);
        _events.push_back(std::move(event));
    }

    /// Records one counter sample (queue depth, running jobs, cancel
    /// counts) at \p timeUs, attributed to the calling thread. No-op unless
    /// enabled. The values write as JSON numbers.
    void RecordCounter(std::string name, std::string category,
                       uint64_t timeUs,
                       std::map<std::string, double> counters) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_enabled) {
            return;
        }
        RigExecProfileEvent event;
        event.name = std::move(name);
        event.category = std::move(category);
        event.startUs = timeUs;
        event.durationUs = 0;
        event.kind = RigExecProfileEventKind::Counter;
        event.threadIndex = _ThreadIndexLocked(std::this_thread::get_id());
        event.counters = std::move(counters);
        _events.push_back(std::move(event));
    }

    /// The per-frame cache lane: one instant point per lookup on the
    /// "frameCache" category. \p hit true records "cacheHit", false
    /// "cacheMiss"; \p frame is the requested time value, carried as a
    /// string arg for the trace reader. No-op unless enabled.
    void RecordCacheLookup(bool hit, double frame) const
    {
        RecordInstant(hit ? "cacheHit" : "cacheMiss",
                      kRigExecProfileCategoryFrameCache, NowUs(),
                      {{"frame", std::to_string(frame)}});
    }

    /// The scheduler lane: one counter sample on the "scheduler" category
    /// with the queue depth, running jobs, and lifetime cancels. No-op
    /// unless enabled.
    void RecordSchedulerQueue(size_t queuedDepth, size_t running,
                              size_t canceled) const
    {
        RecordCounter("warmQueue", kRigExecProfileCategoryScheduler,
                      NowUs(),
                      {{"queuedDepth", double(queuedDepth)},
                       {"running", double(running)},
                       {"canceled", double(canceled)}});
    }

    /// The scheduler lane: one instant point per purge on the "scheduler"
    /// category, with the purged count and the cause ("edit", "playback",
    /// or "shutdown"). No-op unless enabled.
    void RecordSchedulerCancel(size_t purged,
                               const std::string &cause) const
    {
        RecordInstant("warmCancel", kRigExecProfileCategoryScheduler,
                      NowUs(),
                      {{"purged", std::to_string(purged)},
                       {"cause", cause}});
    }

    /// A copy of every recorded event, in completion order.
    std::vector<RigExecProfileEvent> GetEvents() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _events;
    }

    size_t GetEventCount() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _events.size();
    }

    /// How many distinct threads have recorded, or been recorded for. One
    /// means the run really did happen on a single thread -- which, for a
    /// frame that asked for the parallel schedule, is the finding rather
    /// than the default.
    size_t GetThreadCount() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _threadIds.size();
    }

    /// Per-(category, name) totals, sorted by total cost descending.
    std::vector<RigExecProfileSummaryRow> Summarize() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        std::map<std::pair<std::string, std::string>,
                 RigExecProfileSummaryRow>
            rows;
        for (const RigExecProfileEvent &event : _events) {
            // Costs only: instants and counters have no duration to
            // attribute, so the summary leaves them out.
            if (event.kind != RigExecProfileEventKind::Complete) {
                continue;
            }
            RigExecProfileSummaryRow &row =
                rows[{event.category, event.name}];
            row.name = event.name;
            row.category = event.category;
            ++row.count;
            row.totalUs += event.durationUs;
            row.maxUs = std::max(row.maxUs, event.durationUs);
        }
        std::vector<RigExecProfileSummaryRow> summary;
        summary.reserve(rows.size());
        for (const auto &[key, row] : rows) {
            summary.push_back(row);
        }
        std::sort(summary.begin(), summary.end(),
                  [](const RigExecProfileSummaryRow &a,
                     const RigExecProfileSummaryRow &b) {
                      return a.totalUs > b.totalUs;
                  });
        return summary;
    }

    /// Writes every recorded event as Chrome Trace Event JSON. The file can
    /// be opened in Perfetto or chrome://tracing. Returns false with a
    /// message when the file cannot be written.
    bool WriteChromeTrace(const std::string &path,
                          std::string *error = nullptr) const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out) {
            if (error) {
                *error = "cannot open " + path + " for writing";
            }
            return false;
        }
        uint64_t origin = 0;
        for (const RigExecProfileEvent &event : _events) {
            if (origin == 0 || event.startUs < origin) {
                origin = event.startUs;
            }
        }
        out << "{\"traceEvents\":[\n";
        bool first = true;
        // Name the rows before the spans that land on them. Without these
        // metadata events Perfetto labels each row with a bare number, and
        // the one question the rows exist to answer -- which of these is the
        // thread that called Evaluate -- needs a guess. Index 0 is that
        // thread by construction: SetEnabled claims it.
        for (const auto &[id, index] : _threadIds) {
            (void)id;
            if (!first) {
                out << ",\n";
            }
            first = false;
            out << "{\"name\":\"thread_name\",\"ph\":\"M\",\"pid\":1,\"tid\":"
                << index << ",\"args\":{\"name\":\""
                << (index == 0 ? "main (Evaluate)"
                               : "rigExec worker " + std::to_string(index))
                << "\"}}";
        }
        for (const RigExecProfileEvent &event : _events) {
            if (!first) {
                out << ",\n";
            }
            first = false;
            out << "{\"name\":\"" << _EscapeJson(event.name)
                << "\",\"cat\":\"" << _EscapeJson(event.category) << "\",";
            if (event.kind == RigExecProfileEventKind::Instant) {
                out << "\"ph\":\"I\",\"ts\":" << (event.startUs - origin)
                    << ",\"pid\":1,\"tid\":" << event.threadIndex
                    << ",\"s\":\"t\"";
            } else if (event.kind == RigExecProfileEventKind::Counter) {
                out << "\"ph\":\"C\",\"ts\":" << (event.startUs - origin)
                    << ",\"pid\":1,\"tid\":" << event.threadIndex;
            } else {
                out << "\"ph\":\"X\",\"ts\":" << (event.startUs - origin)
                    << ",\"dur\":" << event.durationUs
                    << ",\"pid\":1,\"tid\":" << event.threadIndex;
            }
            if (!event.args.empty() || !event.counters.empty()) {
                out << ",\"args\":{";
                bool firstArg = true;
                for (const auto &[key, value] : event.args) {
                    if (!firstArg) {
                        out << ",";
                    }
                    firstArg = false;
                    out << "\"" << _EscapeJson(key)
                        << "\":\"" << _EscapeJson(value) << "\"";
                }
                for (const auto &[key, value] : event.counters) {
                    if (!firstArg) {
                        out << ",";
                    }
                    firstArg = false;
                    out << "\"" << _EscapeJson(key)
                        << "\":" << _FormatCounter(value);
                }
                out << "}";
            }
            out << "}";
        }
        out << "\n],\"displayTimeUnit\":\"ms\"}\n";
        out.flush();
        if (!out) {
            if (error) {
                *error = "failed while writing " + path;
            }
            return false;
        }
        return true;
    }

    /// Microseconds on the clock scopes use. Exposed so a scope and the
    /// profiler cannot disagree about the time base.
    static uint64_t NowUs()
    {
        const auto now = std::chrono::steady_clock::now();
        return uint64_t(
            std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch())
                .count());
    }

private:
    uint64_t _ThreadIndexLocked(const std::thread::id &id) const
    {
        const auto inserted = _threadIds.insert({id, _threadIds.size()});
        return inserted.first->second;
    }

    /// Formats one counter sample as a JSON number: integers print
    /// without a decimal point, the way a trace reader expects queue
    /// depths to.
    static std::string _FormatCounter(double value)
    {
        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), "%.17g", value);
        return std::string(buffer);
    }

    static std::string _EscapeJson(const std::string &text)
    {
        std::string escaped;
        escaped.reserve(text.size() + 8);
        for (char c : text) {
            switch (c) {
            case '"':
                escaped += "\\\"";
                break;
            case '\\':
                escaped += "\\\\";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buffer[8];
                    std::snprintf(buffer, sizeof(buffer), "\\u%04x", c);
                    escaped += buffer;
                } else {
                    escaped += c;
                }
                break;
            }
        }
        return escaped;
    }

    mutable std::mutex _mutex;
    bool _enabled = false;
    mutable std::vector<RigExecProfileEvent> _events;
    mutable std::map<std::thread::id, uint64_t> _threadIds;
};

/// RAII scope: records one event spanning its lifetime. A null profiler (or
/// a disabled one) disarms it, so the destructor is a single branch when
/// profiling is off.
class RigExecProfileScope {
public:
    RigExecProfileScope(RigExecProfiler *profiler, std::string name,
                        std::string category = "rig")
        : _profiler(profiler && profiler->IsEnabled() ? profiler : nullptr),
          _name(std::move(name)), _category(std::move(category)),
          _startUs(_profiler ? RigExecProfiler::NowUs() : 0)
    {
    }

    ~RigExecProfileScope()
    {
        if (!_profiler) {
            return;
        }
        _profiler->Record(std::move(_name), std::move(_category), _startUs,
                          RigExecProfiler::NowUs(), std::move(_args));
    }

    RigExecProfileScope(const RigExecProfileScope &) = delete;
    RigExecProfileScope &operator=(const RigExecProfileScope &) = delete;
    RigExecProfileScope(RigExecProfileScope &&other) noexcept
        : _profiler(other._profiler), _name(std::move(other._name)),
          _category(std::move(other._category)), _startUs(other._startUs),
          _args(std::move(other._args))
    {
        other._profiler = nullptr;
    }
    RigExecProfileScope &operator=(RigExecProfileScope &&) = delete;

    void AddArg(const std::string &key, const std::string &value)
    {
        if (_profiler) {
            _args[key] = value;
        }
    }

private:
    RigExecProfiler *_profiler = nullptr;
    std::string _name;
    std::string _category;
    uint64_t _startUs = 0;
    std::map<std::string, std::string> _args;
};

/// A scope that moves: closes the interval it is holding and opens the next.
///
/// For a pass whose parts run one after another in a single block rather than
/// nested -- which is what every phase of Compile looks like. A scope per part
/// would need a brace per part, so timing them would mean re-indenting the
/// body and burying the change in whitespace; worse, a part that is a loop
/// over the same code as its neighbour cannot be braced apart at all.
///
/// `Next` names the part that STARTS there, so the marks read in the order the
/// code runs. The last part closes at destruction, and `Close` ends the run
/// early where a pass returns from the middle.
///
/// Disarmed by a null or disabled profiler, like RigExecProfileScope, so an
/// unprofiled compile pays one branch per mark.
class RigExecProfilePhases {
public:
    RigExecProfilePhases(RigExecProfiler *profiler, std::string category)
        : _profiler(profiler && profiler->IsEnabled() ? profiler : nullptr),
          _category(std::move(category))
    {
    }

    ~RigExecProfilePhases() { Close(); }

    RigExecProfilePhases(const RigExecProfilePhases &) = delete;
    RigExecProfilePhases &operator=(const RigExecProfilePhases &) = delete;

    /// Closes the open part, if any, and opens one called \p name.
    void Next(std::string name)
    {
        if (!_profiler) {
            return;
        }
        const uint64_t now = RigExecProfiler::NowUs();
        if (!_name.empty()) {
            _profiler->Record(std::move(_name), _category, _startUs, now);
        }
        _name = std::move(name);
        _startUs = now;
    }

    /// Closes the open part. Idempotent, so an early return and the
    /// destructor cannot record the same interval twice.
    void Close()
    {
        if (!_profiler || _name.empty()) {
            return;
        }
        _profiler->Record(std::move(_name), _category, _startUs,
                          RigExecProfiler::NowUs());
        _name.clear();
    }

private:
    RigExecProfiler *_profiler = nullptr;
    std::string _category;
    std::string _name;
    uint64_t _startUs = 0;
};

#define RIGEXEC_PROFILE_CONCAT_IMPL(a, b) a##b
#define RIGEXEC_PROFILE_CONCAT(a, b) RIGEXEC_PROFILE_CONCAT_IMPL(a, b)

/// Opens a profile scope named by \p nameExpr in category "rig". The name
/// expression is not evaluated at all when profiling is disabled, so call
/// sites can build names from path strings without taxing the unprofiled
/// evaluation.
#define RIGEXEC_PROFILE_SCOPE(profiler, nameExpr)                              \
    ::rigExec::RigExecProfileScope RIGEXEC_PROFILE_CONCAT(                     \
        _rigExecProfileScope_, __LINE__)(                                      \
        (profiler).IsEnabled() ? &(profiler) : nullptr,                         \
        (profiler).IsEnabled() ? std::string(nameExpr) : std::string(), "rig")

/// Opens a profile scope with an explicit category. Like
/// RIGEXEC_PROFILE_SCOPE, neither expression is evaluated when disabled.
#define RIGEXEC_PROFILE_SCOPE_CAT(profiler, nameExpr, catExpr)                  \
    ::rigExec::RigExecProfileScope RIGEXEC_PROFILE_CONCAT(                     \
        _rigExecProfileScope_, __LINE__)(                                      \
        (profiler).IsEnabled() ? &(profiler) : nullptr,                         \
        (profiler).IsEnabled() ? std::string(nameExpr) : std::string(),         \
        (profiler).IsEnabled() ? std::string(catExpr) : std::string())

}  // namespace rigExec

#endif  // RIGEXEC_PROFILER_H
