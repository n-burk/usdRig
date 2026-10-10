#include "rigExec/goldenSuite.h"
#include "rigExec/goldenPose.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>

namespace rigExec {
struct RigExecGoldenSuiteNode {
    unsigned ordinal;
    std::string rig;
    size_t generations = 0;
    std::atomic<bool> closed{false};
    RigExecGoldenSuiteNode *next = nullptr;
};
namespace {
struct Configuration {
    bool enabled = false;
    bool capture = false;
    bool rawValues = false;
    std::filesystem::path directory;
    std::string suite;
    std::string error;
};
// Process configuration is read at load, before evaluator construction.
const Configuration configuration = [] {
    Configuration result;
    const char *value = std::getenv("RIGEXEC_GOLDEN_SUITE");
    if (!value || !*value) return result;
    result.enabled = true;
    const char *raw = std::getenv("RIGEXEC_GOLDEN_SUITE_RAW");
    if (raw && *raw && std::string(raw) != "0" && std::string(raw) != "1") {
        result.error = "RIGEXEC_GOLDEN_SUITE_RAW needs 0 or 1";
        return result;
    }
    result.rawValues = raw && std::string(raw) == "1";
    const std::string text(value);
    const size_t colon = text.find(':');
    if (colon == std::string::npos || colon + 1 == text.size() ||
        (text.substr(0, colon) != "capture" && text.substr(0, colon) != "check")) {
        result.error = "RIGEXEC_GOLDEN_SUITE needs capture:<directory> or check:<directory>";
        return result;
    }
    const char *suite = std::getenv("RIGEXEC_GOLDEN_SUITE_NAME");
    if (!suite || !*suite) {
        result.error = "RIGEXEC_GOLDEN_SUITE_NAME is required";
        return result;
    }
    result.suite = suite;
    if (result.suite.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos) {
        result.error = "suite name must be one path component using letters, digits, '_' or '-'";
        return result;
    }
    result.capture = text.substr(0, colon) == "capture";
    result.directory = std::filesystem::path(text.substr(colon + 1)) / result.suite;
    return result;
}();
[[noreturn]] void Fail(unsigned ordinal, const std::string &message)
{
    std::fprintf(stderr, "golden suite %s evaluator ordinal %u: %s\n",
        configuration.suite.c_str(), ordinal, message.c_str());
    std::fflush(stderr);
    std::_Exit(EXIT_FAILURE);
}

struct SuiteIndex {
    std::atomic<unsigned> evaluators{0};
    std::atomic<unsigned> active{0};
    std::atomic<bool> finalized{false};
    std::atomic<RigExecGoldenSuiteNode *> head{nullptr};
};
static_assert(std::atomic<unsigned>::is_always_lock_free &&
              std::atomic<bool>::is_always_lock_free &&
              std::atomic<RigExecGoldenSuiteNode *>::is_always_lock_free,
              "suite registry atomics must be lock-free");
// No observer state allocation occurs when the feature is disabled.
const auto suiteIndex = configuration.enabled ? std::make_unique<SuiteIndex>() : nullptr;
SuiteIndex &Index() { return *suiteIndex; }
std::string Read(const std::filesystem::path &path, unsigned ordinal)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) Fail(ordinal, "missing original-source capture: " + path.string());
    const std::string result{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    if (input.bad()) Fail(ordinal, "cannot read " + path.string());
    return result;
}
void Write(const std::filesystem::path &path, const std::string &bytes, unsigned ordinal)
{
    std::error_code error;
    if (std::filesystem::exists(path, error)) Fail(ordinal, "capture file already exists: " + path.string());
    if (error) Fail(ordinal, "cannot inspect capture file: " + error.message());
    std::ofstream output(path, std::ios::binary);
    output.write(bytes.data(), std::streamsize(bytes.size()));
    output.close();
    if (!output) Fail(ordinal, "cannot write " + path.string());
}
void Prepare()
{
    if (!configuration.error.empty()) Fail(0, configuration.error);
    std::error_code error;
    if (configuration.capture) {
        std::filesystem::create_directories(configuration.directory, error);
        if (error) Fail(0, "cannot create directory: " + error.message());
        for (const auto &entry : std::filesystem::directory_iterator(configuration.directory, error))
            if (entry.path().extension() == ".golden")
                Fail(0, "capture directory contains an existing judge: " + entry.path().string());
        if (error) Fail(0, "cannot inspect capture directory: " + error.message());
    } else {
        // A missing index also rejects old partial captures and zero-evaluator
        // invocations; per-evaluator files alone cannot establish completeness.
        (void)Read(configuration.directory / "index.golden", 0);
    }
}
void Finalize()
{
    auto &index = Index();
    if (index.finalized.load(std::memory_order_acquire)) return;
    if (index.active.load(std::memory_order_acquire) != 0)
        Fail(0, "suite completed with active evaluators");
    const unsigned evaluators = index.evaluators.load(std::memory_order_acquire);
    std::map<unsigned, RigExecGoldenSuiteNode *> nodes;
    for (auto *node = index.head.load(std::memory_order_acquire); node; node = node->next) {
        if (!node->closed.load(std::memory_order_acquire))
            Fail(node->ordinal, "evaluator was not closed before suite completion");
        if (!nodes.emplace(node->ordinal, node).second)
            Fail(node->ordinal, "duplicate evaluator registry identity");
    }
    if (nodes.size() != evaluators) Fail(0, "evaluator registry is incomplete");
    std::string bytes = "rigexec-suite-index 1\nsuite " + RigExecGoldenEscape(configuration.suite) + "\n";
    size_t generations = 0;
    for (unsigned ordinal = 0; ordinal < evaluators; ++ordinal) {
        const auto found = nodes.find(ordinal);
        if (found == nodes.end()) Fail(ordinal, "missing evaluator completion");
        const auto &node = *found->second;
        generations += node.generations;
        bytes += "evaluator " + std::to_string(ordinal) + " rig " + RigExecGoldenEscape(node.rig) +
            " generations " + std::to_string(node.generations) + "\n";
    }
    bytes += "end evaluators " + std::to_string(evaluators) +
        " generations " + std::to_string(generations) + "\n";
    const auto path = configuration.directory / "index.golden";
    if (configuration.capture) Write(path, bytes, 0);
    else {
        std::string error;
        if (!RigExecCompareGolden(Read(path, 0), bytes, &error)) Fail(0, "suite index: " + error);
    }
    std::set<std::string> files{"index.golden"};
    for (unsigned ordinal = 0; ordinal < evaluators; ++ordinal)
        files.insert(std::to_string(ordinal) + ".golden");
    std::error_code error;
    for (const auto &entry : std::filesystem::directory_iterator(configuration.directory, error)) {
        if (entry.path().extension() != ".golden") continue;
        if (!entry.is_regular_file(error) || !files.erase(entry.path().filename().string()))
            Fail(0, "unexpected suite capture: " + entry.path().string());
        if (error) Fail(0, "cannot inspect suite capture: " + error.message());
    }
    if (error) Fail(0, "cannot list suite capture directory: " + error.message());
    if (!files.empty()) Fail(0, "suite capture inventory is incomplete: " + *files.begin());
    for (const auto &entry : nodes) delete entry.second;
    index.head.store(nullptr, std::memory_order_relaxed);
    index.finalized.store(true, std::memory_order_release);
}
const bool registered = [] {
    if (configuration.enabled) {
        Prepare();
        if (std::atexit(Finalize) != 0) Fail(0, "cannot register suite completion validation");
    }
    return true;
}();
}

class RigExecGoldenSuiteRecorder : public RigExecGoldenSuiteObserver {
public:
    RigExecGoldenSuiteRecorder(const SdfPath &rigPath, unsigned ordinal)
        : _ordinal(ordinal)
    {
        _node = new RigExecGoldenSuiteNode{ordinal, rigPath.GetString()};
        auto &registry = Index().head;
        auto *head = registry.load(std::memory_order_relaxed);
        do { _node->next = head; }
        while (!registry.compare_exchange_weak(head, _node, std::memory_order_release, std::memory_order_relaxed));
        _bytes = "rigexec-suite-golden 1\nsuite " + RigExecGoldenEscape(configuration.suite) +
            " evaluator " + std::to_string(ordinal) + " rig " + RigExecGoldenEscape(rigPath.GetString()) +
            "\nencoding raw-bits " + (configuration.rawValues ? "full-values" : "domain-digests") +
            " work-metadata=excluded\n";
    }
    void Record(const RigExecRigPose &pose) override
    {
        std::vector<RigExecGoldenValue> values;
        std::string error;
        if (!RigExecEncodeGoldenPose(pose, &values, &error)) Fail(_ordinal, error);
        // Generation order and times are recorded in this evaluator's private
        // buffer. Independent evaluator interleavings do not affect the verdict.
        _bytes += RigExecGoldenVisit("generation", _generation++, pose, values, !configuration.rawValues);
    }
    ~RigExecGoldenSuiteRecorder() override
    {
        const auto path = configuration.directory / (std::to_string(_ordinal) + ".golden");
        if (configuration.capture) Write(path, _bytes, _ordinal);
        else {
            std::string error;
            if (!RigExecCompareGolden(Read(path, _ordinal), _bytes, &error)) Fail(_ordinal, error);
        }
        _node->generations = _generation;
        _node->closed.store(true, std::memory_order_release);
        Index().active.fetch_sub(1, std::memory_order_release);
    }
private:
    RigExecGoldenSuiteNode *_node = nullptr;
    unsigned _ordinal = 0;
    size_t _generation = 0;
    std::string _bytes;
};

void RigExecOracleFinalizeGolden()
{
    if (configuration.enabled) Finalize();
}

std::unique_ptr<RigExecGoldenSuiteObserver>
RigExecOracleCreateGolden(const SdfPath &rigPath)
{
    if (!configuration.enabled) return nullptr;
    Index().active.fetch_add(1, std::memory_order_relaxed);
    const unsigned ordinal = Index().evaluators.fetch_add(1, std::memory_order_relaxed);
    return std::unique_ptr<RigExecGoldenSuiteObserver>(new RigExecGoldenSuiteRecorder(rigPath, ordinal));
}
}
