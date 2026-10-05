// rigExecRuntime loader conformance: bake every baking fixture in-process
// at the probe time, open the bytes with the zero-USD reader, and check
// every step's label as the reader names it in error text against the
// program's, and malformed-input refusal, including a truncated file, the
// old container, another file identifier or format version, a step graph
// with a predecessor after its step, step and cluster indices past their
// tables, a cluster graph with a cycle, and static tables that do not match
// the program. Then: a fresh reader's defaults against a fresh evaluator at
// the bake time (every output, the diagnostics and the counters), twice,
// and the input API -- the inputs' names, lookup and defaults, the table's
// control avar and operator input found as inputs, refusals, a set that
// moves the outputs, a reset, and token text the file lacks or holds as
// the empty token. Last, on a bake at the table's first frame:
// the input sampler drives a fresh reader along the table's frames (an
// `inputs` row) or at the bake time (a `static` row), each run against a
// fresh evaluator's generation at that time, counters and summary line
// included.
#include "rigExecBake/bake.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/format.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++failures;                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
        }                                                               \
    } while (0)

#include "rigExecFileEdit.h"
#include "rigExecRuntimeDrive.h"

static SdfPath
_FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

static std::vector<double>
_ParseFrames(const std::string &text)
{
    std::vector<double> frames;
    size_t begin = 0;
    while (begin <= text.size()) {
        const size_t end = text.find(",", begin);
        const std::string piece =
            text.substr(begin, end == std::string::npos
                        ? std::string::npos : end - begin);
        if (!piece.empty()) {
            frames.push_back(std::stod(piece));
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return frames;
}

static void
_TestMalformed()
{
    std::string error;
    // Empty input.
    CHECK(!RigExecRuntimeReader::Open(nullptr, 0, &error));
    CHECK(!error.empty());
    // Truncated garbage.
    const std::vector<uint8_t> garbage = {'R', 'E', 'X', 'B', 1, 2, 3};
    CHECK(!RigExecRuntimeReader::Open(garbage.data(), garbage.size(),
                                      &error));
    CHECK(!error.empty());
    // Wrong magic, plausible size.
    const std::vector<uint8_t> wrong(256, 0);
    CHECK(!RigExecRuntimeReader::Open(wrong.data(), wrong.size(), &error));
    CHECK(!error.empty());
    // The old sectioned container's magic leads the file.
    std::vector<uint8_t> old(64, 0);
    old[0] = 'R';
    old[1] = 'E';
    old[2] = 'X';
    old[3] = 'B';
    old[4] = 3;
    old[6] = 3;
    error.clear();
    CHECK(!RigExecRuntimeReader::Open(old.data(), old.size(), &error));
    CHECK(error == "not a v4 .rigexec (old REXB container); rebake");
}

// What Open says about \p bytes, or "(opened)".
static std::string
_OpenError(const std::vector<uint8_t> &bytes)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    return reader ? std::string("(opened)") : error;
}

static int fileRefusalRows = 0;

// A real bake with another file identifier, and with another format
// version, each refused with its exact message.
static void
_TestFileRefusals(const std::vector<uint8_t> &bytes)
{
    std::vector<uint8_t> identifier = bytes;
    CHECK(identifier.size() > 8);
    if (identifier.size() <= 8) {
        return;
    }
    identifier[4] = 'R';
    identifier[5] = 'I';
    identifier[6] = 'G';
    identifier[7] = 'X';
    std::string got = _OpenError(identifier);
    CHECK(got == "not a .rigexec file (file identifier is not REXB)");
    const std::vector<uint8_t> version =
        RigExecTestEdited(bytes, [](fb::RigExecWireFile *file) {
            file->formatVersion = 4;
        });
    got = _OpenError(version);
    CHECK(got == "unsupported .rigexec format version 4 (this reader reads " +
                     std::to_string(RigExecFormatVersion) + "); rebake");
    if (got.find("format version 4") == std::string::npos) {
        std::printf("version 4: open said '%s'\n", got.c_str());
    }
    ++fileRefusalRows;
}

static int stepGraphFlips = 0;
static int stepGraphRanges = 0;
static int stepGraphCycles = 0;
static int stepGraphProducers = 0;

// Open refuses \p bytes changed by \p edit, saying exactly \p expected
// after the validator's prefix.
template <class Edit>
static void
_ExpectRefusal(const std::string &name, const char *what,
               const std::vector<uint8_t> &bytes, const Edit &edit,
               const std::string &expected)
{
    const std::string want = "invalid .rigexec: " + expected;
    const std::string got = _OpenError(RigExecTestEdited(bytes, edit));
    CHECK(got == want);
    if (got != want) {
        std::printf("%s, %s: open said '%s', expected '%s'\n", name.c_str(),
                    what, got.c_str(), want.c_str());
    }
}

// Step and cluster graphs playback could not walk, edited into a fixture's
// bake, each refused by Open with the step-graph check's exact message:
// the first step with a predecessor names the last step instead; indices
// past the end in a step's predecessors, a step's cluster, a cluster's
// members and a cluster's predecessors, and a clustering that places one
// step too few; and two clusters joined by an edge given the reverse edge
// too.
static void
_TestStepGraphRefusals(const std::string &name,
                       const std::vector<uint8_t> &bytes)
{
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    const std::vector<fb::RigExecWireStep> &steps = file->steps;
    const fb::RigExecWireClustering &clustering = *file->clustering;
    const size_t count = steps.size();
    const auto &clusters = clustering.clusters;
    const std::string stepCount = std::to_string(count);
    const std::string clusterCount = std::to_string(clusters.size());
    for (size_t s = 0; s + 1 < count; ++s) {
        if (steps[s].preds.empty()) {
            continue;
        }
        _ExpectRefusal(name, "flipped predecessor", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->steps[s].preds[0] = int32_t(count - 1);
                       },
                       "step " + std::to_string(s) +
                           " depends on later step " +
                           std::to_string(count - 1));
        _ExpectRefusal(name, "predecessor past the steps", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->steps[s].preds[0] = int32_t(count);
                       },
                       "step " + std::to_string(s) + " names predecessor " +
                           stepCount + ", which is no step");
        ++stepGraphFlips;
        ++stepGraphRanges;
        break;
    }
    // A read past every slot, in a pose domain and in Aggregate, which
    // the blend reads from the same run: nothing writes either.
    if (count > 0) {
        const uint32_t past = uint32_t(1) << 30;
        const std::string last = std::to_string(count - 1);
        const std::pair<fb::SlotDomain, const char *> domains[] = {
            {fb::SlotDomain::PosedM, "PosedM"},
            {fb::SlotDomain::Aggregate, "Aggregate"}};
        for (const auto &[domain, text] : domains) {
            const fb::SlotDomain read = domain;
            _ExpectRefusal(name, "read nothing writes", bytes,
                           [&](fb::RigExecWireFile *edited) {
                               edited->steps.back().reads.push_back(
                                   fb::SlotRange(read, past, past + 1));
                           },
                           "step " + last + " reads " + text + " slots [" +
                               std::to_string(past) + ", " +
                               std::to_string(past + 1) +
                               "), which no earlier step writes");
            ++stepGraphProducers;
        }
    }
    if (count > 0 && !clusters.empty()) {
        _ExpectRefusal(name, "cluster past the clusters", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->steps[0].cluster =
                               int32_t(clusters.size());
                       },
                       "step 0 names cluster " + clusterCount +
                           ", which is no cluster");
        _ExpectRefusal(name, "member past the steps", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->clustering->clusters[0].members.push_back(
                               int32_t(count));
                       },
                       "cluster 0 names member " + stepCount +
                           ", which is no step");
        _ExpectRefusal(name, "cluster predecessor past the clusters", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->clustering->clusters[0].preds.push_back(
                               int32_t(clusters.size()));
                       },
                       "cluster 0 names predecessor " + clusterCount +
                           ", which is no cluster");
        _ExpectRefusal(name, "clustering one step short", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->clustering->clusterOf.pop_back();
                       },
                       "the clustering places " + std::to_string(count - 1) +
                           " steps; the file has " + stepCount);
        stepGraphRanges += 4;
    }
    // Every cluster \p from reaches along succs, itself included.
    const auto reached = [&](int32_t from) {
        std::vector<char> seen(clusters.size(), 0);
        std::vector<int32_t> stack = {from};
        seen[size_t(from)] = 1;
        while (!stack.empty()) {
            const int32_t at = stack.back();
            stack.pop_back();
            for (const int32_t next : clusters[size_t(at)].succs) {
                if (!seen[size_t(next)]) {
                    seen[size_t(next)] = 1;
                    stack.push_back(next);
                }
            }
        }
        return seen;
    };
    // The edge a -> b to reverse: a reaches no other predecessor of b, so
    // the only cycle is a and b, and a is the lowest cluster it reaches.
    // The sort then leaves waiting exactly the clusters a reaches; a is the
    // lowest of them, its one waiting predecessor is b, and b's is a, so
    // the walk from the lowest waiting cluster names a.
    for (size_t b = 0; b < clusters.size(); ++b) {
        for (const int32_t a : clusters[b].preds) {
            const std::vector<char> seen = reached(a);
            const bool otherPath = std::any_of(
                clusters[b].preds.begin(), clusters[b].preds.end(),
                [&](int32_t pred) { return pred != a && seen[size_t(pred)]; });
            const bool lowest =
                std::find(seen.begin(), seen.begin() + a, 1) ==
                seen.begin() + a;
            if (otherPath || !lowest) {
                continue;
            }
            _ExpectRefusal(
                name, "two-cluster cycle", bytes,
                [&](fb::RigExecWireFile *edited) {
                    std::vector<int32_t> &preds =
                        edited->clustering->clusters[size_t(a)].preds;
                    preds.insert(std::lower_bound(preds.begin(), preds.end(),
                                                  int32_t(b)),
                                 int32_t(b));
                    std::vector<int32_t> &succs =
                        edited->clustering->clusters[b].succs;
                    succs.insert(
                        std::lower_bound(succs.begin(), succs.end(), a), a);
                },
                "the cluster graph has a cycle through cluster " +
                    std::to_string(a));
            ++stepGraphCycles;
            return;
        }
    }
}

static int staticTableRows = 0;
static int staticChainRows = 0;
static int staticArrayRows = 0;

// Open refuses static tables that do not match the program, each with the
// validator's exact message naming the field: an xform base past the
// xform slots, a delta-base flag past the delta bases, a chain base
// without its have_base flag, and a constraint's array weights of another
// count than its sources.
static void
_TestStaticTableRefusals(const std::string &name,
                         const std::vector<uint8_t> &bytes)
{
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    const size_t xforms = file->pose->xformBase.size();
    _ExpectRefusal(name, "xform base past the slots", bytes,
                   [](fb::RigExecWireFile *edited) {
                       edited->pose->xformBase.push_back(
                           RigExecWireMatrix4d{});
                   },
                   "pose.xform_base: " + std::to_string(xforms + 1) +
                       " entries, expected " + std::to_string(xforms));
    const size_t deltas = file->geometry->deltaBasePaths.size();
    _ExpectRefusal(name, "delta-base flag past the delta bases", bytes,
                   [](fb::RigExecWireFile *edited) {
                       edited->geometry->deltaBaseOk.push_back(0);
                   },
                   "geometry.delta_base_ok: " + std::to_string(deltas + 1) +
                       " entries, expected " + std::to_string(deltas));
    ++staticTableRows;
    // A chain holding base points, which keeps them without the flag.
    const auto &chains = file->geometry->chains;
    for (size_t c = 0; c < chains.size(); ++c) {
        if (!chains[c].haveBase || chains[c].base == 0) {
            continue;
        }
        _ExpectRefusal(name, "chain base without have_base", bytes,
                       [c](fb::RigExecWireFile *edited) {
                           edited->geometry->chains[c].haveBase = false;
                       },
                       "geometry.chains[" + std::to_string(c) +
                           "]: a base without have_base");
        ++staticChainRows;
        break;
    }
    // A constraint's arrays that resolved, one weight more than sources.
    const auto &arrays = file->pose->constraintArrays;
    for (size_t k = 0; k < arrays.size(); ++k) {
        if (!arrays[k].ok) {
            continue;
        }
        const uint64_t sources = arrays[k].sourceCount;
        _ExpectRefusal(name, "array weights past the sources", bytes,
                       [k](fb::RigExecWireFile *edited) {
                           edited->pose->constraintArrays[k].weights.push_back(
                               1.0);
                       },
                       "pose.constraint_arrays[" + std::to_string(k) +
                           "].weights: " + std::to_string(sources + 1) +
                           " entries, expected " + std::to_string(sources));
        ++staticArrayRows;
        break;
    }
}

static int stepLabelRows = 0;

// Every step's label, as the reader builds it from the tables for error
// text, is the live program's label; a step past the steps is its number.
static void
_TestStepLabels(const RigExecExampleFixture &fixture,
                const RigExecRigEvaluator &evaluator,
                const std::vector<uint8_t> &bytes)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    const RigExecBakedProgram *baked = evaluator.GetBakedProgram();
    CHECK(reader);
    CHECK(baked);
    if (!reader || !baked) {
        return;
    }
    const RigExecBakedProgramImpl &program = baked->GetStepGraph();
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    CHECK(file && file->steps.size() == program.steps.size());
    size_t matched = 0;
    size_t printed = 0;
    for (size_t i = 0; i < program.steps.size(); ++i) {
        const std::string label = reader->GetStepLabelForTesting(i);
        const bool same = label == program.steps[i].label;
        CHECK(same);
        if (same) {
            ++matched;
        } else if (printed++ < 5) {
            std::printf("%s step %zu: label '%s', program '%s'\n",
                        fixture.stage, i, label.c_str(),
                        program.steps[i].label.c_str());
        }
    }
    const size_t count = program.steps.size();
    CHECK(reader->GetStepLabelForTesting(count) == std::to_string(count));
    std::printf("step labels %s: %zu of %zu match\n", fixture.stage, matched,
                count);
    if (matched == count) {
        ++stepLabelRows;
    }
}

// A fresh reader executes its defaults: Execute has no precondition, and
// the run publishes its generation's lines.
static void
_TestFreshExecute(const std::vector<uint8_t> &bytes)
{
    std::string error;
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("open diagnostic: %s\n", error.c_str());
        return;
    }
    CHECK(reader->Execute(&error));
    if (!error.empty()) {
        std::printf("fresh execute: %s\n", error.c_str());
    }
    CHECK(!reader->GetDiagnostics().empty());
}

template <class T>
static bool
_SameBits(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

template <class T>
static bool
_SameBits(const std::vector<T> &a, const std::vector<T> &b)
{
    return a.size() == b.size() &&
           (a.empty() ||
            std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0);
}

// Bitwise equality on the member the tag names.
static bool
_SameInput(const RrInputValue &a, const RrInputValue &b)
{
    if (a.tag != b.tag) {
        return false;
    }
    switch (a.tag) {
    case RrInputTag::Double:
        return _SameBits(a.f64, b.f64);
    case RrInputTag::Float:
        return _SameBits(a.f32, b.f32);
    case RrInputTag::Bool:
        return a.boolean == b.boolean;
    case RrInputTag::Int:
        return a.i32 == b.i32;
    case RrInputTag::Matrix4d:
        return _SameBits(a.matrix, b.matrix);
    case RrInputTag::Token:
        return a.token == b.token;
    case RrInputTag::Vec3d:
        return _SameBits(a.vec, b.vec);
    case RrInputTag::Vec3f:
        return _SameBits(a.vec3f, b.vec3f);
    }
    return false;
}

static bool
_SameProperty(const RrPropertyValue &a, const RrPropertyValue &b)
{
    if (a.tag != b.tag) {
        return false;
    }
    switch (a.tag) {
    case RrPropertyValue::Tag::Float:
        return _SameBits(a.f32, b.f32);
    case RrPropertyValue::Tag::Double:
        return _SameBits(a.f64, b.f64);
    case RrPropertyValue::Tag::Matrix4d:
        return _SameBits(a.matrix, b.matrix);
    case RrPropertyValue::Tag::Vec3f:
        return _SameBits(a.vec, b.vec);
    }
    return false;
}

// The first output domain two readers' last runs differ in, bit for bit,
// or "": joints, points, matrix primvars, weight frames and fields,
// provider transforms and property values, and with \p everything the
// diagnostics and counters too.
static std::string
_CompareRuns(const RigExecRuntimeReader &a, const RigExecRuntimeReader &b,
             bool everything)
{
    {
        const auto &x = a.GetJointMatrices();
        const auto &y = b.GetJointMatrices();
        if (x.size() != y.size()) {
            return "joint matrix count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].matrix, y[i].matrix)) {
                return "joint matrix " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetPoints();
        const auto &y = b.GetPoints();
        if (x.size() != y.size()) {
            return "points count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].points, y[i].points)) {
                return "points " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetMatrixPrimvars();
        const auto &y = b.GetMatrixPrimvars();
        if (x.size() != y.size()) {
            return "matrix primvar count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].matrix, y[i].matrix)) {
                return "matrix primvar " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetWeightFrames();
        const auto &y = b.GetWeightFrames();
        if (x.size() != y.size()) {
            return "weight frame count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].matrix, y[i].matrix)) {
                return "weight frame " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetWeightFields();
        const auto &y = b.GetWeightFields();
        if (x.size() != y.size()) {
            return "weight field count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || x[i].target != y[i].target ||
                !_SameBits(x[i].weights, y[i].weights)) {
                return "weight field " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetProviderXforms();
        const auto &y = b.GetProviderXforms();
        if (x.size() != y.size()) {
            return "provider transform count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].matrix, y[i].matrix) ||
                !_SameBits(x[i].base, y[i].base)) {
                return "provider transform " + x[i].path;
            }
        }
    }
    {
        const auto x = a.GetPropertyValues();
        const auto y = b.GetPropertyValues();
        if (x.size() != y.size()) {
            return "property value count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameProperty(x[i].value, y[i].value)) {
                return "property value " + x[i].path;
            }
        }
    }
    if (!everything) {
        return std::string();
    }
    if (a.GetDiagnostics() != b.GetDiagnostics()) {
        return "diagnostics";
    }
    const RigExecRuntimeCounters p = a.GetCounters();
    const RigExecRuntimeCounters q = b.GetCounters();
    if (p.revisionsExecuted != q.revisionsExecuted ||
        p.revisionsCreated != q.revisionsCreated ||
        p.schedulesBuilt != q.schedulesBuilt ||
        p.chainsBuilt != q.chainsBuilt ||
        p.revisionsBuilt != q.revisionsBuilt) {
        return "counters";
    }
    return std::string();
}

static int inputApiRows = 0;
static int inputApiFields = 0;
static int inputApiMoved = 0;
static int inputApiResets = 0;
static int inputApiTokens = 0;

static int defaultsRows = 0;

// A fresh reader executes the defaults, and they are the bake: a fresh
// evaluator's generation at the bake time (no explicit Compile, so its
// compile notices land in that generation, where the reader replays them)
// matches the reader's first run in every output, the diagnostics and the
// work counters. A second run with nothing set matches the evaluator's
// second generation at the same time, counters included.
static void
_TestDefaults(const RigExecExampleFixture &fixture,
              const UsdStageRefPtr &stage, const SdfPath &rigPath,
              const std::vector<uint8_t> &bytes)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("%s: open: %s\n", fixture.stage, error.c_str());
        return;
    }
    const double t = reader->GetBakeTime();
    CHECK(t == RigExecBakedProbeTime(stage).GetValue());
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
    CHECK(pose.valid);
    CHECK(evaluator.GetBakedGenerationCount() == 1);
    CHECK(reader->Execute(&error));
    if (!error.empty()) {
        std::printf("%s: defaults execute: %s\n", fixture.stage,
                    error.c_str());
    }
    std::vector<std::string> diffs;
    bool same = RigExecCompareRuntime(pose, *reader, true, {}, &diffs);
    CHECK(same);
    {
        const RigExecRigPose again = evaluator.Evaluate(UsdTimeCode(t));
        CHECK(again.valid);
        CHECK(reader->Execute(&error));
        const bool sameAgain =
            RigExecCompareRuntime(again, *reader, true, {}, &diffs);
        CHECK(sameAgain);
        if (!sameAgain) {
            std::printf("defaults %s t=%g: the second run differs\n",
                        fixture.stage, t);
        }
        same = same && sameAgain;
    }
    if (same) {
        std::printf("defaults %s t=%g: == Evaluate(T), twice (%zu "
                    "joints, %zu moved, %zu weight frames, %zu fields, %zu "
                    "provider xforms, %zu diagnostics, counters "
                    "%zu/%zu/%zu)\n",
                    fixture.stage, t, reader->GetJointMatrices().size(),
                    reader->GetPoints().size(),
                    reader->GetWeightFrames().size(),
                    reader->GetWeightFields().size(),
                    reader->GetProviderXforms().size(),
                    reader->GetDiagnostics().size(),
                    size_t(reader->GetCounters().revisionsExecuted),
                    size_t(reader->GetCounters().revisionsCreated),
                    size_t(reader->GetCounters().schedulesBuilt));
        ++defaultsRows;
    } else {
        std::printf("defaults %s t=%g: MISMATCH (%zu differences)\n",
                    fixture.stage, t, diffs.size());
        for (const std::string &line : diffs) {
            std::printf("    %s\n", line.c_str());
        }
    }
}

static int samplerRows = 0;
static int samplerFrames = 0;
static int samplerUnanimatedRows = 0;

// Playback along the stage's timeline: the sampler hands a fresh reader the
// stage's values of its Animated inputs at each time, and each run is a
// fresh evaluator's generation at that time in every output, the
// diagnostics (summary line included) and the work counters. A row whose
// file has no Animated input still moves the counters with time: the
// sampler's TouchAnimatedInputs is what dirties the steps a change of time
// dirties in the program.
static void
_TestSamplerDrive(const RigExecExampleFixture &fixture,
                  const UsdStageRefPtr &stage, const SdfPath &rigPath,
                  const std::vector<uint8_t> &bytes,
                  const std::vector<double> &frames)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("%s: open: %s\n", fixture.stage, error.c_str());
        return;
    }
    CHECK(reader->GetBakeTime() == frames.front());
    RigExecInputSampler sampler;
    CHECK(sampler.Bind(stage, *reader, &error));
    CHECK(sampler.GetWarnings().empty());
    for (const std::string &warning : sampler.GetWarnings()) {
        std::printf("%s: sampler: %s\n", fixture.stage, warning.c_str());
    }
    const bool isStatic = std::string(fixture.animation) == "static";
    const std::vector<double> times =
        isStatic ? std::vector<double>{frames.front()} : frames;
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    size_t matched = 0;
    for (const double t : times) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
        CHECK(pose.valid);
        const bool ran = RigExecTestDrive(reader.get(), &sampler, t, &error);
        CHECK(ran);
        if (!ran) {
            std::printf("%s t=%g: drive: %s\n", fixture.stage, t,
                        error.c_str());
            continue;
        }
        std::vector<std::string> diffs;
        const bool same =
            RigExecCompareRuntime(pose, *reader, true, {}, &diffs);
        CHECK(same);
        if (same) {
            ++matched;
        } else {
            std::printf("sampler %s t=%g: MISMATCH (%zu differences)\n",
                        fixture.stage, t, diffs.size());
            for (const std::string &line : diffs) {
                std::printf("    %s\n", line.c_str());
            }
        }
    }
    CHECK(evaluator.GetBakedGenerationCount() == times.size());
    std::printf("sampler %s: %zu of %zu frame(s) driven == Evaluate(t) "
                "(%s; %zu input(s), %zu sampled per frame)\n",
                fixture.stage, matched, times.size(), fixture.animation,
                reader->GetInputCount(), sampler.GetAnimatedCount());
    if (matched == times.size()) {
        ++samplerRows;
        if (!isStatic && times.size() > 1 &&
            sampler.GetAnimatedCount() == 0) {
            ++samplerUnanimatedRows;
        }
    }
    samplerFrames += int(matched);
}

// The input API over a bake at the probe time, whose slot values are the
// inputs' defaults.
static void
_TestInputApi(const RigExecExampleFixture &fixture,
              const UsdStageRefPtr &stage, const std::vector<uint8_t> &bytes)
{
    const double probe = RigExecBakedProbeTime(stage).GetValue();
    std::string error;
    const auto open = [&] {
        std::string why;
        std::unique_ptr<RigExecRuntimeReader> reader =
            RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &why);
        CHECK(reader);
        if (!reader) {
            std::printf("%s: open: %s\n", fixture.stage, why.c_str());
        }
        return reader;
    };
    const std::unique_ptr<RigExecRuntimeReader> fresh = open();
    if (!fresh) {
        return;
    }
    CHECK(fresh->GetBakeTime() == probe);

    // Names unique and sorted, each found at its own index, and a fresh
    // reader holding every default.
    const size_t count = fresh->GetInputCount();
    CHECK(count > 0);
    size_t animated = 0;
    for (size_t i = 0; i < count; ++i) {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(i);
        CHECK(!info.name.empty());
        if (i > 0) {
            CHECK(fresh->GetInputInfo(i - 1).name < info.name);
        }
        size_t at = count;
        CHECK(fresh->FindInput(info.name, &at) && at == i);
        CHECK(_SameInput(fresh->GetInputValue(i), info.defaultValue));
        CHECK(fresh->GetInputValue(i).tag == info.type);
        animated += info.animated ? 1 : 0;
    }
    CHECK(!fresh->FindInput("/RigExecTest/NoSuch.attr", nullptr));
    CHECK(fresh->GetInputInfo(count).name.empty());

    // Refusals change nothing: an unknown name, a value of another type,
    // non-finite values, an index past the count.
    std::string why;
    CHECK(!fresh->SetInput("/RigExecTest/NoSuch.attr", 1.0, &why));
    CHECK(why == "no input named /RigExecTest/NoSuch.attr");
    {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(0);
        RrInputValue wrong = info.defaultValue;
        wrong.tag = info.type == RrInputTag::Bool ? RrInputTag::Int
                                                  : RrInputTag::Bool;
        why.clear();
        CHECK(!fresh->SetInputAt(0, wrong, &why));
        CHECK(!why.empty());
        CHECK(_SameInput(fresh->GetInputValue(0), info.defaultValue));
    }
    for (size_t i = 0; i < count; ++i) {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(i);
        if (info.type != RrInputTag::Double &&
            info.type != RrInputTag::Float) {
            continue;
        }
        // A Float input also refuses a finite double that narrows to inf.
        std::vector<double> bad = {std::numeric_limits<double>::quiet_NaN(),
                                   -std::numeric_limits<double>::infinity()};
        if (info.type == RrInputTag::Float) {
            bad.push_back(1e300);
        }
        for (const double value : bad) {
            why.clear();
            CHECK(!fresh->SetInput(info.name, value, &why));
            CHECK(why == info.name + " takes finite values only");
        }
        CHECK(_SameInput(fresh->GetInputValue(i), info.defaultValue));
        break;
    }
    why.clear();
    CHECK(!fresh->SetInputAt(count, fresh->GetInputInfo(0).defaultValue,
                             &why));
    CHECK(why == "no input at index " + std::to_string(count) +
                     "; the file lists " + std::to_string(count));

    // The table's control avar and operator input are inputs of the file:
    // the fields every drag of the example lands on. The first is set to
    // its default + 0.25 and the run moves the outputs or not (it moves
    // them on most rows); reset, the run plays the defaults again.
    CHECK(fresh->Execute(&error));
    std::vector<std::string> fields;
    if (fixture.controlPrim[0] != '\0') {
        fields.push_back(std::string(fixture.controlPrim) + "." +
                         fixture.controlAvar);
    }
    if (fixture.operatorPrim[0] != '\0') {
        fields.push_back(std::string(fixture.operatorPrim) + "." +
                         fixture.operatorInput);
    }
    for (const std::string &field : fields) {
        size_t at = count;
        const bool found = fresh->FindInput(field, &at);
        CHECK(found);
        if (found) {
            ++inputApiFields;
        } else {
            std::printf("%s: %s is no input\n", fixture.stage,
                        field.c_str());
        }
    }
    const std::string dragName = fields.empty() ? std::string() : fields[0];
    size_t drag = count;
    if (!dragName.empty()) {
        fresh->FindInput(dragName, &drag);
    }
    std::string moveNote = "n/a";
    std::string resetNote = "n/a";
    if (drag < count) {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(drag);
        const double base = info.type == RrInputTag::Float
                                ? double(info.defaultValue.f32)
                                : info.defaultValue.f64;
        const double value = base + 0.25;
        const std::unique_ptr<RigExecRuntimeReader> input = open();
        if (!input) {
            return;
        }
        CHECK(input->SetInput(dragName, value, &why));
        CHECK(input->Execute(&error));
        const std::string moved = _CompareRuns(*input, *fresh, false);
        if (!moved.empty()) {
            ++inputApiMoved;
            moveNote = "moves " + moved;
        } else {
            moveNote = "outputs unmoved";
        }
        input->ResetInputs();
        CHECK(_SameInput(input->GetInputValue(drag), info.defaultValue));
        CHECK(input->Execute(&error));
        const std::string diff = _CompareRuns(*input, *fresh, false);
        CHECK(diff.empty());
        if (!diff.empty()) {
            std::printf("%s: reset run and fresh run differ at %s\n",
                        fixture.stage, diff.c_str());
        }
        resetNote = "reset == fresh";
        ++inputApiResets;
    }

    // Token text the file lacks is interned once, past the file's path
    // ids, and round-trips; known text resolves to the file's own id, the
    // empty token to id 0.
    std::string tokenNote = "n/a";
    const std::unique_ptr<fb::RigExecWireFile> unpacked =
        RigExecTestUnpack(bytes);
    const size_t paths = unpacked ? unpacked->paths.size() : 0;
    for (size_t i = 0; i < count; ++i) {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(i);
        if (info.type != RrInputTag::Token) {
            continue;
        }
        const std::unique_ptr<RigExecRuntimeReader> tokens = open();
        if (!tokens) {
            return;
        }
        const std::string text = "rigExecTestUnlistedToken";
        CHECK(tokens->SetInputToken(info.name, text, &why));
        const uint32_t id = tokens->GetInputValue(i).token;
        CHECK(tokens->GetInputValue(i).tag == RrInputTag::Token);
        CHECK(paths > 0 && id >= paths);
        CHECK(tokens->GetTokenText(id) == text);
        CHECK(tokens->SetInputToken(info.name, text, &why));
        CHECK(tokens->GetInputValue(i).token == id);
        const std::string known =
            tokens->GetTokenText(info.defaultValue.token);
        CHECK(tokens->SetInputToken(info.name, known, &why));
        CHECK(tokens->GetInputValue(i).token < paths);
        CHECK(tokens->GetTokenText(tokens->GetInputValue(i).token) == known);
        CHECK(tokens->SetInputToken(info.name, "", &why));
        CHECK(tokens->GetInputValue(i).token == 0);
        CHECK(tokens->GetTokenText(0).empty());
        CHECK(tokens->SetInputToken(info.name, text, &why));
        CHECK(tokens->Execute(&error));
        tokenNote = info.name;
        ++inputApiTokens;
        break;
    }
    if (tokenNote == "n/a" && count > 0) {
        // No Token input: SetInputToken refuses every input as not a token.
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(0);
        why.clear();
        CHECK(!fresh->SetInputToken(info.name, "x", &why));
        CHECK(!why.empty());
    }
    std::printf("input api %s: %zu input(s), %zu animated; defaults, "
                "lookup and refusals ok; %zu table field(s) found; %s set: "
                "%s; %s; token: %s\n",
                fixture.stage, count, animated, fields.size(),
                dragName.empty() ? "-" : dragName.c_str(), moveNote.c_str(),
                resetNote.c_str(), tokenNote.c_str());
    ++inputApiRows;
}

static void
_TestFixture(const RigExecExampleFixture &fixture,
             const std::string &stagePath,
             const std::vector<double> &bakeFrames)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = _FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::string error;

    // One record at the probe time: the defaults, then the input API.
    std::vector<uint8_t> atProbe;
    CHECK(RigExecTestBakeAt(evaluator,
                            std::numeric_limits<double>::quiet_NaN(),
                            &atProbe, &error));
    if (atProbe.empty()) {
        std::printf("%s: probe bake: %s\n", fixture.stage, error.c_str());
        return;
    }
    // The bake writes a file the format's validator accepts, so Open
    // takes it.
    CHECK(RigExecRuntimeReader::Open(atProbe.data(), atProbe.size(),
                                     &error));

    _TestStepLabels(fixture, evaluator, atProbe);
    _TestStepGraphRefusals(stagePath, atProbe);
    _TestStaticTableRefusals(stagePath, atProbe);

    // Truncating the tail breaks the buffer, so Open refuses.
    if (atProbe.size() > 64) {
        std::string truncError;
        CHECK(!RigExecRuntimeReader::Open(
            atProbe.data(), atProbe.size() - 32, &truncError));
        CHECK(!truncError.empty());
    }

    _TestDefaults(fixture, stage, rigPath, atProbe);
    _TestInputApi(fixture, stage, atProbe);

    // One record at the first table frame, played along the table.
    std::vector<uint8_t> atFirst;
    if (RigExecBakedProbeTime(stage).GetValue() == bakeFrames.front()) {
        atFirst = atProbe;
    } else {
        CHECK(RigExecTestBakeAt(evaluator, bakeFrames.front(), &atFirst,
                                &error));
    }
    if (atFirst.empty()) {
        std::printf("%s: first-frame bake: %s\n", fixture.stage,
                    error.c_str());
        return;
    }
    _TestSamplerDrive(fixture, stage, rigPath, atFirst, bakeFrames);
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    _TestMalformed();

    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }
    bool sawBaking = false;
    bool sawBytes = false;
    int bakingRows = 0;
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            continue;
        }
        sawBaking = true;
        ++bakingRows;
        const std::string stagePath =
            examplesDir + "/" + fixture.stage;
        const std::vector<double> frames = _ParseFrames(fixture.frames);
        CHECK(!frames.empty());
        if (frames.empty()) {
            continue;
        }
        _TestFixture(fixture, stagePath, frames);
        if (!sawBytes) {
            // One fresh-reader Execute on the first baking rig.
            const UsdStageRefPtr stage = UsdStage::Open(stagePath);
            if (stage && !_FindRig(stage).IsEmpty()) {
                RigExecRigEvaluator evaluator(stage, _FindRig(stage));
                evaluator.SetEvaluationMode(
                    RigExecEvaluationMode::Baked);
                std::vector<uint8_t> bytes;
                std::string error;
                if (RigExecTestBakeAt(evaluator, frames.front(), &bytes,
                                      &error) &&
                    !bytes.empty()) {
                    _TestFreshExecute(bytes);
                    _TestFileRefusals(bytes);
                    sawBytes = true;
                }
            }
        }
    }
    CHECK(sawBaking);
    CHECK(sawBytes);
    // Every baking fixture opened above; the refusals need a step with a
    // predecessor, a cluster and a cluster edge, which some fixture must
    // have.
    std::printf("step graph refusals: %d flipped predecessor(s), %d index "
                "range(s), %d cluster cycle(s), %d unproduced read(s)\n",
                stepGraphFlips, stepGraphRanges, stepGraphCycles,
                stepGraphProducers);
    CHECK(stepGraphProducers > 0);
    CHECK(stepGraphFlips > 0);
    CHECK(stepGraphRanges > 0);
    CHECK(stepGraphCycles > 0);
    std::printf("file refusals: %d row(s) (identifier, format "
                "version)\n",
                fileRefusalRows);
    CHECK(fileRefusalRows == 1);
    std::printf("static table refusals: %d of %d baking row(s); chain base "
                "on %d, constraint arrays on %d\n",
                staticTableRows, bakingRows, staticChainRows,
                staticArrayRows);
    CHECK(staticTableRows == bakingRows);
    CHECK(staticChainRows > 0);
    CHECK(staticArrayRows > 0);
    std::printf("step labels: %d of %d baking row(s) match the "
                "program's\n",
                stepLabelRows, bakingRows);
    CHECK(stepLabelRows == bakingRows);
    // Every baking row ran the input API; the set, the reset and the token
    // cases need a fixture that has each.
    std::printf("input api: %d of %d baking row(s), %d table field(s) "
                "found as inputs, %d set(s) moving the outputs, %d "
                "reset(s), %d token input(s)\n",
                inputApiRows, bakingRows, inputApiFields, inputApiMoved,
                inputApiResets, inputApiTokens);
    CHECK(inputApiRows == bakingRows);
    std::printf("defaults: %d of %d baking row(s) execute their defaults "
                "as the bake time's generation\n",
                defaultsRows, bakingRows);
    CHECK(defaultsRows == bakingRows);
    // The rows with no Animated input are the ones only the sampler's
    // change of time keeps in step with the program.
    std::printf("sampler: %d of %d baking row(s) driven along their frames "
                "(%d frame(s); %d inputs row(s) with no Animated input)\n",
                samplerRows, bakingRows, samplerFrames,
                samplerUnanimatedRows);
    CHECK(samplerRows == bakingRows);
    CHECK(samplerUnanimatedRows > 0);
    CHECK(inputApiFields > 0);
    CHECK(inputApiMoved > 0);
    CHECK(inputApiResets > 0);

    if (failures == 0) {
        std::printf("testRigExecRuntimeLoader: all tests passed\n");
        return 0;
    }
    std::printf("testRigExecRuntimeLoader: %d failures\n", failures);
    return 1;
}
