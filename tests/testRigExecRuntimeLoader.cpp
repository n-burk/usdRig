// rigExecRuntime loader conformance: bake every baking fixture in-process,
// open the bytes with the zero-USD reader, and check sections, frames,
// frame selection, and malformed-input refusal, including a step graph
// with a predecessor after its step, step and cluster indices past their
// tables, and a cluster graph with a cycle.
#include "rigExecBake/bake.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/program.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
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

#include "rigExecSectionEdit.h"

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
}

// The steps and clustering \p bytes carries.
static bool
_DecodeGraph(const std::vector<uint8_t> &bytes,
             std::vector<RigExecWireStep> *steps,
             RigExecWireClustering *clustering)
{
    std::string error;
    const std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    const uint8_t *data = nullptr;
    size_t size = 0;
    if (!reader ||
        !reader->FindSection(RigExecBinarySection::Steps, &data, &size)) {
        return false;
    }
    RigExecWireReader stepCursor(data, size);
    if (!RigExecWireDecodeSteps(&stepCursor, steps, &error) ||
        !reader->FindSection(RigExecBinarySection::Clusters, &data,
                             &size)) {
        return false;
    }
    RigExecWireReader clusterCursor(data, size);
    return RigExecWireDecodeClustering(&clusterCursor, clustering, &error);
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

static int stepGraphFlips = 0;
static int stepGraphRanges = 0;
static int stepGraphCycles = 0;
static int stepGraphProducers = 0;

// Open refuses \p bytes with section \p tag replaced by \p payload, saying
// exactly \p expected.
static void
_ExpectRefusal(const std::string &name, const char *what,
               const std::vector<uint8_t> &bytes, RigExecBinarySection tag,
               const std::vector<uint8_t> &payload,
               const std::string &expected)
{
    const std::string got = _OpenError(_ReplaceSection(bytes, tag, payload));
    CHECK(got == expected);
    if (got != expected) {
        std::printf("%s, %s: open said '%s', expected '%s'\n", name.c_str(),
                    what, got.c_str(), expected.c_str());
    }
}

static std::vector<uint8_t>
_StepsPayload(const std::vector<RigExecWireStep> &steps)
{
    std::vector<uint8_t> payload;
    CHECK(RigExecWireEncodeSteps(steps, &payload));
    return payload;
}

static std::vector<uint8_t>
_ClustersPayload(const RigExecWireClustering &clustering)
{
    std::vector<uint8_t> payload;
    CHECK(RigExecWireEncodeClustering(clustering, &payload));
    return payload;
}

// Step and cluster graphs playback could not walk, cut into a fixture's
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
    std::vector<RigExecWireStep> steps;
    RigExecWireClustering clustering;
    CHECK(_DecodeGraph(bytes, &steps, &clustering));
    const size_t count = steps.size();
    const auto &clusters = clustering.clusters;
    const std::string stepCount = std::to_string(count);
    const std::string clusterCount = std::to_string(clusters.size());
    for (size_t s = 0; s + 1 < count; ++s) {
        if (steps[s].preds.empty()) {
            continue;
        }
        std::vector<RigExecWireStep> edited = steps;
        edited[s].preds[0] = int32_t(count - 1);
        _ExpectRefusal(name, "flipped predecessor", bytes,
                       RigExecBinarySection::Steps, _StepsPayload(edited),
                       "step " + std::to_string(s) +
                           " depends on later step " +
                           std::to_string(count - 1));
        edited[s].preds[0] = int32_t(count);
        _ExpectRefusal(name, "predecessor past the steps", bytes,
                       RigExecBinarySection::Steps, _StepsPayload(edited),
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
        const std::pair<RigExecWireSlotDomain, const char *> domains[] = {
            {RigExecWireSlotDomain::PosedM, "PosedM"},
            {RigExecWireSlotDomain::Aggregate, "Aggregate"}};
        for (const auto &[domain, text] : domains) {
            std::vector<RigExecWireStep> edited = steps;
            edited.back().reads.push_back({domain, past, past + 1});
            _ExpectRefusal(name, "read nothing writes", bytes,
                           RigExecBinarySection::Steps,
                           _StepsPayload(edited),
                           "step " + last + " reads " + text + " slots [" +
                               std::to_string(past) + ", " +
                               std::to_string(past + 1) +
                               "), which no earlier step writes");
            ++stepGraphProducers;
        }
    }
    if (count > 0 && !clusters.empty()) {
        std::vector<RigExecWireStep> editedSteps = steps;
        editedSteps[0].cluster = int32_t(clusters.size());
        _ExpectRefusal(name, "cluster past the clusters", bytes,
                       RigExecBinarySection::Steps,
                       _StepsPayload(editedSteps),
                       "step 0 names cluster " + clusterCount +
                           ", which is no cluster");
        RigExecWireClustering edited = clustering;
        edited.clusters[0].members.push_back(int32_t(count));
        _ExpectRefusal(name, "member past the steps", bytes,
                       RigExecBinarySection::Clusters,
                       _ClustersPayload(edited),
                       "cluster 0 names member " + stepCount +
                           ", which is no step");
        edited = clustering;
        edited.clusters[0].preds.push_back(int32_t(clusters.size()));
        _ExpectRefusal(name, "cluster predecessor past the clusters", bytes,
                       RigExecBinarySection::Clusters,
                       _ClustersPayload(edited),
                       "cluster 0 names predecessor " + clusterCount +
                           ", which is no cluster");
        edited = clustering;
        edited.clusterOf.pop_back();
        _ExpectRefusal(name, "clustering one step short", bytes,
                       RigExecBinarySection::Clusters,
                       _ClustersPayload(edited),
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
            RigExecWireClustering edited = clustering;
            std::vector<int32_t> &preds = edited.clusters[size_t(a)].preds;
            preds.insert(
                std::lower_bound(preds.begin(), preds.end(), int32_t(b)),
                int32_t(b));
            std::vector<int32_t> &succs = edited.clusters[b].succs;
            succs.insert(std::lower_bound(succs.begin(), succs.end(), a), a);
            _ExpectRefusal(name, "two-cluster cycle", bytes,
                           RigExecBinarySection::Clusters,
                           _ClustersPayload(edited),
                           "the cluster graph has a cycle through cluster " +
                               std::to_string(a));
            ++stepGraphCycles;
            return;
        }
    }
}

static void
_TestExecuteNeedsFrame(const std::vector<uint8_t> &bytes)
{
    std::string error;
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("open diagnostic: %s\n", error.c_str());
        return;
    }
    // Execute with no frame selected refuses naming the miss; outputs
    // keep their empty initial state.
    CHECK(!reader->Execute(&error));
    CHECK(error == "no frame selected");
    CHECK(reader->GetJointMatrices().empty());
    CHECK(reader->GetPoints().empty());
}

static void
_TestFixture(const std::string &stagePath,
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
    RigExecBakeOpts opts;
    opts.frames = bakeFrames;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    if (!error.empty()) {
        std::printf("bake diagnostic: %s\n", error.c_str());
    }
    CHECK(!result.bytes.empty());
    if (result.bytes.empty()) {
        return;
    }

    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(),
                                   &error);
    CHECK(reader);
    if (!reader) {
        std::printf("open diagnostic: %s\n", error.c_str());
        return;
    }
    // The bake always emits every section, so the loader decodes all of
    // them; a missing or malformed one fails Open, not a flag.
    CHECK(reader->HasSteps());
    CHECK(reader->HasClusters());
    CHECK(reader->HasCones());
    CHECK(reader->HasSlotMeta());
    CHECK(reader->HasConstants());
    CHECK(reader->HasPoses());
    CHECK(reader->HasGeometry());
    CHECK(reader->HasInputTable());

    CHECK(reader->GetFrameTimes() == bakeFrames);

    // Exact-time selection: a baked frame selects, anything else misses.
    for (double frame : bakeFrames) {
        CHECK(reader->SetFrame(frame, &error));
    }
    CHECK(!reader->SetFrame(-1e9, &error));
    CHECK(!error.empty());

    _TestStepGraphRefusals(stagePath, result.bytes);

    // Truncating the tail breaks a section payload, so Open refuses.
    if (result.bytes.size() > 64) {
        std::string truncError;
        CHECK(!RigExecRuntimeReader::Open(
            result.bytes.data(), result.bytes.size() - 32, &truncError));
        CHECK(!truncError.empty());
    }
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
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            continue;
        }
        sawBaking = true;
        const std::string stagePath =
            examplesDir + "/" + fixture.stage;
        const std::vector<double> frames = _ParseFrames(fixture.frames);
        CHECK(!frames.empty());
        if (frames.empty()) {
            continue;
        }
        _TestFixture(stagePath, frames);
        if (!sawBytes) {
            // One Execute precondition probe on the first baking rig.
            const UsdStageRefPtr stage = UsdStage::Open(stagePath);
            if (stage && !_FindRig(stage).IsEmpty()) {
                RigExecRigEvaluator evaluator(stage, _FindRig(stage));
                evaluator.SetEvaluationMode(
                    RigExecEvaluationMode::Baked);
                RigExecBakeOpts opts;
                opts.frames = frames;
                RigExecBakeResult result;
                std::string error;
                if (RigExecBakeToBinary(evaluator, opts, &result,
                                        &error) &&
                    !result.bytes.empty()) {
                    _TestExecuteNeedsFrame(result.bytes);
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

    if (failures == 0) {
        std::printf("testRigExecRuntimeLoader: all tests passed\n");
        return 0;
    }
    std::printf("testRigExecRuntimeLoader: %d failures\n", failures);
    return 1;
}
