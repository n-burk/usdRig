//
// rigExecRuntime loader conformance: bake every baking fixture in-process,
// open the bytes with the zero-USD reader, and check sections, frames,
// frame selection, and malformed-input refusal.
//

#include "rigExecBake/bake.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
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

    if (failures == 0) {
        std::printf("testRigExecRuntimeLoader: all tests passed\n");
        return 0;
    }
    std::printf("testRigExecRuntimeLoader: %d failures\n", failures);
    return 1;
}
