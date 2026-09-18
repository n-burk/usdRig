//
// rigExecRuntime pose-family parity (M2): baked program vs runtime over
// every baking fixture, comparing fin/base versions, rest -> pose
// matrices and joint matrices bit for bit. Owned by the pose-port worker.
//

#include "rigExecBake/bake.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/container.h"
#include "rigExecBinary/pose.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <map>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;
static int comparedFixtures = 0;
static int deferredFixtures = 0;
static int fullModeFixtures = 0;

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

static bool
_FramesEqual(const RigExecPointFrame &a, const RrPointFrame &b)
{
    if (a.flags != b.flags) {
        return false;
    }
    for (size_t i = 0; i < 4; ++i) {
        for (size_t c = 0; c < 3; ++c) {
            if (a.points[i][c] != b.points[i][c]) {
                return false;
            }
        }
    }
    return true;
}

static bool
_MatricesEqual(const GfMatrix4d &a, const RrMat4d &b)
{
    for (size_t r = 0; r < 4; ++r) {
        const GfVec4d row = a.GetRow(int(r));
        for (size_t c = 0; c < 4; ++c) {
            if (row[c] != b[r][c]) {
                return false;
            }
        }
    }
    return true;
}

// Whether any wire constraint binds a weight object: those fixtures need
// the weights family, so pose-only mode defers them instead of failing.
static bool
_AnyWeightObject(const std::vector<uint8_t> &bytes)
{
    std::string error;
    std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    if (!reader) {
        return true;
    }
    const uint8_t *data = nullptr;
    size_t size = 0;
    if (!reader->FindSection(RigExecBinarySection::DomainPose, &data,
                             &size)) {
        return true;
    }
    RigExecWireReader cursor(data, size);
    RigExecWireDomainPose poses;
    if (!RigExecWireDecodeDomainPose(&cursor, &poses, &error)) {
        return true;
    }
    for (const RigExecWireConstraint &c : poses.constraints) {
        if (c.weightObject != 0) {
            return true;
        }
    }
    return false;
}

static void
_TestFixture(const std::string &name, const std::string &stagePath,
             const std::vector<double> &bakeFrames)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        std::printf("%s: FAILED (no stage)\n", name.c_str());
        return;
    }
    const SdfPath rigPath = _FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        std::printf("%s: FAILED (no rig)\n", name.c_str());
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
        std::printf("%s: FAILED (no bytes)\n", name.c_str());
        return;
    }

    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(),
                                   &error);
    CHECK(reader);
    if (!reader) {
        std::printf("open diagnostic: %s\n", error.c_str());
        std::printf("%s: FAILED (open)\n", name.c_str());
        return;
    }

    // Mask strategy: full mode first; when another family is still
    // landing, fall back to pose-only on fixtures whose constraints bind
    // no weight object, and defer the rest.
    reader->SetRunMaskForTesting(0x7u);
    bool fullMode = true;
    if (fullMode) {
        const RigExecRigPose probe =
            evaluator.Evaluate(UsdTimeCode(bakeFrames[0]));
        if (!probe.valid || !reader->SetFrame(bakeFrames[0], &error) ||
            !reader->Execute(&error)) {
            if (!probe.valid) {
                std::printf("%s: FAILED (probe invalid)\n",
                            name.c_str());
                CHECK(false);
                return;
            }
            if (error.find("not implemented yet") != std::string::npos) {
                fullMode = false;
            } else {
                std::printf("%s frame %.17g: %s\n", name.c_str(),
                            bakeFrames[0], error.c_str());
                CHECK(false);
                return;
            }
        }
    }
    // Fresh evaluator + reader so the measured walk starts clean in the
    // mode the probe selected.
    RigExecRigEvaluator measured(stage, rigPath);
    measured.SetEvaluationMode(RigExecEvaluationMode::Baked);
    reader = RigExecRuntimeReader::Open(
        result.bytes.data(), result.bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("%s: FAILED (reopen)\n", name.c_str());
        return;
    }
    reader->SetRunMaskForTesting(fullMode ? 0x7u : 0x1u);
    if (!fullMode) {
        if (_AnyWeightObject(result.bytes)) {
            ++deferredFixtures;
            std::printf("%s: deferred (pose-only; binds weightObject)\n",
                        name.c_str());
            return;
        }
    } else {
        ++fullModeFixtures;
    }

    bool failed = false;
    int comparedFrames = 0;
    for (double frame : bakeFrames) {
        const RigExecRigPose pose =
            measured.Evaluate(UsdTimeCode(frame));
        if (!pose.valid) {
            std::printf("%s frame %.17g: baked pose invalid\n",
                        name.c_str(), frame);
            CHECK(false);
            failed = true;
            break;
        }
        const RigExecBakedProgram *program =
            measured.GetBakedProgram();
        if (!program) {
            std::printf("%s frame %.17g: no baked program\n",
                        name.c_str(), frame);
            CHECK(false);
            failed = true;
            break;
        }
        const RigExecBakedProgramImpl &graph = program->GetStepGraph();
        if (!reader->SetFrame(frame, &error)) {
            std::printf("setframe diagnostic: %s\n", error.c_str());
            CHECK(false);
            failed = true;
            break;
        }
        if (!reader->Execute(&error)) {
            std::printf("execute diagnostic at %s frame %.17g: %s\n",
                        name.c_str(), frame, error.c_str());
            CHECK(false);
            failed = true;
            break;
        }

        const std::vector<RrPointFrame> &fin = reader->GetFinFrames();
        const std::vector<RrPointFrame> &base = reader->GetBaseFrames();
        const std::vector<RrMat4d> &finalM = reader->GetFinalMatrices();
        const std::vector<RrMat4d> &baseM = reader->GetBaseMatrices();
        if (fin.size() != graph.fin.size() ||
            base.size() != graph.base.size() ||
            finalM.size() != graph.finalMatrix.size() ||
            baseM.size() != graph.baseMatrix.size()) {
            std::printf("%s frame %.17g: pool sizes %zu/%zu/%zu/%zu vs "
                        "%zu/%zu/%zu/%zu\n", name.c_str(), frame,
                        fin.size(), base.size(), finalM.size(),
                        baseM.size(), graph.fin.size(),
                        graph.base.size(), graph.finalMatrix.size(),
                        graph.baseMatrix.size());
            CHECK(false);
            failed = true;
            continue;
        }
        for (size_t i = 0; i < fin.size(); ++i) {
            if (!_FramesEqual(graph.fin[i], fin[i])) {
                std::printf("%s frame %.17g: fin[%zu] differs "
                            "(flags %u vs %u)\n", name.c_str(), frame, i,
                            graph.fin[i].flags, fin[i].flags);
                CHECK(false);
                failed = true;
                break;
            }
        }
        for (size_t i = 0; i < base.size(); ++i) {
            if (!_FramesEqual(graph.base[i], base[i])) {
                std::printf("%s frame %.17g: base[%zu] differs "
                            "(flags %u vs %u)\n", name.c_str(), frame, i,
                            graph.base[i].flags, base[i].flags);
                CHECK(false);
                failed = true;
                break;
            }
        }
        for (size_t i = 0; i < finalM.size(); ++i) {
            if (!_MatricesEqual(graph.finalMatrix[i], finalM[i])) {
                std::printf("%s frame %.17g: finalMatrix[%zu] differs\n",
                            name.c_str(), frame, i);
                CHECK(false);
                failed = true;
                break;
            }
        }
        for (size_t i = 0; i < baseM.size(); ++i) {
            if (!_MatricesEqual(graph.baseMatrix[i], baseM[i])) {
                std::printf("%s frame %.17g: baseMatrix[%zu] differs\n",
                            name.c_str(), frame, i);
                CHECK(false);
                failed = true;
                break;
            }
        }

        // Joint matrices: path plus bitwise matrix.
        std::map<std::string, GfMatrix4d> wantJoints;
        for (const auto &entry : pose.jointMatricesFinal) {
            wantJoints[entry.first.GetString()] = entry.second;
        }
        const std::vector<RigExecRuntimeJointMatrix> &gotJoints =
            reader->GetJointMatrices();
        if (gotJoints.size() != wantJoints.size()) {
            std::printf("%s frame %.17g: %zu joints vs %zu\n",
                        name.c_str(), frame, gotJoints.size(),
                        wantJoints.size());
            CHECK(false);
            failed = true;
        } else {
            for (const RigExecRuntimeJointMatrix &joint : gotJoints) {
                const auto it = wantJoints.find(joint.path);
                if (it == wantJoints.end() ||
                    !_MatricesEqual(it->second, joint.matrix)) {
                    std::printf("%s frame %.17g: joint %s differs\n",
                                name.c_str(), frame, joint.path.c_str());
                    CHECK(false);
                    failed = true;
                    break;
                }
            }
        }

        // Diagnostics verbatim in full mode only: pose-only mode skips
        // whole families, so its generation summary cannot match.
        if (fullMode) {
            const std::vector<std::string> &diagnostics =
                reader->GetDiagnostics();
            if (diagnostics != pose.diagnostics) {
                std::printf("%s frame %.17g: diagnostics differ "
                            "(%zu vs %zu)\n", name.c_str(), frame,
                            diagnostics.size(), pose.diagnostics.size());
                const size_t common =
                    std::min(diagnostics.size(),
                             pose.diagnostics.size());
                for (size_t i = 0; i < common; ++i) {
                    if (diagnostics[i] != pose.diagnostics[i]) {
                        std::printf("  got:  %s\n  want: %s\n",
                                    diagnostics[i].c_str(),
                                    pose.diagnostics[i].c_str());
                        break;
                    }
                }
                CHECK(false);
                failed = true;
            }
        }
        ++comparedFrames;
    }

    if (failed) {
        std::printf("%s: FAILED\n", name.c_str());
    } else {
        ++comparedFixtures;
        std::printf("%s: compared %d frames%s\n", name.c_str(),
                    comparedFrames, fullMode ? " (full)" : " (pose-only)");
    }
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);

    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }
    bool sawBaking = false;
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
        _TestFixture(fixture.stage, stagePath, frames);
    }
    CHECK(sawBaking);

    if (failures == 0) {
        std::printf("testRigExecRuntimePose: all tests passed "
                    "(%d compared, %d deferred, %d full-mode)\n",
                    comparedFixtures, deferredFixtures,
                    fullModeFixtures);
        return 0;
    }
    std::printf("testRigExecRuntimePose: %d failures "
                "(%d compared, %d deferred, %d full-mode)\n", failures,
                comparedFixtures, deferredFixtures, fullModeFixtures);
    return 1;
}
