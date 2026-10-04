// rigExecRuntime pose-family parity (M2): baked program vs runtime over
// every baking fixture, comparing fin/base versions, rest -> pose
// matrices and joint matrices bit for bit. Owned by the pose-port worker.
#include "rigExecBake/bake.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/container.h"
#include "rigExecBinary/inputTable.h"
#include "rigExecBinary/pose.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
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

// Bakes \p stage at \p bakeFrames and compares a fresh reader, with the
// record cross-check on, against a fresh baked evaluator frame by frame.
// \p crossChecked receives the values the cross-check compared.
static void
_TestStage(const std::string &name, const UsdStageRefPtr &stage,
           const std::vector<double> &bakeFrames,
           uint64_t *crossChecked = nullptr)
{
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
    reader->SetCrossCheckForTesting(true);
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
    reader->SetCrossCheckForTesting(true);
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

    if (crossChecked) {
        *crossChecked = reader->GetCrossCheckResultCountForTesting();
    }
    if (failed) {
        std::printf("%s: FAILED\n", name.c_str());
    } else {
        ++comparedFixtures;
        std::printf("%s: compared %d frames%s, %llu cross-checked (%llu "
                    "registered read(s), %llu path read(s))\n",
                    name.c_str(), comparedFrames,
                    fullMode ? " (full)" : " (pose-only)",
                    static_cast<unsigned long long>(
                        reader->GetCrossCheckCountForTesting()),
                    static_cast<unsigned long long>(
                        reader->GetCrossCheckCountForTesting(
                            RrCrossCheckRegisteredRead)),
                    static_cast<unsigned long long>(
                        reader->GetCrossCheckCountForTesting(
                            RrCrossCheckPathRead)));
    }
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
    _TestStage(name, stage, bakeFrames);
}

// \p bytes with section \p tag replaced by \p payload; the string table and
// every other section are copied unchanged.
static std::vector<uint8_t>
_ReplaceSection(const std::vector<uint8_t> &bytes, RigExecBinarySection tag,
                const std::vector<uint8_t> &payload)
{
    std::string error;
    const std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        return {};
    }
    RigExecBinaryWriter writer;
    std::string text;
    for (uint32_t id = 1; reader->GetString(id, &text); ++id) {
        CHECK(writer.AddString(text) == id);
    }
    for (uint32_t t = uint32_t(RigExecBinarySection::Manifest);
         t <= uint32_t(RigExecBinarySection::Computed); ++t) {
        const RigExecBinarySection section = RigExecBinarySection(t);
        const uint8_t *data = nullptr;
        size_t size = 0;
        if (!reader->FindSection(section, &data, &size)) {
            continue;
        }
        if (section == tag) {
            writer.AddSection(section, payload);
        } else {
            writer.AddSection(section, data, size);
        }
    }
    return writer.Finish();
}

static bool
_DecodeInputTable(const std::vector<uint8_t> &bytes,
                  RigExecWireInputTable *table)
{
    std::string error;
    const std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    const uint8_t *data = nullptr;
    size_t size = 0;
    if (!reader ||
        !reader->FindSection(RigExecBinarySection::InputTable, &data,
                             &size)) {
        return false;
    }
    RigExecWireReader cursor(data, size);
    return RigExecWireDecodeInputTable(&cursor, table, &error);
}

// Two transform constraints whose envelopes no mover binds: a StaticWeight
// at 0.5 and a DynamicWeight whose driver connects to a double keyed from
// 0.2 at frame 1 to \p lastDriver at frame 10 (testRigExecBinary's
// envelope bake). Both resolve through the oracle from the Computed
// section's envelope-only entries.
static UsdStageRefPtr
_EnvelopeStage(double lastDriver = 0.8)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const auto xform = [&](const char *path, const GfVec3d &translation) {
        const UsdGeomXform x = UsdGeomXform::Define(stage, SdfPath(path));
        GfMatrix4d m(1.0);
        m.SetTranslateOnly(translation);
        x.MakeMatrixXform().Set(m);
        return x.GetPrim();
    };
    xform("/Asset", GfVec3d(0));
    xform("/Asset/TargetA", GfVec3d(0));
    xform("/Asset/TargetB", GfVec3d(0));
    xform("/Asset/Source", GfVec3d(10, 0, 0));
    const UsdPrim dial = xform("/Asset/Dial", GfVec3d(0));
    const UsdAttribute amount = dial.CreateAttribute(
        TfToken("avars:amount"), SdfValueTypeNames->Double);
    amount.Set(0.2, UsdTimeCode(1.0));
    amount.Set(lastDriver, UsdTimeCode(10.0));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto constraint = [&](const char *name, const char *target,
                                const SdfPath &weightPath) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name),
            TfToken("RigExecPositionConstraint"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath(target)});
        prim.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        prim.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weightPath});
    };
    const UsdPrim fixed = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Fixed"), TfToken("RigExecStaticWeight"));
    fixed.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetA")});
    fixed.CreateAttribute(TfToken("rigExec:defaultWeight"),
                          SdfValueTypeNames->Float)
        .Set(0.5f);
    const UsdPrim driven = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Driven"), TfToken("RigExecDynamicWeight"));
    driven.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetB")});
    driven.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    driven.CreateAttribute(TfToken("inputs:driver"), SdfValueTypeNames->Float)
        .AddConnection(amount.GetPath());
    constraint("A", "/Asset/TargetA", fixed.GetPath());
    constraint("B", "/Asset/TargetB", driven.GetPath());
    return stage;
}

// Runs every frame of \p bytes through a fresh reader; false with the
// first Execute error.
static bool
_RunFrames(const std::vector<uint8_t> &bytes,
           const std::vector<double> &frames, bool crossCheck,
           std::vector<std::vector<RrPointFrame>> *fins, std::string *error)
{
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), error);
    if (!reader) {
        return false;
    }
    reader->SetCrossCheckForTesting(crossCheck);
    for (double frame : frames) {
        if (!reader->SetFrame(frame, error) || !reader->Execute(error)) {
            return false;
        }
        fins->push_back(reader->GetFinFrames());
    }
    return true;
}

// Constraint envelopes are computed, not replayed: the parity path with
// the cross-check compares every envelope against the record; a record
// whose envelope was altered fails the cross-check naming the field, and
// with the cross-check off plays exactly as the untouched file does.
static void
TestComputedEnvelopes()
{
    const std::vector<double> frames = {1.0, 5.0, 10.0};
    uint64_t crossChecked = 0;
    _TestStage("computed envelopes", _EnvelopeStage(), frames,
               &crossChecked);
    // Two envelopes per frame: the Constraint steps run every frame.
    CHECK(crossChecked == 2 * frames.size());

    const UsdStageRefPtr stage = _EnvelopeStage();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    RigExecWireInputTable table;
    CHECK(_DecodeInputTable(result.bytes, &table));
    if (table.frames.empty() || table.frames[0].constraintWeights.size() != 2) {
        CHECK(false);
        return;
    }
    // Constraint 1's envelope at the first frame, moved off what the
    // oracle computes there (0.5 or 0.2).
    table.frames[0].constraintWeights[1] = 0.25f;
    std::vector<uint8_t> payload;
    CHECK(RigExecWireEncodeInputTable(table, &payload));
    const std::vector<uint8_t> tampered =
        _ReplaceSection(result.bytes, RigExecBinarySection::InputTable,
                        payload);

    std::vector<std::vector<RrPointFrame>> untouched, replayed, ignored;
    CHECK(_RunFrames(result.bytes, frames, true, &untouched, &error));
    CHECK(!_RunFrames(tampered, frames, true, &ignored, &error));
    CHECK(error.find("cross-check mismatch at constraintWeights[1]") !=
          std::string::npos);
    std::printf("computed envelopes, tampered record: %s\n", error.c_str());
    CHECK(_RunFrames(tampered, frames, false, &replayed, &error));
    CHECK(replayed == untouched);

    // The dynamic envelope driven to 1.6 at frame 10 breaks its strict
    // range: the oracle's error passes the constraint through with the
    // program's diagnostic (compared verbatim by the parity path), and a
    // failed resolve is not compared against the record.
    uint64_t strictChecked = 0;
    _TestStage("computed envelopes, strict violation", _EnvelopeStage(1.6),
               frames, &strictChecked);
    CHECK(strictChecked == 2 * frames.size() - 1);
}

// One constraint envelope a computed-envelope case reads back. The target
// is a plain Xform at the asset origin pulled toward a source at
// (sourceX, 0, 0), so its revised translation is sourceX * w, and the
// envelope w the runtime applied is recovered exactly: sourceX * double(w)
// needs fewer than 53 significant bits.
struct _EnvelopeProbe {
    const char *target;
    const char *weightObject;
    double sourceX;
};

static const RigExecRuntimeProviderXform *
_FindProviderXform(const RigExecRuntimeReader &reader, const char *path)
{
    for (const RigExecRuntimeProviderXform &xform :
         reader.GetProviderXforms()) {
        if (xform.path == path) {
            return &xform;
        }
    }
    return nullptr;
}

// Plays \p stage's bake with the cross-check on beside the dynamic path:
// an ExecReference evaluator (the exec-authoritative walk every evaluator
// is held to) and the dynamic weight oracle itself
// (RigExecRigEvaluator::_ResolveWeights, which the baked program holds as
// resolveWeights). At every frame each probe's revised transform must
// equal the dynamic evaluator's bit for bit and imply exactly the envelope
// the oracle resolves. \p envelopes receives the oracle's envelopes, one
// row per frame in probe order; \p crossChecked the envelopes the
// cross-check compared. False on any difference.
static bool
_EnvelopesMatchDynamic(const std::string &name, const UsdStageRefPtr &stage,
                       const std::vector<double> &frames,
                       const std::vector<_EnvelopeProbe> &probes,
                       uint64_t *crossChecked,
                       std::vector<std::vector<float>> *envelopes)
{
    const SdfPath rigPath = _FindRig(stage);
    RigExecRigEvaluator baked(stage, rigPath);
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult result;
    std::string error;
    if (!RigExecBakeToBinary(baked, opts, &result, &error)) {
        std::printf("%s: bake failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(),
                                   &error);
    if (!reader) {
        std::printf("%s: open failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
    reader->SetCrossCheckForTesting(true);
    RigExecRigEvaluator dynamic(stage, rigPath);
    dynamic.SetEvaluationMode(RigExecEvaluationMode::ExecReference);

    bool same = true;
    for (double frame : frames) {
        const RigExecRigPose want = dynamic.Evaluate(UsdTimeCode(frame));
        // The oracle reads the stage at the generation its evaluator last
        // ran, so the host evaluates the frame first.
        const RigExecRigPose host = baked.Evaluate(UsdTimeCode(frame));
        const RigExecBakedProgram *program = baked.GetBakedProgram();
        if (!want.valid || !host.valid || !program) {
            std::printf("%s frame %g: no dynamic pose or no program\n",
                        name.c_str(), frame);
            return false;
        }
        if (!reader->SetFrame(frame, &error) || !reader->Execute(&error)) {
            std::printf("%s frame %g: %s\n", name.c_str(), frame,
                        error.c_str());
            return false;
        }
        std::vector<float> row;
        for (const _EnvelopeProbe &probe : probes) {
            std::vector<float> w;
            std::string why;
            if (!program->GetStepGraph().resolveWeights(
                    SdfPath(probe.weightObject), 1, UsdTimeCode(frame), &w,
                    &why, nullptr) ||
                w.size() != 1) {
                std::printf("%s frame %g: the dynamic oracle failed on %s: "
                            "%s\n", name.c_str(), frame, probe.weightObject,
                            why.c_str());
                same = false;
                continue;
            }
            row.push_back(w[0]);
            const auto expected =
                want.providerXforms.find(SdfPath(probe.target));
            const RigExecRuntimeProviderXform *got =
                _FindProviderXform(*reader, probe.target);
            if (expected == want.providerXforms.end() || !got) {
                std::printf("%s frame %g: %s has no revised transform "
                            "(dynamic %d, runtime %d)\n", name.c_str(),
                            frame, probe.target,
                            int(expected != want.providerXforms.end()),
                            int(got != nullptr));
                same = false;
                continue;
            }
            if (!_MatricesEqual(expected->second, got->matrix)) {
                std::printf("%s frame %g: %s differs from the dynamic "
                            "evaluator (x %.17g vs %.17g)\n", name.c_str(),
                            frame, probe.target, got->matrix[3][0],
                            expected->second[3][0]);
                same = false;
            }
            const float applied =
                static_cast<float>(got->matrix[3][0] / probe.sourceX);
            if (std::memcmp(&applied, &w[0], sizeof(float)) != 0) {
                std::printf("%s frame %g: %s applied envelope %.9g, the "
                            "dynamic oracle resolves %.9g\n", name.c_str(),
                            frame, probe.target, double(applied),
                            double(w[0]));
                same = false;
            }
        }
        envelopes->push_back(std::move(row));
    }
    *crossChecked = reader->GetCrossCheckEnvelopeCountForTesting();
    // Transform-domain envelopes only: nothing measures a current phase.
    if (reader->GetCrossCheckPhasePacketCountForTesting() != 0) {
        std::printf("%s: compared current-phase packets it has none of\n",
                    name.c_str());
        same = false;
    }
    return same;
}

// testRigExecConstraints' StaticWeight 0.5 case
// (TestGeometryDomainTargetCompiles, transform arm): a constant
// StaticWeight at 0.5 bound as the constraint's weight object supersedes
// its own inputs:defaultWeight of 0.25, so the target lands halfway to the
// source.
static UsdStageRefPtr
_StaticWeightConstraintStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const auto xform = [&](const char *path, const GfVec3d &translation) {
        const UsdGeomXform x = UsdGeomXform::Define(stage, SdfPath(path));
        x.MakeMatrixXform().Set(GfMatrix4d(
            GfRotation(GfVec3d(0, 0, 1), 0.0), translation));
    };
    xform("/Asset", GfVec3d(0));
    xform("/Asset/Target", GfVec3d(0));
    xform("/Asset/Source", GfVec3d(10, 0, 0));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim pos = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Pos"),
                                          TfToken("RigExecPositionConstraint"));
    CHECK(pos.ApplyAPI(TfToken("RigExecMoverAPI")));
    pos.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Target")});
    pos.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    pos.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.25f);
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Target")});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float)
        .Set(0.5f);
    pos.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({weight.GetPath()});
    return stage;
}

static void
TestStaticWeightEnvelope()
{
    const char *const name = "static weight envelope";
    const std::vector<double> frames = {1.0, 2.0, 3.0};
    uint64_t parityChecked = 0;
    _TestStage(name, _StaticWeightConstraintStage(), frames, &parityChecked);
    uint64_t dynamicChecked = 0;
    std::vector<std::vector<float>> envelopes;
    CHECK(_EnvelopesMatchDynamic(
        name, _StaticWeightConstraintStage(), frames,
        {{"/Asset/Target", "/Asset/Rig/Weights/W", 10.0}}, &dynamicChecked,
        &envelopes));
    // Not vacuous: a Constraint step that binds a weight object reads
    // outside the program and reruns every run, so its envelope is
    // computed and compared at every frame.
    CHECK(parityChecked == frames.size());
    CHECK(dynamicChecked == frames.size());
    CHECK(envelopes.size() == frames.size());
    for (const std::vector<float> &row : envelopes) {
        CHECK(row == std::vector<float>{0.5f});
    }
    std::printf("%s: %llu + %llu envelope(s) cross-checked\n", name,
                static_cast<unsigned long long>(parityChecked),
                static_cast<unsigned long long>(dynamicChecked));
}

// A DynamicWeight driven by a rig control's animated avar. The Dial
// control's avars:tx (a double) is keyed from 0 at frame 1 to 1.8 at frame
// 10; each weight's inputs:driver (a float) connects to it, so the oracle
// reads the double hop and narrows it. Driven: clamp, scale 0.625, bias
// 0.125, which leaves [0, 1] from frame 9 on. Modulated: strict, scale
// 0.625, over an envelope-only base (a constant StaticWeight at 0.7) that
// only rigExec:baseWeight reaches; neither factor is a power of two, so
// (b * d) * s and b * (d * s) differ at some frames. Blend: a
// CombineWeight (max, invert 0.25, strength 0.75) over a constant 0.25
// and the avar scaled by 0.5. With \p clampDial, a float math mover clamps
// the avar to [0, 1] and every driver declares rigExecReadPhase "final",
// so each reads the chain's result through the overlay rather than the
// authored value (undeclared, it would read the chain's base).
static UsdStageRefPtr
_AvarDrivenStage(bool clampDial = false)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const auto xform = [&](const char *path, const GfVec3d &translation) {
        const UsdGeomXform x = UsdGeomXform::Define(stage, SdfPath(path));
        GfMatrix4d m(1.0);
        m.SetTranslateOnly(translation);
        x.MakeMatrixXform().Set(m);
    };
    xform("/Asset", GfVec3d(0));
    xform("/Asset/TargetA", GfVec3d(0));
    xform("/Asset/TargetB", GfVec3d(0));
    xform("/Asset/TargetC", GfVec3d(0));
    xform("/Asset/Source", GfVec3d(10, 0, 0));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim dial = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Dial"), TfToken("RigExecControl"));
    const UsdAttribute tx =
        dial.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    tx.Set(0.0, UsdTimeCode(1.0));
    tx.Set(1.8, UsdTimeCode(10.0));

    const auto dynamicWeight = [&](const char *name, const char *target) {
        const UsdPrim w = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name),
            TfToken("RigExecDynamicWeight"));
        w.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({SdfPath(target)});
        w.CreateAttribute(TfToken("rigExec:representation"),
                          SdfValueTypeNames->Token)
            .Set(TfToken("constant"));
        const UsdAttribute driver = w.CreateAttribute(
            TfToken("inputs:driver"), SdfValueTypeNames->Float);
        driver.AddConnection(tx.GetPath());
        if (clampDial) {
            driver.SetMetadata(TfToken("rigExecReadPhase"),
                               std::string("final"));
        }
        return w;
    };
    const UsdPrim driven = dynamicWeight("Driven", "/Asset/TargetA");
    driven.CreateAttribute(TfToken("rigExec:rangePolicy"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("clamp"));
    driven.CreateAttribute(TfToken("inputs:scale"), SdfValueTypeNames->Float)
        .Set(0.625f);
    driven.CreateAttribute(TfToken("inputs:bias"), SdfValueTypeNames->Float)
        .Set(0.125f);
    const UsdPrim half = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Half"), TfToken("RigExecStaticWeight"));
    half.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetB")});
    half.CreateAttribute(TfToken("rigExec:representation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    half.CreateAttribute(TfToken("rigExec:defaultWeight"),
                         SdfValueTypeNames->Float)
        .Set(0.7f);
    const UsdPrim modulated = dynamicWeight("Modulated", "/Asset/TargetB");
    modulated.CreateRelationship(TfToken("rigExec:baseWeight"))
        .SetTargets({half.GetPath()});
    modulated.CreateAttribute(TfToken("inputs:scale"), SdfValueTypeNames->Float)
        .Set(0.625f);
    // Blend: the max of a constant 0.25 and the avar scaled by 0.5, then
    // inverted by a quarter. Every object it composes is envelope-only.
    const UsdPrim quarter = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Quarter"), TfToken("RigExecStaticWeight"));
    quarter.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetC")});
    quarter.CreateAttribute(TfToken("rigExec:representation"),
                            SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    quarter.CreateAttribute(TfToken("rigExec:defaultWeight"),
                            SdfValueTypeNames->Float)
        .Set(0.25f);
    const UsdPrim ramp = dynamicWeight("Ramp", "/Asset/TargetC");
    ramp.CreateAttribute(TfToken("inputs:scale"), SdfValueTypeNames->Float)
        .Set(0.5f);
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Blend"), TfToken("RigExecCombineWeight"));
    blend.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetC")});
    blend.CreateAttribute(TfToken("rigExec:combineMode"),
                          SdfValueTypeNames->Token)
        .Set(TfToken("max"));
    blend.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({quarter.GetPath(), ramp.GetPath()});
    blend.CreateAttribute(TfToken("inputs:invert"), SdfValueTypeNames->Float)
        .Set(0.25f);
    blend.CreateAttribute(TfToken("inputs:strength"), SdfValueTypeNames->Float)
        .Set(0.75f);
    if (clampDial) {
        const UsdPrim clamp = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/ClampDial"),
            TfToken("RigExecFloatMathMover"));
        CHECK(clamp.ApplyAPI(TfToken("RigExecMoverAPI")));
        clamp.CreateAttribute(TfToken("rigExec:operation"),
                              SdfValueTypeNames->Token)
            .Set(TfToken("clamp"));
        clamp.CreateAttribute(TfToken("inputs:min"), SdfValueTypeNames->Float)
            .Set(0.0f);
        clamp.CreateAttribute(TfToken("inputs:max"), SdfValueTypeNames->Float)
            .Set(1.0f);
        clamp.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({tx.GetPath()});
    }

    const auto constraint = [&](const char *name, const char *target,
                                const UsdPrim &weight) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name),
            TfToken("RigExecPositionConstraint"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath(target)});
        prim.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        prim.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weight.GetPath()});
    };
    constraint("A", "/Asset/TargetA", driven);
    constraint("B", "/Asset/TargetB", modulated);
    constraint("C", "/Asset/TargetC", blend);
    return stage;
}

// Each frame's revised x of \p target from a fresh reader over \p bytes;
// false with the first Execute error.
static bool
_RunTargetX(const std::vector<uint8_t> &bytes,
            const std::vector<double> &frames, bool crossCheck,
            const char *target, std::vector<double> *xs, std::string *error)
{
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), error);
    if (!reader) {
        return false;
    }
    reader->SetCrossCheckForTesting(crossCheck);
    for (double frame : frames) {
        if (!reader->SetFrame(frame, error) || !reader->Execute(error)) {
            return false;
        }
        const RigExecRuntimeProviderXform *xform =
            _FindProviderXform(*reader, target);
        if (!xform) {
            *error = std::string(target) + " has no revised transform";
            return false;
        }
        xs->push_back(xform->matrix[3][0]);
    }
    return true;
}

// The envelopes come from the Computed section's slot values: the Dial
// avar's value at the middle frame, set to 0 in the section alone, makes
// the runtime compute Driven's envelope from 0 there (0.125), which the
// cross-check refuses against the record naming the field, and which with
// the cross-check off moves the target to 1.25 at that frame only.
static void
_TestSlotValuesDriveEnvelopes()
{
    const std::vector<double> frames = {1.0, 5.0, 10.0};
    const UsdStageRefPtr stage = _AvarDrivenStage();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    const std::unique_ptr<RigExecBinaryReader> binary =
        RigExecBinaryReader::Open(result.bytes.data(), result.bytes.size(),
                                  &error);
    const uint8_t *data = nullptr;
    size_t size = 0;
    RigExecWireComputed computed;
    if (!binary ||
        !binary->FindSection(RigExecBinarySection::Computed, &data, &size)) {
        CHECK(false);
        return;
    }
    {
        RigExecWireReader cursor(data, size);
        CHECK(RigExecWireDecodeComputed(&cursor, &computed, &error));
    }
    size_t slot = computed.inputs.size();
    std::string text;
    for (size_t s = 0; s < computed.inputs.size(); ++s) {
        if (binary->GetString(computed.inputs[s].name, &text) &&
            text == "/Asset/Rig/Controls/Dial.avars:tx") {
            slot = s;
        }
    }
    CHECK(slot < computed.inputs.size());
    CHECK(computed.frames.size() == frames.size());
    if (slot >= computed.inputs.size() ||
        computed.frames.size() != frames.size()) {
        return;
    }
    // values[0] is Double +0.0.
    computed.frames[1].values[slot] = 0;
    computed.frames[1].hasValue[slot] = 1;
    std::vector<uint8_t> payload;
    CHECK(RigExecWireEncodeComputed(computed, &payload, &error));
    const std::vector<uint8_t> tampered = _ReplaceSection(
        result.bytes, RigExecBinarySection::Computed, payload);

    std::vector<double> untouched, moved, ignored;
    CHECK(_RunTargetX(result.bytes, frames, true, "/Asset/TargetA",
                      &untouched, &error));
    CHECK(!_RunTargetX(tampered, frames, true, "/Asset/TargetA", &ignored,
                       &error));
    // The registered read of the altered slot (the dial's avar binding) is
    // compared first, against the avar table the record drives.
    CHECK(error.find("registered reads: cross-check mismatch at "
                     "avarBindings[0].input") != std::string::npos);
    std::printf("avar-driven dynamic envelope, altered slot value: %s\n",
                error.c_str());
    CHECK(_RunTargetX(tampered, frames, false, "/Asset/TargetA", &moved,
                      &error));
    CHECK(moved.size() == frames.size() && untouched.size() == frames.size());
    if (moved.size() == frames.size() && untouched.size() == frames.size()) {
        CHECK(moved[0] == untouched[0]);
        CHECK(moved[1] == 10.0 * double(0.125f) && moved[1] != untouched[1]);
        CHECK(moved[2] == untouched[2]);
    }
}

static void
TestAvarDrivenDynamicEnvelope()
{
    const char *const name = "avar-driven dynamic envelope";
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    uint64_t parityChecked = 0;
    _TestStage(name, _AvarDrivenStage(), frames, &parityChecked);
    uint64_t dynamicChecked = 0;
    std::vector<std::vector<float>> envelopes;
    CHECK(_EnvelopesMatchDynamic(
        name, _AvarDrivenStage(), frames,
        {{"/Asset/TargetA", "/Asset/Rig/Weights/Driven", 10.0},
         {"/Asset/TargetB", "/Asset/Rig/Weights/Modulated", 10.0},
         {"/Asset/TargetC", "/Asset/Rig/Weights/Blend", 10.0}},
        &dynamicChecked, &envelopes));
    // Every envelope is computed and compared at every frame: a Constraint
    // step that binds a weight object reruns every run.
    CHECK(parityChecked == 3 * frames.size());
    CHECK(dynamicChecked == 3 * frames.size());

    // The oracle's own arithmetic over the avar read at each frame, which
    // the envelopes above equal, with d the double avar narrowed to float:
    // (1 * d) * 0.625 + 0.125 clamped; (0.7 * d) * 0.625 + 0 strict; and
    // m = max(max(0, 0.25), (1 * d) * 0.5 + 0), then (m + (1 - 2m) * 0.25)
    // * 0.75, strict.
    const UsdStageRefPtr stage = _AvarDrivenStage();
    const UsdAttribute tx = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Controls/Dial.avars:tx"));
    CHECK(envelopes.size() == frames.size());
    size_t clamped = 0;
    size_t reassociated = 0;
    for (size_t f = 0; f < envelopes.size() && f < frames.size(); ++f) {
        double avar = 0.0;
        CHECK(tx.Get(&avar, UsdTimeCode(frames[f])));
        const float d = static_cast<float>(avar);
        float driven = (1.0f * d) * 0.625f + 0.125f;
        if (driven > 1.0f) {
            driven = 1.0f;
            ++clamped;
        }
        const float modulated = (0.7f * d) * 0.625f + 0.0f;
        const float other = 0.7f * (d * 0.625f) + 0.0f;
        reassociated += std::memcmp(&other, &modulated, sizeof(float)) != 0;
        const float m = std::max(std::max(0.0f, 0.25f),
                                 (1.0f * d) * 0.5f + 0.0f);
        const float blend = (m + (1.0f - 2.0f * m) * 0.25f) * 0.75f;
        CHECK(envelopes[f] ==
              (std::vector<float>{driven, modulated, blend}));
    }
    // Frames 9 and 10 exercise the clamp; the operation order shows.
    CHECK(clamped == 2);
    CHECK(reassociated > 0);
    _TestSlotValuesDriveEnvelopes();
    std::printf("%s: %llu + %llu envelope(s) cross-checked over %zu "
                "frames\n", name,
                static_cast<unsigned long long>(parityChecked),
                static_cast<unsigned long long>(dynamicChecked),
                frames.size());
}

// The avar-driven stage with a float math mover clamping the Dial avar
// to [0, 1]: each driver's walk crosses the chain's target, so the oracle
// reads the chain's double result through the generation's overlay, not
// the authored avar. The parity path compares the runtime against the
// baked program (cross-check on), the dynamic comparison against
// ExecReference and the dynamic oracle, and the envelopes must be the
// oracle's arithmetic over the clamped value: from frame 7 on that differs
// from the authored value's.
static void
TestChainDrivenEnvelope()
{
    const char *const name = "chain-driven envelope";
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    uint64_t parityChecked = 0;
    _TestStage(name, _AvarDrivenStage(true), frames, &parityChecked);
    uint64_t dynamicChecked = 0;
    std::vector<std::vector<float>> envelopes;
    CHECK(_EnvelopesMatchDynamic(
        name, _AvarDrivenStage(true), frames,
        {{"/Asset/TargetA", "/Asset/Rig/Weights/Driven", 10.0},
         {"/Asset/TargetB", "/Asset/Rig/Weights/Modulated", 10.0},
         {"/Asset/TargetC", "/Asset/Rig/Weights/Blend", 10.0}},
        &dynamicChecked, &envelopes));
    // Per frame: three envelopes, the clamp chain's one published value,
    // and the avar binding of Dial.avars:tx, a registered read whose head
    // is the chain's target.
    CHECK(parityChecked == 3 * frames.size() + frames.size() + frames.size());
    CHECK(dynamicChecked == 3 * frames.size());
    const UsdStageRefPtr stage = _AvarDrivenStage(true);
    const UsdAttribute tx = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Controls/Dial.avars:tx"));
    CHECK(envelopes.size() == frames.size());
    size_t chained = 0;
    for (size_t f = 0; f < envelopes.size() && f < frames.size(); ++f) {
        double avar = 0.0;
        CHECK(tx.Get(&avar, UsdTimeCode(frames[f])));
        const float authored = static_cast<float>(avar);
        // The chain computes a double target in float and publishes the
        // double; the oracle narrows it back.
        const float d = std::min(std::max(authored, 0.0f), 1.0f);
        const float driven =
            std::min((1.0f * d) * 0.625f + 0.125f, 1.0f);
        const float modulated = (0.7f * d) * 0.625f + 0.0f;
        const float m = std::max(std::max(0.0f, 0.25f),
                                 (1.0f * d) * 0.5f + 0.0f);
        const float blend = (m + (1.0f - 2.0f * m) * 0.25f) * 0.75f;
        CHECK(envelopes[f] ==
              (std::vector<float>{driven, modulated, blend}));
        chained += d != authored;
    }
    CHECK(chained == 4);
    std::printf("%s: %llu value(s) (envelopes, property values, chain "
                "reads) + %llu "
                "envelope(s) cross-checked, %zu frame(s) read through the "
                "chain\n", name,
                static_cast<unsigned long long>(parityChecked),
                static_cast<unsigned long long>(dynamicChecked), chained);
}

// \p bytes without section \p tag; everything else copied unchanged.
static std::vector<uint8_t>
_WithoutSection(const std::vector<uint8_t> &bytes, RigExecBinarySection tag)
{
    std::string error;
    const std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        return {};
    }
    RigExecBinaryWriter writer;
    std::string text;
    for (uint32_t id = 1; reader->GetString(id, &text); ++id) {
        CHECK(writer.AddString(text) == id);
    }
    for (uint32_t t = uint32_t(RigExecBinarySection::Manifest);
         t <= uint32_t(RigExecBinarySection::Computed); ++t) {
        const RigExecBinarySection section = RigExecBinarySection(t);
        const uint8_t *data = nullptr;
        size_t size = 0;
        if (section != tag && reader->FindSection(section, &data, &size)) {
            writer.AddSection(section, data, size);
        }
    }
    return writer.Finish();
}

// The text Open fails \p bytes with; empty when it opens.
static std::string
_OpenError(const std::vector<uint8_t> &bytes)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    return reader ? std::string() : error;
}

// Open refuses a file whose envelopes the Computed section does not
// cover, each with its own text: no section at all, a malformed one, one
// that does not match the tables it extends, an envelope index naming
// another object, and a weight-object read that is not a float.
static void
TestComputedOpenRefusals()
{
    const UsdStageRefPtr stage = _EnvelopeStage();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = {1.0, 5.0};
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(_OpenError(result.bytes).empty());
    const std::unique_ptr<RigExecBinaryReader> binary =
        RigExecBinaryReader::Open(result.bytes.data(), result.bytes.size(),
                                  &error);
    const uint8_t *data = nullptr;
    size_t size = 0;
    RigExecWireComputed computed;
    if (!binary ||
        !binary->FindSection(RigExecBinarySection::Computed, &data, &size)) {
        CHECK(false);
        return;
    }
    const std::vector<uint8_t> original(data, data + size);
    {
        RigExecWireReader cursor(data, size);
        CHECK(RigExecWireDecodeComputed(&cursor, &computed, &error));
    }
    const auto expect = [&](const char *what,
                            const std::vector<uint8_t> &bytes,
                            const std::string &text) {
        const std::string got = _OpenError(bytes);
        if (got != text) {
            std::printf("%s: open said '%s', expected '%s'\n", what,
                        got.c_str(), text.c_str());
            CHECK(false);
        }
    };
    const auto with = [&](const RigExecWireComputed &edited) {
        std::vector<uint8_t> payload;
        std::string why;
        CHECK(RigExecWireEncodeComputed(edited, &payload, &why));
        return _ReplaceSection(result.bytes, RigExecBinarySection::Computed,
                               payload);
    };

    expect("no computed section",
           _WithoutSection(result.bytes, RigExecBinarySection::Computed),
           "constraint /Asset/Rig/Movers/B resolves an envelope, which needs "
           "the computed section this file does not carry; rebake it");
    {
        std::vector<uint8_t> truncated = original;
        truncated.pop_back();
        const std::string got = _OpenError(_ReplaceSection(
            result.bytes, RigExecBinarySection::Computed, truncated));
        CHECK(got.rfind("malformed computed section: computed section: ",
                        0) == 0);
        std::printf("truncated computed section: %s\n", got.c_str());
    }
    {
        RigExecWireComputed edited = computed;
        edited.frames.pop_back();
        expect("one frame short", with(edited),
               "computed section: one slot record per input-table frame "
               "expected");
    }
    // The pose table orders constraint B before A.
    CHECK(computed.constraintWeightObjectIndex.size() == 2);
    if (computed.constraintWeightObjectIndex.size() == 2) {
        RigExecWireComputed edited = computed;
        std::swap(edited.constraintWeightObjectIndex[0],
                  edited.constraintWeightObjectIndex[1]);
        expect("swapped envelope indices", with(edited),
               "the computed section gives constraint /Asset/Rig/Movers/B "
               "the envelope of /Asset/Rig/Weights/Fixed, not of its "
               "weight object /Asset/Rig/Weights/Driven");
        edited = computed;
        edited.constraintWeightObjectIndex[0] = -1;
        expect("missing envelope index", with(edited),
               "constraint /Asset/Rig/Movers/B resolves an envelope the "
               "computed section does not carry");
    }
    {
        // A double-tagged read (its constant re-pointed at values[0], the
        // Double +0.0, so the section itself stays consistent).
        RigExecWireComputed edited = computed;
        CHECK(!edited.weightObjects.empty());
        if (!edited.weightObjects.empty()) {
            v4::RigExecWireWeightObject &object = edited.weightObjects.back();
            object.driver.tag = v4::InputTag::Double;
            object.driver.constant = 0;
            std::string text;
            CHECK(binary->GetString(object.path, &text));
            expect("double read", with(edited),
                   "weight object " + text +
                       " carries a read that is not a float");
        }
    }
    std::printf("computed open refusals: checked\n");
}

// The runtime's property results against the dynamic evaluator's
// published values (RigExecRigPose::movedProperties): same type, same bits.
static bool
_SamePropertyValue(const VtValue &want, const RrPropertyValue &got)
{
    using Tag = RrPropertyValue::Tag;
    if (want.IsHolding<float>()) {
        const float w = want.UncheckedGet<float>();
        return got.tag == Tag::Float &&
               std::memcmp(&w, &got.f32, sizeof(float)) == 0;
    }
    if (want.IsHolding<double>()) {
        const double w = want.UncheckedGet<double>();
        return got.tag == Tag::Double &&
               std::memcmp(&w, &got.f64, sizeof(double)) == 0;
    }
    if (want.IsHolding<GfMatrix4d>()) {
        const GfMatrix4d &w = want.UncheckedGet<GfMatrix4d>();
        if (got.tag != Tag::Matrix4d) {
            return false;
        }
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                const double a = w[r][c];
                const double b = got.matrix[size_t(r)][size_t(c)];
                if (std::memcmp(&a, &b, sizeof(double)) != 0) {
                    return false;
                }
            }
        }
        return true;
    }
    if (want.IsHolding<GfVec3f>()) {
        const GfVec3f &w = want.UncheckedGet<GfVec3f>();
        return got.tag == Tag::Vec3f &&
               std::memcmp(w.data(), got.vec.data(), sizeof(float) * 3) == 0;
    }
    return false;
}

// A value only a property chain publishes into movedProperties.
static bool
_IsPropertyResult(const VtValue &value)
{
    return value.IsHolding<float>() || value.IsHolding<double>() ||
           value.IsHolding<GfMatrix4d>() || value.IsHolding<GfVec3f>();
}

// The lines property chains write.
static std::vector<std::string>
_ChainLines(const std::vector<std::string> &lines)
{
    std::vector<std::string> out;
    for (const std::string &line : lines) {
        if (line.rfind("property chain ", 0) == 0 ||
            line.rfind("diag ", 0) == 0) {
            out.push_back(line);
        }
    }
    return out;
}

static bool
_SameBits(const RrPropertyValue &a, const RrPropertyValue &b)
{
    using Tag = RrPropertyValue::Tag;
    if (a.tag != b.tag) {
        return false;
    }
    switch (a.tag) {
    case Tag::Float:
        return std::memcmp(&a.f32, &b.f32, sizeof(float)) == 0;
    case Tag::Double:
        return std::memcmp(&a.f64, &b.f64, sizeof(double)) == 0;
    case Tag::Matrix4d:
        for (size_t r = 0; r < 4; ++r) {
            if (std::memcmp(a.matrix[r], b.matrix[r], sizeof(double) * 4) !=
                0) {
                return false;
            }
        }
        return true;
    case Tag::Vec3f:
        return std::memcmp(a.vec.data(), b.vec.data(), sizeof(float) * 3) ==
               0;
    }
    return false;
}

// One frame of a chain case: the runtime's property results by path, and
// the chains' diagnostic lines.
struct _ChainFrame {
    std::map<std::string, RrPropertyValue> values;
    std::vector<std::string> lines;
};

// Plays \p stage's bake with the cross-check on beside an ExecReference
// evaluator (the exec-authoritative walk every evaluator is held to). At
// every frame each property result the runtime computed must equal the
// dynamic evaluator's published value bit for bit, the two must publish
// the same set, and the chains' diagnostic lines must agree verbatim.
// \p rows receives the runtime's results and lines per frame; \p checked
// the property values the cross-check compared. False on any difference.
static bool
_ChainsMatchDynamic(const std::string &name, const UsdStageRefPtr &stage,
                    const std::vector<double> &frames, uint64_t *checked,
                    std::vector<_ChainFrame> *rows)
{
    const SdfPath rigPath = _FindRig(stage);
    RigExecRigEvaluator baked(stage, rigPath);
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult result;
    std::string error;
    if (!RigExecBakeToBinary(baked, opts, &result, &error)) {
        std::printf("%s: bake failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(),
                                   &error);
    if (!reader) {
        std::printf("%s: open failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
    reader->SetCrossCheckForTesting(true);
    RigExecRigEvaluator dynamic(stage, rigPath);
    dynamic.SetEvaluationMode(RigExecEvaluationMode::ExecReference);

    bool same = true;
    for (double frame : frames) {
        const RigExecRigPose want = dynamic.Evaluate(UsdTimeCode(frame));
        if (!want.valid) {
            std::printf("%s frame %g: no dynamic pose\n", name.c_str(),
                        frame);
            return false;
        }
        if (!reader->SetFrame(frame, &error) || !reader->Execute(&error)) {
            std::printf("%s frame %g: %s\n", name.c_str(), frame,
                        error.c_str());
            return false;
        }
        _ChainFrame row;
        for (const RigExecRuntimePropertyValue &got :
             reader->GetPropertyResultsForTesting()) {
            row.values[got.path] = got.value;
            const auto found = want.movedProperties.find(SdfPath(got.path));
            if (found == want.movedProperties.end() ||
                !_SamePropertyValue(found->second, got.value)) {
                std::printf("%s frame %g: %s differs from the dynamic "
                            "evaluator (%s)\n", name.c_str(), frame,
                            got.path.c_str(),
                            found == want.movedProperties.end()
                                ? "not published there"
                                : "value");
                same = false;
            }
        }
        for (const auto &[path, value] : want.movedProperties) {
            if (_IsPropertyResult(value) &&
                !row.values.count(path.GetString())) {
                std::printf("%s frame %g: the runtime publishes nothing at "
                            "%s\n", name.c_str(), frame,
                            path.GetText());
                same = false;
            }
        }
        const std::vector<std::string> got =
            _ChainLines(reader->GetDiagnostics());
        const std::vector<std::string> expected =
            _ChainLines(want.diagnostics);
        if (got != expected) {
            std::printf("%s frame %g: chain lines differ (%zu vs %zu)\n",
                        name.c_str(), frame, got.size(), expected.size());
            for (size_t i = 0; i < std::max(got.size(), expected.size());
                 ++i) {
                std::printf("  got:  %s\n  want: %s\n",
                            i < got.size() ? got[i].c_str() : "",
                            i < expected.size() ? expected[i].c_str() : "");
            }
            same = false;
        }
        row.lines = got;
        rows->push_back(std::move(row));
    }
    *checked = reader->GetCrossCheckPropertyValueCountForTesting();
    // Property chains only: nothing else computed is in these rigs.
    if (reader->GetCrossCheckEnvelopeCountForTesting() != 0 ||
        reader->GetCrossCheckPhasePacketCountForTesting() != 0) {
        std::printf("%s: compared envelopes or packets it has none of\n",
                    name.c_str());
        same = false;
    }
    return same;
}

// A rig of property movers only: channels under /Asset/Rig/Channels,
// movers under /Asset/Rig/Movers, frames 1 to 10.
static UsdStageRefPtr
_ChainBaseStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(10.0);
    UsdGeomXform::Define(stage, SdfPath("/Asset"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Channels"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    return stage;
}

static UsdAttribute
_Channel(const UsdStageRefPtr &stage, const char *scope, const char *name,
         const SdfValueTypeName &type)
{
    const UsdPrim prim = stage->DefinePrim(
        SdfPath(std::string("/Asset/Rig/Channels/") + scope),
        TfToken("Scope"));
    return prim.CreateAttribute(TfToken(name), type);
}

// A math mover at /Asset/Rig/Movers/<path> moving \p target. A nested
// mover revises before its parent, which is how a case fixes the order.
static UsdPrim
_MathMover(const UsdStageRefPtr &stage, const std::string &path,
           const char *type, const char *operation,
           const UsdAttribute &target)
{
    const UsdPrim prim =
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers/" + path), TfToken(type));
    CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
    prim.CreateAttribute(TfToken("rigExec:operation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken(operation));
    prim.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({target.GetPath()});
    return prim;
}

template <class T>
static void
_SetInput(const UsdPrim &prim, const char *name, const SdfValueTypeName &type,
          const T &value)
{
    prim.CreateAttribute(TfToken(name), type).Set(value);
}

static const GfVec2f kShapeKeys[] = {
    GfVec2f(0.0f, 0.0f), GfVec2f(0.5f, 0.8f), GfVec2f(2.0f, 1.2f)};
static const GfVec2f kShapeTangents[] = {
    GfVec2f(1.5f, 1.5f), GfVec2f(0.6f, 0.6f), GfVec2f(0.1f, 0.1f)};

// A float chain: a Hermite curve with tangents, then a clamp to [0.1, 1],
// over a keyed base that leaves the curve's keys on both sides. Beside it
// a curve of twelve linear keys (no tangents, the binary-search segment
// lookup) on a second channel.
static UsdStageRefPtr
_FloatCurveClampStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(-0.25f, UsdTimeCode(1.0));
    amount.Set(0.75f, UsdTimeCode(4.0));
    amount.Set(1.5f, UsdTimeCode(7.0));
    amount.Set(3.0f, UsdTimeCode(10.0));
    const UsdPrim limit = _MathMover(stage, "Limit", "RigExecFloatMathMover",
                                     "clamp", amount);
    _SetInput(limit, "inputs:min", SdfValueTypeNames->Float, 0.1f);
    _SetInput(limit, "inputs:max", SdfValueTypeNames->Float, 1.0f);
    const UsdPrim shape = _MathMover(stage, "Limit/Shape",
                                     "RigExecFloatMathMover", "curve", amount);
    _SetInput(shape, "inputs:keys", SdfValueTypeNames->Float2Array,
              VtArray<GfVec2f>(std::begin(kShapeKeys), std::end(kShapeKeys)));
    _SetInput(shape, "inputs:tangents", SdfValueTypeNames->Float2Array,
              VtArray<GfVec2f>(std::begin(kShapeTangents),
                               std::end(kShapeTangents)));

    const UsdAttribute ramp =
        _Channel(stage, "Ramp", "rigExec:amount", SdfValueTypeNames->Float);
    ramp.Set(-1.0f, UsdTimeCode(1.0));
    ramp.Set(12.5f, UsdTimeCode(10.0));
    VtArray<GfVec2f> steps;
    for (int i = 0; i < 12; ++i) {
        steps.push_back(GfVec2f(float(i), float(i * i) * 0.125f));
    }
    const UsdPrim staircase = _MathMover(
        stage, "Staircase", "RigExecFloatMathMover", "curve", ramp);
    _SetInput(staircase, "inputs:keys", SdfValueTypeNames->Float2Array,
              steps);
    return stage;
}

// A double chain multiplied by 1.25 computed in float, over a base keyed
// between values a float cannot hold exactly.
static UsdStageRefPtr
_DoubleChainStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute level =
        _Channel(stage, "Wide", "rigExec:level", SdfValueTypeNames->Double);
    level.Set(0.1, UsdTimeCode(1.0));
    level.Set(1.0 / 3.0, UsdTimeCode(10.0));
    const UsdPrim scale = _MathMover(stage, "Scale", "RigExecFloatMathMover",
                                     "multiply", level);
    _SetInput(scale, "inputs:value", SdfValueTypeNames->Float, 1.1f);
    return stage;
}

// A matrix chain: multiply by a rotation and translation under
// inputs:defaultWeight 0.5, then blend toward a translation under 0.25,
// then a multiply whose operand is not finite (inputs unusable).
static UsdStageRefPtr
_MatrixChainStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute local =
        _Channel(stage, "Space", "rigExec:local", SdfValueTypeNames->Matrix4d);
    local.Set(GfMatrix4d(GfRotation(GfVec3d(0, 1, 0), 10.0),
                         GfVec3d(1.0, 2.0, 3.0)),
              UsdTimeCode(1.0));
    local.Set(GfMatrix4d(GfRotation(GfVec3d(0, 1, 0), 70.0),
                         GfVec3d(4.0, -2.0, 0.5)),
              UsdTimeCode(10.0));
    const UsdPrim bad = _MathMover(stage, "Bad", "RigExecMatrixMathMover",
                                   "multiply", local);
    GfMatrix4d infinite(1.0);
    infinite[3][0] = std::numeric_limits<double>::infinity();
    _SetInput(bad, "inputs:value", SdfValueTypeNames->Matrix4d, infinite);
    const UsdPrim snap = _MathMover(stage, "Bad/Snap",
                                    "RigExecMatrixMathMover", "blend", local);
    _SetInput(snap, "inputs:value", SdfValueTypeNames->Matrix4d,
              GfMatrix4d(1.0).SetTranslate(GfVec3d(2.0, 2.0, 2.0)));
    _SetInput(snap, "inputs:defaultWeight", SdfValueTypeNames->Float, 0.25f);
    const UsdPrim offset = _MathMover(stage, "Bad/Snap/Offset",
                                      "RigExecMatrixMathMover", "multiply",
                                      local);
    _SetInput(offset, "inputs:value", SdfValueTypeNames->Matrix4d,
              GfMatrix4d(GfRotation(GfVec3d(0, 0, 1), 30.0),
                         GfVec3d(0.0, 5.0, 0.0)));
    _SetInput(offset, "inputs:defaultWeight", SdfValueTypeNames->Float, 0.5f);
    return stage;
}

// A vec3f chain: add, then clamp per component, then remap under 0.5.
static UsdStageRefPtr
_Vec3fChainStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute offset =
        _Channel(stage, "Vec", "rigExec:offset", SdfValueTypeNames->Float3);
    offset.Set(GfVec3f(0.0f, 0.0f, 0.0f), UsdTimeCode(1.0));
    offset.Set(GfVec3f(2.0f, 5.0f, -3.0f), UsdTimeCode(10.0));
    const UsdPrim spread = _MathMover(stage, "Spread",
                                      "RigExecVec3fMathMover", "remap", offset);
    _SetInput(spread, "inputs:min", SdfValueTypeNames->Float3,
              GfVec3f(-2.0f, -2.0f, -2.0f));
    _SetInput(spread, "inputs:max", SdfValueTypeNames->Float3,
              GfVec3f(2.0f, 3.0f, 2.0f));
    _SetInput(spread, "inputs:defaultWeight", SdfValueTypeNames->Float, 0.5f);
    const UsdPrim bound = _MathMover(stage, "Spread/Bound",
                                     "RigExecVec3fMathMover", "clamp", offset);
    _SetInput(bound, "inputs:min", SdfValueTypeNames->Float3,
              GfVec3f(-1.0f, -1.0f, -1.0f));
    _SetInput(bound, "inputs:max", SdfValueTypeNames->Float3,
              GfVec3f(1.0f, 4.5f, 1.0f));
    const UsdPrim lift = _MathMover(stage, "Spread/Bound/Lift",
                                    "RigExecVec3fMathMover", "add", offset);
    _SetInput(lift, "inputs:value", SdfValueTypeNames->Float3,
              GfVec3f(0.0f, 2.0f, 0.0f));
    return stage;
}

// Disabled revisions: one always (inputs:enabled false), one keyed off from
// frame 4 to frame 7, each passing the value through with its line.
static UsdStageRefPtr
_DisabledStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(0.2f, UsdTimeCode(1.0));
    amount.Set(0.8f, UsdTimeCode(10.0));
    const UsdPrim off = _MathMover(stage, "Off", "RigExecFloatMathMover",
                                   "add", amount);
    _SetInput(off, "inputs:value", SdfValueTypeNames->Float, 100.0f);
    _SetInput(off, "inputs:enabled", SdfValueTypeNames->Bool, false);
    const UsdPrim toggle = _MathMover(stage, "Off/Toggle",
                                      "RigExecFloatMathMover", "multiply",
                                      amount);
    _SetInput(toggle, "inputs:value", SdfValueTypeNames->Float, 2.0f);
    const UsdAttribute enabled = toggle.CreateAttribute(
        TfToken("inputs:enabled"), SdfValueTypeNames->Bool);
    enabled.Set(true, UsdTimeCode(1.0));
    enabled.Set(false, UsdTimeCode(4.0));
    enabled.Set(true, UsdTimeCode(8.0));
    return stage;
}

// Envelopes on property revisions: a blend under a StaticWeight 0.5, an add
// under a clamped DynamicWeight an avar drives, an add under a strict one
// the avar drives out of range from frame 8 (the oracle's error passes the
// revision through), an add under inputs:defaultWeight 0.25, and three
// whose defaultWeight is out of range: 1.5, -0.25 and NaN.
static UsdStageRefPtr
_EnvelopeChainStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdPrim dial = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Dial"), TfToken("RigExecControl"));
    const UsdAttribute ty =
        dial.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double);
    ty.Set(0.0, UsdTimeCode(1.0));
    ty.Set(3.0, UsdTimeCode(10.0));
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(0.1f, UsdTimeCode(1.0));
    amount.Set(0.45f, UsdTimeCode(5.0));
    amount.Set(0.2f, UsdTimeCode(10.0));

    const UsdPrim half = stage->DefinePrim(SdfPath("/Asset/Rig/Weights/Half"),
                                           TfToken("RigExecStaticWeight"));
    half.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({amount.GetPath()});
    half.CreateAttribute(TfToken("rigExec:representation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    half.CreateAttribute(TfToken("rigExec:defaultWeight"),
                         SdfValueTypeNames->Float)
        .Set(0.5f);
    const auto dynamicWeight = [&](const char *name, float scale,
                                   const char *rangePolicy) {
        const UsdPrim w = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Weights/") + name),
            TfToken("RigExecDynamicWeight"));
        w.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({amount.GetPath()});
        w.CreateAttribute(TfToken("rigExec:representation"),
                          SdfValueTypeNames->Token)
            .Set(TfToken("constant"));
        w.CreateAttribute(TfToken("rigExec:rangePolicy"),
                          SdfValueTypeNames->Token)
            .Set(TfToken(rangePolicy));
        w.CreateAttribute(TfToken("inputs:driver"), SdfValueTypeNames->Float)
            .AddConnection(ty.GetPath());
        w.CreateAttribute(TfToken("inputs:scale"), SdfValueTypeNames->Float)
            .Set(scale);
        return w;
    };
    const UsdPrim ramp = dynamicWeight("Ramp", 0.5f, "clamp");
    const UsdPrim steep = dynamicWeight("Steep", 0.45f, "strict");

    const UsdPrim over = _MathMover(stage, "Over", "RigExecFloatMathMover",
                                    "add", amount);
    _SetInput(over, "inputs:value", SdfValueTypeNames->Float, 100.0f);
    _SetInput(over, "inputs:defaultWeight", SdfValueTypeNames->Float, 1.5f);
    const UsdPrim soft = _MathMover(stage, "Over/Soft",
                                    "RigExecFloatMathMover", "add", amount);
    _SetInput(soft, "inputs:value", SdfValueTypeNames->Float, 0.5f);
    _SetInput(soft, "inputs:defaultWeight", SdfValueTypeNames->Float, 0.25f);
    const UsdPrim kick = _MathMover(stage, "Over/Soft/Kick",
                                    "RigExecFloatMathMover", "add", amount);
    _SetInput(kick, "inputs:value", SdfValueTypeNames->Float, 0.125f);
    kick.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({steep.GetPath()});
    const UsdPrim nudge = _MathMover(stage, "Over/Soft/Kick/Nudge",
                                     "RigExecFloatMathMover", "add", amount);
    _SetInput(nudge, "inputs:value", SdfValueTypeNames->Float, 0.05f);
    nudge.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({ramp.GetPath()});
    const UsdPrim fade = _MathMover(stage, "Over/Soft/Kick/Nudge/Fade",
                                    "RigExecFloatMathMover", "blend", amount);
    _SetInput(fade, "inputs:value", SdfValueTypeNames->Float, 0.3f);
    fade.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({half.GetPath()});
    // Refused at both other edges of the defaultWeight range: below 0 (an
    // envelope that would otherwise leave the value as it is) and NaN.
    const UsdPrim negative =
        _MathMover(stage, "Over/Soft/Kick/Nudge/Fade/Negative",
                   "RigExecFloatMathMover", "add", amount);
    _SetInput(negative, "inputs:value", SdfValueTypeNames->Float, 100.0f);
    _SetInput(negative, "inputs:defaultWeight", SdfValueTypeNames->Float,
              -0.25f);
    const UsdPrim notANumber =
        _MathMover(stage, "Over/Soft/Kick/Nudge/Fade/Negative/NotANumber",
                   "RigExecFloatMathMover", "add", amount);
    _SetInput(notANumber, "inputs:value", SdfValueTypeNames->Float, 100.0f);
    _SetInput(notANumber, "inputs:defaultWeight", SdfValueTypeNames->Float,
              std::numeric_limits<float>::quiet_NaN());
    return stage;
}

// Phased consumers: a float chain (multiply by 2, then add 0.25) read at
// its base and after its first revision, and a double chain's base read
// into a float input, each by a float math mover whose result is a chain of
// its own.
static UsdStageRefPtr
_PhasedStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(0.1f, UsdTimeCode(1.0));
    amount.Set(0.7f, UsdTimeCode(10.0));
    const UsdPrim bias = _MathMover(stage, "Bias", "RigExecFloatMathMover",
                                    "add", amount);
    _SetInput(bias, "inputs:value", SdfValueTypeNames->Float, 0.25f);
    const UsdPrim gain = _MathMover(stage, "Bias/Gain",
                                    "RigExecFloatMathMover", "multiply",
                                    amount);
    _SetInput(gain, "inputs:value", SdfValueTypeNames->Float, 2.0f);
    const UsdAttribute level =
        _Channel(stage, "Wide", "rigExec:level", SdfValueTypeNames->Double);
    level.Set(0.1, UsdTimeCode(1.0));
    level.Set(1.0 / 3.0, UsdTimeCode(10.0));
    const UsdPrim scale = _MathMover(stage, "Scale", "RigExecFloatMathMover",
                                     "multiply", level);
    _SetInput(scale, "inputs:value", SdfValueTypeNames->Float, 1.25f);

    const auto readout = [&](const char *name, const UsdAttribute &source,
                             const char *phase) {
        const UsdAttribute target =
            _Channel(stage, "Readouts", (std::string("rigExec:") + name).c_str(),
                     SdfValueTypeNames->Float);
        target.Set(0.0f);
        const UsdPrim mover =
            _MathMover(stage, std::string("Readouts/") + name,
                       "RigExecFloatMathMover", "add", target);
        const UsdAttribute value = mover.CreateAttribute(
            TfToken("inputs:value"), SdfValueTypeNames->Float);
        value.AddConnection(source.GetPath());
        value.SetMetadata(TfToken("rigExecReadPhase"), std::string(phase));
    };
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Readouts"), TfToken("Scope"));
    readout("base", amount, "base");
    readout("early", amount, "/Asset/Rig/Movers/Bias/Gain");
    readout("narrow", level, "base");
    return stage;
}

// The default read phase: a dial keyed from 0.1 to 0.45, doubled by Gain and
// clamped to 0.6 by Limit, read by add movers into readout channels --
// undeclared (the base), `final` (no record: the dial's published value
// answers it), `base` on a hop, `final` through that hop (a record of every
// revision), and undeclared through a `final` hop (the base).
static UsdStageRefPtr
_DefaultPhaseStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute amount =
        _Channel(stage, "Dial", "rigExec:amount", SdfValueTypeNames->Float);
    amount.Set(0.1f, UsdTimeCode(1.0));
    amount.Set(0.45f, UsdTimeCode(10.0));
    const UsdPrim limit = _MathMover(stage, "Limit", "RigExecFloatMathMover",
                                     "clamp", amount);
    _SetInput(limit, "inputs:min", SdfValueTypeNames->Float, 0.0f);
    _SetInput(limit, "inputs:max", SdfValueTypeNames->Float, 0.6f);
    const UsdPrim gain = _MathMover(stage, "Limit/Gain",
                                    "RigExecFloatMathMover", "multiply",
                                    amount);
    _SetInput(gain, "inputs:value", SdfValueTypeNames->Float, 2.0f);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Readouts"), TfToken("Scope"));
    const auto readout = [&](const char *name, const SdfPath &source,
                             const char *phase) {
        const UsdAttribute target =
            _Channel(stage, "Readouts", (std::string("rigExec:") + name).c_str(),
                     SdfValueTypeNames->Float);
        target.Set(0.0f);
        const UsdPrim mover =
            _MathMover(stage, std::string("Readouts/") + name,
                       "RigExecFloatMathMover", "add", target);
        const UsdAttribute value = mover.CreateAttribute(
            TfToken("inputs:value"), SdfValueTypeNames->Float);
        value.AddConnection(source);
        if (phase) {
            value.SetMetadata(TfToken("rigExecReadPhase"), std::string(phase));
        }
    };
    readout("undeclared", amount.GetPath(), nullptr);
    readout("final", amount.GetPath(), "final");
    readout("hop", amount.GetPath(), "base");
    readout("finalViaHop",
            SdfPath("/Asset/Rig/Movers/Readouts/hop.inputs:value"), "final");
    readout("finalHop", amount.GetPath(), "final");
    readout("baseViaFinalHop",
            SdfPath("/Asset/Rig/Movers/Readouts/finalHop.inputs:value"),
            nullptr);
    return stage;
}

// Bases a chain cannot use and results it refuses: a float base keyed to
// +inf at frame 3 and NaN at frame 5; a double base of 1e300, finite as a
// double but not as the float it is tested through; a multiply that
// overflows a float from frame 7 on; an add whose input is +inf.
static UsdStageRefPtr
_NonFiniteStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const UsdAttribute bad =
        _Channel(stage, "Bad", "rigExec:amount", SdfValueTypeNames->Float);
    bad.Set(0.5f, UsdTimeCode(1.0));
    bad.Set(std::numeric_limits<float>::infinity(), UsdTimeCode(3.0));
    bad.Set(std::numeric_limits<float>::quiet_NaN(), UsdTimeCode(5.0));
    bad.Set(0.25f, UsdTimeCode(7.0));
    const UsdPrim plus = _MathMover(stage, "Plus", "RigExecFloatMathMover",
                                    "add", bad);
    _SetInput(plus, "inputs:value", SdfValueTypeNames->Float, 1.0f);

    const UsdAttribute huge =
        _Channel(stage, "Huge", "rigExec:level", SdfValueTypeNames->Double);
    huge.Set(1e300);
    const UsdPrim keep = _MathMover(stage, "Keep", "RigExecFloatMathMover",
                                    "multiply", huge);
    _SetInput(keep, "inputs:value", SdfValueTypeNames->Float, 1.0f);

    const UsdAttribute blow =
        _Channel(stage, "Blow", "rigExec:amount", SdfValueTypeNames->Float);
    blow.Set(0.5f, UsdTimeCode(1.0));
    blow.Set(2.0f, UsdTimeCode(7.0));
    const UsdPrim inf = _MathMover(stage, "Inf", "RigExecFloatMathMover",
                                   "add", blow);
    _SetInput(inf, "inputs:value", SdfValueTypeNames->Float,
              std::numeric_limits<float>::infinity());
    const UsdPrim overflow = _MathMover(stage, "Inf/Overflow",
                                        "RigExecFloatMathMover", "multiply",
                                        blow);
    _SetInput(overflow, "inputs:value", SdfValueTypeNames->Float, 3e38f);
    return stage;
}

static float
_Float(const _ChainFrame &row, const std::string &path, bool *found)
{
    const auto it = row.values.find(path);
    *found = it != row.values.end() &&
             it->second.tag == RrPropertyValue::Tag::Float;
    return *found ? it->second.f32 : 0.0f;
}

static bool
_HasLine(const _ChainFrame &row, const std::string &line)
{
    return std::find(row.lines.begin(), row.lines.end(), line) !=
           row.lines.end();
}

// One chain case: baked parity with verbatim diagnostics and the
// cross-check on, then the dynamic comparison. Both readers must compare
// exactly \p expected property values over the frames.
static bool
_RunChainCase(const char *name, UsdStageRefPtr (*build)(),
              const std::vector<double> &frames, uint64_t expected,
              std::vector<_ChainFrame> *rows)
{
    const int failuresBefore = failures;
    uint64_t parity = 0;
    _TestStage(name, build(), frames, &parity);
    uint64_t dynamicChecked = 0;
    CHECK(_ChainsMatchDynamic(name, build(), frames, &dynamicChecked, rows));
    // These rigs compute nothing but property chains.
    CHECK(parity == expected);
    CHECK(dynamicChecked == expected);
    CHECK(dynamicChecked > 0);
    CHECK(rows->size() == frames.size());
    std::printf("%s: %llu + %llu property value(s) cross-checked\n", name,
                static_cast<unsigned long long>(parity),
                static_cast<unsigned long long>(dynamicChecked));
    return failures == failuresBefore && rows->size() == frames.size();
}


static void
TestFloatChainCurveClamp()
{
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_ChainFrame> rows;
    // Two chains, each published at every frame.
    if (!_RunChainCase("float chain, curve then clamp",
                       _FloatCurveClampStage, frames, 2 * frames.size(),
                       &rows)) {
        return;
    }
    // The evaluator's own kernels over the authored base: the Hermite
    // curve, then the clamp; and the twelve linear keys.
    const UsdStageRefPtr stage = _FloatCurveClampStage();
    const UsdAttribute amount = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
    const UsdAttribute ramp = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Ramp.rigExec:amount"));
    const UsdAttribute steps = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Movers/Staircase.inputs:keys"));
    VtArray<GfVec2f> staircase;
    CHECK(steps.Get(&staircase));
    size_t low = 0, high = 0, inside = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        float base = 0.0f;
        CHECK(amount.Get(&base, UsdTimeCode(frames[f])));
        RigExecPropertyMathParams<float> curve;
        curve.op = RigExecPropertyOp::Curve;
        curve.keys = kShapeKeys;
        curve.keyCount = 3;
        curve.tangents = kShapeTangents;
        curve.tangentCount = 3;
        RigExecPropertyMathParams<float> clamp;
        clamp.op = RigExecPropertyOp::Clamp;
        clamp.min = 0.1f;
        clamp.max = 1.0f;
        const float expected = RigExecApplyFloatMath(
            RigExecApplyFloatMath(base, curve), clamp);
        bool found = false;
        const float got =
            _Float(rows[f], "/Asset/Rig/Channels/Dial.rigExec:amount", &found);
        CHECK(found && std::memcmp(&got, &expected, sizeof(float)) == 0);
        low += got == 0.1f;
        high += got == 1.0f;
        inside += got > 0.1f && got < 1.0f;

        float x = 0.0f;
        CHECK(ramp.Get(&x, UsdTimeCode(frames[f])));
        const float linear = RigExecEvaluateLinearKeys(
            staircase.cdata(), staircase.size(), x);
        const float gotLinear =
            _Float(rows[f], "/Asset/Rig/Channels/Ramp.rigExec:amount", &found);
        CHECK(found && std::memcmp(&gotLinear, &linear, sizeof(float)) == 0);
    }
    // The clamp holds both bounds at some frames and neither at others.
    CHECK(low > 0 && high > 0 && inside > 0);
}

static void
TestDoubleChainInFloat()
{
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("double chain computed in float", _DoubleChainStage,
                       frames, frames.size(), &rows)) {
        return;
    }
    const UsdStageRefPtr stage = _DoubleChainStage();
    const UsdAttribute level = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Wide.rigExec:level"));
    size_t narrowed = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        double base = 0.0;
        CHECK(level.Get(&base, UsdTimeCode(frames[f])));
        // Narrowed, multiplied in float, widened back.
        const double expected = double(float(base) * 1.1f);
        const auto it =
            rows[f].values.find("/Asset/Rig/Channels/Wide.rigExec:level");
        CHECK(it != rows[f].values.end() &&
              it->second.tag == RrPropertyValue::Tag::Double &&
              std::memcmp(&it->second.f64, &expected, sizeof(double)) == 0);
        narrowed += it != rows[f].values.end() &&
                    it->second.f64 != base * double(1.1f);
    }
    // Computing in double would have given other bits at some frames.
    CHECK(narrowed > 0);
    std::printf("double chain computed in float: %zu of %zu frame(s) differ "
                "from the double product\n",
                narrowed, frames.size());
}

static void
TestMatrixChain()
{
    const std::vector<double> frames = {1, 4, 7, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("matrix chain", _MatrixChainStage, frames,
                       frames.size(), &rows)) {
        return;
    }
    // The evaluator's kernels over the authored base: multiply under 0.5,
    // then blend under 0.25; the non-finite multiply passes through with its
    // line.
    const UsdStageRefPtr stage = _MatrixChainStage();
    const UsdAttribute local = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Space.rigExec:local"));
    const GfMatrix4d offset(GfRotation(GfVec3d(0, 0, 1), 30.0),
                            GfVec3d(0.0, 5.0, 0.0));
    const GfMatrix4d snap = GfMatrix4d(1.0).SetTranslate(GfVec3d(2, 2, 2));
    const std::string unusable =
        "diag /Asset/Rig/Movers/Bad: inputs unusable; revision passed through";
    for (size_t f = 0; f < frames.size(); ++f) {
        GfMatrix4d base;
        CHECK(local.Get(&base, UsdTimeCode(frames[f])));
        GfMatrix4d once, twice;
        CHECK(RigExecApplyMatrixMath(base, RigExecPropertyOp::Multiply,
                                     offset, 0.5f, &once));
        CHECK(RigExecApplyMatrixMath(once, RigExecPropertyOp::Blend, snap,
                                     0.25f, &twice));
        const auto it =
            rows[f].values.find("/Asset/Rig/Channels/Space.rigExec:local");
        CHECK(it != rows[f].values.end() &&
              _SamePropertyValue(VtValue(twice), it->second));
        CHECK(_HasLine(rows[f], unusable));
    }
}

static void
TestVec3fChain()
{
    const std::vector<double> frames = {1, 4, 7, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("vec3f chain", _Vec3fChainStage, frames,
                       frames.size(), &rows)) {
        return;
    }
    // Add, clamp, then remap under 0.5, component by component.
    const UsdStageRefPtr stage = _Vec3fChainStage();
    const UsdAttribute offset = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Vec.rigExec:offset"));
    size_t clamped = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        GfVec3f base;
        CHECK(offset.Get(&base, UsdTimeCode(frames[f])));
        RigExecPropertyMathParams<GfVec3f> lift, bound, spread;
        lift.op = RigExecPropertyOp::Add;
        lift.value = GfVec3f(0.0f, 2.0f, 0.0f);
        bound.op = RigExecPropertyOp::Clamp;
        bound.min = GfVec3f(-1.0f);
        bound.max = GfVec3f(1.0f, 4.5f, 1.0f);
        spread.op = RigExecPropertyOp::Remap;
        spread.min = GfVec3f(-2.0f);
        spread.max = GfVec3f(2.0f, 3.0f, 2.0f);
        spread.weight = 0.5f;
        const GfVec3f lifted = RigExecApplyVec3fMath(base, lift);
        const GfVec3f held = RigExecApplyVec3fMath(lifted, bound);
        const GfVec3f expected = RigExecApplyVec3fMath(held, spread);
        const auto it =
            rows[f].values.find("/Asset/Rig/Channels/Vec.rigExec:offset");
        CHECK(it != rows[f].values.end() &&
              _SamePropertyValue(VtValue(expected), it->second));
        clamped += held != lifted;
    }
    CHECK(clamped > 0);
}

static void
TestDisabledRevision()
{
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("disabled revisions", _DisabledStage, frames,
                       frames.size(), &rows)) {
        return;
    }
    // Doubled where the toggle is on, passed through where it is off; the
    // add of 100 never applies.
    const UsdStageRefPtr stage = _DisabledStage();
    const UsdAttribute amount = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
    const std::string off =
        "diag /Asset/Rig/Movers/Off: disabled; revision passed through";
    const std::string toggle =
        "diag /Asset/Rig/Movers/Off/Toggle: disabled; revision passed "
        "through";
    size_t passed = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        float base = 0.0f;
        CHECK(amount.Get(&base, UsdTimeCode(frames[f])));
        const bool on = frames[f] < 4.0 || frames[f] >= 8.0;
        bool found = false;
        const float got =
            _Float(rows[f], "/Asset/Rig/Channels/Dial.rigExec:amount", &found);
        CHECK(found && got == (on ? base * 2.0f : base));
        CHECK(_HasLine(rows[f], off));
        CHECK(_HasLine(rows[f], toggle) == !on);
        passed += !on;
    }
    CHECK(passed == 4);
}

static void
TestEnvelopeRevisions()
{
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase("envelope-weighted revisions", _EnvelopeChainStage,
                       frames, frames.size(), &rows)) {
        return;
    }
    // The strict weight leaves [0, 1] from frame 8 (ty 7/3 times 0.45);
    // the out-of-range defaultWeights refuse at every frame.
    const std::string strict =
        "diag /Asset/Rig/Movers/Over/Soft/Kick: strict range violation on "
        "/Asset/Rig/Weights/Steep; revision passed through";
    const auto outOfRange = [](const std::string &mover) {
        return "diag /Asset/Rig/Movers/" + mover +
               ": inputs:defaultWeight must be finite and in [0, 1]; "
               "revision passed through";
    };
    const std::string over = outOfRange("Over");
    const std::string negative =
        outOfRange("Over/Soft/Kick/Nudge/Fade/Negative");
    const std::string notANumber =
        outOfRange("Over/Soft/Kick/Nudge/Fade/Negative/NotANumber");
    size_t refused = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        CHECK(_HasLine(rows[f], over));
        CHECK(_HasLine(rows[f], negative));
        CHECK(_HasLine(rows[f], notANumber));
        CHECK(_HasLine(rows[f], strict) == (frames[f] >= 8.0));
        refused += _HasLine(rows[f], strict);
    }
    CHECK(refused == 3);
}

static void
TestPhasedConsumers()
{
    const std::vector<double> frames = {1, 4, 7, 10};
    std::vector<_ChainFrame> rows;
    // Per frame: the two chains, three readout chains, three consumers.
    if (!_RunChainCase("phased consumers", _PhasedStage, frames,
                       8 * frames.size(), &rows)) {
        return;
    }
    const UsdStageRefPtr stage = _PhasedStage();
    const UsdAttribute amount = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
    const UsdAttribute level = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Wide.rigExec:level"));
    for (size_t f = 0; f < frames.size(); ++f) {
        float base = 0.0f;
        double wide = 0.0;
        CHECK(amount.Get(&base, UsdTimeCode(frames[f])));
        CHECK(level.Get(&wide, UsdTimeCode(frames[f])));
        bool found = false;
        CHECK(_Float(rows[f], "/Asset/Rig/Movers/Readouts/base.inputs:value",
                     &found) == base &&
              found);
        CHECK(_Float(rows[f],
                     "/Asset/Rig/Movers/Readouts/early.inputs:value",
                     &found) == base * 2.0f &&
              found);
        // The double chain's base, narrowed into the float input.
        CHECK(_Float(rows[f],
                     "/Asset/Rig/Movers/Readouts/narrow.inputs:value",
                     &found) == float(wide) &&
              found);
        CHECK(_Float(rows[f], "/Asset/Rig/Channels/Dial.rigExec:amount",
                     &found) == base * 2.0f + 0.25f &&
              found);
    }
}

// The default read phase through the .rigexec: the runtime replays each
// reader at its own phase, bit for bit with the baked program and the
// dynamic evaluator, and the computed section carries a record per reader
// the overlay walk alone would answer differently -- applied 0 for the
// undeclared ones, every revision for the `final` behind a recorded hop.
static void
TestDefaultReadPhaseRoundTrip()
{
    const std::vector<double> frames = {1, 4, 7, 10};
    std::vector<_ChainFrame> rows;
    // Per frame: the dial's chain, six readout chains, four records.
    if (!_RunChainCase("default read phase", _DefaultPhaseStage, frames,
                       11 * frames.size(), &rows)) {
        return;
    }
    const UsdStageRefPtr stage = _DefaultPhaseStage();
    const UsdAttribute amount = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Dial.rigExec:amount"));
    const auto readout = [&](size_t f, const char *name) {
        bool found = false;
        const float value =
            _Float(rows[f],
                   std::string("/Asset/Rig/Channels/Readouts.rigExec:") + name,
                   &found);
        CHECK(found);
        return value;
    };
    const auto consumer = [&](size_t f, const char *name, bool *found) {
        return _Float(rows[f],
                      std::string("/Asset/Rig/Movers/Readouts/") + name +
                          ".inputs:value",
                      found);
    };
    size_t clamped = 0;
    for (size_t f = 0; f < frames.size(); ++f) {
        float base = 0.0f;
        CHECK(amount.Get(&base, UsdTimeCode(frames[f])));
        const float final = std::min(base * 2.0f, 0.6f);
        clamped += final != base * 2.0f;
        CHECK(readout(f, "undeclared") == base);
        CHECK(readout(f, "final") == final);
        CHECK(readout(f, "hop") == base);
        CHECK(readout(f, "finalViaHop") == final);
        CHECK(readout(f, "finalHop") == final);
        CHECK(readout(f, "baseViaFinalHop") == base);
        // What the records publish on the readers themselves; none on the
        // `final` readers the dial's value answers.
        bool found = false;
        CHECK(consumer(f, "undeclared", &found) == base && found);
        CHECK(consumer(f, "finalViaHop", &found) == final && found);
        CHECK(consumer(f, "baseViaFinalHop", &found) == base && found);
        consumer(f, "final", &found);
        CHECK(!found);
        consumer(f, "finalHop", &found);
        CHECK(!found);
    }
    CHECK(clamped > 0 && clamped < frames.size());

    // The round trip: the records the computed section carries.
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    const std::unique_ptr<RigExecBinaryReader> binary =
        RigExecBinaryReader::Open(result.bytes.data(), result.bytes.size(),
                                  &error);
    CHECK(binary);
    const uint8_t *data = nullptr;
    size_t size = 0;
    if (!binary ||
        !binary->FindSection(RigExecBinarySection::Computed, &data, &size)) {
        CHECK(false);
        return;
    }
    RigExecWireReader cursor(data, size);
    RigExecWireComputed computed;
    CHECK(RigExecWireDecodeComputed(&cursor, &computed, &error));
    std::map<std::string, uint32_t> applied;
    std::string text;
    for (const v4::PhasedConsumer &record : computed.phasedConsumers) {
        CHECK(record.consumer < computed.inputs.size());
        if (record.consumer < computed.inputs.size() &&
            binary->GetString(computed.inputs[record.consumer].name, &text)) {
            applied[text] = record.applied;
        }
    }
    const std::string readers = "/Asset/Rig/Movers/Readouts/";
    CHECK(applied.size() == 4);
    CHECK(applied.count(readers + "undeclared.inputs:value") &&
          applied[readers + "undeclared.inputs:value"] == 0);
    CHECK(applied.count(readers + "hop.inputs:value") &&
          applied[readers + "hop.inputs:value"] == 0);
    CHECK(applied.count(readers + "finalViaHop.inputs:value") &&
          applied[readers + "finalViaHop.inputs:value"] == 2);
    CHECK(applied.count(readers + "baseViaFinalHop.inputs:value") &&
          applied[readers + "baseViaFinalHop.inputs:value"] == 0);
    std::printf("default read phase: %zu record(s) round-tripped\n",
                applied.size());
}

static void
TestNonFiniteBase()
{
    const std::vector<double> frames = {1, 3, 5, 7};
    std::vector<_ChainFrame> rows;
    // Bad publishes at frames 1 and 7, Huge never, Blow at every frame.
    if (!_RunChainCase("non-finite bases and results", _NonFiniteStage,
                       frames, 6, &rows)) {
        return;
    }
    const std::string badPath = "/Asset/Rig/Channels/Bad.rigExec:amount";
    const std::string hugePath = "/Asset/Rig/Channels/Huge.rigExec:level";
    const std::string blowPath = "/Asset/Rig/Channels/Blow.rigExec:amount";
    const std::string hugeLine =
        "property chain " + hugePath +
        ": authored base is not finite; chain skipped";
    const std::string infLine =
        "diag /Asset/Rig/Movers/Inf: inputs unusable; revision passed through";
    bool found = false;
    CHECK(_Float(rows[0], badPath, &found) == 1.5f && found);
    CHECK(!rows[1].values.count(badPath) && !rows[2].values.count(badPath));
    CHECK(_Float(rows[3], badPath, &found) == 1.25f && found);
    for (const _ChainFrame &row : rows) {
        CHECK(!row.values.count(hugePath));
        CHECK(_HasLine(row, hugeLine));
        CHECK(_HasLine(row, infLine));
    }
    const std::string notFinite =
        "property chain " + badPath +
        ": authored base is not finite; chain skipped";
    CHECK(!_HasLine(rows[0], notFinite) && _HasLine(rows[1], notFinite) &&
          _HasLine(rows[2], notFinite) && !_HasLine(rows[3], notFinite));
    // 0.5 and 1.0 times 3e38 stay finite; 1.5 and 2.0 overflow, and the
    // revision passes the base through.
    const std::string overflow =
        "diag /Asset/Rig/Movers/Inf/Overflow: produced a non-finite value; "
        "revision passed through";
    CHECK(_Float(rows[0], blowPath, &found) == 0.5f * 3e38f && found);
    CHECK(_Float(rows[1], blowPath, &found) == 1.0f * 3e38f && found);
    CHECK(_Float(rows[2], blowPath, &found) == 1.5f && found);
    CHECK(_Float(rows[3], blowPath, &found) == 2.0f && found);
    CHECK(!_HasLine(rows[0], overflow) && !_HasLine(rows[1], overflow) &&
          _HasLine(rows[2], overflow) && _HasLine(rows[3], overflow));
}

// One played frame of a file: what a consumer reads, plus the property
// results.
struct _PlayedFrame {
    std::vector<RigExecRuntimePropertyValue> properties;
    std::vector<RrPointFrame> fin;
    std::vector<RigExecRuntimeProviderXform> xforms;
    std::vector<std::string> diagnostics;
};

static bool
_SamePlayed(const _PlayedFrame &a, const _PlayedFrame &b)
{
    if (a.properties.size() != b.properties.size() || a.fin != b.fin ||
        a.diagnostics != b.diagnostics || a.xforms.size() != b.xforms.size()) {
        return false;
    }
    for (size_t i = 0; i < a.properties.size(); ++i) {
        if (a.properties[i].path != b.properties[i].path ||
            !_SameBits(a.properties[i].value, b.properties[i].value)) {
            return false;
        }
    }
    for (size_t i = 0; i < a.xforms.size(); ++i) {
        if (a.xforms[i].path != b.xforms[i].path ||
            a.xforms[i].matrix != b.xforms[i].matrix ||
            a.xforms[i].base != b.xforms[i].base) {
            return false;
        }
    }
    return true;
}

// Plays \p frames of \p bytes through a fresh reader; \p chainReads, when
// given, receives the chain-crossing reads the cross-check compared.
static bool
_PlayFrames(const std::vector<uint8_t> &bytes,
            const std::vector<double> &frames, bool crossCheck,
            std::vector<_PlayedFrame> *out, std::string *error,
            uint64_t *chainReads = nullptr)
{
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), error);
    if (!reader) {
        return false;
    }
    reader->SetCrossCheckForTesting(crossCheck);
    for (double frame : frames) {
        if (!reader->SetFrame(frame, error) || !reader->Execute(error)) {
            return false;
        }
        out->push_back({reader->GetPropertyResultsForTesting(),
                        reader->GetFinFrames(), reader->GetProviderXforms(),
                        reader->GetDiagnostics()});
    }
    if (chainReads) {
        *chainReads = reader->GetCrossCheckChainReadCountForTesting();
    }
    return true;
}

// tests/fixtures/computed_chains.usda: the chain cases together (a curve
// with tangents, envelopes on property revisions, double, vec3f and matrix
// chains, the chains' diagnostics, phased consumers) under the baked parity
// path and against the dynamic evaluator, 10 values per frame, plus the
// constraint weight Follow reads through the dial's chain at its declared
// `final`. Then the record
// is shown unread: a recorded property value, or Follow's recorded weight,
// moved off what the runtime computes fails the cross-check naming it and,
// with the cross-check off, plays exactly as the untouched file does; and a
// slot value a chain reads, changed in the computed section alone, changes
// the chain's result and the transform of the constraint reading it.
static void
TestComputedChainsFixture()
{
    const char *const name = "computed chains fixture";
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "computed_chains.usda")
            .string();
    const std::vector<double> frames = {1.0, 3.0, 5.0, 7.0, 10.0};
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    uint64_t parity = 0;
    _TestStage(name, stage, frames, &parity);
    uint64_t dynamicChecked = 0;
    std::vector<_ChainFrame> rows;
    CHECK(_ChainsMatchDynamic(name, stage, frames, &dynamicChecked, &rows));
    // Per frame: 10 property values and Follow's inputs:defaultWeight.
    CHECK(parity == 10 * frames.size() + frames.size());
    CHECK(dynamicChecked == 10 * frames.size());

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    const std::unique_ptr<RigExecBinaryReader> binary =
        RigExecBinaryReader::Open(result.bytes.data(), result.bytes.size(),
                                  &error);
    CHECK(binary);
    if (!binary) {
        return;
    }
    // Without the computed section nothing can run the chains.
    CHECK(_OpenError(_WithoutSection(result.bytes,
                                     RigExecBinarySection::Computed)) ==
          "the rig runs property chains, which need the computed section "
          "this file does not carry; rebake it");

    const std::string dial = "/Asset/Rig/Channels/Dial.rigExec:amount";
    const std::string follow = "/Asset/Rig/Movers/Follow.inputs:defaultWeight";
    std::vector<_PlayedFrame> untouched;
    uint64_t chainReads = 0;
    CHECK(_PlayFrames(result.bytes, frames, true, &untouched, &error,
                      &chainReads));
    CHECK(chainReads == frames.size());

    // The dial's recorded value at the first frame, moved by 1/8.
    RigExecWireInputTable table;
    CHECK(_DecodeInputTable(result.bytes, &table));
    bool moved = false;
    std::string text;
    if (!table.frames.empty()) {
        RigExecWireFrameInputs &record = table.frames[0];
        for (size_t i = 0; i < record.propertyPaths.size() &&
                           i < record.propertyValues.size();
             ++i) {
            if (binary->GetString(record.propertyPaths[i], &text) &&
                text == dial) {
                record.propertyValues[i].f32 += 0.125f;
                moved = true;
            }
        }
    }
    CHECK(moved);
    std::vector<uint8_t> payload;
    CHECK(RigExecWireEncodeInputTable(table, &payload));
    const std::vector<uint8_t> tampered = _ReplaceSection(
        result.bytes, RigExecBinarySection::InputTable, payload);
    std::vector<_PlayedFrame> ignored, replayed;
    CHECK(!_PlayFrames(tampered, frames, true, &ignored, &error));
    CHECK(error.find("cross-check mismatch at propertyValues[" + dial + "]") !=
          std::string::npos);
    std::printf("%s, tampered record: %s\n", name, error.c_str());
    CHECK(_PlayFrames(tampered, frames, false, &replayed, &error));
    CHECK(replayed.size() == untouched.size());
    for (size_t f = 0; f < replayed.size() && f < untouched.size(); ++f) {
        CHECK(_SamePlayed(replayed[f], untouched[f]));
    }

    // Follow's recorded weight at the first frame, moved by 1/8: the
    // constraint takes what the chain computes, so only the cross-check
    // reads the record.
    {
        RigExecWireInputTable weights;
        CHECK(_DecodeInputTable(result.bytes, &weights));
        bool movedWeight = false;
        for (size_t uid = 0; uid < weights.directory.size(); ++uid) {
            if (weights.frames.empty() ||
                !binary->GetString(weights.directory[uid].head, &text) ||
                text != follow) {
                continue;
            }
            RigExecWireFrameInputs &record = weights.frames[0];
            for (size_t k = 0;
                 k < record.uids.size() && k < record.values.size(); ++k) {
                if (record.uids[k] == uid) {
                    CHECK(record.values[k].tag ==
                          RigExecWireInput::Tag::Float);
                    record.values[k].f32 += 0.125f;
                    movedWeight = true;
                }
            }
        }
        CHECK(movedWeight);
        payload.clear();
        CHECK(RigExecWireEncodeInputTable(weights, &payload));
        const std::vector<uint8_t> weightTampered = _ReplaceSection(
            result.bytes, RigExecBinarySection::InputTable, payload);
        ignored.clear();
        CHECK(!_PlayFrames(weightTampered, frames, true, &ignored, &error));
        CHECK(error.find("chain reads: cross-check mismatch at values[uid ") !=
                  std::string::npos &&
              error.find(", " + follow + "]") != std::string::npos);
        std::printf("%s, tampered chain read: %s\n", name, error.c_str());
        std::vector<_PlayedFrame> weightReplayed;
        CHECK(_PlayFrames(weightTampered, frames, false, &weightReplayed,
                          &error));
        CHECK(weightReplayed.size() == untouched.size());
        for (size_t f = 0;
             f < weightReplayed.size() && f < untouched.size(); ++f) {
            CHECK(_SamePlayed(weightReplayed[f], untouched[f]));
        }
    }

    // Controls/Dial.avars:tx (Gain's multiplier, read through a connection)
    // set to +0 at frame 5 in the computed section alone.
    RigExecWireComputed computed;
    {
        const uint8_t *data = nullptr;
        size_t size = 0;
        CHECK(binary->FindSection(RigExecBinarySection::Computed, &data,
                                  &size));
        RigExecWireReader cursor(data, size);
        CHECK(RigExecWireDecodeComputed(&cursor, &computed, &error));
    }
    size_t slot = computed.inputs.size();
    for (size_t s = 0; s < computed.inputs.size(); ++s) {
        if (binary->GetString(computed.inputs[s].name, &text) &&
            text == "/Asset/Rig/Controls/Dial.avars:tx") {
            slot = s;
        }
    }
    CHECK(slot < computed.inputs.size() &&
          computed.frames.size() == frames.size());
    if (slot >= computed.inputs.size() ||
        computed.frames.size() != frames.size()) {
        return;
    }
    computed.frames[2].values[slot] = 0;  // values[0] is Double +0.0
    computed.frames[2].hasValue[slot] = 1;
    payload.clear();
    CHECK(RigExecWireEncodeComputed(computed, &payload, &error));
    const std::vector<uint8_t> altered = _ReplaceSection(
        result.bytes, RigExecBinarySection::Computed, payload);
    std::vector<_PlayedFrame> recomputed;
    ignored.clear();
    CHECK(!_PlayFrames(altered, frames, true, &ignored, &error));
    CHECK(error.find("cross-check mismatch at propertyValues[" + dial + "]") !=
          std::string::npos);
    std::printf("%s, altered slot value: %s\n", name, error.c_str());
    CHECK(_PlayFrames(altered, frames, false, &recomputed, &error));
    CHECK(recomputed.size() == untouched.size());
    for (size_t f = 0; f < recomputed.size() && f < untouched.size(); ++f) {
        const auto dialOf = [&](const _PlayedFrame &played) {
            for (const RigExecRuntimePropertyValue &v : played.properties) {
                if (v.path == dial) {
                    return v.value.f32;
                }
            }
            return -1.0f;
        };
        CHECK((dialOf(recomputed[f]) != dialOf(untouched[f])) == (f == 2));
        // Follow reads the dial's result (it declares `final`) as its
        // weight, so the transform it revises moves with it.
        const auto targetOf = [&](const _PlayedFrame &played,
                                  bool *found) {
            for (const RigExecRuntimeProviderXform &x : played.xforms) {
                if (x.path == "/Asset/Target") {
                    *found = true;
                    return x.matrix;
                }
            }
            *found = false;
            return RrMat4d(1.0);
        };
        bool inRecomputed = false, inUntouched = false;
        const RrMat4d moved = targetOf(recomputed[f], &inRecomputed);
        const RrMat4d kept = targetOf(untouched[f], &inUntouched);
        CHECK(inRecomputed && inUntouched);
        CHECK((moved != kept) == (f == 2));
    }
}

// tests/fixtures/computed_ik_space.usda: a TwoBoneIk measured in an
// animated rigExec:space composed with a keyed, non-uniform
// rigExec:spaceMatrix, beside one reading a constant spaceMatrix. The arm's
// spaceMatrix is a registered read the frame records carry at every frame
// (a bake that left it out of the directory refused the rig), the runtime
// plays the fixture bit for bit with the baked program, and the solver
// consumes the value: the recorded matrix moved at one frame fails the
// registered-read cross-check naming solvers[k].ikSpace and, with the
// cross-check off, moves the arm at that frame only.
static void
TestIkSpaceFixture()
{
    const char *const name = "ik space fixture";
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "computed_ik_space.usda")
            .string();
    const std::vector<double> frames = {1.0, 3.0, 5.0, 7.0, 10.0};
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    _TestStage(name, stage, frames);

    RigExecRigEvaluator evaluator(stage, SdfPath("/IkSpaceAsset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    const std::unique_ptr<RigExecBinaryReader> binary =
        RigExecBinaryReader::Open(result.bytes.data(), result.bytes.size(),
                                  &error);
    CHECK(binary);
    if (!binary) {
        return;
    }
    const std::string spaceMatrix =
        "/IkSpaceAsset/Rig/Solvers/ArmIK.rigExec:spaceMatrix";
    RigExecWireInputTable table;
    CHECK(_DecodeInputTable(result.bytes, &table));
    int64_t uid = -1;
    std::string text;
    for (size_t k = 0; k < table.directory.size(); ++k) {
        if (binary->GetString(table.directory[k].head, &text) &&
            text == spaceMatrix) {
            CHECK(uid < 0);
            CHECK(table.directory[k].tag == RigExecWireInput::Tag::Matrix4d);
            uid = int64_t(k);
        }
    }
    CHECK(uid >= 0);
    CHECK(table.frames.size() == frames.size());
    if (uid < 0 || table.frames.size() != frames.size()) {
        return;
    }
    // Recorded at every frame, and not the identity at any.
    for (const RigExecWireFrameInputs &record : table.frames) {
        bool held = false;
        for (size_t k = 0; k < record.uids.size(); ++k) {
            if (record.uids[k] == uint32_t(uid)) {
                held = true;
                CHECK(record.values[k].matrix[0] != 1.0);
            }
        }
        CHECK(held);
    }

    std::vector<_PlayedFrame> untouched;
    CHECK(_PlayFrames(result.bytes, frames, true, &untouched, &error));
    // The arm's recorded spaceMatrix at frame 3, its x axis doubled.
    for (size_t k = 0; k < table.frames[1].uids.size(); ++k) {
        if (table.frames[1].uids[k] == uint32_t(uid)) {
            table.frames[1].values[k].matrix[0] *= 2.0;
        }
    }
    std::vector<uint8_t> payload;
    CHECK(RigExecWireEncodeInputTable(table, &payload));
    const std::vector<uint8_t> tampered = _ReplaceSection(
        result.bytes, RigExecBinarySection::InputTable, payload);
    std::vector<_PlayedFrame> ignored, replayed;
    CHECK(!_PlayFrames(tampered, frames, true, &ignored, &error));
    CHECK(error.find("registered reads: cross-check mismatch at solvers[") !=
              std::string::npos &&
          error.find("].ikSpace (uid " + std::to_string(uid) + ", " +
                     spaceMatrix + ").matrix[0][0]") != std::string::npos);
    std::printf("%s, tampered record: %s\n", name, error.c_str());
    CHECK(_PlayFrames(tampered, frames, false, &replayed, &error));
    CHECK(replayed.size() == untouched.size());
    for (size_t f = 0; f < replayed.size() && f < untouched.size(); ++f) {
        CHECK((replayed[f].fin != untouched[f].fin) == (f == 1));
    }

    // A constant spaceMatrix and no space keep the leg on the solver's
    // constant arm, whose bone lengths are measured in that matrix when the
    // rests are refreshed: the knee's rest is keyed, so the ladder varies
    // and they are refreshed every frame. The baked program publishes the
    // rest-length handles there, which the dynamic computation does not, so
    // this case is compared with the baked program only.
    const UsdStageRefPtr constant = UsdStage::Open(fixture);
    CHECK(constant);
    if (!constant) {
        return;
    }
    constant->SetEditTarget(constant->GetSessionLayer());
    const UsdPrim leg =
        constant->GetPrimAtPath(SdfPath("/IkSpaceAsset/Rig/Solvers/LegIK"));
    CHECK(leg);
    GfMatrix4d scale(1.0);
    scale.SetScale(GfVec3d(0.75, 0.75, 0.75));
    CHECK(leg.CreateAttribute(TfToken("rigExec:spaceMatrix"),
                              SdfValueTypeNames->Matrix4d)
              .Set(scale));
    const UsdAttribute kneeRest =
        constant->GetPrimAtPath(SdfPath("/IkSpaceAsset/Rig/Joints/Hip/Knee"))
            .GetAttribute(TfToken("rest:space"));
    CHECK(kneeRest);
    GfMatrix4d knee(1.0);
    knee.SetTranslate(GfVec3d(0.0, -2.5, 0.2));
    CHECK(kneeRest.Set(knee, UsdTimeCode(1.0)));
    knee.SetTranslate(GfVec3d(0.0, -2.0, 0.6));
    CHECK(kneeRest.Set(knee, UsdTimeCode(10.0)));
    _TestStage("ik space fixture, constant leg space", constant, frames);
}

// The read phase on inputs outside the movers, replayed: the docs
// single-chain IK with its twist connected to a channel a math mover
// doubles (keyed 10 to 30 degrees), a control whose avars:ty reads a
// height channel a mover scales by 1.5 (keyed 2 to 4) -- a double channel,
// or a float one that only a record widens -- and the docs space switch
// reading its index from a channel outside the rig that reads a pick a
// mover adds 1 to. Undeclared each reads the channel's base, `final` the
// revised value (testRigExecArm checks which); here the runtime must match
// the baked program bit for bit at every frame either way.
static UsdStageRefPtr
_TwistReadStage(const std::string &examplesDir, bool final)
{
    const UsdStageRefPtr stage = UsdStage::Open(
        examplesDir + "/../docs/examples/single_chain_ik_constraint.usda");
    CHECK(stage);
    if (!stage) {
        return stage;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdAttribute degrees =
        stage->DefinePrim(SdfPath("/ScIkAsset/Rig/Channels/Twist"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:degrees"),
                             SdfValueTypeNames->Double);
    degrees.Set(10.0, UsdTimeCode(1001.0));
    degrees.Set(30.0, UsdTimeCode(1024.0));
    const UsdPrim twice = stage->DefinePrim(
        SdfPath("/ScIkAsset/Rig/Movers/TwistTwice"),
        TfToken("RigExecFloatMathMover"));
    CHECK(twice.ApplyAPI(TfToken("RigExecMoverAPI")));
    twice.CreateAttribute(TfToken("rigExec:operation"),
                          SdfValueTypeNames->Token)
        .Set(TfToken("multiply"));
    twice.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .Set(2.0f);
    twice.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({degrees.GetPath()});
    const UsdAttribute twist = stage->GetAttributeAtPath(
        SdfPath("/ScIkAsset/Rig/Movers/Pose/ArmIK.inputs:twistDegrees"));
    twist.SetConnections({degrees.GetPath()});
    if (final) {
        twist.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    }
    return stage;
}

static UsdStageRefPtr
_LiftReadStage(bool final, bool floatChannel = false)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(10.0);
    UsdGeomXform::Define(stage, SdfPath("/Asset"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim lift = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Lift"), TfToken("RigExecControl"));
    lift.CreateAttribute(TfToken("rest:space"), SdfValueTypeNames->Matrix4d)
        .Set(GfMatrix4d(1.0));
    const UsdAttribute height =
        stage->DefinePrim(SdfPath("/Asset/Rig/Channels/Lift"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:height"),
                             floatChannel ? SdfValueTypeNames->Float
                                          : SdfValueTypeNames->Double);
    if (floatChannel) {
        height.Set(2.0f, UsdTimeCode(1.0));
        height.Set(4.0f, UsdTimeCode(10.0));
    } else {
        height.Set(2.0, UsdTimeCode(1.0));
        height.Set(4.0, UsdTimeCode(10.0));
    }
    const UsdPrim raise = _MathMover(stage, "Raise", "RigExecFloatMathMover",
                                     "multiply", height);
    _SetInput(raise, "inputs:value", SdfValueTypeNames->Float, 1.5f);
    const UsdAttribute ty = lift.CreateAttribute(TfToken("avars:ty"),
                                                 SdfValueTypeNames->Double);
    ty.SetConnections({height.GetPath()});
    if (final) {
        ty.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    }
    return stage;
}

static UsdStageRefPtr
_OutsideSpaceStage(const std::string &examplesDir, bool final)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/../docs/examples/space_switch.usda");
    CHECK(stage);
    if (!stage) {
        return stage;
    }
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdAttribute pick =
        stage->DefinePrim(SdfPath("/SpaceSwitchAsset/Rig/Channels"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:pick"),
                             SdfValueTypeNames->Double);
    pick.Set(0.0);
    const UsdPrim next = stage->DefinePrim(
        SdfPath("/SpaceSwitchAsset/Rig/Movers/NextSpace"),
        TfToken("RigExecFloatMathMover"));
    CHECK(next.ApplyAPI(TfToken("RigExecMoverAPI")));
    next.CreateAttribute(TfToken("rigExec:operation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("add"));
    next.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .Set(1.0f);
    next.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({pick.GetPath()});
    const UsdAttribute space =
        stage->DefinePrim(SdfPath("/SpaceSwitchAsset/Dials"),
                          TfToken("Scope"))
            .CreateAttribute(TfToken("rigExec:space"),
                             SdfValueTypeNames->Double);
    space.SetConnections({pick.GetPath()});
    if (final) {
        space.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));
    }
    stage->GetPrimAtPath(SdfPath("/SpaceSwitchAsset/Rig/Spaces/HandSpaces"))
        .GetRelationship(TfToken("rigExec:activeSpaceAttribute"))
        .SetTargets({space.GetPath()});
    return stage;
}

static void
TestConstraintAndAvarReadersReplay(const std::string &examplesDir)
{
    for (const bool final : {false, true}) {
        const UsdStageRefPtr twist = _TwistReadStage(examplesDir, final);
        if (twist) {
            _TestStage(final ? "twist read final" : "twist read at its base",
                       twist, {1001.0, 1006.0, 1012.0, 1024.0});
        }
        _TestStage(final ? "avar read final" : "avar read at its base",
                   _LiftReadStage(final), {1.0, 4.0, 10.0});
        _TestStage(final ? "avar read final, float channel"
                         : "avar read at its base, float channel",
                   _LiftReadStage(final, true), {1.0, 4.0, 10.0});
        const UsdStageRefPtr space = _OutsideSpaceStage(examplesDir, final);
        if (space) {
            _TestStage(final ? "outside space index read final"
                             : "outside space index read at its base",
                       space, {1001.0, 1012.0, 1024.0, 1036.0});
        }
    }
}

// A drag on an avar that reads a chain at its base. The USD evaluators
// stand the reader aside and pose with the drag; the runtime replaces the
// avar's read with the override after the prologue, so SetAvar accepts it
// and every frame the step graph pools hold matches the baked program
// under the same drag bit for bit. An avar a math mover revises stays
// refused: its value is the chain's to compute.
static void
TestSetAvarOnPhasedReader()
{
    const UsdStageRefPtr stage = _LiftReadStage(false);
    const UsdPrim lift =
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Controls/Lift"));
    const UsdAttribute tx =
        lift.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    tx.Set(1.0);
    const UsdPrim shift =
        _MathMover(stage, "Shift", "RigExecFloatMathMover", "add", tx);
    _SetInput(shift, "inputs:value", SdfValueTypeNames->Float, 0.5f);

    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 4.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(),
                                   &error);
    CHECK(reader);
    if (!reader) {
        std::printf("set avar on a phased reader: open: %s\n",
                    error.c_str());
        return;
    }
    reader->SetRunMaskForTesting(0x7u);
    CHECK(!reader->SetAvar("/Asset/Rig/Controls/Lift.avars:tx", 2.0,
                           &error));
    CHECK(error.find("property-mover output") != std::string::npos);
    const double drag = 5.5;
    CHECK(reader->SetAvar("/Asset/Rig/Controls/Lift.avars:ty", drag,
                          &error));

    RigExecRigEvaluator dragged(stage, rigPath);
    dragged.SetEvaluationMode(RigExecEvaluationMode::Baked);
    dragged.SetInteractiveOverrides({RigExecValueOverride{
        lift.GetPath(), TfToken(), TfToken("avars:ty"), VtValue(drag)}});
    for (const double frame : frames) {
        const RigExecRigPose pose = dragged.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid);
        const auto control = pose.controlFrames.find(lift.GetPath());
        CHECK(control != pose.controlFrames.end() &&
              control->second.points[0][1] == drag);
        const RigExecBakedProgram *program = dragged.GetBakedProgram();
        CHECK(program);
        if (!pose.valid || !program) {
            return;
        }
        const RigExecBakedProgramImpl &graph = program->GetStepGraph();
        CHECK(reader->SetFrame(frame, &error));
        CHECK(reader->Execute(&error));
        const std::vector<RrPointFrame> &fin = reader->GetFinFrames();
        CHECK(fin.size() == graph.fin.size());
        bool same = fin.size() == graph.fin.size();
        bool sawDrag = false;
        for (size_t i = 0; same && i < fin.size(); ++i) {
            same = _FramesEqual(graph.fin[i], fin[i]);
            sawDrag = sawDrag || fin[i].points[0][1] == drag;
        }
        if (!same) {
            std::printf("set avar on a phased reader frame %.17g: the "
                        "runtime's frames differ from the baked drag\n",
                        frame);
        }
        CHECK(same);
        CHECK(sawDrag);
    }
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestComputedEnvelopes();
    TestStaticWeightEnvelope();
    TestAvarDrivenDynamicEnvelope();
    TestChainDrivenEnvelope();
    TestComputedOpenRefusals();
    TestFloatChainCurveClamp();
    TestDoubleChainInFloat();
    TestMatrixChain();
    TestVec3fChain();
    TestDisabledRevision();
    TestEnvelopeRevisions();
    TestPhasedConsumers();
    TestDefaultReadPhaseRoundTrip();
    TestNonFiniteBase();
    TestComputedChainsFixture();
    TestIkSpaceFixture();

    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }
    TestConstraintAndAvarReadersReplay(examplesDir);
    TestSetAvarOnPhasedReader();
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
