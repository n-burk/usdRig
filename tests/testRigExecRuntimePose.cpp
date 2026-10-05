// rigExecRuntime pose-family parity (M2): baked program vs runtime over
// every baking fixture, comparing fin/base versions, rest -> pose
// matrices and joint matrices bit for bit. Each stage is baked at one time
// and the binary is played through its inputs: the input sampler hands it
// the stage's animated inputs at each frame, and a drag is an input set to
// an authored value, held to the evaluator with the same value authored in
// the session layer.
#include "rigExecBake/bake.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/container.h"
#include "rigExecBinary/inputTable.h"
#include "rigExecBinary/pose.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
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
static int comparedFrames = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++failures;                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
        }                                                               \
    } while (0)

#include "rigExecSectionEdit.h"
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

// One played frame of a file: what a consumer reads, plus the property
// results.
struct _PlayedFrame {
    std::vector<RigExecRuntimePropertyValue> properties;
    std::vector<RrPointFrame> fin;
    std::vector<RigExecRuntimeProviderXform> xforms;
    std::vector<std::string> diagnostics;
};

static _PlayedFrame
_Snapshot(const RigExecRuntimeReader &reader)
{
    return {reader.GetPropertyValues(), reader.GetFinFrames(),
            reader.GetProviderXforms(), reader.GetDiagnostics()};
}

// Whether two played frames publish the same values bit for bit: property
// results, version pool and provider transforms, and with \p diagnostics
// the generation's lines too.
static bool
_SamePlayed(const _PlayedFrame &a, const _PlayedFrame &b,
            bool diagnostics = true)
{
    if (a.properties.size() != b.properties.size() || a.fin != b.fin ||
        (diagnostics && a.diagnostics != b.diagnostics) ||
        a.xforms.size() != b.xforms.size()) {
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

// The file plays a value computed from its inputs, not one it holds: its
// outputs at the first and the last frame differ, while it holds the bake
// time's static data alone.
static void
_CheckFramesDiffer(const char *name, const std::vector<_PlayedFrame> &rows)
{
    const bool differ =
        rows.size() > 1 && !_SamePlayed(rows.front(), rows.back(), false);
    CHECK(differ);
    if (!differ) {
        std::printf("%s: the first and last frames play the same outputs\n",
                    name);
    }
}

// Bakes \p stage at the first of \p frames and plays the file through the
// input sampler beside a fresh baked evaluator, frame by frame: the step
// graph's version pools and rest -> pose matrices, the joint matrices and
// the diagnostics, bit for bit. A `static` stage holds an animated source
// in static data at the bake time, so it plays that time alone. \p played,
// when given, receives each frame's outputs.
static void
_TestStage(const std::string &name, const UsdStageRefPtr &stage,
           const std::vector<double> &frames,
           std::vector<_PlayedFrame> *played = nullptr,
           bool staticClass = false)
{
    const SdfPath rigPath = _FindRig(stage);
    CHECK(!rigPath.IsEmpty() && !frames.empty());
    if (rigPath.IsEmpty() || frames.empty()) {
        std::printf("%s: FAILED (no rig or no frames)\n", name.c_str());
        return;
    }
    const std::vector<double> times =
        staticClass ? std::vector<double>{frames.front()} : frames;
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        if (!RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error)) {
            std::printf("%s: FAILED (bake: %s)\n", name.c_str(),
                        error.c_str());
            CHECK(false);
            return;
        }
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: FAILED (open: %s)\n", name.c_str(), error.c_str());
        CHECK(false);
        return;
    }
    // A fresh evaluator beside a fresh reader: both start at the bake
    // time's generation, the compile notices in it.
    RigExecRigEvaluator measured(stage, rigPath);
    measured.SetEvaluationMode(RigExecEvaluationMode::Baked);

    bool failed = false;
    int compared = 0;
    for (double frame : times) {
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
        if (!player.Play(frame, &error)) {
            std::printf("execute diagnostic at %s frame %.17g: %s\n",
                        name.c_str(), frame, error.c_str());
            CHECK(false);
            failed = true;
            break;
        }
        const RigExecRuntimeReader &reader = player.Reader();

        const std::vector<RrPointFrame> &fin = reader.GetFinFrames();
        const std::vector<RrPointFrame> &base = reader.GetBaseFrames();
        const std::vector<RrMat4d> &finalM = reader.GetFinalMatrices();
        const std::vector<RrMat4d> &baseM = reader.GetBaseMatrices();
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
            reader.GetJointMatrices();
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

        // Diagnostics verbatim, the summary line included.
        const std::vector<std::string> &diagnostics = reader.GetDiagnostics();
        if (diagnostics != pose.diagnostics) {
            std::printf("%s frame %.17g: diagnostics differ "
                        "(%zu vs %zu)\n", name.c_str(), frame,
                        diagnostics.size(), pose.diagnostics.size());
            const size_t common =
                std::min(diagnostics.size(), pose.diagnostics.size());
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
        if (played) {
            played->push_back(_Snapshot(reader));
        }
        ++compared;
    }

    if (failed) {
        std::printf("%s: FAILED\n", name.c_str());
    } else {
        ++comparedFixtures;
        comparedFrames += compared;
        std::printf("%s: compared %d frame(s) of a bake at %.17g%s (%zu "
                    "input(s), %zu sampled per frame)\n",
                    name.c_str(), compared, frames.front(),
                    staticClass ? ", static" : "",
                    player->GetInputCount(),
                    player.Sampler().GetAnimatedCount());
    }
}

static void
_TestFixture(const std::string &name, const std::string &stagePath,
             const std::vector<double> &frames, bool staticClass = false)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        std::printf("%s: FAILED (no stage)\n", name.c_str());
        return;
    }
    _TestStage(name, stage, frames, nullptr, staticClass);
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

// Constraint envelopes are computed, not replayed: a file baked at the
// first frame plays every frame bit for bit with the baked program, and
// the dynamic envelope's transform moves between the first and last frame
// although the file holds the first frame's static data alone.
static void
TestComputedEnvelopes()
{
    const std::vector<double> frames = {1.0, 5.0, 10.0};
    std::vector<_PlayedFrame> rows;
    _TestStage("computed envelopes", _EnvelopeStage(), frames, &rows);
    _CheckFramesDiffer("computed envelopes", rows);

    // The dynamic envelope driven to 1.6 at frame 10 breaks its strict
    // range: the oracle's error passes the constraint through with the
    // program's diagnostic (compared verbatim by the parity path).
    rows.clear();
    _TestStage("computed envelopes, strict violation", _EnvelopeStage(1.6),
               frames, &rows);
    _CheckFramesDiffer("computed envelopes, strict violation", rows);
    // The violation happens at frame 10 alone: the driver reaches 1.6 there
    // and stays inside the range at frames 1 and 5.
    const auto violates = [](const _PlayedFrame &row) {
        for (const std::string &line : row.diagnostics) {
            if (line.find("strict range violation") != std::string::npos) {
                return true;
            }
        }
        return false;
    };
    CHECK(rows.size() == frames.size());
    if (rows.size() == frames.size()) {
        CHECK(!violates(rows[0]) && !violates(rows[1]) && violates(rows[2]));
    }
}

// An Animated input the stage holds no value for at a time: a blocked
// sample from frame 5 on, and Default on an attribute keyed alone. The
// sampler leaves the input with no value there, so the binary reads its
// fallback as the evaluator reads the stage, rather than the bake time's
// value.
static void
TestSampledInputWithNoValue()
{
    const UsdStageRefPtr stage = _EnvelopeStage();
    const UsdAttribute amount =
        stage->GetAttributeAtPath(SdfPath("/Asset/Dial.avars:amount"));
    CHECK(amount && amount.Set(SdfValueBlock(), UsdTimeCode(5.0)));
    double probe = 0.0;
    CHECK(!amount.Get(&probe, UsdTimeCode(7.0)) &&
          !amount.Get(&probe, UsdTimeCode::Default()));
    const double defaultTime = UsdTimeCode::Default().GetValue();
    std::vector<_PlayedFrame> rows;
    _TestStage("sampled input with no value", stage,
               {1.0, 7.0, defaultTime, 10.0}, &rows);
    // Frames 7 and Default read the fallback, frame 1 the keyed 0.2.
    CHECK(rows.size() == 4);
    if (rows.size() == 4) {
        CHECK(!_SamePlayed(rows[0], rows[1], false));
        CHECK(_SamePlayed(rows[1], rows[2], false));
    }
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

// Plays \p stage's bake at the first frame through its inputs beside the
// dynamic path: an ExecReference evaluator (the exec-authoritative walk
// every evaluator is held to) and the dynamic weight oracle itself
// (RigExecRigEvaluator::_ResolveWeights, which the baked program holds as
// resolveWeights). At every frame each probe's revised transform must
// equal the dynamic evaluator's bit for bit and imply exactly the envelope
// the oracle resolves. \p envelopes receives the oracle's envelopes, one
// row per frame in probe order. False on any difference.
static bool
_EnvelopesMatchDynamic(const std::string &name, const UsdStageRefPtr &stage,
                       const std::vector<double> &frames,
                       const std::vector<_EnvelopeProbe> &probes,
                       std::vector<std::vector<float>> *envelopes)
{
    const SdfPath rigPath = _FindRig(stage);
    RigExecRigEvaluator baked(stage, rigPath);
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    if (!RigExecTestBakeAt(baked, frames.front(), &bytes, &error)) {
        std::printf("%s: bake failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
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
        if (!player.Play(frame, &error)) {
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
                _FindProviderXform(player.Reader(), probe.target);
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
    _TestStage(name, _StaticWeightConstraintStage(), frames);
    std::vector<std::vector<float>> envelopes;
    CHECK(_EnvelopesMatchDynamic(
        name, _StaticWeightConstraintStage(), frames,
        {{"/Asset/Target", "/Asset/Rig/Weights/W", 10.0}}, &envelopes));
    // The applied envelope is the weight object's at every frame.
    CHECK(envelopes.size() == frames.size());
    for (const std::vector<float> &row : envelopes) {
        CHECK(row == std::vector<float>{0.5f});
    }
    std::printf("%s: %zu frame(s) applied the weight object's envelope\n",
                name, envelopes.size());
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

// The avar as an input set on a file baked at frame 1: Dial.avars:tx set
// to 1.2 is what a fresh evaluator in the dynamic and the baked mode
// publishes at frame 1 with 1.2 authored on the avar in the session
// layer, every output bit for bit. The set moves Driven's envelope off the
// keyed value's to (1 * 1.2f) * 0.625 + 0.125.
static void
_TestAvarInputMatchesSessionEdit()
{
    const char *const name = "avar-driven envelope, input set";
    const UsdStageRefPtr stage = _AvarDrivenStage();
    const SdfPath rigPath("/Asset/Rig");
    const std::string tx = "/Asset/Rig/Controls/Dial.avars:tx";
    const double bakeTime = 1.0;
    const double value = 1.2;
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(RigExecTestBakeAt(evaluator, bakeTime, &bytes, &error));
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    CHECK(player.Play(bakeTime, &error));
    const RigExecRuntimeProviderXform *defaults =
        _FindProviderXform(player.Reader(), "/Asset/TargetA");
    const double defaultX = defaults ? defaults->matrix[3][0] : 0.0;
    CHECK(player->SetInput(tx, value, &error));
    CHECK(player.Play(bakeTime, &error));
    const RigExecRuntimeProviderXform *moved =
        _FindProviderXform(player.Reader(), "/Asset/TargetA");
    CHECK(defaults && moved);
    if (moved) {
        const float driven = (1.0f * static_cast<float>(value)) * 0.625f +
                             0.125f;
        CHECK(moved->matrix[3][0] == 10.0 * double(driven));
        CHECK(moved->matrix[3][0] != defaultX);
    }
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        std::vector<RigExecRigPose> poses;
        CHECK(RigExecTestEditedPoses(stage, rigPath, mode,
                                     {{SdfPath(tx), VtValue(value)}},
                                     {bakeTime}, &poses, &error));
        std::vector<std::string> diffs;
        const bool same =
            poses.size() == 1 &&
            RigExecCompareRuntimeOutputs(poses[0], player.Reader(), &diffs);
        CHECK(same);
        std::printf("%s, %s: %s\n", name,
                    mode == RigExecEvaluationMode::Dynamic ? "dynamic"
                                                           : "baked",
                    same ? "binary == session edit" : "MISMATCH");
        for (const std::string &line : diffs) {
            std::printf("    %s\n", line.c_str());
        }
    }
}

static void
TestAvarDrivenDynamicEnvelope()
{
    const char *const name = "avar-driven dynamic envelope";
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_PlayedFrame> rows;
    _TestStage(name, _AvarDrivenStage(), frames, &rows);
    _CheckFramesDiffer(name, rows);
    std::vector<std::vector<float>> envelopes;
    CHECK(_EnvelopesMatchDynamic(
        name, _AvarDrivenStage(), frames,
        {{"/Asset/TargetA", "/Asset/Rig/Weights/Driven", 10.0},
         {"/Asset/TargetB", "/Asset/Rig/Weights/Modulated", 10.0},
         {"/Asset/TargetC", "/Asset/Rig/Weights/Blend", 10.0}},
        &envelopes));

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
    _TestAvarInputMatchesSessionEdit();
    std::printf("%s: %zu frame(s) of envelopes equal the oracle's\n", name,
                envelopes.size());
}

// The avar-driven stage with a float math mover clamping the Dial avar
// to [0, 1]: each driver's walk crosses the chain's target, so the oracle
// reads the chain's double result through the generation's overlay, not
// the authored avar. The parity path compares the runtime against the
// baked program, the dynamic comparison against ExecReference and the
// dynamic oracle, and the envelopes must be the oracle's arithmetic over
// the clamped value: from frame 7 on that differs from the authored
// value's.
static void
TestChainDrivenEnvelope()
{
    const char *const name = "chain-driven envelope";
    const std::vector<double> frames = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<_PlayedFrame> rows;
    _TestStage(name, _AvarDrivenStage(true), frames, &rows);
    _CheckFramesDiffer(name, rows);
    std::vector<std::vector<float>> envelopes;
    CHECK(_EnvelopesMatchDynamic(
        name, _AvarDrivenStage(true), frames,
        {{"/Asset/TargetA", "/Asset/Rig/Weights/Driven", 10.0},
         {"/Asset/TargetB", "/Asset/Rig/Weights/Modulated", 10.0},
         {"/Asset/TargetC", "/Asset/Rig/Weights/Blend", 10.0}},
        &envelopes));
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
    std::printf("%s: %zu frame(s) read through the chain\n", name, chained);
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
    opts.time = 1.0;
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
           "the file carries no computed section, which every input read "
           "needs; rebake it");
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

// One frame of a chain case: the runtime's property results by path, and
// the chains' diagnostic lines.
struct _ChainFrame {
    std::map<std::string, RrPropertyValue> values;
    std::vector<std::string> lines;
};

// Plays \p stage's bake at the first frame through its inputs beside an
// ExecReference evaluator (the exec-authoritative walk every evaluator is
// held to). At every frame each property result the runtime computed must
// equal the dynamic evaluator's published value bit for bit, the two must
// publish the same set, and the chains' diagnostic lines must agree
// verbatim. \p rows receives the runtime's results and lines per frame.
// False on any difference.
static bool
_ChainsMatchDynamic(const std::string &name, const UsdStageRefPtr &stage,
                    const std::vector<double> &frames,
                    std::vector<_ChainFrame> *rows)
{
    const SdfPath rigPath = _FindRig(stage);
    RigExecRigEvaluator baked(stage, rigPath);
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    if (!RigExecTestBakeAt(baked, frames.front(), &bytes, &error)) {
        std::printf("%s: bake failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open failed: %s\n", name.c_str(), error.c_str());
        return false;
    }
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
        if (!player.Play(frame, &error)) {
            std::printf("%s frame %g: %s\n", name.c_str(), frame,
                        error.c_str());
            return false;
        }
        _ChainFrame row;
        for (const RigExecRuntimePropertyValue &got :
             player->GetPropertyValues()) {
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
            _ChainLines(player->GetDiagnostics());
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

// One chain case: the binary baked at the first frame and played through
// its inputs against the baked program (verbatim diagnostics), its first
// and last frames apart, then against the dynamic evaluator, which must
// see exactly \p expected property values published over the frames.
static bool
_RunChainCase(const char *name, UsdStageRefPtr (*build)(),
              const std::vector<double> &frames, size_t expected,
              std::vector<_ChainFrame> *rows)
{
    const int failuresBefore = failures;
    std::vector<_PlayedFrame> played;
    _TestStage(name, build(), frames, &played);
    if (frames.size() > 1) {
        _CheckFramesDiffer(name, played);
    }
    CHECK(_ChainsMatchDynamic(name, build(), frames, rows));
    // These rigs compute nothing but property chains.
    size_t published = 0;
    for (const _ChainFrame &row : *rows) {
        published += row.values.size();
    }
    CHECK(published == expected);
    CHECK(published > 0);
    CHECK(rows->size() == frames.size());
    std::printf("%s: %zu property value(s) equal the dynamic evaluator's\n",
                name, published);
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
    opts.time = frames.front();
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
    const char *const name = "non-finite bases and results";
    // Bad's base is +inf at frame 3 and NaN at frame 5. The sampler hands
    // the reader the stage's own values, non-finite ones included, so one
    // bake at frame 1 plays all four frames. Bad publishes at frames 1 and
    // 7, Huge never, Blow at every frame.
    std::vector<_ChainFrame> rows;
    if (!_RunChainCase(name, _NonFiniteStage, {1, 3, 5, 7}, 6, &rows)) {
        return;
    }
    // A bake at frame 3 or 5 holds the non-finite base as its default.
    for (const double frame : {3.0, 5.0}) {
        std::vector<_ChainFrame> baked;
        if (!_RunChainCase(name, _NonFiniteStage, {frame}, 1, &baked)) {
            return;
        }
    }
    const std::string badPath = "/Asset/Rig/Channels/Bad.rigExec:amount";
    {
        // A host's set takes finite values only; the stage's +inf reaches
        // the reader through the sampler alone.
        const UsdStageRefPtr stage = _NonFiniteStage();
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::vector<uint8_t> bytes;
        std::string error;
        CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
        RigExecTestPlayer player;
        CHECK(player.Open(bytes, stage, &error));
        error.clear();
        CHECK(!player->SetInput(badPath,
                                double(std::numeric_limits<float>::infinity()),
                                &error));
        CHECK(error == badPath + " takes finite values only");
        CHECK(player.Play(3.0, &error));
        size_t index = 0;
        CHECK(player->FindInput(badPath, &index));
        const RrInputValue &held = player->GetInputValue(index);
        CHECK(held.tag == RrInputTag::Float && std::isinf(held.f32));
    }
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

// tests/fixtures/computed_chains.usda: the chain cases together (a curve
// with tangents, envelopes on property revisions, double, vec3f and matrix
// chains, the chains' diagnostics, phased consumers) played through the
// inputs of a bake at the first frame, against the baked program and the
// dynamic evaluator, 10 values per frame, the constraint weight Follow
// reads through the dial's chain at its declared `final` included. Without
// the computed section the file does not open: nothing could read an
// input.
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
    std::vector<_PlayedFrame> played;
    _TestStage(name, stage, frames, &played);
    _CheckFramesDiffer(name, played);
    std::vector<_ChainFrame> rows;
    CHECK(_ChainsMatchDynamic(name, stage, frames, &rows));
    size_t published = 0;
    for (const _ChainFrame &row : rows) {
        published += row.values.size();
    }
    CHECK(published == 10 * frames.size());

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    CHECK(_OpenError(_WithoutSection(bytes,
                                     RigExecBinarySection::Computed)) ==
          "the file carries no computed section, which every input read "
          "needs; rebake it");
}

// tests/fixtures/computed_ik_space.usda: a TwoBoneIk measured in an
// animated rigExec:space composed with a keyed, non-uniform
// rigExec:spaceMatrix, beside one reading a constant spaceMatrix. The arm's
// spaceMatrix is a registered read the record directory lists and the
// record holds (a bake that left it out of the directory refused the rig),
// and the runtime plays the fixture through its inputs bit for bit with
// the baked program.
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
    std::vector<_PlayedFrame> played;
    _TestStage(name, stage, frames, &played);
    _CheckFramesDiffer(name, played);

    RigExecRigEvaluator evaluator(stage, SdfPath("/IkSpaceAsset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    const std::unique_ptr<RigExecBinaryReader> binary =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(binary);
    if (!binary) {
        return;
    }
    const std::string spaceMatrix =
        "/IkSpaceAsset/Rig/Solvers/ArmIK.rigExec:spaceMatrix";
    RigExecWireInputTable table;
    CHECK(_DecodeInputTable(bytes, &table));
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
    CHECK(table.frames.size() == 1);
    if (uid < 0 || table.frames.size() != 1) {
        return;
    }
    // Recorded, and not the identity.
    bool held = false;
    const RigExecWireFrameInputs &record = table.frames[0];
    for (size_t k = 0; k < record.uids.size(); ++k) {
        if (record.uids[k] == uint32_t(uid)) {
            held = true;
            CHECK(record.values[k].matrix[0] != 1.0);
        }
    }
    CHECK(held);

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
    played.clear();
    _TestStage("ik space fixture, constant leg space", constant, frames,
               &played);
    _CheckFramesDiffer("ik space fixture, constant leg space", played);
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

// One evaluation of a drag's reference: the pose and the program's
// version pools.
struct _Reference {
    RigExecRigPose pose;
    std::vector<RigExecPointFrame> fin, base;
};

// A fresh BakedWithParityCheck evaluator compiled on \p stage, handed
// \p edits in the session layer and evaluated at each of \p frames
// (RigExecTestEditedPoses, compiled after the edits with
// \p compileAfterEdits), with its version pools after each. The parity
// check holds the baked program to the dynamic evaluator.
static std::vector<_Reference>
_References(const UsdStageRefPtr &stage, const SdfPath &rigPath,
            const std::vector<RigExecTestEdit> &edits,
            const std::vector<double> &frames,
            bool compileAfterEdits = false)
{
    std::vector<RigExecRigPose> poses;
    std::vector<_Reference> out(frames.size());
    std::string error;
    const bool ok = RigExecTestEditedPoses(
        stage, rigPath, RigExecEvaluationMode::BakedWithParityCheck, edits,
        frames, &poses, &error,
        [&](const RigExecRigEvaluator &evaluator, size_t i) {
            const RigExecBakedProgram *program = evaluator.GetBakedProgram();
            if (program) {
                out[i].fin = program->GetStepGraph().fin;
                out[i].base = program->GetStepGraph().base;
            }
        },
        compileAfterEdits);
    CHECK(ok);
    if (!ok) {
        std::printf("reference: %s\n", error.c_str());
    }
    for (size_t i = 0; i < poses.size() && i < out.size(); ++i) {
        out[i].pose = std::move(poses[i]);
    }
    return out;
}

// \p value on the attribute at \p path, typed to it.
static RigExecTestEdit
_Edit(const UsdStageRefPtr &stage, const std::string &path, double value)
{
    return {SdfPath(path),
            RigExecTestTypedValue(stage->GetAttributeAtPath(SdfPath(path)),
                                  value)};
}

// Whether the runtime's version pools equal \p fin and \p base bit for
// bit, naming the first slot that differs.
static bool
_SamePools(const char *what, double frame, const RigExecRuntimeReader &reader,
           const std::vector<RigExecPointFrame> &wantFin,
           const std::vector<RigExecPointFrame> &wantBase)
{
    const std::vector<RrPointFrame> &fin = reader.GetFinFrames();
    const std::vector<RrPointFrame> &base = reader.GetBaseFrames();
    if (fin.size() != wantFin.size() || base.size() != wantBase.size()) {
        std::printf("%s frame %.17g: the version pools differ in size\n",
                    what, frame);
        return false;
    }
    for (size_t i = 0; i < fin.size(); ++i) {
        if (!_FramesEqual(wantFin[i], fin[i])) {
            std::printf("%s frame %.17g: fin[%zu] differs\n", what, frame,
                        i);
            return false;
        }
    }
    for (size_t i = 0; i < base.size(); ++i) {
        if (!_FramesEqual(wantBase[i], base[i])) {
            std::printf("%s frame %.17g: base[%zu] differs\n", what, frame,
                        i);
            return false;
        }
    }
    return true;
}

static bool
_SamePools(const char *what, double frame, const RigExecRuntimeReader &reader,
           const RigExecBakedProgramImpl &graph)
{
    return _SamePools(what, frame, reader, graph.fin, graph.base);
}

static bool
_SamePools(const char *what, double frame, const RigExecRuntimeReader &reader,
           const _Reference &reference)
{
    return _SamePools(what, frame, reader, reference.fin, reference.base);
}

// The control's point \p axis of landmark 0 in \p pose, NaN when the pose
// has no frame for it.
static double
_ControlAt(const RigExecRigPose &pose, const SdfPath &control, size_t axis)
{
    const auto found = pose.controlFrames.find(control);
    return found == pose.controlFrames.end() ? std::nan("")
                                             : found->second.points[0][axis];
}

// An input set on an avar that reads a chain at its base through a
// connection: Lift's avars:ty, connected to the height channel the Raise
// mover revises. The set is an authored value on the head of that
// connection, and the walk reads past it to the channel, as the evaluators
// do with 5.5 authored on the avar in the session layer: the reader does
// not stand aside, and the pose is the undragged one. Played through the
// inputs of a bake at frame 1 at frames 1 and 4, every version-pool frame
// equals the baked program's under the session edit bit for bit.
static void
TestSetInputOnPhasedReader()
{
    const char *const name = "set input on a phased reader";
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
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const std::string ty = "/Asset/Rig/Controls/Lift.avars:ty";
    const double drag = 5.5;
    const std::vector<_Reference> undragged =
        _References(stage, rigPath, {}, frames);
    const std::vector<_Reference> dragged =
        _References(stage, rigPath, {_Edit(stage, ty, drag)}, frames);
    CHECK(player.Hold(ty, drag, &error));
    for (size_t f = 0; f < frames.size() && f < dragged.size(); ++f) {
        CHECK(player.Play(frames[f], &error));
        CHECK(_SamePools(name, frames[f], player.Reader(), dragged[f]));
        CHECK(_SamePools(name, frames[f], player.Reader(), undragged[f]));
        CHECK(_ControlAt(dragged[f].pose, lift.GetPath(), 1) != drag);
    }
    std::printf("%s: checked\n", name);
}

// Inputs set on avars partway along other readers' connections. Hop's
// avars:tx reads the Raise chain's target at its base and Lift's avars:ty
// reads that chain through Hop's tx, so both are phased readers; Reach's
// avars:tz reads Pin's avars:tx, which no chain revises. Set on Hop's tx,
// an authored value is shadowed by the readable channel upstream of it:
// Lift and Hop keep reading the channel, and both readers still publish.
// Set on Pin's tx, it reaches Reach. Before, while and after the sets
// stand, every version-pool frame equals the baked program's under the
// same values authored in the session layer, bit for bit, and the baked
// program agrees with the dynamic evaluator.
static void
TestSetInputOnHop()
{
    const char *const name = "set input on a hop";
    const UsdStageRefPtr stage = _LiftReadStage(false);
    const UsdAttribute height = stage->GetAttributeAtPath(
        SdfPath("/Asset/Rig/Channels/Lift.rigExec:height"));
    const auto control = [&](const char *path) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        return prim;
    };
    const UsdPrim hop = control("/Asset/Rig/Controls/Hop");
    const UsdAttribute hopTx =
        hop.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    hopTx.SetConnections({height.GetPath()});
    const UsdPrim lift =
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Controls/Lift"));
    lift.GetAttribute(TfToken("avars:ty")).SetConnections({hopTx.GetPath()});
    const UsdPrim pin = control("/Asset/Rig/Controls/Pin");
    const UsdAttribute pinTx =
        pin.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    pinTx.Set(0.25);
    const UsdPrim reach = control("/Asset/Rig/Controls/Reach");
    reach.CreateAttribute(TfToken("avars:tz"), SdfValueTypeNames->Double)
        .SetConnections({pinTx.GetPath()});

    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 4.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const std::string hopPath = hopTx.GetPath().GetString();
    const std::string pinPath = pinTx.GetPath().GetString();
    const std::string liftPath = "/Asset/Rig/Controls/Lift.avars:ty";
    // Each phased reader's record is published whatever stands.
    const auto published = [&](const std::string &path) {
        for (const RigExecRuntimePropertyValue &value :
             player->GetPropertyValues()) {
            if (value.path == path) {
                return true;
            }
        }
        return false;
    };

    // One pass over the frames against \p references.
    const auto pass = [&](const char *what, bool dragging,
                          const std::vector<_Reference> &references) {
        bool same = references.size() == frames.size();
        for (size_t f = 0; f < frames.size() && f < references.size();
             ++f) {
            const RigExecRigPose &pose = references[f].pose;
            if (!player.Play(frames[f], &error)) {
                std::printf("%s, %s frame %.17g: %s\n", name, what,
                            frames[f], error.c_str());
                CHECK(false);
                return false;
            }
            same = _SamePools(what, frames[f], player.Reader(),
                              references[f]) &&
                   same;
            // The set on the hop is shadowed upstream; the set on the
            // unrevised avar reaches its reader.
            CHECK(_ControlAt(pose, lift.GetPath(), 1) != 5.5 &&
                  _ControlAt(pose, lift.GetPath(), 1) ==
                      _ControlAt(pose, hop.GetPath(), 0));
            CHECK(_ControlAt(pose, reach.GetPath(), 2) ==
                  (dragging ? -0.75 : 0.25));
            CHECK(published(liftPath) && published(hopPath));
        }
        CHECK(same);
        return same;
    };
    const std::vector<_Reference> undragged =
        _References(stage, rigPath, {}, frames);
    CHECK(pass("undragged", false, undragged));
    CHECK(player.Hold(hopPath, 5.5, &error));
    CHECK(player.Hold(pinPath, -0.75, &error));
    CHECK(pass("dragged", true,
               _References(stage, rigPath,
                           {_Edit(stage, hopPath, 5.5),
                            _Edit(stage, pinPath, -0.75)},
                           frames)));
    player.ReleaseAll();
    CHECK(pass("released", false, undragged));
    std::printf("%s: checked\n", name);
}

// An input set on an avar a provider ladder reads: Ctl's rest:tx is
// connected to Pin's constant avars:tx, so no frame recomposes the ladder
// until a value is set on Pin's tx, and the run after its reset recomposes
// it once more to write the authored rest back. Every version-pool frame
// equals the baked program's under the same value authored in the session
// layer, bit for bit: unset, set, reset at the frame the set stood on, and
// reset; and the set moves the pools.
static void
TestSetInputOnLadderInput()
{
    const char *const name = "set input on a ladder input";
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(4.0);
    UsdGeomXform::Define(stage, SdfPath("/Asset"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *path) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        return prim;
    };
    const UsdPrim pin = control("/Asset/Rig/Controls/Pin");
    const UsdAttribute pinTx =
        pin.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    pinTx.Set(0.25);
    const UsdPrim ctl = control("/Asset/Rig/Controls/Ctl");
    ctl.CreateAttribute(TfToken("rest:tx"), SdfValueTypeNames->Double)
        .SetConnections({pinTx.GetPath()});

    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 4.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    // One run at \p frame against \p reference: the pools into \p fin.
    const auto run = [&](const char *what, double frame,
                         const _Reference &reference,
                         std::vector<std::vector<RrPointFrame>> *fin) {
        if (!player.Play(frame, &error)) {
            std::printf("%s, %s frame %.17g: %s\n", name, what, frame,
                        error.c_str());
            CHECK(false);
            return false;
        }
        fin->push_back(player->GetFinFrames());
        return _SamePools(what, frame, player.Reader(), reference);
    };
    const auto pass = [&](const char *what,
                          const std::vector<_Reference> &references,
                          std::vector<std::vector<RrPointFrame>> *fin) {
        bool same = references.size() == frames.size();
        for (size_t f = 0; f < frames.size() && f < references.size();
             ++f) {
            same = run(what, frames[f], references[f], fin) && same;
        }
        return same;
    };
    const std::string pinPath = pinTx.GetPath().GetString();
    const std::vector<_Reference> unset =
        _References(stage, rigPath, {}, frames);
    std::vector<std::vector<RrPointFrame>> undragged, dragged, releasedHere,
        released;
    CHECK(pass("undragged", unset, &undragged));
    CHECK(player.Hold(pinPath, -0.75, &error));
    // Compiled with the value authored: a value edit of an avar a ladder
    // reads through a connection, taken by an evaluator compiled before
    // it, breaks the baked program's parity with the dynamic evaluator.
    CHECK(pass("dragged",
               _References(stage, rigPath, {_Edit(stage, pinPath, -0.75)},
                           frames, true),
               &dragged));
    CHECK(dragged.size() == frames.size() &&
          undragged.size() == frames.size());
    for (size_t f = 0; f < dragged.size() && f < undragged.size(); ++f) {
        CHECK(dragged[f] != undragged[f]);
    }
    player.ReleaseAll();
    if (!unset.empty()) {
        CHECK(run("released in place", frames.back(), unset.back(),
                  &releasedHere));
    }
    CHECK(!releasedHere.empty() && releasedHere.back() == undragged.back());
    CHECK(pass("released", unset, &released));
    CHECK(released == undragged);
    std::printf("%s: checked\n", name);
}

// The runtime's property results against \p pose's published values, both
// ways, bit for bit.
static bool
_SamePropertyResults(const char *what, const RigExecRigPose &pose,
                     const RigExecRuntimeReader &reader)
{
    bool same = true;
    std::map<std::string, RrPropertyValue> got;
    for (const RigExecRuntimePropertyValue &value :
         reader.GetPropertyValues()) {
        got[value.path] = value.value;
        const auto found = pose.movedProperties.find(SdfPath(value.path));
        if (found == pose.movedProperties.end() ||
            !_SamePropertyValue(found->second, value.value)) {
            std::printf("%s: %s differs from the baked program\n", what,
                        value.path.c_str());
            same = false;
        }
    }
    for (const auto &[path, value] : pose.movedProperties) {
        if (_IsPropertyResult(value) && !got.count(path.GetString())) {
            std::printf("%s: the runtime publishes nothing at %s\n", what,
                        path.GetText());
            same = false;
        }
    }
    return same;
}

// An input set at a fixed frame on \p control's avars:tx, which a property
// chain's input reads through a connection, the chain's result at
// \p chainPath being what a constraint moving /Asset/Target weighs by.
// The frame does not change, so the set reaches the constraint only
// through the chain result it moves; baked with every step in its own
// cluster (RIGEXEC_BAKED_GRAIN_US=0), only the steps the input's own
// dirtying reaches rerun. Under the same value authored in the session
// layer the runtime's property results and version pools equal the baked
// program's bit for bit, set and reset, and the set moves the chain's
// result and the constrained transform.
static void
_SetChainInput(const std::string &name, const UsdStageRefPtr &stage,
               const SdfPath &control, const std::string &chainPath)
{
    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 5.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name.c_str(), error.c_str());
        CHECK(false);
        return;
    }
    // One run at \p frame against \p reference: the property results and
    // pools; the chain's result and Target's transform out.
    const auto run = [&](const char *what, double frame,
                         const _Reference &reference, float *chainValue,
                         RrMat4d *target) {
        if (!player.Play(frame, &error)) {
            std::printf("%s, %s frame %.17g: %s\n", name.c_str(), what,
                        frame, error.c_str());
            CHECK(false);
            return false;
        }
        bool chainFound = false, targetFound = false;
        for (const RigExecRuntimePropertyValue &value :
             player->GetPropertyValues()) {
            if (value.path == chainPath) {
                *chainValue = value.value.f32;
                chainFound = true;
            }
        }
        for (const RigExecRuntimeProviderXform &x :
             player->GetProviderXforms()) {
            if (x.path == "/Asset/Target") {
                *target = x.matrix;
                targetFound = true;
            }
        }
        CHECK(chainFound && targetFound);
        const std::string label = name + ", " + what;
        const bool properties = _SamePropertyResults(
            label.c_str(), reference.pose, player.Reader());
        return _SamePools(label.c_str(), frame, player.Reader(),
                          reference) &&
               properties;
    };
    const std::string tx = control.GetString() + ".avars:tx";
    const std::vector<_Reference> unset =
        _References(stage, rigPath, {}, frames);
    const std::vector<_Reference> set =
        _References(stage, rigPath, {_Edit(stage, tx, 2.0)}, {frames.back()});
    if (unset.size() != frames.size() || set.size() != 1) {
        CHECK(false);
        return;
    }
    float undraggedChain = 0.0f, draggedChain = 0.0f, releasedChain = 0.0f;
    RrMat4d undraggedTarget(1.0), draggedTarget(1.0), releasedTarget(1.0);
    float ignoredChain = 0.0f;
    RrMat4d ignoredTarget(1.0);
    CHECK(run("first frame", frames.front(), unset.front(), &ignoredChain,
              &ignoredTarget));
    CHECK(run("unset", frames.back(), unset.back(), &undraggedChain,
              &undraggedTarget));
    CHECK(player.Hold(tx, 2.0, &error));
    CHECK(run("set", frames.back(), set.front(), &draggedChain,
              &draggedTarget));
    CHECK(draggedChain != undraggedChain);
    CHECK(draggedTarget != undraggedTarget);
    player.ReleaseAll();
    CHECK(run("reset", frames.back(), unset.back(), &releasedChain,
              &releasedTarget));
    CHECK(std::memcmp(&releasedChain, &undraggedChain, sizeof(float)) == 0);
    CHECK(releasedTarget == undraggedTarget);
    std::printf("%s: checked\n", name.c_str());
}

// A position constraint whose own inputs:defaultWeight a math mover
// multiplies by Controls/Knob.avars:tx through a connection. Nothing
// declares a read phase, so the runtime runs only what a frame dirties.
static UsdStageRefPtr
_RevisedWeightStage()
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    UsdGeomXform::Define(stage, SdfPath("/Asset/Target"))
        .AddTransformOp()
        .Set(GfMatrix4d(1.0));
    UsdGeomXform::Define(stage, SdfPath("/Asset/Source"))
        .AddTransformOp()
        .Set(GfMatrix4d(1.0).SetTranslate(GfVec3d(10.0, 0.0, 0.0)));
    const UsdPrim knob = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Knob"), TfToken("RigExecControl"));
    knob.CreateAttribute(TfToken("rest:space"), SdfValueTypeNames->Matrix4d)
        .Set(GfMatrix4d(1.0));
    const UsdAttribute tx =
        knob.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    tx.Set(0.25, UsdTimeCode(1.0));
    tx.Set(0.75, UsdTimeCode(10.0));
    const UsdPrim follow =
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Follow"),
                          TfToken("RigExecPositionConstraint"));
    CHECK(follow.ApplyAPI(TfToken("RigExecMoverAPI")));
    const UsdAttribute weight = follow.CreateAttribute(
        TfToken("inputs:defaultWeight"), SdfValueTypeNames->Float);
    weight.Set(0.5f);
    follow.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Target")});
    follow.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    const UsdPrim gain =
        _MathMover(stage, "Gain", "RigExecFloatMathMover", "multiply", weight);
    gain.CreateAttribute(TfToken("inputs:value"), SdfValueTypeNames->Float)
        .SetConnections({tx.GetPath()});
    return stage;
}

// computed_chains' Gain multiplies the dial by Controls/Dial.avars:tx and
// Follow's weight reads the dial's result at its declared `final`; and the
// rig above, where nothing declares a phase.
static void
TestSetInputOnChainInput()
{
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "computed_chains.usda")
            .string();
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (stage) {
        _SetChainInput("set input on a chain input", stage,
                       SdfPath("/Asset/Rig/Controls/Dial"),
                       "/Asset/Rig/Channels/Dial.rigExec:amount");
    }
    _SetChainInput("set input on a revised weight's input",
                   _RevisedWeightStage(), SdfPath("/Asset/Rig/Controls/Knob"),
                   "/Asset/Rig/Movers/Follow.inputs:defaultWeight");
}

// The blink on a control avar: Dial.avars:tx authored at 0.2 (or
// \p authored), revised by Offset (add 0.1) and then Clamp ([0, 1]), and
// read by other controls' avars undeclared (its base), at Offset's
// checkpoint and at `final` (no record: the walk reads the dial's result),
// and by readout chains inside the chain loop, undeclared and `final`.
static UsdStageRefPtr
_DialTargetStage(double authored)
{
    const UsdStageRefPtr stage = _ChainBaseStage();
    const auto control = [&](const char *name) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Controls/") + name),
            TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        return prim;
    };
    const UsdAttribute tx = control("Dial").CreateAttribute(
        TfToken("avars:tx"), SdfValueTypeNames->Double);
    tx.Set(authored);
    const UsdPrim clamp =
        _MathMover(stage, "Clamp", "RigExecFloatMathMover", "clamp", tx);
    _SetInput(clamp, "inputs:min", SdfValueTypeNames->Float, 0.0f);
    _SetInput(clamp, "inputs:max", SdfValueTypeNames->Float, 1.0f);
    const UsdPrim offset = _MathMover(stage, "Clamp/Offset",
                                      "RigExecFloatMathMover", "add", tx);
    _SetInput(offset, "inputs:value", SdfValueTypeNames->Float, 0.1f);
    const auto read = [&](const UsdAttribute &input, const char *phase) {
        input.SetConnections({tx.GetPath()});
        if (phase) {
            input.SetMetadata(TfToken("rigExecReadPhase"),
                              std::string(phase));
        }
    };
    read(control("BaseReader")
             .CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double),
         nullptr);
    read(control("CheckReader")
             .CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double),
         "/Asset/Rig/Movers/Clamp/Offset");
    read(control("FinalReader")
             .CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double),
         "final");
    for (const char *name : {"Base", "Final"}) {
        const UsdAttribute channel =
            _Channel(stage, "Readouts", name, SdfValueTypeNames->Float);
        channel.Set(0.0f);
        const UsdPrim mover =
            _MathMover(stage, std::string("Readouts/") + name,
                       "RigExecFloatMathMover", "add", channel);
        read(mover.CreateAttribute(TfToken("inputs:value"),
                                   SdfValueTypeNames->Float),
             std::string(name) == "Final" ? "final" : nullptr);
    }
    return stage;
}

// One played frame: the version pools, the property results and the
// diagnostics.
struct _PlayedPose {
    std::vector<RrPointFrame> fin, base;
    std::vector<RigExecRuntimePropertyValue> properties;
    std::vector<std::string> diagnostics;
};

static _PlayedPose
_Played(const RigExecRuntimeReader &reader)
{
    return {reader.GetFinFrames(), reader.GetBaseFrames(),
            reader.GetPropertyValues(), reader.GetDiagnostics()};
}

// The first thing \p a and \p b disagree on, or empty when they agree bit
// for bit.
static std::string
_PlayedDiffer(const _PlayedPose &a, const _PlayedPose &b)
{
    const auto samePool = [](const std::vector<RrPointFrame> &x,
                             const std::vector<RrPointFrame> &y) {
        if (x.size() != y.size()) {
            return false;
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].flags != y[i].flags ||
                std::memcmp(x[i].points.data(), y[i].points.data(),
                            sizeof(x[i].points)) != 0) {
                return false;
            }
        }
        return true;
    };
    if (!samePool(a.fin, b.fin)) {
        return "fin pool";
    }
    if (!samePool(a.base, b.base)) {
        return "base pool";
    }
    if (a.properties.size() != b.properties.size()) {
        return "property count";
    }
    for (size_t i = 0; i < a.properties.size(); ++i) {
        if (a.properties[i].path != b.properties[i].path ||
            !_SameBits(a.properties[i].value, b.properties[i].value)) {
            return "property " + a.properties[i].path;
        }
    }
    if (a.diagnostics != b.diagnostics) {
        return "diagnostics";
    }
    return std::string();
}

// An input set on an avar math movers revise is the chain's base, as the
// value authored there is in the USD evaluators. Set to 1.4, the dial's
// base readers read 1.4, Offset's checkpoint 1.5 and the final readers and
// the dial itself 1.0. At frames 1 and 4 every version-pool frame equals
// the baked program's under 1.4 authored in the session layer (which the
// parity check holds to the dynamic evaluator), and the played pose --
// pools, property results and diagnostics -- equals bit for bit what a
// file baked with 1.4 authored on the dial plays. The same holds as the
// held value moves to 1.5 (the final holds at 1.0) and then to 0.5 without
// being reset. Reset, the input plays the original file again.
static void
TestSetInputOnChainTarget()
{
    const char *const name = "set input on a chain target";
    const SdfPath rigPath("/Asset/Rig");
    const std::string dialTx = "/Asset/Rig/Controls/Dial.avars:tx";
    const std::vector<double> frames = {1.0, 4.0};
    const double drag = 1.4;
    const auto bake = [&](const UsdStageRefPtr &stage,
                          std::vector<uint8_t> *bytes) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::string error;
        const bool ok =
            RigExecTestBakeAt(evaluator, frames.front(), bytes, &error);
        if (!ok) {
            std::printf("%s: bake: %s\n", name, error.c_str());
        }
        return ok;
    };
    const auto play = [&](RigExecTestPlayer &player, double frame,
                          _PlayedPose *out) {
        std::string error;
        if (!player.Play(frame, &error)) {
            std::printf("%s frame %.17g: %s\n", name, frame, error.c_str());
            return false;
        }
        *out = _Played(player.Reader());
        return true;
    };

    const UsdStageRefPtr stage = _DialTargetStage(0.2);
    std::vector<uint8_t> original;
    CHECK(bake(stage, &original));
    std::string error;
    RigExecTestPlayer player;
    if (!player.Open(original, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    std::vector<_PlayedPose> undragged(frames.size());
    for (size_t f = 0; f < frames.size(); ++f) {
        CHECK(play(player, frames[f], &undragged[f]));
    }

    // The set, then moved without a reset: 1.4 -> 1.5 leaves the clamped
    // final on 1.0 while the base and checkpoint readers move, and 1.5 ->
    // 0.5 moves every reader.
    struct HeldDrag {
        double drag, final, checkpoint;
    };
    const HeldDrag held[] = {
        {drag, 1.0, 1.5}, {1.5, 1.0, 1.6}, {0.5, 0.6, 0.6}};
    for (const HeldDrag &h : held) {
        char what[64];
        std::snprintf(what, sizeof(what), "chain target set to %.9g",
                      h.drag);
        CHECK(player.Hold(dialTx, h.drag, &error));
        const std::vector<_Reference> references = _References(
            stage, rigPath, {_Edit(stage, dialTx, h.drag)}, frames);
        if (references.size() != frames.size()) {
            CHECK(false);
            return;
        }
        std::vector<_PlayedPose> dragged(frames.size());
        for (size_t f = 0; f < frames.size(); ++f) {
            const RigExecRigPose &pose = references[f].pose;
            if (!play(player, frames[f], &dragged[f])) {
                CHECK(false);
                return;
            }
            CHECK(_SamePools(what, frames[f], player.Reader(),
                             references[f]));
            CHECK(!_PlayedDiffer(undragged[f], dragged[f]).empty());
            const auto ty = [&](const char *control) {
                return _ControlAt(
                    pose,
                    SdfPath(std::string("/Asset/Rig/Controls/") + control),
                    1);
            };
            const double dial =
                _ControlAt(pose, SdfPath("/Asset/Rig/Controls/Dial"), 0);
            CHECK(std::abs(dial - h.final) < 1e-6);
            CHECK(ty("BaseReader") == h.drag);
            CHECK(std::abs(ty("FinalReader") - h.final) < 1e-6);
            CHECK(std::abs(ty("CheckReader") - h.checkpoint) < 1e-6);
        }

        // The held value authored on the dial and baked again.
        stage->GetAttributeAtPath(SdfPath(dialTx)).Set(h.drag);
        std::vector<uint8_t> authored;
        CHECK(bake(stage, &authored));
        stage->GetAttributeAtPath(SdfPath(dialTx)).Set(0.2);
        RigExecTestPlayer released;
        if (!released.Open(authored, stage, &error)) {
            std::printf("%s: open released: %s\n", name, error.c_str());
            CHECK(false);
            return;
        }
        for (size_t f = 0; f < frames.size(); ++f) {
            _PlayedPose played;
            CHECK(play(released, frames[f], &played));
            const std::string why = _PlayedDiffer(dragged[f], played);
            if (!why.empty()) {
                std::printf("%s, frame %.17g: the set and the rebake "
                            "differ (%s)\n",
                            what, frames[f], why.c_str());
            }
            CHECK(why.empty());
        }
    }

    // Reset: the original file's poses again.
    player.ReleaseAll();
    for (size_t f = 0; f < frames.size(); ++f) {
        _PlayedPose played;
        CHECK(play(player, frames[f], &played));
        CHECK(_PlayedDiffer(undragged[f], played).empty());
    }
    std::printf("%s: checked\n", name);
}

// Open refuses a Computed section whose registered reads do not bind the
// runtime's tables exactly, each with its own text: two avar bindings
// headed by one attribute, a table field bound twice, a uid given twice,
// a table field left unbound, a chain read that differs from the
// registered read of its uid in its constant, an override number reaching
// another attribute than the program registered it on, and a walk through
// an attribute the program registered no override on. An input refuses a
// name the file does not list, a value of another type, a non-finite value
// and an index past the count; a chain target, which no avar override
// reached, takes a set like any input.
static void
TestRegisteredReadRefusals()
{
    const char *const name = "registered read refusals";
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "computed_chains.usda")
            .string();
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.time = 1.0;
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
    {
        RigExecWireReader cursor(data, size);
        CHECK(RigExecWireDecodeComputed(&cursor, &computed, &error));
    }
    const auto with = [&](const RigExecWireComputed &edited) {
        std::vector<uint8_t> payload;
        std::string why;
        CHECK(RigExecWireEncodeComputed(edited, &payload, &why));
        return _ReplaceSection(result.bytes, RigExecBinarySection::Computed,
                               payload);
    };
    const auto expect = [&](const char *what,
                            const RigExecWireComputed &edited,
                            const std::string &text) {
        const std::string got = _OpenError(with(edited));
        if (got != text) {
            std::printf("%s, %s: open said '%s', expected '%s'\n", name,
                        what, got.c_str(), text.c_str());
            CHECK(false);
        }
    };
    using Family = RigExecWireRegisteredFamily;
    const std::vector<RigExecWireRegisteredRead> &reads =
        computed.registeredReads;
    const auto isAvar = [](const RigExecWireRegisteredRead &entry) {
        return entry.family == Family::AvarBinding ||
               entry.family == Family::AvarConstantBinding;
    };
    // An avar binding with a uid (Dial's keyed tx), an avar binding with
    // none and an override number, another avar's, and a ladder field.
    size_t recorded = reads.size(), constant = reads.size();
    size_t other = reads.size(), ladder = reads.size();
    for (size_t k = 0; k < reads.size(); ++k) {
        if (isAvar(reads[k]) && reads[k].uid >= 0 &&
            recorded == reads.size()) {
            recorded = k;
        }
        if (isAvar(reads[k]) && reads[k].uid < 0 &&
            reads[k].read.overrideIndex >= 0 && !reads[k].read.walk.empty() &&
            constant == reads.size()) {
            constant = k;
        }
        if (reads[k].family == Family::Ladder && ladder == reads.size()) {
            ladder = k;
        }
    }
    for (size_t k = 0; k < reads.size() && constant < reads.size(); ++k) {
        if (isAvar(reads[k]) && k < constant && reads[k].avar >= 0 &&
            reads[k].avar != reads[constant].avar &&
            !reads[k].read.walk.empty()) {
            other = k;
        }
    }
    CHECK(recorded < reads.size() && constant < reads.size() &&
          other < reads.size() && ladder < reads.size() &&
          !computed.chainReads.empty());
    if (recorded >= reads.size() || constant >= reads.size() ||
        other >= reads.size() || ladder >= reads.size() ||
        computed.chainReads.empty()) {
        return;
    }
    const std::string prefix = "the computed section's registered read ";
    {
        RigExecWireComputed edited = computed;
        edited.registeredReads[constant].read.walk[0] =
            reads[other].read.walk[0];
        expect("two avars at one head", edited,
               prefix + std::to_string(constant) + " heads two avars");
    }
    {
        RigExecWireComputed edited = computed;
        edited.registeredReads.push_back(reads[ladder]);
        expect("a field bound twice", edited,
               prefix + std::to_string(reads.size()) +
                   " binds a field another read binds");
    }
    {
        RigExecWireComputed edited = computed;
        edited.registeredReads.push_back(reads[recorded]);
        expect("a uid given twice", edited,
               prefix + std::to_string(reads.size()) + " repeats uid " +
                   std::to_string(reads[recorded].uid));
    }
    {
        RigExecWireComputed edited = computed;
        edited.registeredReads.erase(edited.registeredReads.begin() +
                                     std::ptrdiff_t(ladder));
        expect("a field left unbound", edited,
               "the computed section binds no read to a field of ladders[" +
                   std::to_string(reads[ladder].object) + "]");
    }
    {
        // The chain read's constant re-pointed at another value of its
        // tag: a read the steps never make.
        RigExecWireComputed edited = computed;
        v4::RigExecWireInput &read = edited.chainReads[0].read;
        size_t another = computed.values.size();
        for (size_t v = 0; v < computed.values.size(); ++v) {
            if (v != read.constant && computed.values[v].tag == read.tag) {
                another = v;
                break;
            }
        }
        CHECK(another < computed.values.size());
        read.constant = uint32_t(another);
        expect("a chain read of another constant", edited,
               "the computed section's chain read of uid " +
                   std::to_string(computed.chainReads[0].uid) +
                   " restates no registered read");
    }
    {
        // The constant binding's override number given another's: its own
        // attribute now reaches a number the program put elsewhere.
        RigExecWireComputed edited = computed;
        edited.registeredReads[constant].read.overrideIndex =
            reads[other].read.overrideIndex;
        std::string head;
        CHECK(binary->GetString(
            computed.inputs[reads[constant].read.walk[0]].name, &head));
        expect("an override number moved", edited,
               "the computed section's reads reach other override numbers "
               "at " + head + " than the program registered there");
    }
    {
        // A slot no overridable read walks, appended to one that does.
        std::vector<char> walked(computed.inputs.size(), 0);
        for (const RigExecWireRegisteredRead &entry : reads) {
            if (entry.read.overrideIndex >= 0) {
                for (const uint32_t slot : entry.read.walk) {
                    walked[slot] = 1;
                }
            }
        }
        const auto unwalked = std::find(walked.begin(), walked.end(), 0);
        CHECK(unwalked != walked.end());
        if (unwalked != walked.end()) {
            RigExecWireComputed edited = computed;
            edited.registeredReads[constant].read.walk.push_back(
                uint32_t(unwalked - walked.begin()));
            expect("a walk through an unregistered attribute", edited,
                   "the computed section's reads reach attributes the "
                   "program registered no override on");
        }
    }

    // An input refuses, changing nothing, a name the file does not list,
    // a value of another type, a non-finite value and an index past the
    // count.
    RigExecTestPlayer player;
    if (!player.Open(result.bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const std::string amount = "/Asset/Rig/Channels/Dial.rigExec:amount";
    size_t index = 0;
    CHECK(player->FindInput(amount, &index));
    const RrInputValue before = player->GetInputValue(index);
    const auto unchanged = [&] {
        const RrInputValue &now = player->GetInputValue(index);
        return now.tag == before.tag &&
               std::memcmp(&now.f32, &before.f32, sizeof(float)) == 0;
    };
    error.clear();
    CHECK(!player->SetInput("/Asset/Rig/Controls/Dial.avars:none", 1.0,
                            &error));
    CHECK(error == "no input named /Asset/Rig/Controls/Dial.avars:none");
    RrInputValue wrong = before;
    wrong.tag = RrInputTag::Bool;
    error.clear();
    CHECK(!player->SetInputAt(index, wrong, &error));
    CHECK(error == amount + " is a float input, not a bool");
    error.clear();
    CHECK(!player->SetInput(amount, std::nan(""), &error));
    CHECK(error == amount + " takes finite values only");
    error.clear();
    CHECK(!player->SetInputAt(player->GetInputCount(), before, &error));
    CHECK(error == "no input at index " +
                       std::to_string(player->GetInputCount()) +
                       "; the file lists " +
                       std::to_string(player->GetInputCount()));
    CHECK(unchanged());

    // What only an avar override refused is an authored value now: the
    // chain target's base, set to 0.3, plays what the baked program plays
    // with 0.3 authored on it in the session layer.
    CHECK(player->SetInput(amount, 0.3, &error));
    CHECK(player.Play(1.0, &error));
    const std::vector<_Reference> set = _References(
        stage, SdfPath("/Asset/Rig"), {_Edit(stage, amount, 0.3)}, {1.0});
    CHECK(set.size() == 1);
    if (set.size() == 1) {
        CHECK(_SamePools(name, 1.0, player.Reader(), set[0]));
        CHECK(_SamePropertyResults(name, set[0].pose, player.Reader()));
    }
    std::printf("%s: checked\n", name);
}

// tests/fixtures/<file>.
static std::string
_FixturePath(const char *file)
{
    return (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
            "fixtures" / file)
        .string();
}

// \p bytes' section \p tag, decoded by \p decode; false when absent or
// malformed.
template <class T, class Decode>
static bool
_DecodeSection(const std::vector<uint8_t> &bytes, RigExecBinarySection tag,
               T *out, Decode decode)
{
    std::string error;
    const std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    const uint8_t *data = nullptr;
    size_t size = 0;
    if (!reader || !reader->FindSection(tag, &data, &size)) {
        return false;
    }
    RigExecWireReader cursor(data, size);
    return decode(&cursor, out, &error) && cursor.Exhausted();
}

// The provider slot \p path names in \p bytes' slot inventory, or -1.
static int
_SlotOfPath(const std::vector<uint8_t> &bytes, const std::string &path,
            RigExecWireSlotMeta *meta)
{
    std::string error;
    const std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    if (!reader ||
        !_DecodeSection(bytes, RigExecBinarySection::SlotMeta, meta,
                        RigExecWireDecodeSlotMeta)) {
        return -1;
    }
    std::string text;
    for (size_t i = 0; i < meta->paths.size(); ++i) {
        if (reader->GetString(meta->paths[i], &text) && text == path) {
            return int(i);
        }
    }
    return -1;
}

static bool
_SameVersion(const RigExecWireFrameVersion &read, int anchor,
             const std::vector<int32_t> &recompose)
{
    return read.anchor == anchor && read.recompose == recompose;
}

// Space switches nested under switched controls, read at the versions the
// program bound at Build: each fixture's version pools, joints and
// diagnostics equal the baked program's bit for bit at every frame, played
// through the inputs of a bake at the first frame. In the nested fixture S
// reads P recomposed from its avars before P's switch, so its parent
// version is {-1, {P}}. In the carry fixture P hangs under G and S's space
// Q under P, so S reads {G, {P}} for its parent and {G, {P, Q}} for its
// space. The dial fixture's indices are keyed with no default: each is a
// live input the record directory lists, whose record holds its key at
// the bake time.
static void
TestSpaceSwitchVersionFixtures()
{
    const std::vector<double> frames = {0.0, 1.0, 2.0, 3.0};
    for (const char *file :
         {"space_switch_nested.usda", "space_switch_same_round.usda",
          "space_switch_dial.usda", "space_switch_carry.usda"}) {
        _TestFixture(file, _FixturePath(file), frames);
    }

    const auto bake = [&](const char *file, RigExecBakeResult *result) {
        const UsdStageRefPtr stage = UsdStage::Open(_FixturePath(file));
        CHECK(stage);
        if (!stage) {
            return false;
        }
        RigExecRigEvaluator evaluator(stage, SdfPath("/Rig"));
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        RigExecBakeOpts opts;
        opts.time = frames.front();
        std::string error;
        const bool ok = RigExecBakeToBinary(evaluator, opts, result, &error);
        CHECK(ok);
        return ok;
    };

    RigExecBakeResult nested;
    if (bake("space_switch_nested.usda", &nested)) {
        RigExecWireSlotMeta meta;
        const int p = _SlotOfPath(nested.bytes, "/Rig/Controls/P", &meta);
        const int s = _SlotOfPath(nested.bytes, "/Rig/Controls/P/S", &meta);
        const int c =
            _SlotOfPath(nested.bytes, "/Rig/Controls/P/S/C", &meta);
        std::vector<RigExecWireSpaceSwitch> switches;
        CHECK(p >= 0 && s >= 0 && c >= 0 &&
              _DecodeSection(
                  nested.bytes, RigExecBinarySection::SpaceSwitch,
                  &switches,
                  [&](RigExecWireReader *cursor,
                      std::vector<RigExecWireSpaceSwitch> *out,
                      std::string *error) {
                      return RigExecWireDecodeSpaceSwitches(
                          cursor, meta.parent, out, error);
                  }));
        CHECK(switches.size() == 2);
        for (const RigExecWireSpaceSwitch &sw : switches) {
            if (sw.slot == s) {
                // P resolves after S: S reads it before its switch.
                CHECK(_SameVersion(sw.parentRead, -1, {p}));
            } else {
                // C, under S, resolves before P: its last version.
                CHECK(sw.slot == p && sw.sourceReads.size() == 2 &&
                      _SameVersion(sw.sourceReads[0], c, {}));
            }
        }
    }

    RigExecBakeResult carry;
    if (bake("space_switch_carry.usda", &carry)) {
        RigExecWireSlotMeta meta;
        const auto slot = [&](const char *path) {
            return _SlotOfPath(carry.bytes, path, &meta);
        };
        const int other = slot("/Rig/Controls/Other");
        const int g = slot("/Rig/Controls/G");
        const int p = slot("/Rig/Controls/G/P");
        const int q = slot("/Rig/Controls/G/P/Q");
        const int s = slot("/Rig/Controls/G/P/S");
        const int c = slot("/Rig/Controls/G/P/S/C");
        std::vector<RigExecWireSpaceSwitch> switches;
        CHECK(other >= 0 && g >= 0 && p >= 0 && q >= 0 && s >= 0 &&
              c >= 0 &&
              _DecodeSection(
                  carry.bytes, RigExecBinarySection::SpaceSwitch, &switches,
                  [&](RigExecWireReader *cursor,
                      std::vector<RigExecWireSpaceSwitch> *out,
                      std::string *error) {
                      return RigExecWireDecodeSpaceSwitches(
                          cursor, meta.parent, out, error);
                  }));
        CHECK(switches.size() == 2);
        for (const RigExecWireSpaceSwitch &sw : switches) {
            CHECK(sw.sourceReads.size() == 2);
            if (sw.sourceReads.size() != 2) {
                continue;
            }
            if (sw.slot == s) {
                // P resolves after S: P recomposed on G, then Q on that.
                CHECK(_SameVersion(sw.parentRead, g, {p}));
                CHECK(sw.spaceSlot == q &&
                      _SameVersion(sw.spaceRead, g, {p, q}));
                CHECK(_SameVersion(sw.sourceReads[0], other, {}));
                CHECK(_SameVersion(sw.sourceReads[1], -1, {}));
            } else {
                CHECK(sw.slot == p && _SameVersion(sw.parentRead, g, {}) &&
                      _SameVersion(sw.sourceReads[0], c, {}) &&
                      _SameVersion(sw.spaceRead, -1, {}));
            }
        }
    }

    RigExecBakeResult dial;
    if (!bake("space_switch_dial.usda", &dial)) {
        return;
    }
    std::string error;
    const std::unique_ptr<RigExecBinaryReader> binary =
        RigExecBinaryReader::Open(dial.bytes.data(), dial.bytes.size(),
                                  &error);
    RigExecWireInputTable table;
    CHECK(binary && _DecodeInputTable(dial.bytes, &table));
    if (!binary || table.frames.size() != 1) {
        CHECK(false);
        return;
    }
    struct Dial {
        const char *path;
        std::array<double, 4> keys;
        int64_t uid;
    };
    std::vector<Dial> dials = {
        {"/Rig/Controls/P.spaces:active", {0.0, 1.0, 0.5, 0.0}, -1},
        {"/Rig/Controls/P/S.spaces:active", {0.5, 0.0, 1.0, 0.5}, -1}};
    std::string text;
    for (Dial &entry : dials) {
        for (size_t k = 0; k < table.directory.size(); ++k) {
            if (binary->GetString(table.directory[k].head, &text) &&
                text == entry.path) {
                CHECK(entry.uid < 0);
                CHECK(table.directory[k].tag ==
                      RigExecWireInput::Tag::Double);
                entry.uid = int64_t(k);
            }
        }
        // Live: listed in the directory, and recorded at its key at the
        // bake time.
        CHECK(entry.uid >= 0);
        const RigExecWireFrameInputs &record = table.frames[0];
        bool held = false;
        for (size_t k = 0; k < record.uids.size() && entry.uid >= 0; ++k) {
            if (record.uids[k] == uint32_t(entry.uid)) {
                held = true;
                CHECK(record.values[k].f64 == entry.keys[0]);
            }
        }
        CHECK(held);
    }
    std::printf("space switch version fixtures: checked\n");
}

// Switch reads set by hand. S, under P under G, switches between Arm
// (under G) and world; the program binds its parent read to P's last
// version and its source read to Arm's. The SpaceSwitch section is
// rewritten so S reads its parent as {anchor -1, recompose {P}} -- P
// composed from its avars against identity instead of G's frame -- and
// then also Arm as {-1, {Arm}}. No binding of this rig produces either.
// The runtime playing the rewritten section through its inputs and the
// program with the same versions on its switch, its compose steps run through
// RigExecBakedRunPoseStep, agree bit for bit on every version-pool frame.
// The parent enters a switched compose only through the round trip of
// `local`, so it moves S in the last digits; the source moves S outright.
// A third rewrite anchors the parent read on G, {G, {P}}: what a switch on
// P resolving in S's round or later binds. P has no switch here, so the
// recompose reproduces P's last version and S does not move at all.
static void
TestHandBuiltSwitchReads()
{
    const char *const name = "hand-built switch reads";
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    UsdGeomXform::Define(stage, SdfPath("/Asset"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *path, double degrees,
                             const GfVec3d &offset) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        GfMatrix4d rest(1.0);
        rest.SetRotate(GfRotation(GfVec3d(0, 0, 1), degrees));
        rest.SetTranslateOnly(offset);
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(rest);
        return prim;
    };
    const auto key = [](const UsdPrim &prim, const char *avar,
                        const std::array<double, 3> &values) {
        const UsdAttribute attribute = prim.CreateAttribute(
            TfToken(avar), SdfValueTypeNames->Double);
        for (size_t f = 0; f < values.size(); ++f) {
            attribute.Set(values[f], UsdTimeCode(double(f + 1)));
        }
    };
    const UsdPrim g =
        control("/Asset/Rig/Controls/G", 0.0, GfVec3d(0, 100, 0));
    const UsdPrim arm =
        control("/Asset/Rig/Controls/G/Arm", 0.0, GfVec3d(-20, 0.5, 0));
    const UsdPrim p =
        control("/Asset/Rig/Controls/G/P", 30.0, GfVec3d(10.5, 0.25, 0));
    const UsdPrim target =
        control("/Asset/Rig/Controls/G/P/S", 0.0, GfVec3d(5.5, 0, 0));
    control("/Asset/Rig/Controls/G/P/S/C", 0.0, GfVec3d(5, 0, 0));
    key(g, "avars:rz", {10.0, 25.0, -15.0});
    key(g, "avars:tx", {1.0, -2.0, 3.0});
    key(arm, "avars:tx", {0.0, 10.0, 20.0});
    key(arm, "avars:rz", {-5.0, 12.0, 40.0});
    key(p, "avars:rz", {5.0, -20.0, 30.0});
    key(target, "avars:rz", {7.0, -13.0, 21.0});
    key(target, "avars:ty", {0.3, -0.7, 1.1});
    const UsdPrim sw = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/sSpaces"), TfToken("RigExecSpaceSwitch"));
    sw.CreateRelationship(TfToken("rigExec:target"))
        .SetTargets({target.GetPath()});
    sw.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({arm.GetPath(), SdfPath("/Asset/Rig")});
    key(sw, "inputs:activeSpace", {0.0, 0.5, 0.25});

    const SdfPath rigPath("/Asset/Rig");
    const std::vector<double> frames = {1.0, 2.0, 3.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.time = frames.front();
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    RigExecWireSlotMeta meta;
    const int pSlot =
        _SlotOfPath(result.bytes, "/Asset/Rig/Controls/G/P", &meta);
    const int sSlot =
        _SlotOfPath(result.bytes, "/Asset/Rig/Controls/G/P/S", &meta);
    const int armSlot =
        _SlotOfPath(result.bytes, "/Asset/Rig/Controls/G/Arm", &meta);
    const int gSlot =
        _SlotOfPath(result.bytes, "/Asset/Rig/Controls/G", &meta);
    std::vector<RigExecWireSpaceSwitch> switches;
    CHECK(pSlot >= 0 && sSlot >= 0 && armSlot >= 0 && gSlot >= 0 &&
          _DecodeSection(result.bytes, RigExecBinarySection::SpaceSwitch,
                         &switches,
                         [&](RigExecWireReader *cursor,
                             std::vector<RigExecWireSpaceSwitch> *out,
                             std::string *why) {
                             return RigExecWireDecodeSpaceSwitches(
                                 cursor, meta.parent, out, why);
                         }));
    CHECK(switches.size() == 1);
    if (pSlot < 0 || sSlot < 0 || armSlot < 0 || gSlot < 0 ||
        switches.size() != 1 ||
        switches[0].slot != sSlot || switches[0].sourceReads.size() != 2) {
        std::printf("%s: FAILED (no switch on S)\n", name);
        CHECK(false);
        return;
    }
    // Nothing above S is switched: the program reads last versions.
    CHECK(_SameVersion(switches[0].parentRead, pSlot, {}));
    CHECK(_SameVersion(switches[0].sourceReads[0], armSlot, {}));

    enum class Rewrite { Parent, ParentAndSource, AnchoredParent };
    for (const Rewrite rewrite :
         {Rewrite::Parent, Rewrite::ParentAndSource,
          Rewrite::AnchoredParent}) {
        const bool rewriteSource = rewrite == Rewrite::ParentAndSource;
        const int anchor = rewrite == Rewrite::AnchoredParent ? gSlot : -1;
        const char *const what =
            rewrite == Rewrite::Parent            ? "parent read"
            : rewrite == Rewrite::ParentAndSource ? "parent and source read"
                                                  : "anchored parent read";
        std::vector<RigExecWireSpaceSwitch> handBuilt = switches;
        handBuilt[0].parentRead = RigExecWireFrameVersion{anchor, {pSlot}};
        if (rewriteSource) {
            handBuilt[0].sourceReads[0] =
                RigExecWireFrameVersion{-1, {armSlot}};
        }
        std::vector<uint8_t> payload;
        CHECK(RigExecWireEncodeSpaceSwitches(handBuilt, &payload));
        const std::vector<uint8_t> bytes = _ReplaceSection(
            result.bytes, RigExecBinarySection::SpaceSwitch, payload);
        size_t movedFrames = 0;
        double largest = 0.0;
        for (const double frame : frames) {
            // A fresh program and reader per frame: every compose step
            // runs.
            RigExecRigEvaluator program(stage, rigPath);
            program.SetEvaluationMode(RigExecEvaluationMode::Baked);
            const RigExecRigPose pose = program.Evaluate(UsdTimeCode(frame));
            const RigExecBakedProgram *baked = program.GetBakedProgram();
            CHECK(pose.valid && baked);
            if (!pose.valid || !baked) {
                return;
            }
            RigExecBakedProgramImpl &B =
                const_cast<RigExecBakedProgramImpl &>(baked->GetStepGraph());
            CHECK(B.paths.size() == meta.paths.size() &&
                  B.paths[size_t(pSlot)] == p.GetPath() &&
                  B.paths[size_t(armSlot)] == arm.GetPath() &&
                  B.spaceSwitches.size() == 1 &&
                  B.spaceSwitches[0].slot == sSlot);
            if (B.spaceSwitches.size() != 1 ||
                B.spaceSwitches[0].sourceReads.size() != 2) {
                return;
            }
            const std::vector<RigExecPointFrame> natural = B.fin;
            RigExecBakedProgramImpl::SpaceSwitch &bakedSwitch =
                B.spaceSwitches[0];
            bakedSwitch.parentRead.anchor = anchor;
            bakedSwitch.parentRead.recompose = {pSlot};
            if (rewriteSource) {
                bakedSwitch.sourceReads[0].anchor = -1;
                bakedSwitch.sourceReads[0].recompose = {armSlot};
            }
            for (RigExecBakedStep &step : B.steps) {
                if (step.kind == RigExecBakedStepKind::ComposeSubtree) {
                    RigExecBakedRunPoseStep(&B, &step, UsdTimeCode(frame));
                }
            }
            RigExecTestPlayer player;
            if (!player.Open(bytes, stage, &error) ||
                !player.Play(frame, &error)) {
                std::printf("%s, %s frame %.17g: %s\n", name, what, frame,
                            error.c_str());
                CHECK(false);
                return;
            }
            CHECK(_SamePools(what, frame, player.Reader(), B));
            const RrPointFrame &got = player->GetFinFrames()[size_t(sSlot)];
            const RigExecPointFrame &bound = natural[size_t(sSlot)];
            if (!_FramesEqual(bound, got)) {
                ++movedFrames;
            }
            for (size_t k = 0; k < 4; ++k) {
                for (size_t c = 0; c < 3; ++c) {
                    largest = std::max(largest,
                                       std::abs(bound.points[k][c] -
                                                got.points[k][c]));
                }
            }
        }
        std::printf("%s, %s: S differs from the bound read at %zu of %zu "
                    "frames, by up to %.17g\n",
                    name, what, movedFrames, frames.size(), largest);
        if (rewrite == Rewrite::ParentAndSource) {
            CHECK(movedFrames == frames.size() && largest > 1e-3);
        } else if (rewrite == Rewrite::Parent) {
            CHECK(movedFrames > 0 && largest < 1e-9);
        } else {
            CHECK(movedFrames == 0);
        }
    }
    std::printf("%s: checked\n", name);
}

// The nested fixture's switch indices set as inputs: pSpaces' and
// sSpaces' inputs:activeSpace (keyed) set to 0.25 and 0.75, held over
// every frame -- set again after the stage's keys at each frame, as a host
// holding a drag does -- then reset; then, at one frame, an input set on
// P's rz alone. S's switch reads P only recomposed from those avars, and
// its compose step is emitted before P's, so only the closure's dirtying
// of the steps that recompose P re-runs it. Every frame's version pools
// equal the baked program's under the same values authored in the session
// layer bit for bit, and the baked program agrees with the dynamic
// evaluator.
static void
TestSpaceSwitchIndexDrag()
{
    const char *const name = "space switch index drag";
    const UsdStageRefPtr stage =
        UsdStage::Open(_FixturePath("space_switch_nested.usda"));
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/Rig");
    const std::vector<double> frames = {0.0, 1.0, 2.0, 3.0};
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: open: %s\n", name, error.c_str());
        CHECK(false);
        return;
    }
    const auto run = [&](const char *what, double frame,
                         const _Reference &reference,
                         std::vector<std::vector<RrPointFrame>> *fin) {
        if (!player.Play(frame, &error)) {
            std::printf("%s, %s frame %.17g: %s\n", name, what, frame,
                        error.c_str());
            CHECK(false);
            return false;
        }
        fin->push_back(player->GetFinFrames());
        return _SamePools(what, frame, player.Reader(), reference);
    };
    const auto pass = [&](const char *what,
                          const std::vector<_Reference> &references,
                          std::vector<std::vector<RrPointFrame>> *fin) {
        bool same = references.size() == frames.size();
        for (size_t f = 0; f < frames.size() && f < references.size();
             ++f) {
            same = run(what, frames[f], references[f], fin) && same;
        }
        return same;
    };
    const std::string pIndex = "/Rig/Movers/pSpaces.inputs:activeSpace";
    const std::string sIndex = "/Rig/Movers/sSpaces.inputs:activeSpace";
    const std::vector<_Reference> unset =
        _References(stage, rigPath, {}, frames);
    std::vector<std::vector<RrPointFrame>> undragged, dragged, released;
    CHECK(pass("undragged", unset, &undragged));
    CHECK(player.Hold(pIndex, 0.25, &error));
    CHECK(player.Hold(sIndex, 0.75, &error));
    CHECK(pass("indices held",
               _References(stage, rigPath,
                           {_Edit(stage, pIndex, 0.25),
                            _Edit(stage, sIndex, 0.75)},
                           frames),
               &dragged));
    CHECK(dragged.size() == frames.size() &&
          undragged.size() == frames.size());
    // At frame 0 every control rests, where a switch holds its target in
    // place whatever its index; from frame 1 on the held indices move the
    // pools.
    for (size_t f = 1; f < dragged.size() && f < undragged.size(); ++f) {
        CHECK(dragged[f] != undragged[f]);
    }
    player.ReleaseAll();
    CHECK(pass("indices released", unset, &released));
    CHECK(released == undragged);

    // At one frame, only P's avars move.
    const size_t at = 2;
    const double frame = frames[at];
    const std::string pRz = "/Rig/Controls/P.avars:rz";
    std::vector<std::vector<RrPointFrame>> still, turned, back;
    CHECK(run("before the rz drag", frame, unset[at], &still));
    CHECK(run("before the rz drag, again", frame, unset[at], &still));
    CHECK(player.Hold(pRz, 40.0, &error));
    const std::vector<_Reference> rz =
        _References(stage, rigPath, {_Edit(stage, pRz, 40.0)}, {frame});
    CHECK(rz.size() == 1);
    if (rz.size() == 1) {
        CHECK(run("rz dragged", frame, rz[0], &turned));
    }
    player.ReleaseAll();
    CHECK(run("rz released", frame, unset[at], &back));
    CHECK(still.size() == 2 && turned.size() == 1 && back.size() == 1);
    if (still.size() == 2 && turned.size() == 1 && back.size() == 1) {
        CHECK(turned[0] != still[1]);
        CHECK(back[0] == still[1]);
    }
    std::printf("%s: checked\n", name);
}

// Open refuses a SpaceSwitch section that reads a slot the inventory does
// not hold, a recompose through a slot the compose has no avars for, and a
// version bound to a world source or a missing space; and refuses a cone
// lookup table naming a cluster that is not there. Each is cut into the
// carry fixture's bake, whose S switch reads {G, {P}} and {G, {P, Q}}.
static void
TestSpaceSwitchOpenRefusals()
{
    const char *const name = "space switch open refusals";
    const UsdStageRefPtr stage =
        UsdStage::Open(_FixturePath("space_switch_carry.usda"));
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.time = 0.0;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(_OpenError(result.bytes).empty());
    RigExecWireSlotMeta meta;
    const int g = _SlotOfPath(result.bytes, "/Rig/Controls/G", &meta);
    const int p = _SlotOfPath(result.bytes, "/Rig/Controls/G/P", &meta);
    const int s = _SlotOfPath(result.bytes, "/Rig/Controls/G/P/S", &meta);
    std::vector<RigExecWireSpaceSwitch> switches;
    RigExecWireCones cones;
    CHECK(g >= 0 && p >= 0 && s >= 0 &&
          _DecodeSection(result.bytes, RigExecBinarySection::SpaceSwitch,
                         &switches,
                         [&](RigExecWireReader *cursor,
                             std::vector<RigExecWireSpaceSwitch> *out,
                             std::string *why) {
                             return RigExecWireDecodeSpaceSwitches(
                                 cursor, meta.parent, out, why);
                         }) &&
          _DecodeSection(result.bytes, RigExecBinarySection::Cones, &cones,
                         RigExecWireDecodeCones));
    size_t at = switches.size();
    for (size_t k = 0; k < switches.size(); ++k) {
        if (switches[k].slot == s) {
            at = k;
        }
    }
    if (g < 0 || p < 0 || s < 0 || at == switches.size() ||
        switches[at].sourceReads.size() != 2 ||
        switches[at].sourceSlots[1] != -1 || switches[at].spaceSlot < 0 ||
        size_t(s) >= cones.avarCluster.size()) {
        std::printf("%s: FAILED (no switch on S)\n", name);
        CHECK(false);
        return;
    }
    const int32_t slots = int32_t(meta.paths.size());
    const auto expect = [&](const char *what,
                            const std::vector<uint8_t> &bytes,
                            const std::string &text) {
        const std::string got = _OpenError(bytes);
        if (got != text) {
            std::printf("%s, %s: open said '%s', expected '%s'\n", name,
                        what, got.c_str(), text.c_str());
            CHECK(false);
        }
    };
    const auto withSwitch =
        [&](const std::function<void(RigExecWireSpaceSwitch &)> &edit) {
            std::vector<RigExecWireSpaceSwitch> edited = switches;
            edit(edited[at]);
            std::vector<uint8_t> payload;
            CHECK(RigExecWireEncodeSpaceSwitches(edited, &payload));
            return _ReplaceSection(result.bytes,
                                   RigExecBinarySection::SpaceSwitch,
                                   payload);
        };
    const std::string noSlot = "space switch reads a version of no slot";
    const std::string unread =
        "space switch reads a version of a world source or a missing space";
    expect("parent anchor past the slots",
           withSwitch([&](RigExecWireSpaceSwitch &sw) {
               sw.parentRead.anchor = slots;
           }),
           noSlot);
    expect("source anchor past the slots",
           withSwitch([&](RigExecWireSpaceSwitch &sw) {
               sw.sourceReads[0].anchor = slots;
           }),
           noSlot);
    expect("space recompose past the slots",
           withSwitch([&](RigExecWireSpaceSwitch &sw) {
               sw.spaceRead.recompose.push_back(slots);
           }),
           noSlot);
    expect("space slot past the slots",
           withSwitch([&](RigExecWireSpaceSwitch &sw) {
               sw.spaceSlot = slots;
           }),
           "space switch names no space slot");
    expect("world source with a version",
           withSwitch([&](RigExecWireSpaceSwitch &sw) {
               sw.sourceReads[1] = RigExecWireFrameVersion{g, {}};
           }),
           unread);
    expect("world source with a recompose",
           withSwitch([&](RigExecWireSpaceSwitch &sw) {
               sw.sourceReads[1] = RigExecWireFrameVersion{-1, {p}};
           }),
           unread);
    expect("missing space with a version",
           withSwitch([&](RigExecWireSpaceSwitch &sw) {
               sw.spaceSlot = -1;
           }),
           unread);
    {
        // P, which both of S's versions recompose, without the avars and
        // ladder a recompose composes from.
        RigExecWireSlotMeta flipped = meta;
        flipped.slotKind[size_t(p)] = RigExecWireSlotKind::XformDerived;
        std::vector<uint8_t> payload;
        CHECK(RigExecWireEncodeSlotMeta(flipped, &payload));
        expect("recompose of a slot that is not FirstFramePose",
               _ReplaceSection(result.bytes, RigExecBinarySection::SlotMeta,
                               payload),
               noSlot);
    }
    const int32_t clusters = int32_t(cones.cone.size());
    for (const int32_t cluster : {int32_t(-1), clusters}) {
        RigExecWireCones edited = cones;
        edited.avarCluster[size_t(s)] = cluster;
        std::vector<uint8_t> payload;
        CHECK(RigExecWireEncodeCones(edited, &payload));
        expect(cluster < 0 ? "avar cluster -1" : "avar cluster past the end",
               _ReplaceSection(result.bytes, RigExecBinarySection::Cones,
                               payload),
               "a cone lookup table names no cluster");
    }
    std::printf("%s: checked\n", name);
}

// The carry fixture, at a frame where S blends Other (twist-filtered) with
// world and at one where S is at world. S's space reads Q recomposed
// through P and Q on G's frame, so an input set on G's, P's or Q's avars
// alone moves S outright. P's and Q's reach S's compose step only through
// the closure's dirtying of the steps that recompose them; G's through S's
// read of its posedM. Each set and its reset match the baked program's
// version pools under the same value authored in the session layer bit for
// bit, with parity against the dynamic walk.
static void
TestSpaceSwitchCarryDrag()
{
    const char *const name = "space switch carry drag";
    const UsdStageRefPtr stage =
        UsdStage::Open(_FixturePath("space_switch_carry.usda"));
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/Rig");
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, 0.0, &bytes, &error));
    RigExecWireSlotMeta meta;
    const int s = _SlotOfPath(bytes, "/Rig/Controls/G/P/S", &meta);
    RigExecTestPlayer player;
    const bool opened = player.Open(bytes, stage, &error);
    CHECK(opened && s >= 0);
    if (!opened || s < 0) {
        std::printf("%s: open: %s\n", name, error.c_str());
        return;
    }
    const auto run = [&](const std::string &what, double frame,
                         const _Reference &reference,
                         std::vector<std::vector<RrPointFrame>> *fin) {
        if (!player.Play(frame, &error)) {
            std::printf("%s, %s frame %.17g: %s\n", name, what.c_str(),
                        frame, error.c_str());
            CHECK(false);
            return false;
        }
        fin->push_back(player->GetFinFrames());
        return _SamePools(what.c_str(), frame, player.Reader(), reference);
    };
    struct Drag {
        const char *prim;
        const char *avar;
        double value;
    };
    const Drag drags[] = {{"/Rig/Controls/G/P", "avars:rz", 40.0},
                          {"/Rig/Controls/G/P/Q", "avars:tx", 6.0},
                          {"/Rig/Controls/G/P/Q", "avars:rz", -30.0},
                          {"/Rig/Controls/G", "avars:rz", 35.0}};
    for (const double frame : {0.0, 2.0}) {
        const std::vector<_Reference> unset =
            _References(stage, rigPath, {}, {frame});
        if (unset.size() != 1) {
            CHECK(false);
            continue;
        }
        for (const Drag &drag : drags) {
            const std::string what =
                std::string(drag.prim) + "." + drag.avar;
            const std::vector<_Reference> set = _References(
                stage, rigPath, {_Edit(stage, what, drag.value)}, {frame});
            std::vector<std::vector<RrPointFrame>> still, turned, back;
            CHECK(run(what + " before", frame, unset[0], &still));
            CHECK(run(what + " before, again", frame, unset[0], &still));
            CHECK(player.Hold(what, drag.value, &error));
            if (set.size() == 1) {
                CHECK(run(what + " dragged", frame, set[0], &turned));
            }
            player.ReleaseAll();
            CHECK(run(what + " released", frame, unset[0], &back));
            CHECK(still.size() == 2 && turned.size() == 1 &&
                  back.size() == 1);
            if (still.size() != 2 || turned.size() != 1 ||
                back.size() != 1 || size_t(s) >= still[1].size() ||
                size_t(s) >= turned[0].size()) {
                continue;
            }
            double moved = 0.0;
            for (size_t k = 0; k < 4; ++k) {
                for (size_t c = 0; c < 3; ++c) {
                    moved = std::max(
                        moved, std::abs(turned[0][size_t(s)].points[k][c] -
                                        still[1][size_t(s)].points[k][c]));
                }
            }
            std::printf("%s, %s at frame %g: S moves by %.17g\n", name,
                        what.c_str(), frame, moved);
            CHECK(moved > 1e-3);
            CHECK(back[0] == still[1]);
        }
    }
    std::printf("%s: checked\n", name);
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
    TestSampledInputWithNoValue();
    TestComputedChainsFixture();
    TestIkSpaceFixture();

    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }
    TestConstraintAndAvarReadersReplay(examplesDir);
    TestSetInputOnPhasedReader();
    TestSetInputOnHop();
    TestSetInputOnLadderInput();
    TestSetInputOnChainInput();
    TestSetInputOnChainTarget();
    TestRegisteredReadRefusals();
    TestSpaceSwitchVersionFixtures();
    TestHandBuiltSwitchReads();
    TestSpaceSwitchIndexDrag();
    TestSpaceSwitchOpenRefusals();
    TestSpaceSwitchCarryDrag();
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
        // A `static` row holds an animated source in static data at the
        // bake time: it plays that time alone.
        _TestFixture(fixture.stage, stagePath, frames,
                     std::string(fixture.animation) == "static");
    }
    CHECK(sawBaking);

    if (failures == 0) {
        std::printf("testRigExecRuntimePose: all tests passed "
                    "(%d stage(s) compared over %d frame(s))\n",
                    comparedFixtures, comparedFrames);
        return 0;
    }
    std::printf("testRigExecRuntimePose: %d failures "
                "(%d stage(s) compared over %d frame(s))\n", failures,
                comparedFixtures, comparedFrames);
    return 1;
}
