//
// .rigexec container + bake conformance.
//

#include "rigExecBake/bake.h"
#include "rigExecBake/capture.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/container.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

// After the macro: the comparison helpers report through CHECK.
#include "rigExecBinaryCompare.h"

static std::vector<uint8_t>
Bytes(const std::string &path)
{
    std::ifstream stream(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(stream),
                                std::istreambuf_iterator<char>());
}

static void
TestContainerRoundTrip()
{
    RigExecBinaryWriter writer;
    CHECK(writer.AddString("") == 0);
    const uint32_t a = writer.AddString("/World/Rig/Joint");
    const uint32_t b = writer.AddString("/World/Rig/Joint");
    CHECK(a == b);
    CHECK(a != 0);
    const uint32_t c = writer.AddString("/World/Rig/Other");
    CHECK(c != a);
    const std::vector<uint8_t> manifest = {'{', '}'};
    writer.AddSection(RigExecBinarySection::Manifest, manifest);
    // An unknown tag is a well-formed file: the reader skips what it does
    // not know, which is the whole minor-version story.
    const std::vector<uint8_t> future = {1, 2, 3, 4};
    writer.AddSection(RigExecBinarySection(0x100), future);
    const std::vector<uint8_t> bytes = writer.Finish();

    std::string error;
    std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("open diagnostic: %s\n", error.c_str());
        return;
    }
    CHECK(reader->GetVersion() == RigExecBinaryVersion);
    const uint8_t *data = nullptr;
    size_t size = 0;
    CHECK(reader->FindSection(RigExecBinarySection::Manifest, &data, &size));
    CHECK(size == manifest.size() && data[0] == '{' && data[1] == '}');
    CHECK(reader->FindSection(RigExecBinarySection(0x100), &data, &size));
    CHECK(size == future.size());
    CHECK(!reader->FindSection(RigExecBinarySection::Steps, &data, &size));
    std::string text;
    CHECK(reader->GetString(0, &text) && text.empty());
    CHECK(reader->GetString(a, &text) && text == "/World/Rig/Joint");
    CHECK(reader->GetString(c, &text) && text == "/World/Rig/Other");
    CHECK(!reader->GetString(c + 1, &text));
}

static void
TestContainerRejections()
{
    RigExecBinaryWriter writer;
    writer.AddString("/World/Rig");
    writer.AddSection(RigExecBinarySection::Manifest,
                      std::vector<uint8_t>{'{', '}'});
    const std::vector<uint8_t> good = writer.Finish();
    std::string error;

    // Every corruption below is rejected with a reason; none of them may
    // open, read out of bounds, or throw.
    std::vector<uint8_t> bad = good;
    bad[0] = 'X';
    CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));
    CHECK(error.find("magic") != std::string::npos);

    CHECK(!RigExecBinaryReader::Open(good.data(), 3, &error));

    bad = good;
    bad[4] = 2;  // major version 2
    CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));
    CHECK(error.find("version") != std::string::npos);

    // A section count the buffer cannot hold.
    bad = good;
    bad[8] = 64;
    CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));

    // A section running past the end (first entry's offset -> huge).
    bad = good;
    bad[16 + 4] = 0xff;
    bad[16 + 5] = 0xff;
    CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));

    // An unterminated final string.
    bad = good;
    // The string table is the FIRST section, so its last byte is read out
    // of the section table, not off the end of the file (which is the
    // manifest payload).
    {
        const size_t tableOffset =
            size_t(bad[20]) | (size_t(bad[21]) << 8) |
            (size_t(bad[22]) << 16) | (size_t(bad[23]) << 24) |
            (size_t(bad[24]) << 32) | (size_t(bad[25]) << 40) |
            (size_t(bad[26]) << 48) | (size_t(bad[27]) << 56);
        const size_t tableSize =
            size_t(bad[28]) | (size_t(bad[29]) << 8) |
            (size_t(bad[30]) << 16) | (size_t(bad[31]) << 24) |
            (size_t(bad[32]) << 32) | (size_t(bad[33]) << 40) |
            (size_t(bad[34]) << 48) | (size_t(bad[35]) << 56);
        bad[tableOffset + tableSize - 1] = 'x';
    }
    CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));
}

static SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

/// Whether a fixture's capture is expected to vary across the baked
/// frames. Probed from pose outputs (body hash, frame header excluded):
/// the animated stages move over their table frames (several peak
/// mid-range with static endpoints, so endpoints alone mislabel
/// them); the static ones never move. Unknown stages skip the guard.
enum class _BinaryVariance {
    Unknown,
    Static,
    Animated,
};

static _BinaryVariance
_BinaryExpectedVariance(const std::string &fixture)
{
    const std::string name =
        std::filesystem::path(fixture).filename().string();
    static const char *animated[] = {
        "01_FkChainTail.usda", "02_TwoBoneIkLeg.usda",
        "03_IkFkBlendClamp.usda", "04_BlendShapeFace.usda",
        "05_TwistRibbonSpine.usda", "06_LatticeBulge.usda",
        "07_SurfaceDrape.usda", "08_AimEyes.usda",
        "09_PropertyMathMovers.usda", "10_AimXformTurret.usda",
        "11_VolumeWeights.usda", "12_CurvenetProfile.usda",
        "13_ReadPhases.usda", "14_VolumeConstrainedSweep.usda",
        "aimtest.usda", "aimtest_points.usda",
        "rotateConstraint.usda", "rigexec_flat.usda",
        "par_rot_aim.usd", "par_rot_aim_redorder.usd",
        "rot_par_combo.usd", "aim_par_combo_flattened.usd",
        "ArmShotAnim.usda", "simple_rig_anim.usd", "Biped_anim.usda",
    };
    static const char *statics[] = {
        "ArmRig.usda", "spider_leg.usd", "spider_leg_ik.usd",
        "simple_rig.usd", "spider_legs_assembly_ref.usda",
        "Biped.usda", "Biped_layered.usda",
    };
    for (const char *known : animated) {
        if (name == known) {
            return _BinaryVariance::Animated;
        }
    }
    for (const char *known : statics) {
        if (name == known) {
            return _BinaryVariance::Static;
        }
    }
    return _BinaryVariance::Unknown;
}

static std::vector<double>
_ParseTableFrames(const std::string &text)
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

// The weight-gather capture hole (M2): the volume gather target and
// driver-curve arrays must reach the frame record. Fixture 11 TipCurve
// is an UNMOVED driver no chain base covers, so without the recorder
// hook in the gather it is unrepresentable and the runtime builds the
// wrong field. Filename-keyed like _BinaryExpectedVariance.
static void
_BinaryCheckWeightGatherReads(const std::string &fixture,
                              const std::set<std::string> &seenPathReads)
{
    const std::string base =
        std::filesystem::path(fixture).filename().string();
    std::vector<std::string> want;
    if (base == "11_VolumeWeights.usda") {
        want = {"/VolumeAsset/Drivers/TipCurve.points",
                "/VolumeAsset/Geom/Strip.points"};
    } else if (base == "14_VolumeConstrainedSweep.usda") {
        want = {"/SweepAsset/Geom/Strip.points"};
    }
    for (const std::string &path : want) {
        if (!seenPathReads.count(path)) {
            std::printf("weight-gather read missing from %s: %s\n",
                        base.c_str(), path.c_str());
        }
        CHECK(seenPathReads.count(path));
    }
}
static void
TestBake(const std::string &fixture, const std::vector<double> &bakeFrames,
         const std::filesystem::path &scratch)
{
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    // Two fresh evaluators, one bake each: a bake is deterministic, so the
    // bytes are identical -- which is what makes a byte golden meaningful
    // for every slice that follows.
    std::vector<uint8_t> first;
    for (int pass = 0; pass < 2; ++pass) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        RigExecBakeOpts opts;
        opts.frames = bakeFrames;
        RigExecBakeResult result;
        std::string error;
        CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
        if (error.empty()) {
            // keep the diagnostic visible when the CHECK above fails
        } else {
            std::printf("bake diagnostic: %s\n", error.c_str());
        }
        if (result.bytes.empty()) {
            return;
        }
        std::unique_ptr<RigExecBinaryReader> reader =
            RigExecBinaryReader::Open(result.bytes.data(),
                                      result.bytes.size(), &error);
        CHECK(reader);
        if (!reader) {
            return;
        }
        const uint8_t *data = nullptr;
        size_t size = 0;
        CHECK(reader->FindSection(RigExecBinarySection::Manifest, &data,
                                 &size));
        CHECK(size == result.manifestJson.size());
        CHECK(result.manifestJson.find(rigPath.GetString()) !=
              std::string::npos);
        if (pass == 0) {
            _BinaryCompareProgram(evaluator, result.bytes);
        }
        if (pass == 0) {
            first = result.bytes;
        } else {
            CHECK(result.bytes == first);
        }
    }
    // Capture fidelity: drive the capture API frame by frame, encoding
    // and decoding the table per frame and comparing against the live
    // program before the next Evaluate moves it on.
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::vector<std::string> compileErrors;
        CHECK(evaluator.Compile(&compileErrors));
        RigExecBinaryWriter writer;
        std::string captureError;
        RigExecBakeCapture capture(evaluator, &writer, &captureError);
        CHECK(capture.Valid());
        const RigExecBakedProgramImpl &program =
            evaluator.GetBakedProgram()->GetStepGraph();
        const std::vector<_BinaryOracleInput> oracle =
            _BinaryCollectOracle(program);
        const std::vector<double> &frames = bakeFrames;
        bool recordsVary = false;
        std::string varyDiff;
        bool haveFirst = false;
        RigExecWireFrameInputs firstRecord;
        std::set<std::string> seenPathReads;
        for (double frame : frames) {
            RigExecRigPose pose;
            if (!capture.CaptureFrame(frame, &pose, &captureError)) {
                CHECK(captureError.empty());
                std::printf("capture diagnostic: %s\n",
                            captureError.c_str());
                return;
            }
            // The string table so far, as bytes: the comparisons below
            // resolve references the way every other section does.
            const std::vector<uint8_t> bytes = writer.Finish();
            std::string openError;
            std::unique_ptr<RigExecBinaryReader> reader =
                RigExecBinaryReader::Open(bytes.data(), bytes.size(),
                                          &openError);
            CHECK(reader);
            if (!reader) {
                return;
            }
            std::vector<uint8_t> payload;
            CHECK(RigExecWireEncodeInputTable(capture.GetTable(),
                                              &payload));
            RigExecWireReader cursor(payload.data(), payload.size());
            RigExecWireInputTable decoded;
            std::string decodeError;
            CHECK(RigExecWireDecodeInputTable(&cursor, &decoded,
                                              &decodeError));
            _BinaryCompareTableStatic(program, decoded, oracle, *reader);
            CHECK(decoded.frames.size() ==
                  capture.GetTable().frames.size());
            _BinaryCompareTableFrame(program, decoded.frames.back(),
                                     oracle, decoded, *reader);
            for (const RigExecWirePathRead &read :
                 decoded.frames.back().pathReads) {
                std::string path;
                if (reader->GetString(read.path, &path)) {
                    seenPathReads.insert(path);
                }
            }
            if (!haveFirst) {
                firstRecord = decoded.frames.back();
                haveFirst = true;
            } else if (!_BinaryFrameInputsEqual(firstRecord,
                                               decoded.frames.back())) {
                if (!recordsVary) {
                    varyDiff = _BinaryFrameInputsDiff(firstRecord,
                                                      decoded.frames.back());
                }
                recordsVary = true;
            }
        }
        // The varies-guard: animated fixtures must capture different
        // records across frames (or the varying-input sampling is
        // vacuous), and static fixtures must capture identical ones
        // (or the capture is nondeterministic).
        const _BinaryVariance expected =
            _BinaryExpectedVariance(fixture);
        if (expected == _BinaryVariance::Animated && !recordsVary) {
            std::printf(
                "capture is static on animated fixture %s\n",
                fixture.c_str());
        }
        CHECK(expected != _BinaryVariance::Animated || recordsVary);
        if (expected == _BinaryVariance::Static && recordsVary) {
            std::printf(
                "capture varies on static fixture %s: %s\n",
                fixture.c_str(), varyDiff.c_str());
        }
        CHECK(expected != _BinaryVariance::Static || !recordsVary);
        _BinaryCheckWeightGatherReads(fixture, seenPathReads);
    }
    // The bytes survive a trip through a file: the CLI writes exactly this
    // vector, so the vector is the format, not an in-memory sketch of it.
    std::string flat = fixture;
    for (char &c : flat) {
        if (c == '/' || c == '\\' || c == ':') {
            c = '_';
        }
    }
    const std::string file = (scratch / (flat + ".rigexec")).string();
    {
        std::ofstream stream(file, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(first.data()),
                     std::streamsize(first.size()));
    }
    CHECK(Bytes(file) == first);

    // No frames is an error, not an empty binary.
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    RigExecBakeResult result;
    std::string error;
    CHECK(!RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(error.find("frames") != std::string::npos);
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const auto scratch =
        std::filesystem::temp_directory_path() /
        ("rigexec-binary-conformance-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()));
    if (!std::filesystem::create_directory(scratch)) {
        return 1;
    }
    TestContainerRoundTrip();
    TestContainerRejections();
    auto BakeOne = [&](const std::string &stage,
                       const std::vector<double> &frames) {
        std::printf("bake conformance: %s\n", stage.c_str());
        TestBake(stage, frames, scratch);
    };
    if (argc > 1 && std::filesystem::is_directory(argv[1])) {
        for (const RigExecExampleFixture &fixture :
             kRigExecExampleFixtures) {
            if (!fixture.bakesToday) {
                std::printf("bake conformance: skip %s (blocked by %s)\n",
                            fixture.stage, fixture.blockedBy);
                continue;
            }
            const std::string stage =
                (std::filesystem::path(argv[1]) / fixture.stage).string();
            BakeOne(stage, _ParseTableFrames(fixture.frames));
        }
    } else if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            BakeOne(argv[i], {1001, 1024, 1048});
        }
    } else {
        for (const RigExecExampleFixture &fixture :
             kRigExecExampleFixtures) {
            if (!fixture.bakesToday) {
                continue;
            }
            const std::string stage =
                (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) /
                 fixture.stage).string();
            BakeOne(stage, _ParseTableFrames(fixture.frames));
        }
    }
    if (!failures) {
        std::filesystem::remove_all(scratch);
    }
    if (failures) {
        std::printf("testRigExecBinary: %d failures; fixture %s\n", failures,
                    scratch.string().c_str());
        return 1;
    }
    std::puts("testRigExecBinary: all tests passed");
    return 0;
}
