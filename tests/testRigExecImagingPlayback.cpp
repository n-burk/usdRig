//
// testRigExecImagingPlayback (M2b): a rig played from its .rigexec file
// publishes the same geometry the live bridge does.
//
// For every baking fixture: bake in process, publish each frame through a
// live RigExecImagingBridge and through a RigExecBakedPlayback, and compare
// the two generations leaf by leaf -- points, normals, extents and driven
// transforms, bitwise. Guide payloads and movedFloats are playback's two
// documented v1 gaps (see playback.h) and are not compared: the assertion
// is that every geometry leaf the live path owns, playback owns with the
// same value, and that playback owns no geometry leaf the live path lacks.
//
// Also covered: nearest-frame time mapping (ties to the lower frame),
// Open() refusing an unreadable file, and registry selection -- a stage
// whose rig names rigExec:asset activates into a guideless playback
// generation, while the same stage without the attribute draws guides.
//

#include "rigExecBake/bake.h"
#include "rigExecImaging/bridge.h"
#include "rigExecImaging/playback.h"
#include "rigExecImaging/registry.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++failures;                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                               \
    } while (0)

static SdfPath
_FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath::EmptyPath();
}

static std::vector<double>
_ParseFrames(const std::string &text)
{
    std::vector<double> frames;
    size_t begin = 0;
    while (begin <= text.size()) {
        const size_t end = text.find(",", begin);
        const std::string piece = text.substr(
            begin, end == std::string::npos ? std::string::npos : end - begin);
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
_SamePoints(const VtVec3fArray &a, const VtVec3fArray &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

static bool
_SameMatrices(const GfMatrix4d &a, const GfMatrix4d &b)
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (a[r][c] != b[r][c]) {
                return false;
            }
        }
    }
    return true;
}

// Every geometry leaf the live generation owns, playback owns with the same
// value -- and playback owns no geometry leaf the live one lacks. Guide-only
// prims (joints drawn as skeletons, controls as shapes) exist only on the
// live side and are skipped, not failed: guides are playback's documented
// v1 gap.
static void
_CompareGenerations(const RigExecImagingSnapshot &live,
                    const RigExecImagingSnapshot &play,
                    const std::string &what)
{
    for (const auto &[path, expected] : live.prims) {
        const bool liveGeometry = expected.hasPoints ||
                                  expected.hasNormals ||
                                  expected.hasExtent || expected.hasXform;
        if (!liveGeometry) {
            continue;
        }
        const auto found = play.prims.find(path);
        if (found == play.prims.end()) {
            std::printf("playback lacks live geometry prim %s [%s]\n",
                        path.GetString().c_str(), what.c_str());
            CHECK(found != play.prims.end());
            continue;
        }
        const RigExecPublishedPrim &got = found->second;
        if (expected.hasPoints != got.hasPoints ||
            (expected.hasPoints &&
             !_SamePoints(expected.points, got.points))) {
            std::printf("points differ on %s [%s]\n",
                        path.GetString().c_str(), what.c_str());
            CHECK(false);
        }
        if (expected.hasNormals != got.hasNormals ||
            (expected.hasNormals &&
             !_SamePoints(expected.normals, got.normals))) {
            std::printf("normals differ on %s [%s]\n",
                        path.GetString().c_str(), what.c_str());
            CHECK(false);
        }
        if (expected.hasExtent != got.hasExtent ||
            (expected.hasExtent &&
             (expected.extentMin != got.extentMin ||
              expected.extentMax != got.extentMax))) {
            std::printf("extent differs on %s [%s]\n",
                        path.GetString().c_str(), what.c_str());
            CHECK(false);
        }
        if (expected.hasXform != got.hasXform ||
            (expected.hasXform &&
             (!_SameMatrices(expected.xform, got.xform) ||
              !_SameMatrices(expected.xformBase, got.xformBase)))) {
            std::printf("xform differs on %s [%s]\n",
                        path.GetString().c_str(), what.c_str());
            CHECK(false);
        }
        if (expected.hasWeightOverlay != got.hasWeightOverlay ||
            (expected.hasWeightOverlay &&
             expected.weightOverlay != got.weightOverlay)) {
            std::printf("overlay differs on %s [%s]\n",
                        path.GetString().c_str(), what.c_str());
            CHECK(false);
        }
    }
    for (const auto &[path, got] : play.prims) {
        const bool playGeometry = got.hasPoints || got.hasNormals ||
                                  got.hasExtent || got.hasXform;
        if (!playGeometry) {
            continue;
        }
        const auto found = live.prims.find(path);
        if (found == live.prims.end() ||
            (!found->second.hasPoints && !found->second.hasNormals &&
             !found->second.hasExtent && !found->second.hasXform)) {
            std::printf("playback owns geometry %s the live path lacks "
                        "[%s]\n",
                        path.GetString().c_str(), what.c_str());
            CHECK(false);
        }
    }
}

static void
_TestFixture(const std::string &stagePath, const std::string &frameText,
             const std::string &operatorPrim,
             const std::filesystem::path &scratch)
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
    const std::vector<double> frames = _ParseFrames(frameText);
    CHECK(!frames.empty());

    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = frames;
    RigExecBakeResult baked;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &baked, &error));
    if (baked.bytes.empty()) {
        std::printf("bake diagnostic: %s\n", error.c_str());
        return;
    }
    std::string flat = stagePath;
    for (char &c : flat) {
        if (c == '/' || c == '\\' || c == ':') {
            c = '_';
        }
    }
    const std::string binary =
        (scratch / (flat + ".rigexec")).string();
    {
        std::ofstream stream(binary, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(baked.bytes.data()),
                     std::streamsize(baked.bytes.size()));
    }

    auto liveStore = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge live(stage, rigPath, liveStore);
    std::vector<std::string> compileErrors;
    CHECK(live.Compile(&compileErrors));
    auto playStore = std::make_shared<RigExecSnapshotStore>();
    RigExecBakedPlayback play(stage, rigPath, playStore);
    CHECK(play.Open(binary, &error));
    if (!error.empty()) {
        std::printf("open diagnostic: %s\n", error.c_str());
    }
    // The table's operator prim, when it names a weight object, paints the
    // overlay on both legs; when it names anything else both legs paint
    // nothing, which the comparison also holds them to.
    if (!operatorPrim.empty()) {
        live.SetWeightOverlay(SdfPath(operatorPrim));
        play.SetWeightOverlay(SdfPath(operatorPrim));
    }
    for (double frame : frames) {
        const UsdTimeCode time(frame);
        const RigExecImagingBridge::PublishResult liveResult =
            live.EvaluateAndPublishResult(time);
        CHECK(liveResult.ok);
        const RigExecImagingBridge::PublishResult playResult =
            play.EvaluateAndPublishResult(time);
        if (!playResult.ok) {
            std::printf("playback failed at frame %g of %s\n", frame,
                        stagePath.c_str());
        }
        CHECK(playResult.ok);
        RigExecImagingSnapshotConstPtr a = liveStore->Get();
        RigExecImagingSnapshotConstPtr b = playStore->Get();
        CHECK(a && b);
        if (!a || !b) {
            continue;
        }
        CHECK(a->Describes(UsdStageWeakPtr(stage), time));
        CHECK(b->Describes(UsdStageWeakPtr(stage), time));
        _CompareGenerations(*a, *b,
                            stagePath + " frame " + std::to_string(frame));
    }

    // Nearest-frame mapping on this binary's own frame list.
    if (frames.size() >= 2) {
        const double first = frames.front();
        const double second = frames[1];
        CHECK(play.MapTimeToFrame(UsdTimeCode(first)) == first);
        CHECK(play.MapTimeToFrame(UsdTimeCode(second)) == second);
        const double mid = 0.5 * (first + second);
        CHECK(play.MapTimeToFrame(UsdTimeCode(mid)) == first);
        CHECK(play.MapTimeToFrame(UsdTimeCode(first - 100000.0)) == first);
        CHECK(play.MapTimeToFrame(UsdTimeCode(frames.back() + 100000.0)) ==
              frames.back());
    }
}

// Registry selection: the same rig with rigExec:asset authored plays its
// binary (geometry, no guides), and without the attribute evaluates live
// (guides drawn). One fixture carries the assertion; the sweep above
// carries every fixture's values.
static void
_TestSelection(const std::string &stagePath, const std::string &frameText,
               const std::filesystem::path &scratch)
{
    const std::vector<double> frames = _ParseFrames(frameText);
    CHECK(!frames.empty());
    if (frames.empty()) {
        return;
    }
    const auto hasGuides = [](const RigExecImagingSnapshot &snapshot) {
        for (const auto &[path, prim] : snapshot.prims) {
            if (prim.hasGuides || prim.hasControlGuide ||
                prim.hasVolumeGuides) {
                return true;
            }
        }
        return false;
    };
    const auto hasGeometry = [](const RigExecImagingSnapshot &snapshot) {
        for (const auto &[path, prim] : snapshot.prims) {
            if (prim.hasPoints || prim.hasNormals || prim.hasExtent ||
                prim.hasXform) {
                return true;
            }
        }
        return false;
    };

    // Live first, on a stage without the attribute.
    {
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage);
        if (!stage) {
            return;
        }
        RigExecImagingRegistry &registry =
            RigExecImagingRegistry::GetInstance();
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, _FindRig(stage),
                                UsdTimeCode(frames.front()), &errors));
        RigExecImagingSnapshotConstPtr current =
            registry.GetStore()->Get();
        CHECK(current && hasGeometry(*current));
        if (!current || !hasGuides(*current)) {
            std::printf("live leg of %s drew no guides; the selection "
                        "test cannot distinguish\n",
                        stagePath.c_str());
        }
        CHECK(current && hasGuides(*current));
        registry.Deactivate();
    }

    // Then playback, with the attribute authored in the session layer.
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
    opts.frames = frames;
    RigExecBakeResult baked;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &baked, &error));
    if (baked.bytes.empty()) {
        return;
    }
    const std::string binary =
        (scratch / "selection.rigexec").string();
    {
        std::ofstream stream(binary, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(baked.bytes.data()),
                     std::streamsize(baked.bytes.size()));
    }
    UsdPrim rig = stage->GetPrimAtPath(rigPath);
    const UsdAttribute asset = rig.CreateAttribute(
        TfToken("rigExec:asset"), SdfValueTypeNames->Asset);
    CHECK(asset);
    asset.Set(SdfAssetPath(binary));
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rigPath, UsdTimeCode(frames.front()),
                            &errors));
    RigExecImagingSnapshotConstPtr current = registry.GetStore()->Get();
    CHECK(current && hasGeometry(*current));
    if (current && hasGuides(*current)) {
        std::printf("playback leg of %s drew guides; the asset was not "
                    "selected\n",
                    stagePath.c_str());
    }
    CHECK(!current || !hasGuides(*current));
    registry.Deactivate();
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    if (argc < 2 || !std::filesystem::is_directory(argv[1])) {
        std::printf("usage: testRigExecImagingPlayback <examples-dir>\n");
        return 2;
    }
    const auto scratch =
        std::filesystem::temp_directory_path() /
        ("rigexec-playback-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()));
    if (!std::filesystem::create_directory(scratch)) {
        return 1;
    }
    // Open() refuses what it cannot read.
    {
        auto store = std::make_shared<RigExecSnapshotStore>();
        RigExecBakedPlayback play(UsdStageRefPtr(), SdfPath::EmptyPath(),
                                  store);
        std::string error;
        CHECK(!play.Open(
            (scratch / "missing.rigexec").string(), &error));
        CHECK(!error.empty());
    }
    bool selectionDone = false;
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            continue;
        }
        const std::string stage =
            (std::filesystem::path(argv[1]) / fixture.stage).string();
        std::printf("playback conformance: %s\n", stage.c_str());
        _TestFixture(stage, fixture.frames, fixture.operatorPrim, scratch);
        if (!selectionDone) {
            _TestSelection(stage, fixture.frames, scratch);
            selectionDone = true;
        }
    }
    if (failures == 0) {
        std::printf("testRigExecImagingPlayback: all tests passed\n");
    } else {
        std::printf("testRigExecImagingPlayback: %d failures\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
