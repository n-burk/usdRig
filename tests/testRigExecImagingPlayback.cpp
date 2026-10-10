// testRigExecImagingPlayback (M2b): a rig played from its .rigexec file
// publishes the same geometry the live bridge does.
// For every baking fixture: bake in process at the table's first frame,
// publish through a live RigExecImagingBridge and through a
// RigExecBakedPlayback (which samples the stage's Animated inputs at each
// time), and compare the two generations leaf by leaf -- points, normals,
// extents and driven transforms, bitwise. An `inputs` row is compared at
// every table frame and at the midpoint of each consecutive pair, times the
// file never held; a `static` row, whose binary holds an animated source as
// static data, at the bake time alone. Guide payloads and movedFloats are
// playback's two documented v1 gaps (see playback.h) and are not compared:
// the assertion is that every geometry leaf the live path owns, playback
// owns with the same value, and that playback owns no geometry leaf the
// live path lacks. Also covered: Open() refusing an unreadable file;
// registry selection -- a stage whose rig names rigExec:asset activates
// into a guideless playback generation, while the same stage without the
// attribute draws guides; and a registry playback that reads its static
// inputs once, then again after each stage notice, so a Default edit or
// keys authored after it opened play as a fresh playback plays them.
#include "rigExecBake/bake.h"
#include "rigExecBinary/format.h"
#include "rigExecImaging/bridge.h"
#include "rigExecImaging/playback.h"
#include "rigExecImaging/registry.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
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

// Whether two generations own the same geometry leaves with the same
// values, as _CompareGenerations holds them, without reporting.
static bool
_SameGenerations(const RigExecImagingSnapshot &a,
                 const RigExecImagingSnapshot &b)
{
    const auto geometry = [](const RigExecPublishedPrim &prim) {
        return prim.hasPoints || prim.hasNormals || prim.hasExtent ||
               prim.hasXform;
    };
    for (const auto &[path, x] : a.prims) {
        const auto found = b.prims.find(path);
        if (found == b.prims.end()) {
            if (geometry(x)) {
                return false;
            }
            continue;
        }
        const RigExecPublishedPrim &y = found->second;
        if (x.hasPoints != y.hasPoints || x.hasNormals != y.hasNormals ||
            x.hasExtent != y.hasExtent || x.hasXform != y.hasXform ||
            (x.hasPoints && !_SamePoints(x.points, y.points)) ||
            (x.hasNormals && !_SamePoints(x.normals, y.normals)) ||
            (x.hasExtent && (x.extentMin != y.extentMin ||
                             x.extentMax != y.extentMax)) ||
            (x.hasXform && (!_SameMatrices(x.xform, y.xform) ||
                            !_SameMatrices(x.xformBase, y.xformBase)))) {
            return false;
        }
    }
    for (const auto &[path, y] : b.prims) {
        if (geometry(y) && a.prims.find(path) == a.prims.end()) {
            return false;
        }
    }
    return true;
}

static int comparedRows = 0;
static int comparedTimes = 0;

static void
_TestFixture(const std::string &stagePath, const std::string &frameText,
             const std::string &operatorPrim, const std::string &animation,
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
    if (frames.empty()) {
        return;
    }

    RigExecRigEvaluator evaluator(stage, rigPath);

    RigExecBakeOpts opts;
    opts.time = frames.front();
    RigExecBakeResult baked;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &baked, &error));
    if (baked.bytes.empty()) {
        std::printf("bake diagnostic: %s\n", error.c_str());
        return;
    }
    // The times played: an inputs row's table frames with each midpoint
    // between them, a static row's bake time.
    std::vector<double> times;
    if (animation == "static") {
        times.push_back(frames.front());
    } else {
        for (size_t i = 0; i < frames.size(); ++i) {
            if (i > 0) {
                times.push_back(0.5 * (frames[i - 1] + frames[i]));
            }
            times.push_back(frames[i]);
        }
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
    for (double frame : times) {
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
        ++comparedTimes;
    }
    std::printf("  %s: %zu time(s) compared, baked at %g (%s)\n",
                stagePath.c_str(), times.size(), frames.front(),
                animation.c_str());
    ++comparedRows;
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

    RigExecBakeOpts opts;
    opts.time = frames.front();
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

// A registry playback session reads its static inputs (not Animated, not
// keyed) only after a stage notice, and the registry forwards every notice
// of its stage. A Default edit of such an input then plays at the next
// time exactly as a playback opened after the edit plays it, and keys
// authored on it afterwards are followed at every time. Playbacks outside
// the registry hear of no edit and keep what they read: the one opened
// before the edit shows that the edit moves the rig, the one opened after
// it that the keys do.
static void
_TestPlaybackSeesAStaticProviderEdit(const std::string &examples,
                                     const std::filesystem::path &scratch)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examples + "/biped/Biped_anim.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = _FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    const double bakeTime = 1.0, t1 = 2.0, t2 = 3.0, t3 = 5.0, t4 = 6.0;
    RigExecBakeResult baked;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        RigExecBakeOpts opts;
        opts.time = bakeTime;
        CHECK(RigExecBakeToBinary(evaluator, opts, &baked, &error));
    }
    if (baked.bytes.empty()) {
        std::printf("static edit: bake: %s\n", error.c_str());
        return;
    }
    const std::string binary = (scratch / "static_edit.rigexec").string();
    {
        std::ofstream stream(binary, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(baked.bytes.data()),
                     std::streamsize(baked.bytes.size()));
    }
    // The input slots the file holds and does not mark Animated, by path.
    std::set<std::string> staticSlots;
    {
        std::unique_ptr<fb::RigExecWireFile> file;
        CHECK(RigExecFormatOpen(baked.bytes.data(), baked.bytes.size(), &file,
                                &error));
        if (file) {
            for (const auto &input : file->inputs) {
                if (!(input.flags() &
                      uint8_t(fb::InputSlotFlags::Animated))) {
                    staticSlots.insert(
                        RigExecFormatPathText(*file, input.name()));
                }
            }
        }
    }
    // Rest avars the file holds as static inputs, the provider values a
    // playback samples from their source, in stage order.
    const TfToken restAvars[] = {TfToken("rest:tx"), TfToken("rest:ty"),
                                 TfToken("rest:tz"), TfToken("rest:rx"),
                                 TfToken("rest:ry"), TfToken("rest:rz")};
    std::vector<UsdAttribute> candidates;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        for (const TfToken &name : restAvars) {
            const UsdAttribute attribute = prim.GetAttribute(name);
            if (attribute &&
                attribute.GetTypeName() == SdfValueTypeNames->Double &&
                attribute.GetNumTimeSamples() == 0 &&
                !attribute.HasAuthoredConnections() &&
                staticSlots.count(attribute.GetPath().GetString())) {
                candidates.push_back(attribute);
            }
        }
    }
    CHECK(!candidates.empty());
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        const UsdAttribute asset =
            stage->GetPrimAtPath(rigPath).CreateAttribute(
                TfToken("rigExec:asset"), SdfValueTypeNames->Asset);
        CHECK(asset && asset.Set(SdfAssetPath(binary)));
    }
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rigPath, UsdTimeCode(bakeTime), &errors));
    for (const std::string &line : errors) {
        std::printf("static edit: activation: %s\n", line.c_str());
    }
    CHECK(registry.GetPlayback(rigPath) != nullptr);
    if (!registry.GetPlayback(rigPath) || candidates.empty()) {
        registry.Deactivate();
        return;
    }
    // The registry's generation at \p frame.
    const auto published = [&](double frame) {
        CHECK(registry.SetTime(UsdTimeCode(frame)));
        RigExecImagingSnapshotConstPtr snapshot = registry.GetStore()->Get();
        CHECK(snapshot && snapshot->Describes(UsdStageWeakPtr(stage),
                                              UsdTimeCode(frame)));
        return snapshot;
    };
    // A playback outside the registry, opened over the stage as it is now,
    // and its generation at a time.
    struct _Outside {
        std::shared_ptr<RigExecSnapshotStore> store;
        std::unique_ptr<RigExecBakedPlayback> play;
    };
    const auto outside = [&]() {
        _Outside out;
        out.store = std::make_shared<RigExecSnapshotStore>();
        out.play = std::make_unique<RigExecBakedPlayback>(stage, rigPath,
                                                          out.store);
        std::string why;
        const bool opened = out.play->Open(binary, &why);
        if (!opened) {
            std::printf("static edit: open: %s\n", why.c_str());
        }
        CHECK(opened);
        return out;
    };
    const auto at = [&](_Outside &out, double frame) {
        CHECK(out.play->EvaluateAndPublishResult(UsdTimeCode(frame)).ok);
        RigExecImagingSnapshotConstPtr snapshot = out.store->Get();
        CHECK(snapshot && snapshot->Describes(UsdStageWeakPtr(stage),
                                              UsdTimeCode(frame)));
        return snapshot;
    };

    // Unedited, the session and a playback outside it agree.
    _Outside before = outside();
    const RigExecImagingSnapshotConstPtr g1 = published(t1);
    const RigExecImagingSnapshotConstPtr b1 = at(before, t1);
    const RigExecImagingSnapshotConstPtr b2 = at(before, t2);
    if (!g1 || !b1 || !b2) {
        registry.Deactivate();
        return;
    }
    _CompareGenerations(*g1, *b1, "static edit, unedited");

    // A candidate whose Default edit moves the rig at t2: a probe told of
    // each edit reads it at t1 and holds it at t2.
    _Outside probe = outside();
    UsdAttribute edited;
    double original = 0.0;
    size_t tried = 0;
    for (const UsdAttribute &attribute : candidates) {
        double value = 0.0;
        if (tried == 48 || !attribute.Get(&value, UsdTimeCode(t2))) {
            continue;
        }
        ++tried;
        {
            UsdEditContext context(stage, stage->GetSessionLayer());
            CHECK(attribute.Set(value - 1.0));
        }
        probe.play->NoteStageChanged();
        at(probe, t1);
        const RigExecImagingSnapshotConstPtr p2 = at(probe, t2);
        if (p2 && !_SameGenerations(*p2, *b2)) {
            edited = attribute;
            original = value;
            break;
        }
        UsdEditContext context(stage, stage->GetSessionLayer());
        CHECK(attribute.Clear());
    }
    std::printf("static edit: %zu of %zu candidate(s) tried; edited %s\n",
                tried, candidates.size(),
                edited ? edited.GetPath().GetText() : "none");
    CHECK(edited);
    if (!edited) {
        registry.Deactivate();
        return;
    }
    // At t2 the session plays the edit as a playback opened after it.
    _Outside after = outside();
    const RigExecImagingSnapshotConstPtr a2 = at(after, t2);
    CHECK(a2 && !_SameGenerations(*a2, *b2));
    const RigExecImagingSnapshotConstPtr g2 = published(t2);
    if (g2 && a2) {
        _CompareGenerations(*g2, *a2, "static edit, Default at t2");
    }

    // Keys after the session opened: the edit's value at t3, the original
    // at t4. The playback opened after the Default edit hears of no keys
    // and still plays the edit at t4, so there the keys move the rig.
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        CHECK(edited.Set(original - 1.0, UsdTimeCode(t3)));
        CHECK(edited.Set(original, UsdTimeCode(t4)));
    }
    _Outside keyed3 = outside();
    _Outside keyed4 = outside();
    const RigExecImagingSnapshotConstPtr k3 = at(keyed3, t3);
    const RigExecImagingSnapshotConstPtr k4 = at(keyed4, t4);
    const RigExecImagingSnapshotConstPtr e4 = at(after, t4);
    CHECK(k4 && e4 && !_SameGenerations(*e4, *k4));
    for (const double frame : {t3, t4, t3}) {
        const RigExecImagingSnapshotConstPtr g = published(frame);
        const RigExecImagingSnapshotConstPtr &want = frame == t3 ? k3 : k4;
        if (g && want) {
            _CompareGenerations(*g, *want,
                                "static edit, keyed, at " +
                                    std::to_string(frame));
        }
    }
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
    int bakingRows = 0;
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            continue;
        }
        ++bakingRows;
        const std::string stage =
            (std::filesystem::path(argv[1]) / fixture.stage).string();
        std::printf("playback conformance: %s\n", stage.c_str());
        _TestFixture(stage, fixture.frames, fixture.operatorPrim,
                     fixture.animation, scratch);
        if (!selectionDone) {
            _TestSelection(stage, fixture.frames, scratch);
            selectionDone = true;
        }
    }
    std::printf("playback: %d of %d baking row(s) compared at %d time(s)\n",
                comparedRows, bakingRows, comparedTimes);
    CHECK(comparedRows == bakingRows);
    _TestPlaybackSeesAStaticProviderEdit(argv[1], scratch);
    if (failures == 0) {
        std::printf("testRigExecImagingPlayback: all tests passed\n");
    } else {
        std::printf("testRigExecImagingPlayback: %d failures\n", failures);
    }
    return failures == 0 ? 0 : 1;
}
