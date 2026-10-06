// rigExecRuntime geometry-family parity (M2): baked program vs runtime
// over every baking fixture, comparing published chain points bit for
// bit. Each stage is baked at one time and the binary is played through
// its inputs: the input sampler hands it the stage's animated inputs at
// each frame, and a drag is an input set to an authored value, held to the
// evaluator with the same value authored in the session layer.
#include "rigExecBake/bake.h"
#include "rigExecBake/staticReport.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/format.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

// The baked epilogue's summary line is the only place chainsBuilt and
// revisionsBuilt are published; the runtime reader emits the same line.
// Parse its five numbers so the counters are checked against it too.
static bool
_ParseSummary(const std::string &line, size_t *chains, size_t *revisions,
              size_t *created, size_t *executed, size_t *schedules)
{
    size_t values[5] = {0, 0, 0, 0, 0};
    int matched = 0;
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    matched = std::sscanf(line.c_str(),
                           "mover graph: %zu chain(s), %zu revision(s); "
                           "%zu created, %zu executed, %zu schedule(s) built",
                           &values[0], &values[1], &values[2], &values[3],
                           &values[4]);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
    *chains = values[0];
    *revisions = values[1];
    *created = values[2];
    *executed = values[3];
    *schedules = values[4];
    return matched == 5;
}

// Bakes \p stage at the first of \p frames and plays the file through the
// input sampler beside a fresh baked evaluator, frame by frame: the moved
// points, the diagnostics (summary line included) and the work counters,
// bit for bit. A `static` stage holds an animated source in static data
// at the bake time, so it plays that time alone. \p played, when given,
// receives each frame's points.
static void
_TestStage(const std::string &name, const UsdStageRefPtr &stage,
           const std::vector<double> &frames,
           std::vector<std::vector<RigExecRuntimePoints>> *played = nullptr,
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
        RigExecRigEvaluator baker(stage, rigPath);
        baker.SetEvaluationMode(RigExecEvaluationMode::Baked);
        if (!RigExecTestBakeAt(baker, frames.front(), &bytes, &error)) {
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
    // time's generation, the compile notices in it, with the same history.
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);

    bool failed = false;
    int compared = 0;
    for (double frame : times) {
        const RigExecRigPose pose = evaluator.Evaluate(frame);
        if (!pose.valid) {
            CHECK(false);
            failed = true;
            break;
        }
        if (!player.Play(frame, &error)) {
            std::printf("execute diagnostic at %s frame %.17g: %s\n",
                        name.c_str(), frame, error.c_str());
            CHECK(false);
            failed = true;
            break;
        }
        const RigExecRuntimeReader *reader = &player.Reader();

        // Points: path plus bitwise points per moved property, both sides
        // in path order.
        const std::vector<RigExecRuntimePoints> &readerPoints =
            reader->GetPoints();
        std::vector<const RigExecRuntimePoints *> points;
        points.reserve(readerPoints.size());
        for (const RigExecRuntimePoints &moved : readerPoints) {
            points.push_back(&moved);
        }
        std::sort(points.begin(), points.end(),
                  [](const RigExecRuntimePoints *a,
                     const RigExecRuntimePoints *b) {
                      return a->path < b->path;
                  });
        // Geometry owns the point-array entries; scalar property
        // results (inputs:weight, rigExec:gain, ...) are the pose
        // family's publication, compared by its own test.
        std::vector<std::pair<std::string, VtValue>> wantPoints;
        wantPoints.reserve(pose.movedProperties.size());
        for (const auto &entry : pose.movedProperties) {
            if (!entry.second.IsHolding<VtVec3fArray>()) {
                continue;
            }
            wantPoints.push_back(
                {entry.first.GetString(), entry.second});
        }
        std::sort(wantPoints.begin(), wantPoints.end(),
                  [](const std::pair<std::string, VtValue> &a,
                     const std::pair<std::string, VtValue> &b) {
                      return a.first < b.first;
                  });
        if (points.size() != wantPoints.size()) {
            std::printf("%s frame %.17g: %zu moved properties vs %zu\n",
                        name.c_str(), frame, points.size(),
                        wantPoints.size());
            for (const auto *moved : points) {
                std::printf("  got:  %s (%zu points)\n",
                            moved->path.c_str(), moved->points.size());
            }
            for (const auto &entry : wantPoints) {
                std::printf("  want: %s\n", entry.first.c_str());
            }
            CHECK(false);
            failed = true;
            continue;
        }
        for (size_t at = 0; at < wantPoints.size(); ++at) {
            const std::string &path = wantPoints[at].first;
            if (points[at]->path != path) {
                std::printf("%s frame %.17g: moved property %zu is %s, "
                            "expected %s\n", name.c_str(), frame, at,
                            points[at]->path.c_str(), path.c_str());
                CHECK(false);
                failed = true;
                break;
            }
            const VtVec3fArray &want =
                wantPoints[at].second.UncheckedGet<VtVec3fArray>();
            const std::vector<RrVec3f> &got = points[at]->points;
            if (got.size() != want.size() ||
                (want.size() > 0 &&
                 std::memcmp(got.data(), want.cdata(),
                             want.size() * sizeof(GfVec3f)) != 0)) {
                size_t firstDiff = 0;
                while (firstDiff < got.size() &&
                       firstDiff < want.size() &&
                       std::memcmp(&got[firstDiff], &want[firstDiff],
                                   sizeof(GfVec3f)) == 0) {
                    ++firstDiff;
                }
                std::printf("%s frame %.17g: %s points differ "
                            "(%zu vs %zu, first at %zu)\n", name.c_str(),
                            frame, path.c_str(), got.size(), want.size(),
                            firstDiff);
                if (firstDiff < got.size() && firstDiff < want.size()) {
                    std::printf("  got:  %.9g %.9g %.9g\n  want: %.9g "
                                "%.9g %.9g\n", got[firstDiff][0],
                                got[firstDiff][1], got[firstDiff][2],
                                want[firstDiff][0], want[firstDiff][1],
                                want[firstDiff][2]);
                }
                CHECK(false);
                failed = true;
                break;
            }
        }

        // Diagnostics: verbatim, summary line included. The summary's
        // five numbers are also checked against the counters below.
        const std::vector<std::string> &diagnostics =
            reader->GetDiagnostics();
        const std::vector<std::string> &wantDiags = pose.diagnostics;
        size_t wantChains = 0, wantRevisions = 0, wantCreated = 0;
        size_t wantExecuted = 0, wantSchedules = 0;
        bool haveSummary = false;
        if (!wantDiags.empty() &&
            wantDiags.back().compare(0, 12, "mover graph:") == 0) {
            haveSummary = _ParseSummary(
                wantDiags.back(), &wantChains, &wantRevisions,
                &wantCreated, &wantExecuted, &wantSchedules);
            if (!haveSummary) {
                std::printf("%s frame %.17g: unparsable summary: %s\n",
                            name.c_str(), frame,
                            wantDiags.back().c_str());
                CHECK(false);
                failed = true;
            }
        } else {
            std::printf("%s frame %.17g: no summary line\n", name.c_str(),
                        frame);
            CHECK(false);
            failed = true;
        }
        if (diagnostics != wantDiags) {
            std::printf("%s frame %.17g: diagnostics differ "
                        "(%zu vs %zu)\n", name.c_str(), frame,
                        diagnostics.size(), wantDiags.size());
            const size_t common =
                std::min(diagnostics.size(), wantDiags.size());
            for (size_t i = 0; i < common; ++i) {
                if (diagnostics[i] != wantDiags[i]) {
                    std::printf("  got:  %s\n  want: %s\n",
                                diagnostics[i].c_str(),
                                wantDiags[i].c_str());
                    break;
                }
            }
            if (diagnostics.size() > common) {
                std::printf("  extra got:  %s\n",
                            diagnostics[common].c_str());
            }
            if (wantDiags.size() > common) {
                std::printf("  extra want: %s\n",
                            wantDiags[common].c_str());
            }
            CHECK(false);
            failed = true;
        }

        // Counters: the pose's three plus the summary's five.
        const RigExecRuntimeCounters counters = reader->GetCounters();
        if (counters.revisionsExecuted !=
                pose.moverGraphRevisionsExecuted ||
            counters.revisionsCreated !=
                pose.moverGraphRevisionsCreated ||
            counters.schedulesBuilt != pose.moverGraphSchedulesBuilt) {
            std::printf("%s frame %.17g: counters %u/%u/%u vs %zu/%zu/%zu\n",
                        name.c_str(), frame, counters.revisionsCreated,
                        counters.revisionsExecuted, counters.schedulesBuilt,
                        pose.moverGraphRevisionsCreated,
                        pose.moverGraphRevisionsExecuted,
                        pose.moverGraphSchedulesBuilt);
            CHECK(false);
            failed = true;
        }
        if (haveSummary &&
            (counters.chainsBuilt != wantChains ||
             counters.revisionsBuilt != wantRevisions ||
             counters.revisionsCreated != wantCreated ||
             counters.revisionsExecuted != wantExecuted ||
             counters.schedulesBuilt != wantSchedules)) {
            std::printf("%s frame %.17g: counters vs summary differ\n",
                        name.c_str(), frame);
            CHECK(false);
            failed = true;
        }
        if (played) {
            played->push_back(readerPoints);
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

// A geometry-domain constraint whose sphere weight reads `preceding`: the
// revision's current-phase field is measured against the points entering
// it, with the sphere's height keyed so the field moves
// (testRigExecBinary's current-phase bake).
static UsdStageRefPtr
_CurrentPhaseStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Points"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0, 4, 0),
                          GfVec3f(0, 8, 0)});
    const UsdGeomXform source =
        UsdGeomXform::Define(stage, SdfPath("/Asset/PullTo"));
    source.MakeMatrixXform().Set(
        GfMatrix4d(1.0).SetTranslate(GfVec3d(6, 0, 0)));
    const UsdPrim sphere = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sphere"), TfToken("RigExecSphereWeight"));
    sphere.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(0.0);
    const UsdAttribute height =
        sphere.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double);
    height.Set(0.0, UsdTimeCode(1.0));
    height.Set(8.0, UsdTimeCode(2.0));
    sphere.CreateAttribute(TfToken("avars:tz"), SdfValueTypeNames->Double)
        .Set(0.0);
    sphere.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    sphere.CreateAttribute(TfToken("inputs:falloffMin"),
                           SdfValueTypeNames->Float)
        .Set(0.0f);
    sphere.CreateAttribute(TfToken("inputs:falloffMax"),
                           SdfValueTypeNames->Float)
        .Set(8.0f);
    sphere.CreateAttribute(TfToken("rigExec:falloffProfile"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("linear"));
    sphere.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetMetadata(TfToken("rigExecReadPhase"), std::string("preceding"));
    const UsdPrim constraint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Sweep/Pull"),
        TfToken("RigExecPositionConstraint"));
    constraint.ApplyAPI(TfToken("RigExecMoverAPI"));
    constraint.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    constraint.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({source.GetPath()});
    constraint.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({sphere.GetPath()});
    return stage;
}

static bool
_SamePoints(const std::vector<std::vector<RigExecRuntimePoints>> &a,
            const std::vector<std::vector<RigExecRuntimePoints>> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t f = 0; f < a.size(); ++f) {
        if (a[f].size() != b[f].size()) {
            return false;
        }
        for (size_t p = 0; p < a[f].size(); ++p) {
            if (a[f][p].path != b[f][p].path ||
                a[f][p].points.size() != b[f][p].points.size() ||
                (!a[f][p].points.empty() &&
                 std::memcmp(a[f][p].points.data(), b[f][p].points.data(),
                             a[f][p].points.size() * sizeof(RrVec3f)) !=
                     0)) {
                return false;
            }
        }
    }
    return true;
}

// The file plays points computed from its inputs, not ones it holds: the
// first and the last frame's points differ, while it holds the bake time's
// static data alone.
static void
_CheckFramesDiffer(const std::string &name,
                   const std::vector<std::vector<RigExecRuntimePoints>> &rows)
{
    const bool differ =
        rows.size() > 1 && !_SamePoints({rows.front()}, {rows.back()});
    CHECK(differ);
    if (!differ) {
        std::printf("%s: the first and last frames play the same points\n",
                    name.c_str());
    }
}

// Current-phase weight packets are computed, not replayed: a file baked at
// the first frame plays every frame bit for bit with the baked program, and
// the field moves the points between the first and last frame.
static void
TestComputedCurrentPhase()
{
    const char *const name = "computed current phase";
    std::vector<std::vector<RigExecRuntimePoints>> rows;
    _TestStage(name, _CurrentPhaseStage(), {1.0, 2.0}, &rows);
    _CheckFramesDiffer(name, rows);
}

// The assemblers' connection-following scalar reads of every type
// (tests/fixtures/computed_path_reads.usda): the file's read rows are
// exactly those sites, each reading its head when its walk yields nothing,
// including the two floats whose connection ends on a double with no
// value, which read their own authored value. The fixture's mesh points
// are keyed and held as a chain base, static data the file holds at the
// bake time: the binary plays that time (rigExecPose --verify-binary drags
// its inputs there). The same stage with the points held by a stronger
// default (tests/rigExecPathReadsHeldPoints.usda) reads every animated
// source as an input, so its binary plays every frame, the keyed double3,
// matrix4d and float reads computed at each.
static void
TestPathReadsFixture()
{
    const char *const name = "path reads fixture";
    const std::string fixture = std::string(RIGEXEC_EXAMPLES_DIR) +
                                "/../tests/fixtures/computed_path_reads.usda";
    const std::vector<double> frames = {1.0, 4.0, 7.0, 10.0};
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    _TestStage(name, stage, frames, nullptr, true);
    {
        const std::string heldName = std::string(name) + ", points held";
        const UsdStageRefPtr held =
            UsdStage::Open(std::string(RIGEXEC_EXAMPLES_DIR) +
                           "/../tests/rigExecPathReadsHeldPoints.usda");
        CHECK(held);
        if (held) {
            std::vector<std::vector<RigExecRuntimePoints>> rows;
            _TestStage(heldName, held, frames, &rows);
            _CheckFramesDiffer(heldName, rows);
        }
    }

    RigExecRigEvaluator evaluator(stage, _FindRig(stage));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    const std::string step = "/PathReadAsset/Rig/Movers/Mush.inputs:step";
    const std::string held = "/PathReadAsset/Rig/Dials.held";
    // The read rows are exactly those sites, each reading its head when its
    // walk yields nothing; the two that end on the valueless double walk
    // two hops, and their head holds the float the site reads.
    size_t rows = 0;
    size_t twoHops = 0;
    for (const fb::RigExecWirePathRead &row : file->geometry->pathReads) {
        if (!row.read) {
            continue;
        }
        ++rows;
        CHECK(row.headFallback && !row.rest);
        const std::string text = RigExecFormatPathText(*file, row.path);
        if (text != step && text != held) {
            continue;
        }
        const fb::RigExecWireInput &read = *row.read;
        CHECK(read.walk.size() == 2);
        CHECK(read.tag == fb::InputTag::Float);
        if (read.walk.size() != 2) {
            continue;
        }
        // The head's own float, and a double hop with no value.
        const fb::InputSlot &head = file->inputs[read.walk[0]];
        const fb::InputSlot &hop = file->inputs[read.walk[1]];
        CHECK(head.type() == fb::InputTag::Float &&
              (head.flags() & uint8_t(fb::InputSlotFlags::HasValue)) != 0);
        CHECK(hop.type() == fb::InputTag::Double &&
              (hop.flags() & uint8_t(fb::InputSlotFlags::HasValue)) == 0);
        const uint32_t bits = uint32_t(file->values[head.value()].bits);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        CHECK(value == (text == step ? 0.3f : 0.6f));
        ++twoHops;
    }
    CHECK(rows == 13);
    CHECK(twoHops == 2);
}

// A blend channel whose inputs:weight is connected to a pose
// interpolator's output (testRigExecPoseInterpolator's rig, the driver
// keyed through the poses). With \p published a float math mover also
// revises the weight itself, so the assembly reads that result instead of
// the pose weight.
static UsdStageRefPtr
_PoseDrivenBlendStage(bool published)
{
    const SdfPath driver("/Asset/Rig/Controls/Shoulder/Driver");
    const SdfPath interpolator("/Asset/Rig/PoseInterpolators/Swing");
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls/Shoulder"),
                      TfToken("RigExecControl"));
    const UsdPrim control = stage->DefinePrim(driver,
                                              TfToken("RigExecControl"));
    const UsdAttribute rz = control.GetAttribute(TfToken("avars:rz"));
    rz.Set(0.0, UsdTimeCode(1.0));
    rz.Set(30.0, UsdTimeCode(2.0));
    rz.Set(45.0, UsdTimeCode(3.0));
    const VtVec3fArray base{GfVec3f(0.0f, 0.0f, 0.0f),
                            GfVec3f(1.0f, 0.0f, 0.0f)};
    stage->DefinePrim(SdfPath("/Asset/Geom/Mesh"), TfToken("Mesh"))
        .GetAttribute(TfToken("points"))
        .Set(base);
    VtVec3fArray full = base;
    for (GfVec3f &p : full) {
        p[2] += 10.0f;
    }
    stage->DefinePrim(SdfPath("/Asset/Targets/Full"), TfToken("Points"))
        .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(full);
    stage->DefinePrim(SdfPath("/Asset/Rig/PoseInterpolators"),
                      TfToken("Scope"));
    const UsdPrim swing =
        stage->DefinePrim(interpolator, TfToken("RigExecPoseInterpolator"));
    swing.CreateRelationship(TfToken("rigExec:driver")).SetTargets({driver});
    swing.GetAttribute(TfToken("rigExec:kernel")).Set(TfToken("gaussian"));
    swing.GetAttribute(TfToken("rigExec:twistAxis")).Set(TfToken("Z"));
    const auto addPose = [&](const char *poseName, double degrees) {
        const UsdPrim pose = stage->DefinePrim(
            interpolator.AppendChild(TfToken(poseName)), TfToken("RigExecPose"));
        const double half = GfDegreesToRadians(degrees) * 0.5;
        pose.GetAttribute(TfToken("rigExec:poseType")).Set(TfToken("whole"));
        pose.GetAttribute(TfToken("rigExec:rotation"))
            .Set(GfQuatf(float(std::cos(half)),
                         GfVec3f(0.0f, 0.0f, float(std::sin(half)))));
        pose.GetAttribute(TfToken("rigExec:rotationRadius"))
            .Set(float(GfDegreesToRadians(45.0)));
        return pose;
    };
    addPose("neutral", 0.0);
    const UsdPrim forward = addPose("Forward", 45.0);
    addPose("Back", -45.0);
    const UsdPrim input = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/Corrective"),
        TfToken("RigExecBlendInput"));
    const UsdAttribute weight = input.GetAttribute(TfToken("inputs:weight"));
    weight.Set(0.0f);
    weight.AddConnection(
        forward.GetPath().AppendProperty(TfToken("outputs:weight")));
    const UsdPrim sample = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/Corrective/Full"),
        TfToken("RigExecBlendSample"));
    sample.GetAttribute(TfToken("rigExec:activation")).Set(1.0f);
    sample.CreateRelationship(TfToken("rigExec:targetPoints"))
        .SetTargets({SdfPath("/Asset/Targets/Full.points")});
    input.CreateRelationship(TfToken("rigExec:samples"))
        .SetTargets({sample.GetPath()});
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Blend"), TfToken("RigExecBlendShapeMover"));
    blend.ApplyAPI(TfToken("RigExecMoverAPI"));
    blend.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/Mesh.points")});
    blend.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    blend.CreateRelationship(TfToken("rigExec:blendInputs"))
        .SetTargets({input.GetPath()});
    if (published) {
        const UsdPrim offset = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Offset"),
            TfToken("RigExecFloatMathMover"));
        CHECK(offset.ApplyAPI(TfToken("RigExecMoverAPI")));
        offset.CreateAttribute(TfToken("rigExec:operation"),
                               SdfValueTypeNames->Token)
            .Set(TfToken("add"));
        offset.CreateAttribute(TfToken("inputs:value"),
                               SdfValueTypeNames->Float)
            .Set(0.25f);
        offset.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({weight.GetPath()});
    }
    return stage;
}

// A pose-driven blend channel's weight is the pose weight slot, played
// through the inputs at every frame, so the points move between the first
// and the last frame; with a value published at the weight itself it is
// that value (the authored 0 offset by 0.25, the same at every frame).
static void
TestPoseDrivenBlendWeights()
{
    const std::vector<double> frames = {1.0, 2.0, 3.0};
    for (const bool published : {false, true}) {
        const std::string name =
            published ? "pose-driven blend channel, published weight"
                      : "pose-driven blend channel";
        std::vector<std::vector<RigExecRuntimePoints>> rows;
        _TestStage(name, _PoseDrivenBlendStage(published), frames, &rows);
        if (!published) {
            _CheckFramesDiffer(name, rows);
        }
    }
}

// A blend channel of two dense samples: Half's rigExec:activation is
// connected to a control's keyed avars:tx, Full's is authored 1. Dragged
// to 1.5, the avar carries Half's activation past Full's, which reorders
// the channel's samples. Half's shape is off the line from the base to
// Full's, so at each baked frame the drag moves the points.
static UsdStageRefPtr
_ActivationDragStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));
    const UsdPrim control = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Shape"), TfToken("RigExecControl"));
    control.CreateAttribute(TfToken("rest:space"), SdfValueTypeNames->Matrix4d)
        .Set(GfMatrix4d(1.0));
    const UsdAttribute tx =
        control.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    tx.Set(0.25, UsdTimeCode(1.0));
    tx.Set(0.75, UsdTimeCode(3.0));
    const VtVec3fArray base{GfVec3f(0.0f, 0.0f, 0.0f),
                            GfVec3f(1.0f, 0.0f, 0.0f)};
    stage->DefinePrim(SdfPath("/Asset/Geom/Mesh"), TfToken("Mesh"))
        .GetAttribute(TfToken("points"))
        .Set(base);
    const auto target = [&](const char *path, float dz) {
        VtVec3fArray points = base;
        for (GfVec3f &p : points) {
            p[2] += dz;
        }
        stage->DefinePrim(SdfPath(path), TfToken("Points"))
            .CreateAttribute(TfToken("points"),
                             SdfValueTypeNames->Point3fArray)
            .Set(points);
    };
    target("/Asset/Targets/Half", 4.0f);
    target("/Asset/Targets/Full", 10.0f);
    const UsdPrim input = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/Smile"), TfToken("RigExecBlendInput"));
    input.GetAttribute(TfToken("inputs:weight")).Set(0.6f);
    const auto sample = [&](const char *name, const char *targetPath) {
        const UsdPrim prim = stage->DefinePrim(
            input.GetPath().AppendChild(TfToken(name)),
            TfToken("RigExecBlendSample"));
        prim.CreateRelationship(TfToken("rigExec:targetPoints"))
            .SetTargets({SdfPath(targetPath)});
        return prim;
    };
    const UsdPrim half = sample("Half", "/Asset/Targets/Half.points");
    half.GetAttribute(TfToken("rigExec:activation"))
        .SetConnections({tx.GetPath()});
    const UsdPrim full = sample("Full", "/Asset/Targets/Full.points");
    full.GetAttribute(TfToken("rigExec:activation")).Set(1.0f);
    input.CreateRelationship(TfToken("rigExec:samples"))
        .SetTargets({half.GetPath(), full.GetPath()});
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Blend"), TfToken("RigExecBlendShapeMover"));
    blend.ApplyAPI(TfToken("RigExecMoverAPI"));
    blend.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/Mesh.points")});
    blend.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    blend.CreateRelationship(TfToken("rigExec:blendInputs"))
        .SetTargets({input.GetPath()});
    return stage;
}

// Whether the runtime's moved points equal \p pose's bit for bit, path for
// path.
static bool
_SameMovedPoints(const RigExecRigPose &pose,
                 const std::vector<RigExecRuntimePoints> &got)
{
    size_t want = 0;
    for (const auto &[path, value] : pose.movedProperties) {
        if (!value.IsHolding<VtVec3fArray>()) {
            continue;
        }
        ++want;
        const VtVec3fArray &points = value.UncheckedGet<VtVec3fArray>();
        const auto found = std::find_if(
            got.begin(), got.end(), [&](const RigExecRuntimePoints &moved) {
                return moved.path == path.GetString();
            });
        if (found == got.end() || found->points.size() != points.size() ||
            (!points.empty() &&
             std::memcmp(found->points.data(), points.cdata(),
                         points.size() * sizeof(GfVec3f)) != 0)) {
            return false;
        }
    }
    return want == got.size();
}

// An input set on the avar a blend sample's activation reads through its
// connection. The runtime reads each activation over the input slots as
// the baked gather reads it through the resolved inputs, so with the avar
// set to 1.5 and held over the frames (set again after the stage's keys
// at each frame) its points equal, bit for bit, the baked program's with
// 1.5 authored on the avar in the session layer (and the baked program
// agrees with the dynamic evaluator): before, while and after the set, and
// reset at the frame it stood on.
static void
TestBlendActivationDrag()
{
    const char *const name = "blend activation drag";
    const std::vector<double> frames = {1.0, 2.0, 3.0};
    const UsdStageRefPtr stage = _ActivationDragStage();
    std::vector<std::vector<RigExecRuntimePoints>> rows;
    _TestStage(name, stage, frames, &rows);
    _CheckFramesDiffer(name, rows);

    const SdfPath rigPath("/Asset/Rig");
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
    const std::string txPath = "/Asset/Rig/Controls/Shape.avars:tx";
    // The references: a fresh evaluator with nothing authored, and one with
    // 1.5 authored on the avar.
    const auto references = [&](const std::vector<RigExecTestEdit> &edits) {
        std::vector<RigExecRigPose> poses;
        const bool ok = RigExecTestEditedPoses(
            stage, rigPath, RigExecEvaluationMode::BakedWithParityCheck,
            edits, frames, &poses, &error);
        CHECK(ok);
        if (!ok) {
            std::printf("%s: reference: %s\n", name, error.c_str());
        }
        return poses;
    };
    const std::vector<RigExecRigPose> unset = references({});
    const std::vector<RigExecRigPose> set =
        references({{SdfPath(txPath), VtValue(1.5)}});
    if (unset.size() != frames.size() || set.size() != frames.size()) {
        CHECK(false);
        return;
    }
    // One frame against \p pose.
    const auto run = [&](const char *what, size_t f,
                         const RigExecRigPose &pose,
                         std::vector<std::vector<RigExecRuntimePoints>>
                             *played) {
        if (!player.Play(frames[f], &error)) {
            std::printf("%s, %s frame %.17g: %s\n", name, what, frames[f],
                        error.c_str());
            CHECK(false);
            return false;
        }
        played->push_back(player->GetPoints());
        if (!_SameMovedPoints(pose, played->back())) {
            std::printf("%s, %s frame %.17g: the runtime's points differ "
                        "from the baked program's\n",
                        name, what, frames[f]);
            return false;
        }
        return true;
    };
    const auto pass = [&](const char *what,
                          const std::vector<RigExecRigPose> &poses,
                          std::vector<std::vector<RigExecRuntimePoints>>
                              *played) {
        bool same = true;
        for (size_t f = 0; f < frames.size(); ++f) {
            same = run(what, f, poses[f], played) && same;
        }
        return same;
    };
    std::vector<std::vector<RigExecRuntimePoints>> undragged, dragged,
        released, releasedHere;
    CHECK(pass("undragged", unset, &undragged));
    CHECK(player.Hold(txPath, 1.5, &error));
    CHECK(pass("dragged", set, &dragged));
    CHECK(dragged.size() == frames.size() &&
          undragged.size() == frames.size());
    for (size_t f = 0; f < dragged.size() && f < undragged.size(); ++f) {
        CHECK(!_SamePoints({dragged[f]}, {undragged[f]}));
    }
    // Reset at the frame the set stood on, then over every frame.
    player.ReleaseAll();
    CHECK(run("released in place", frames.size() - 1, unset.back(),
              &releasedHere));
    CHECK(!releasedHere.empty() && !undragged.empty() &&
          _SamePoints({releasedHere.back()}, {undragged.back()}));
    CHECK(pass("released", unset, &released));
    CHECK(_SamePoints(released, undragged));
    std::printf("%s: checked\n", name);
}

// A blend shape mover whose two channels read UsdSkelBlendShape prims: one
// dense (an offset per point, no pointIndices, the schema's own form) and
// one sparse. Each channel's weight is keyed from 0 to 1.
static UsdStageRefPtr
_BlendShapeLayoutStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const VtVec3fArray base{GfVec3f(0.0f, 0.0f, 0.0f),
                            GfVec3f(1.0f, 0.0f, 0.0f),
                            GfVec3f(1.0f, 1.0f, 0.0f),
                            GfVec3f(0.0f, 1.0f, 0.0f)};
    stage->DefinePrim(SdfPath("/Asset/Geom/Mesh"), TfToken("Mesh"))
        .GetAttribute(TfToken("points"))
        .Set(base);
    const auto shape = [&](const char *path, const VtVec3fArray &offsets,
                           const VtIntArray *indices) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("BlendShape"));
        prim.CreateAttribute(TfToken("offsets"),
                             SdfValueTypeNames->Vector3fArray, false,
                             SdfVariabilityUniform)
            .Set(offsets);
        if (indices) {
            prim.CreateAttribute(TfToken("pointIndices"),
                                 SdfValueTypeNames->IntArray, false,
                                 SdfVariabilityUniform)
                .Set(*indices);
        }
    };
    shape("/Asset/Targets/Dense",
          {GfVec3f(0.0f, 0.0f, 1.0f), GfVec3f(0.0f, 0.0f, 2.0f),
           GfVec3f(0.0f, 0.5f, 0.0f), GfVec3f(0.25f, 0.0f, 0.0f)},
          nullptr);
    const VtIntArray sparse{1, 3};
    shape("/Asset/Targets/Sparse",
          {GfVec3f(0.0f, 0.0f, -1.0f), GfVec3f(0.5f, 0.0f, 0.0f)}, &sparse);
    SdfPathVector inputs;
    for (const char *name : {"Dense", "Sparse"}) {
        const UsdPrim input = stage->DefinePrim(
            SdfPath("/Asset/Rig/BlendInputs").AppendChild(TfToken(name)),
            TfToken("RigExecBlendInput"));
        const UsdAttribute weight =
            input.GetAttribute(TfToken("inputs:weight"));
        weight.Set(0.0f, UsdTimeCode(1.0));
        weight.Set(1.0f, UsdTimeCode(3.0));
        const UsdPrim sample = stage->DefinePrim(
            input.GetPath().AppendChild(TfToken("Full")),
            TfToken("RigExecBlendSample"));
        sample.GetAttribute(TfToken("rigExec:activation")).Set(1.0f);
        sample.CreateRelationship(TfToken("rigExec:blendShape"))
            .SetTargets({SdfPath("/Asset/Targets").AppendChild(
                TfToken(name))});
        input.CreateRelationship(TfToken("rigExec:samples"))
            .SetTargets({sample.GetPath()});
        inputs.push_back(input.GetPath());
    }
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Blend"), TfToken("RigExecBlendShapeMover"));
    blend.ApplyAPI(TfToken("RigExecMoverAPI"));
    blend.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/Mesh.points")});
    blend.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    blend.CreateRelationship(TfToken("rigExec:blendInputs")).SetTargets(inputs);
    return stage;
}

// Both UsdSkelBlendShape forms bake, and the file plays them bit for bit
// with the baked program: the dense layout's offsets run over every point.
static void
TestBlendShapeLayouts()
{
    const char *const name = "blend shape layouts";
    std::vector<std::vector<RigExecRuntimePoints>> rows;
    _TestStage(name, _BlendShapeLayoutStage(), {1.0, 2.0, 3.0}, &rows);
    _CheckFramesDiffer(name, rows);
}

static const RigExecRuntimeWeightField *
_FindWeightField(const RigExecRuntimeReader &reader, const char *path)
{
    for (const RigExecRuntimeWeightField &field : reader.GetWeightFields()) {
        if (field.path == path) {
            return &field;
        }
    }
    return nullptr;
}

static const RigExecRuntimePoints *
_FindPoints(const RigExecRuntimeReader &reader, const char *path)
{
    for (const RigExecRuntimePoints &moved : reader.GetPoints()) {
        if (moved.path == path) {
            return &moved;
        }
    }
    return nullptr;
}

// Plays \p stage's bake at the first frame through its inputs beside an
// ExecReference evaluator (the exec-authoritative walk every evaluator is
// held to). At every frame the runtime's published field for
// \p weightObject -- the current-phase field its oracle measured against
// the points entering the revision -- must equal the dynamic evaluator's
// weightFields entry bit for bit, and so must the points at \p pointsPath.
// \p fields receives the dynamic fields, one per frame. False on any
// difference.
static bool
_FieldsMatchDynamic(const std::string &name, const UsdStageRefPtr &stage,
                    const std::vector<double> &frames,
                    const char *weightObject, const char *pointsPath,
                    std::vector<std::vector<float>> *fields)
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
            std::printf("%s frame %g: dynamic pose invalid\n", name.c_str(),
                        frame);
            return false;
        }
        if (!player.Play(frame, &error)) {
            std::printf("%s frame %g: %s\n", name.c_str(), frame,
                        error.c_str());
            return false;
        }
        const auto wantField = want.weightFields.find(SdfPath(weightObject));
        const RigExecRuntimeWeightField *gotField =
            _FindWeightField(player.Reader(), weightObject);
        if (wantField == want.weightFields.end() || !gotField) {
            std::printf("%s frame %g: no field for %s (dynamic %d, runtime "
                        "%d)\n", name.c_str(), frame, weightObject,
                        int(wantField != want.weightFields.end()),
                        int(gotField != nullptr));
            return false;
        }
        const std::vector<float> &w = wantField->second.weights;
        if (gotField->weights.size() != w.size() ||
            (!w.empty() &&
             std::memcmp(gotField->weights.data(), w.data(),
                         w.size() * sizeof(float)) != 0)) {
            std::printf("%s frame %g: the runtime's field for %s differs "
                        "from the dynamic evaluator's\n", name.c_str(),
                        frame, weightObject);
            for (size_t i = 0; i < gotField->weights.size() || i < w.size();
                 ++i) {
                std::printf("  [%zu] runtime %.9g, dynamic %.9g\n", i,
                            i < gotField->weights.size()
                                ? double(gotField->weights[i]) : -1.0,
                            i < w.size() ? double(w[i]) : -1.0);
            }
            same = false;
        }
        fields->push_back(w);
        const auto wantPoints = want.movedProperties.find(SdfPath(pointsPath));
        const RigExecRuntimePoints *gotPoints =
            _FindPoints(player.Reader(), pointsPath);
        if (wantPoints == want.movedProperties.end() || !gotPoints ||
            !wantPoints->second.IsHolding<VtVec3fArray>()) {
            std::printf("%s frame %g: %s did not move on both sides\n",
                        name.c_str(), frame, pointsPath);
            same = false;
            continue;
        }
        const VtVec3fArray &p =
            wantPoints->second.UncheckedGet<VtVec3fArray>();
        if (gotPoints->points.size() != p.size() ||
            (!p.empty() &&
             std::memcmp(gotPoints->points.data(), p.cdata(),
                         p.size() * sizeof(GfVec3f)) != 0)) {
            std::printf("%s frame %g: %s differs from the dynamic "
                        "evaluator's\n", name.c_str(), frame, pointsPath);
            same = false;
        }
    }
    return same;
}

// testRigExecBakedMode's geometry-domain arm (MakeAConstrainedVolumeRig)
// without the constraint that moves the volume, the one part of that rig
// the program refuses: the sphere follows the Lift control through a
// connection on its own avars:ty instead. The geometry constraint's weight
// object is then a `preceding` sphere the oracle measures at every frame.
// Lift's avars:ty is keyed 0, 8 and 2 at frames 1, 3 and 5; the sphere's
// inputs:falloffMax is keyed 8, 8 and 6 there, so a step-backed scalar read
// changes between frames too. With \p narrowFalloff a float math mover
// halves inputs:falloffMax, and the oracle reads the chain's result
// through the overlay.
static UsdStageRefPtr
_GeometryDomainArmStage(bool narrowFalloff = false)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim slab =
        stage->DefinePrim(SdfPath("/Asset/Geom/Slab"), TfToken("Points"));
    slab.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0, 4, 0),
                          GfVec3f(0, 8, 0)});

    const UsdPrim lift = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Lift"), TfToken("RigExecControl"));
    const UsdAttribute liftTy =
        lift.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double);
    liftTy.Set(0.0, UsdTimeCode(1.0));
    liftTy.Set(8.0, UsdTimeCode(3.0));
    liftTy.Set(2.0, UsdTimeCode(5.0));

    const UsdPrim sphere = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sphere"), TfToken("RigExecSphereWeight"));
    sphere.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Geom/Slab.points")});
    sphere.CreateAttribute(TfToken("inputs:falloffMin"),
                           SdfValueTypeNames->Float)
        .Set(0.0f);
    const UsdAttribute falloffMax = sphere.CreateAttribute(
        TfToken("inputs:falloffMax"), SdfValueTypeNames->Float);
    falloffMax.Set(8.0f, UsdTimeCode(1.0));
    falloffMax.Set(8.0f, UsdTimeCode(3.0));
    falloffMax.Set(6.0f, UsdTimeCode(5.0));
    sphere.CreateAttribute(TfToken("rigExec:falloffProfile"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("linear"));
    sphere.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetMetadata(TfToken("rigExecReadPhase"), std::string("preceding"));
    sphere.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
        .AddConnection(liftTy.GetPath());
    if (narrowFalloff) {
        const UsdPrim narrow = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Narrow"),
            TfToken("RigExecFloatMathMover"));
        CHECK(narrow.ApplyAPI(TfToken("RigExecMoverAPI")));
        narrow.CreateAttribute(TfToken("rigExec:operation"),
                               SdfValueTypeNames->Token)
            .Set(TfToken("multiply"));
        narrow.CreateAttribute(TfToken("inputs:value"),
                               SdfValueTypeNames->Float)
            .Set(0.5f);
        narrow.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({falloffMax.GetPath()});
    }

    const UsdGeomXform pull =
        UsdGeomXform::Define(stage, SdfPath("/Asset/PullTo"));
    pull.MakeMatrixXform().Set(
        GfMatrix4d(1.0).SetTranslate(GfVec3d(6, 0, 0)));
    const UsdPrim constraint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Sweep/Pull"),
        TfToken("RigExecPositionConstraint"));
    constraint.ApplyAPI(TfToken("RigExecMoverAPI"));
    constraint.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/Slab.points")});
    constraint.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({pull.GetPath()});
    constraint.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({sphere.GetPath()});
    return stage;
}

// The current-phase field's scalars are inputs: on a file baked at frame
// 3, the sphere's inputs:falloffMax (keyed, 8 there) set to 4 narrows the
// field (the point 4 from the sphere drops from 0.5 to 0), and that is
// what a fresh evaluator in the dynamic and the baked mode publishes at
// frame 3 with 4 authored on the attribute in the session layer, every
// output bit for bit.
static void
_TestFalloffInputMatchesSessionEdit()
{
    const char *const name = "geometry-domain arm, input set";
    const char *const sphere = "/Asset/Rig/Weights/Sphere";
    const std::string falloffMax = std::string(sphere) + ".inputs:falloffMax";
    const double bakeTime = 3.0;
    const UsdStageRefPtr stage = _GeometryDomainArmStage();
    const SdfPath rigPath("/Asset/Rig");
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
    const RigExecRuntimeWeightField *field =
        _FindWeightField(player.Reader(), sphere);
    CHECK(field &&
          field->weights == (std::vector<float>{0.0f, 0.5f, 1.0f}));
    CHECK(player->SetInput(falloffMax, 4.0, &error));
    CHECK(player.Play(bakeTime, &error));
    field = _FindWeightField(player.Reader(), sphere);
    CHECK(field &&
          field->weights == (std::vector<float>{0.0f, 0.0f, 1.0f}));
    for (const RigExecEvaluationMode mode :
         {RigExecEvaluationMode::Dynamic, RigExecEvaluationMode::Baked}) {
        std::vector<RigExecRigPose> poses;
        CHECK(RigExecTestEditedPoses(stage, rigPath, mode,
                                     {{SdfPath(falloffMax), VtValue(4.0f)}},
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
TestGeometryDomainArm()
{
    const char *const name = "geometry-domain arm";
    const std::vector<double> frames = {1, 2, 3, 4, 5};
    std::vector<std::vector<RigExecRuntimePoints>> rows;
    _TestStage(name, _GeometryDomainArmStage(), frames, &rows);
    _CheckFramesDiffer(name, rows);
    std::vector<std::vector<float>> fields;
    CHECK(_FieldsMatchDynamic(name, _GeometryDomainArmStage(), frames,
                              "/Asset/Rig/Weights/Sphere",
                              "/Asset/Geom/Slab.points", &fields));
    // The field follows the sphere: 1 - d/8 about y = 0 at frame 1 and
    // about y = 8 at frame 3.
    CHECK(fields.size() == frames.size());
    if (fields.size() == frames.size()) {
        CHECK(fields[0] == (std::vector<float>{1.0f, 0.5f, 0.0f}));
        CHECK(fields[2] == (std::vector<float>{0.0f, 0.5f, 1.0f}));
    }
    _TestFalloffInputMatchesSessionEdit();
    std::printf("%s: %zu field(s) equal the dynamic evaluator's\n", name,
                fields.size());

    // The same arm with inputs:falloffMax halved by a float math mover: the
    // step-backed read's walk crosses the chain's target, so the oracle
    // reads 4 where 8 is authored (3 where 6 is).
    const char *const chained = "geometry-domain arm through a chain";
    rows.clear();
    _TestStage(chained, _GeometryDomainArmStage(true), frames, &rows);
    _CheckFramesDiffer(chained, rows);
    std::vector<std::vector<float>> narrowed;
    CHECK(_FieldsMatchDynamic(chained, _GeometryDomainArmStage(true), frames,
                              "/Asset/Rig/Weights/Sphere",
                              "/Asset/Geom/Slab.points", &narrowed));
    CHECK(narrowed.size() == frames.size());
    if (narrowed.size() == frames.size()) {
        CHECK(narrowed[0] == (std::vector<float>{1.0f, 0.0f, 0.0f}));
        CHECK(narrowed[2] == (std::vector<float>{0.0f, 0.0f, 1.0f}));
    }
    std::printf("%s: %zu field(s) equal the dynamic evaluator's\n", chained,
                narrowed.size());
}

// testRigExecVolumeWeights' TestCurrentPhaseThroughCombine with the lift
// animated. A matrix mover binds a combine around a `preceding` sphere at
// the origin reaching 3, and a nested mover lifts every point first, by
// avars:ty keyed 5 at frame 1 down to 0 at frame 4. The sphere measures
// the lifted points, so the field differs from the rest-pose field until
// the lift is gone: nothing at frames 1 and 2, the rest-pose field at 4.
// \p strength, when not 1, is authored as the combine's inputs:strength.
static UsdStageRefPtr
_CurrentPhaseThroughCombineStage(float strength = 1.0f)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath target("/Asset/Geom/M.points");
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Points"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0),
                          GfVec3f(1, 0, 0), GfVec3f(2, 0, 0)});
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    // Placed through the translate avars, as the volume-weight suite does.
    const auto place = [](const UsdPrim &prim, double tx, double ty,
                          double tz) {
        const char *const names[3] = {"avars:tx", "avars:ty", "avars:tz"};
        const double at[3] = {tx, ty, tz};
        for (int k = 0; k < 3; ++k) {
            prim.CreateAttribute(TfToken(names[k]), SdfValueTypeNames->Double)
                .Set(at[k]);
        }
    };
    const UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/J"),
                                            TfToken("RigExecJoint"));
    place(joint, 0, 2, 0);
    const UsdPrim lift = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Lift"),
                                           TfToken("RigExecJoint"));
    place(lift, 0, 0, 0);
    const UsdAttribute liftTy = lift.GetAttribute(TfToken("avars:ty"));
    liftTy.Clear();
    liftTy.Set(5.0, UsdTimeCode(1.0));
    liftTy.Set(0.0, UsdTimeCode(4.0));

    const UsdPrim full = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Full"), TfToken("RigExecStaticWeight"));
    full.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    full.CreateAttribute(TfToken("rigExec:representation"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("dense"));
    full.CreateAttribute(TfToken("rigExec:values"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1.0f, 1.0f, 1.0f, 1.0f});
    full.CreateAttribute(TfToken("rigExec:defaultWeight"),
                         SdfValueTypeNames->Float)
        .Set(0.0f);

    const UsdPrim sphere = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sphere"), TfToken("RigExecSphereWeight"));
    place(sphere, 0, 0, 0);
    sphere.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    sphere.CreateAttribute(TfToken("inputs:falloffMin"),
                           SdfValueTypeNames->Float)
        .Set(0.0f);
    sphere.CreateAttribute(TfToken("inputs:falloffMax"),
                           SdfValueTypeNames->Float)
        .Set(3.0f);
    sphere.CreateAttribute(TfToken("rigExec:falloffProfile"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("linear"));
    sphere.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetMetadata(TfToken("rigExecReadPhase"), std::string("preceding"));

    // The mover binds the combine, not the sphere.
    const UsdPrim combine = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Wrapped"),
        TfToken("RigExecCombineWeight"));
    combine.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    combine.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({sphere.GetPath()});
    if (strength != 1.0f) {
        combine.CreateAttribute(TfToken("inputs:strength"),
                                SdfValueTypeNames->Float)
            .Set(strength);
    }

    const auto mover = [&](const char *path, const UsdPrim &transform,
                           const UsdPrim &weight) {
        const UsdPrim m =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecMatrixMover"));
        CHECK(m.ApplyAPI(TfToken("RigExecMoverAPI")));
        m.CreateRelationship(TfToken("rigExec:moves")).SetTargets({target});
        m.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({transform.GetPath()});
        m.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weight.GetPath()});
    };
    // Composed post-order: the nested First runs before Second.
    mover("/Asset/Rig/Movers/Second", joint, combine);
    mover("/Asset/Rig/Movers/Second/First", lift, full);
    return stage;
}

static void
TestCurrentPhaseThroughCombine()
{
    const char *const name = "current phase through a combine";
    const std::vector<double> frames = {1, 2, 3, 4};
    std::vector<std::vector<RigExecRuntimePoints>> rows;
    _TestStage(name, _CurrentPhaseThroughCombineStage(), frames, &rows);
    _CheckFramesDiffer(name, rows);
    std::vector<std::vector<float>> fields;
    CHECK(_FieldsMatchDynamic(name, _CurrentPhaseThroughCombineStage(),
                              frames, "/Asset/Rig/Weights/Wrapped",
                              "/Asset/Geom/M.points", &fields));
    CHECK(fields.size() == frames.size());
    if (fields.size() == frames.size()) {
        // Lifted 5 and 10/3: every point is beyond the sphere's reach.
        CHECK(fields[0] == std::vector<float>(4, 0.0f));
        CHECK(fields[1] == std::vector<float>(4, 0.0f));
        // Lifted 5/3: inside, and nearer zero than the rest-pose field.
        for (size_t i = 0; i < fields[2].size(); ++i) {
            CHECK(fields[2][i] > 0.0f && fields[2][i] < fields[3][i]);
        }
        // Unlifted: the rest-pose field 1 - d/3 at d = 0, 0.5, 1, 2.
        CHECK(fields[3].size() == 4 && fields[3][0] == 1.0f);
    }
    std::printf("%s: %zu field(s) equal the dynamic evaluator's\n", name,
                fields.size());
}

// Bakes \p stage at the first of \p frames and plays the file through its
// inputs, collecting each frame's diagnostics; false with the first bake,
// open or Execute error.
static bool
_RunDiagnostics(const UsdStageRefPtr &stage, const std::vector<double> &frames,
                std::vector<std::vector<std::string>> *diagnostics,
                std::string *error)
{
    std::vector<uint8_t> bytes;
    {
        RigExecRigEvaluator evaluator(stage, _FindRig(stage));
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        if (!RigExecTestBakeAt(evaluator, frames.front(), &bytes, error)) {
            return false;
        }
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, error)) {
        return false;
    }
    for (double frame : frames) {
        if (!player.Play(frame, error)) {
            return false;
        }
        diagnostics->push_back(player->GetDiagnostics());
    }
    return true;
}

static bool
_HasLine(const std::vector<std::string> &lines, const std::string &line)
{
    return std::find(lines.begin(), lines.end(), line) != lines.end();
}

// A current-phase field the oracle fails to resolve: the combine's
// strength 2 lifts the rest-pose field past 1 under its strict range
// policy at frame 4. The revision then holds an empty packet and the
// generation the program's own diagnostic, compared verbatim by the parity
// path.
static void
TestCurrentPhaseFailure()
{
    const char *const name = "current phase, failed resolve";
    const std::vector<double> frames = {1, 2, 3, 4};
    _TestStage(name, _CurrentPhaseThroughCombineStage(2.0f), frames);
    std::vector<std::vector<std::string>> diagnostics;
    std::string error;
    CHECK(_RunDiagnostics(_CurrentPhaseThroughCombineStage(2.0f), frames,
                          &diagnostics, &error));
    if (!error.empty()) {
        std::printf("%s: %s\n", name, error.c_str());
    }
    const std::string failed = "current-phase weight failed: strict range "
                               "violation on /Asset/Rig/Weights/Wrapped";
    CHECK(diagnostics.size() == frames.size());
    if (diagnostics.size() == frames.size()) {
        CHECK(!_HasLine(diagnostics[0], failed));
        CHECK(_HasLine(diagnostics[3], failed));
    }
    std::printf("%s: checked\n", name);
}

// A current-phase combine over the three volume kinds, each read where the
// oracle reads it: a `preceding` sphere with per-axis and signed scales
// (in-flight samples), a plane that samples the authored mesh (the static
// sample points), and a `preceding` curve weight (the static curve
// points). The lift moves the points in flight from 1.5 at frame 1 to 0 at
// frame 4.
struct _MixOptions {
    const char *mode = "max";
    const char *bounds = "unbounded";
    const char *axis = "x";
    /// The plane samples a separate Points prim whose points are keyed.
    bool animatedSampler = false;
    /// The curve's points are keyed.
    bool animatedCurve = false;
};

static UsdStageRefPtr
_VolumeMixStage(const _MixOptions &options)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath target("/Asset/Geom/M.points");
    const VtVec3fArray rest{GfVec3f(0, 0, 0),         GfVec3f(0.5f, 0.25f, 0),
                            GfVec3f(-1, -0.5f, 0.25f), GfVec3f(2, 0, -0.5f),
                            GfVec3f(0.25f, 1, 0.5f),  GfVec3f(-0.5f, 0.5f, -1)};
    stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Points"))
        .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(rest);
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto place = [](const UsdPrim &prim, double tx, double ty,
                          double tz) {
        const char *const names[3] = {"avars:tx", "avars:ty", "avars:tz"};
        const double at[3] = {tx, ty, tz};
        for (int k = 0; k < 3; ++k) {
            prim.CreateAttribute(TfToken(names[k]), SdfValueTypeNames->Double)
                .Set(at[k]);
        }
    };
    const auto setFloat = [](const UsdPrim &prim, const char *name, float v) {
        prim.CreateAttribute(TfToken(name), SdfValueTypeNames->Float).Set(v);
    };
    const auto setToken = [](const UsdPrim &prim, const char *name,
                             const char *v) {
        prim.CreateAttribute(TfToken(name), SdfValueTypeNames->Token)
            .Set(TfToken(v));
    };
    const auto preceding = [&](const UsdPrim &prim) {
        prim.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({target});
        prim.GetRelationship(TfToken("rigExec:weightTarget"))
            .SetMetadata(TfToken("rigExecReadPhase"), std::string("preceding"));
    };
    const UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/J"),
                                            TfToken("RigExecJoint"));
    place(joint, 0, 2, 0);
    const UsdPrim lift = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Lift"),
                                           TfToken("RigExecJoint"));
    place(lift, 0, 0, 0);
    const UsdAttribute liftTy = lift.GetAttribute(TfToken("avars:ty"));
    liftTy.Clear();
    liftTy.Set(1.5, UsdTimeCode(1.0));
    liftTy.Set(0.0, UsdTimeCode(4.0));

    const UsdPrim full = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Full"), TfToken("RigExecStaticWeight"));
    full.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    setToken(full, "rigExec:representation", "dense");
    full.CreateAttribute(TfToken("rigExec:values"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray(rest.size(), 1.0f));
    setFloat(full, "rigExec:defaultWeight", 0.0f);

    const UsdPrim sphere = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sphere"), TfToken("RigExecSphereWeight"));
    place(sphere, 0, 0, 0);
    preceding(sphere);
    setFloat(sphere, "inputs:falloffMin", 0.0f);
    setFloat(sphere, "inputs:falloffMax", 2.5f);
    setToken(sphere, "rigExec:falloffProfile", "linear");
    setFloat(sphere, "inputs:scaleX", 1.5f);
    setFloat(sphere, "inputs:scaleY", 0.75f);
    setFloat(sphere, "inputs:scaleZ", 1.25f);
    setFloat(sphere, "inputs:scaleXNeg", 0.5f);
    setFloat(sphere, "inputs:scaleYPos", 2.0f);
    setFloat(sphere, "inputs:scaleZNeg", 1.5f);

    // The plane samples the authored points, never the ones in flight.
    const UsdPrim plane = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Plane"), TfToken("RigExecPlaneWeight"));
    place(plane, -0.5, 0, 0);
    plane.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    if (options.animatedSampler) {
        const UsdAttribute sampler =
            stage->DefinePrim(SdfPath("/Asset/Geom/Sampler"),
                              TfToken("Points"))
                .CreateAttribute(TfToken("points"),
                                 SdfValueTypeNames->Point3fArray);
        VtVec3fArray moved = rest;
        for (GfVec3f &p : moved) {
            p[0] += 1.0f;
        }
        sampler.Set(rest, UsdTimeCode(1.0));
        sampler.Set(moved, UsdTimeCode(4.0));
        plane.CreateRelationship(TfToken("rigExec:sampleSource"))
            .SetTargets({sampler.GetPath()});
    }
    setFloat(plane, "inputs:falloffMin", 0.0f);
    setFloat(plane, "inputs:falloffMax", 2.0f);
    setToken(plane, "rigExec:falloffProfile", "linear");
    setToken(plane, "rigExec:planeAxis", options.axis);
    setToken(plane, "rigExec:planeBounds", options.bounds);
    setFloat(plane, "inputs:extentU", 1.0f);
    setFloat(plane, "inputs:extentV", 0.6f);

    const UsdAttribute curvePoints =
        stage->DefinePrim(SdfPath("/Asset/Geom/CurveSource"),
                          TfToken("BasisCurves"))
            .CreateAttribute(TfToken("points"),
                             SdfValueTypeNames->Point3fArray);
    const VtVec3fArray along{GfVec3f(-1, 0.5f, 0), GfVec3f(1, 0.5f, 0)};
    if (options.animatedCurve) {
        curvePoints.Set(along, UsdTimeCode(1.0));
        curvePoints.Set(VtVec3fArray{GfVec3f(-1, 0, 0), GfVec3f(1, 0, 0)},
                        UsdTimeCode(4.0));
    } else {
        curvePoints.Set(along);
    }
    const UsdPrim curve = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Curve"), TfToken("RigExecCurveWeight"));
    place(curve, 0, 0, 0);
    preceding(curve);
    curve.CreateRelationship(TfToken("rigExec:curve"))
        .SetTargets({curvePoints.GetPath()});
    setFloat(curve, "inputs:falloffMin", 0.0f);
    setFloat(curve, "inputs:falloffMax", 1.5f);
    setToken(curve, "rigExec:falloffProfile", "linear");

    const UsdPrim combine = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Mix"), TfToken("RigExecCombineWeight"));
    combine.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    combine.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({sphere.GetPath(), plane.GetPath(), curve.GetPath()});
    setToken(combine, "rigExec:combineMode", options.mode);
    setToken(combine, "rigExec:rangePolicy", "clamp");

    const auto mover = [&](const char *path, const UsdPrim &transform,
                           const UsdPrim &weight) {
        const UsdPrim m =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecMatrixMover"));
        CHECK(m.ApplyAPI(TfToken("RigExecMoverAPI")));
        m.CreateRelationship(TfToken("rigExec:moves")).SetTargets({target});
        m.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({transform.GetPath()});
        m.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weight.GetPath()});
    };
    mover("/Asset/Rig/Movers/Second", joint, combine);
    mover("/Asset/Rig/Movers/Second/First", lift, full);
    return stage;
}

// The oracle's plane, curve, per-axis and signed sphere scales, and the
// combine modes no other stage folds several fields with, each against the
// baked program and the dynamic evaluator bit for bit.
static void
TestVolumeMixCurrentPhase()
{
    const std::vector<double> frames = {1, 2, 3, 4};
    const std::pair<const char *, const char *> cases[] = {
        {"subtract", "bounded"},
        {"min", "unbounded"},
        {"average", "bounded"},
        {"overlay", "unbounded"},
    };
    for (const auto &[mode, bounds] : cases) {
        _MixOptions options;
        options.mode = mode;
        options.bounds = bounds;
        const std::string name =
            std::string("volume mix, ") + mode + ", " + bounds + " plane";
        std::vector<std::vector<RigExecRuntimePoints>> rows;
        _TestStage(name, _VolumeMixStage(options), frames, &rows);
        _CheckFramesDiffer(name, rows);
        std::vector<std::vector<float>> fields;
        CHECK(_FieldsMatchDynamic(name, _VolumeMixStage(options), frames,
                                  "/Asset/Rig/Weights/Mix",
                                  "/Asset/Geom/M.points", &fields));
        // Not vacuous: some point lies strictly inside the field.
        size_t inside = 0;
        for (const std::vector<float> &field : fields) {
            for (float w : field) {
                inside += w > 0.0f && w < 1.0f;
            }
        }
        CHECK(inside > 0);
        std::printf("%s: %zu field(s) equal the dynamic evaluator's, %zu "
                    "weight(s) strictly inside\n", name.c_str(),
                    fields.size(), inside);
    }
}

// An authored empty rigExec:planeAxis: the oracle reads the attribute as
// authored and fails ("unknown rigExec:planeAxis "), so every current-phase
// resolve fails with the program's diagnostic, though the plane's own
// packet substitutes the fallback axis.
static void
TestEmptyPlaneAxis()
{
    const char *const name = "volume mix, empty planeAxis";
    const std::vector<double> frames = {1, 2, 3, 4};
    _MixOptions options;
    options.axis = "";
    _TestStage(name, _VolumeMixStage(options), frames);
    std::vector<std::vector<std::string>> diagnostics;
    std::string error;
    CHECK(_RunDiagnostics(_VolumeMixStage(options), frames, &diagnostics,
                          &error));
    if (!error.empty()) {
        std::printf("%s: %s\n", name, error.c_str());
    }
    const std::string failed = "current-phase weight failed: "
                               "/Asset/Rig/Weights/Plane: unknown "
                               "rigExec:planeAxis ";
    CHECK(diagnostics.size() == frames.size());
    for (const std::vector<std::string> &lines : diagnostics) {
        CHECK(_HasLine(lines, failed));
    }
    std::printf("%s: checked\n", name);
}

// The file's weight objects hold the oracle's static sample and curve
// points (geometry.weight_objects oracle_samples and oracle_curve) at the
// bake time. A current-phase field that reads animated ones still bakes --
// the binary holds them at that time only -- and the static report names
// the object and the attribute; the same stage unanimated reports
// nothing.
static void
TestAnimatedStaticPointsReported()
{
    const auto report = [&](const _MixOptions &options) {
        const UsdStageRefPtr stage = _VolumeMixStage(options);
        RigExecRigEvaluator evaluator(stage, _FindRig(stage));
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        RigExecBakeOpts opts;
        opts.time = 1.0;
        RigExecBakeResult result;
        std::string error;
        CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
        if (!error.empty()) {
            std::printf("animated static points bake: %s\n", error.c_str());
        }
        std::vector<std::pair<std::string, std::string>> listed;
        std::vector<RigExecBakeStaticEntry> entries;
        CHECK(RigExecBakeStaticReport(evaluator, &entries, &error));
        for (const RigExecBakeStaticEntry &entry : entries) {
            std::printf("  static %s: %s\n", entry.field.c_str(),
                        entry.source.c_str());
            listed.emplace_back(entry.field, entry.source);
        }
        return listed;
    };
    using _Listed = std::vector<std::pair<std::string, std::string>>;
    CHECK(report(_MixOptions()).empty());
    _MixOptions sampler;
    sampler.animatedSampler = true;
    // Each array is static twice over: the WeightPacket step gathers it,
    // and the oracle facts the current-phase field resolves with hold it.
    CHECK(report(sampler) ==
          _Listed({{"weight object /Asset/Rig/Weights/Plane gather",
                    "/Asset/Geom/Sampler.points"},
                   {"weight object /Asset/Rig/Weights/Plane oracle",
                    "/Asset/Geom/Sampler.points"}}));
    _MixOptions curve;
    curve.animatedCurve = true;
    CHECK(report(curve) ==
          _Listed({{"weight object /Asset/Rig/Weights/Curve gather",
                    "/Asset/Geom/CurveSource.points"},
                   {"weight object /Asset/Rig/Weights/Curve oracle",
                    "/Asset/Geom/CurveSource.points"}}));
    std::printf("animated sample source and curve: baked, reported\n");
}

// Open refuses an envelope index on a geometry-domain constraint (whose
// weight resolves per point on its revision), and a main revision without
// its inputs:defaultWeight read, each refused by the format's validator
// naming the field.
static void
TestComputedOpenRefusals()
{
    const UsdStageRefPtr stage = _CurrentPhaseStage();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.time = 1.0;
    RigExecBakeResult result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    const auto openError = [](const std::vector<uint8_t> &bytes) {
        std::string why;
        const std::unique_ptr<RigExecRuntimeReader> reader =
            RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &why);
        return reader ? std::string() : why;
    };
    CHECK(openError(result.bytes).empty());
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(result.bytes);
    if (!file) {
        return;
    }
    const std::vector<fb::RigExecWireConstraint> &constraints =
        file->pose->constraints;
    CHECK(constraints.size() == 1 && constraints[0].weightObjectIndex == -1);
    CHECK(!file->geometry->weightObjects.empty());
    if (constraints.size() != 1 || file->geometry->weightObjects.empty()) {
        return;
    }
    std::string got = openError(
        RigExecTestEdited(result.bytes, [](fb::RigExecWireFile *edited) {
            edited->pose->constraints[0].weightObjectIndex = 0;
        }));
    CHECK(got == "invalid .rigexec: pose.constraints[0]: "
                 "weight_object_index does not name the envelope of its "
                 "weight object");
    std::printf("envelope index on a geometry constraint: %s\n",
                got.c_str());
    CHECK(!file->geometry->chains.empty() &&
          !file->geometry->chains[0].revisions.empty());
    if (file->geometry->chains.empty() ||
        file->geometry->chains[0].revisions.empty()) {
        return;
    }
    got = openError(
        RigExecTestEdited(result.bytes, [](fb::RigExecWireFile *edited) {
            edited->geometry->chains[0].revisions[0].defaultWeight.reset();
        }));
    CHECK(got == "invalid .rigexec: geometry.chains[0].revisions[0]."
                 "default_weight: present on a derived revision or missing "
                 "on a main one");
    std::printf("main revision without its default weight: %s\n",
                got.c_str());
}

// A skin whose per-point layout holds, inside its rows, index-0 entries of
// weight +0 and -0 (which the file's sparse form drops), a zero weight on
// another index (which it keeps), and in one variant a negative weight,
// under classicLinear and dualQuaternion: the file keeps exactly the
// entries other than (0, +-0), and plays every frame bit for bit with the
// baked program.
static UsdStageRefPtr
_SparseSkinStage(const char *method, bool negative)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *path, const char *avar,
                             const std::array<double, 3> &values) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        const UsdAttribute attribute = prim.CreateAttribute(
            TfToken(avar), SdfValueTypeNames->Double);
        for (size_t f = 0; f < values.size(); ++f) {
            attribute.Set(values[f], UsdTimeCode(double(f + 1)));
        }
        return prim;
    };
    const UsdPrim a = control("/Asset/Rig/A", "avars:rz", {0.0, 35.0, -60.0});
    const UsdPrim b = control("/Asset/Rig/B", "avars:tx", {0.0, 2.5, -4.0});
    const UsdPrim c = control("/Asset/Rig/C", "avars:ry", {10.0, -25.0, 80.0});
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/Mesh"), TfToken("Mesh"));
    mesh.GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(1, 0, 0), GfVec3f(0, 2, 0.5f),
                          GfVec3f(-1, 1, 2), GfVec3f(3, -1, 1)});
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Skin"),
                                           TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/Mesh.points")});
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({a.GetPath(), b.GetPath(), c.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int)
        .Set(4);
    skin.CreateAttribute(TfToken("rigExec:skinningMethod"),
                         SdfValueTypeNames->Token)
        .Set(TfToken(method));
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray)
        .Set(VtIntArray{0, 1, 0, 2,   //
                        2, 1, 0, 0,   //
                        1, 0, 2, 0,   //
                        0, 0, 1, 2});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{0.0f, 0.5f, -0.0f, 0.5f,                    //
                          0.0f, 1.0f, 0.0f, -0.0f,                    //
                          negative ? -0.25f : 0.25f, 0.75f, 0.0f, 0.0f,  //
                          -0.0f, 0.0f, 0.6f, 0.4f});
    return stage;
}

static void
TestSparseSkinTopology()
{
    for (const char *method : {"classicLinear", "dualQuaternion"}) {
        for (const bool negative : {false, true}) {
            const std::string name = std::string("sparse skin topology, ") +
                                     method +
                                     (negative ? ", negative weight" : "");
            const UsdStageRefPtr stage = _SparseSkinStage(method, negative);
            std::vector<std::vector<RigExecRuntimePoints>> rows;
            _TestStage(name, stage, {1.0, 2.0, 3.0}, &rows);
            if (!negative) {
                _CheckFramesDiffer(name, rows);
            }
            // The file's layout: the 16 entries less the seven (0, +-0)
            // ones, the zero on index 2 kept. A negative weight fails the
            // layout's validation, so the bake keeps no epoch layout and
            // both sides read the per-frame arrays.
            RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
            evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
            std::vector<uint8_t> bytes;
            std::string error;
            CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
            const std::unique_ptr<fb::RigExecWireFile> file =
                RigExecTestUnpack(bytes);
            const fb::RigExecWireSkinTopology *topology =
                file && !file->geometry->chains.empty() &&
                        !file->geometry->chains[0].revisions.empty()
                    ? file->geometry->chains[0].revisions[0].topology.get()
                    : nullptr;
            CHECK(file && (topology != nullptr) == !negative);
            if (!topology) {
                continue;
            }
            CHECK(topology->elementSize == 4 && topology->pointCount == 4);
            CHECK(topology->validated);
            CHECK(topology->counts8 ==
                  (std::vector<uint8_t>{2, 2, 3, 2}));
            CHECK(topology->indexWidth == 1);
            CHECK(topology->indices8 ==
                  (std::vector<uint8_t>{1, 2, 2, 1, 1, 0, 2, 1, 2}));
            CHECK(topology->weights ==
                  (std::vector<float>{0.5f, 0.5f, 0.0f, 1.0f, 0.25f, 0.75f,
                                      0.0f, 0.6f, 0.4f}));
        }
    }
    std::printf("sparse skin topology: checked\n");
}

// The C runtime's environment, the one std::getenv reads (TfSetenv on
// Windows writes the process block only). Empty clears \p name.
static std::string
_GetEnv(const char *name)
{
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char *value = std::getenv(name);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
    return value ? value : "";
}

static void
_SetEnv(const char *name, const std::string &value)
{
#ifdef _WIN32
    _putenv_s(name, value.c_str());
#else
    if (value.empty()) {
        unsetenv(name);
    } else {
        setenv(name, value.c_str(), 1);
    }
#endif
}

// Three point sets over two controls: Shift translates by whole numbers,
// Turn rotates. Lift is a matrix mover by Shift at weights 0 and 1 alone;
// Bend one by Turn at interior weights too; Skin a classicLinear skin over
// both, three of its rows on Shift alone.
static UsdStageRefPtr
_SimdSettingStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));
    const auto control = [&](const char *path,
                             const std::vector<std::pair<const char *,
                                                         double>> &avars) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("rest:space"),
                             SdfValueTypeNames->Matrix4d)
            .Set(GfMatrix4d(1.0));
        for (const auto &[avar, value] : avars) {
            prim.CreateAttribute(TfToken(avar), SdfValueTypeNames->Double)
                .Set(value);
        }
        return prim.GetPath();
    };
    const SdfPath shift = control(
        "/Asset/Rig/Controls/Shift",
        {{"avars:tx", 3.0}, {"avars:ty", -2.0}, {"avars:tz", 5.0}});
    const SdfPath turn = control(
        "/Asset/Rig/Controls/Turn",
        {{"avars:rz", 33.0}, {"avars:tx", 1.5}, {"avars:ty", -0.5}});

    // Dyadic coordinates, so a whole-number translation is exact in float
    // and lands on no zero.
    const VtVec3fArray rest = {
        GfVec3f(1.5f, 2.25f, -3.5f),   GfVec3f(-2.5f, 0.75f, 4.25f),
        GfVec3f(3.25f, -1.5f, 0.5f),   GfVec3f(-0.25f, 5.5f, -2.75f),
        GfVec3f(4.5f, -3.25f, 1.25f),  GfVec3f(-5.75f, 1.5f, 2.5f),
        GfVec3f(0.5f, -4.5f, -1.75f),  GfVec3f(2.75f, 3.5f, -0.25f)};
    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const auto points = [&](const char *path) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("Points"));
        prim.CreateAttribute(TfToken("points"),
                             SdfValueTypeNames->Point3fArray)
            .Set(rest);
        return prim.GetPath().AppendProperty(TfToken("points"));
    };
    const SdfPath lift = points("/Asset/Geom/Lift");
    const SdfPath bend = points("/Asset/Geom/Bend");
    const SdfPath skinned = points("/Asset/Geom/Skin");

    stage->DefinePrim(SdfPath("/Asset/Rig/Weights"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto matrixMover = [&](const char *name, const SdfPath &target,
                                 const SdfPath &transform,
                                 const VtFloatArray &values) {
        const SdfPath weightPath =
            SdfPath("/Asset/Rig/Weights").AppendChild(TfToken(name));
        const UsdPrim weight =
            stage->DefinePrim(weightPath, TfToken("RigExecStaticWeight"));
        weight.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({target});
        weight.CreateAttribute(TfToken("rigExec:representation"),
                               SdfValueTypeNames->Token)
            .Set(TfToken("dense"));
        weight.CreateAttribute(TfToken("rigExec:values"),
                               SdfValueTypeNames->FloatArray)
            .Set(values);
        weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                               SdfValueTypeNames->Float)
            .Set(0.0f);
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers").AppendChild(TfToken(name)),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({target});
        mover.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({transform});
        mover.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weightPath});
    };
    matrixMover("Lift", lift, shift, {0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f,
                                      0.0f, 1.0f});
    matrixMover("Bend", bend, turn, {0.0f, 1.0f, 0.3f, 0.5f, 1.0f, 0.0f,
                                     0.7f, 0.15f});

    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves")).SetTargets({skinned});
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({shift, turn});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int)
        .Set(2);
    skin.CreateAttribute(TfToken("rigExec:skinningMethod"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("classicLinear"));
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray)
        .Set(VtIntArray{0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1.0f, 0.0f,  0.5f, 0.5f,   //
                          1.0f, 0.0f,  0.25f, 0.75f, //
                          0.0f, 1.0f,  1.0f, 0.0f,   //
                          0.6f, 0.4f,  0.9f, 0.1f});
    return stage;
}

// RIGEXEC_ENABLE_SIMD is read at Open, once per reader: two readers of one
// file, opened with the switch off and then on, keep their own setting
// whatever the environment says when they run. The SSE2 kernels match the
// scalar ones within 1e-6 x extent and exactly where the arithmetic is: a
// weight-0 point passes through, and a weight-1 point under a whole-number
// translation lands on the same float.
static void
TestSimdSettingPerReader()
{
    const char *const name = "SIMD setting per reader";
    const char *const variable = "RIGEXEC_ENABLE_SIMD";
    const UsdStageRefPtr stage = _SimdSettingStage();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<uint8_t> bytes;
    std::string error;
    // Baked before the switch moves, so the evaluator's own reading of it
    // is the suite's.
    const bool baked = RigExecTestBakeAt(evaluator, 0.0, &bytes, &error);
    CHECK(baked);
    if (!baked) {
        std::printf("%s: bake: %s\n", name, error.c_str());
        return;
    }
    const std::string original = _GetEnv(variable);
    _SetEnv(variable, "0");
    const std::unique_ptr<RigExecRuntimeReader> scalar =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    _SetEnv(variable, "1");
    const std::unique_ptr<RigExecRuntimeReader> simd =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(scalar && simd);
    if (!scalar || !simd) {
        _SetEnv(variable, original);
        std::printf("%s: open: %s\n", name, error.c_str());
        return;
    }

    // Lift: rows exact at every point. Bend: rows exact where the weight
    // is 0. Skin: rows exact where Shift alone weighs.
    struct Target {
        const char *path;
        std::vector<bool> exact;
    };
    const std::vector<Target> targets = {
        {"/Asset/Geom/Lift.points", std::vector<bool>(8, true)},
        {"/Asset/Geom/Bend.points",
         {true, false, false, false, false, true, false, false}},
        {"/Asset/Geom/Skin.points",
         {true, false, true, false, false, true, false, false}}};
    size_t differing = 0;
    const auto compare = [&](const char *what, const char *environment) {
        _SetEnv(variable, environment);
        CHECK(!scalar->GetSimdEnabledForTesting());
        CHECK(simd->GetSimdEnabledForTesting());
        const bool ranScalar = scalar->Execute(&error);
        CHECK(ranScalar);
        const bool ranSimd = simd->Execute(&error);
        CHECK(ranSimd);
        if (!ranScalar || !ranSimd) {
            std::printf("%s, %s: execute: %s\n", name, what, error.c_str());
            return;
        }
        for (const Target &target : targets) {
            const RigExecRuntimePoints *a = _FindPoints(*scalar, target.path);
            const RigExecRuntimePoints *b = _FindPoints(*simd, target.path);
            CHECK(a && b && a->points.size() == target.exact.size() &&
                  b->points.size() == target.exact.size());
            if (!a || !b || a->points.size() != target.exact.size() ||
                b->points.size() != target.exact.size()) {
                continue;
            }
            double extent = 1.0;
            for (const RrVec3f &p : b->points) {
                for (int k = 0; k < 3; ++k) {
                    extent = std::max(extent, std::abs(double(p[k])));
                }
            }
            for (size_t i = 0; i < target.exact.size(); ++i) {
                const RrVec3f &x = a->points[i];
                const RrVec3f &y = b->points[i];
                const bool same =
                    std::memcmp(&x, &y, sizeof(RrVec3f)) == 0;
                differing += same ? 0 : 1;
                double distance = 0.0;
                for (int k = 0; k < 3; ++k) {
                    const double d = double(x[k]) - double(y[k]);
                    distance += d * d;
                }
                const bool close = std::sqrt(distance) <= 1e-6 * extent;
                if ((target.exact[i] && !same) || !close) {
                    ++failures;
                    std::printf("FAIL %s, %s: %s point %zu is (%.9g %.9g "
                                "%.9g) scalar, (%.9g %.9g %.9g) SIMD\n",
                                name, what, target.path, i, x[0], x[1],
                                x[2], y[0], y[1], y[2]);
                }
            }
        }
    };
    // The switch flipped against each reader before every run.
    compare("defaults", "1");
    const std::vector<std::pair<const char *, double>> drags = {
        {"/Asset/Rig/Controls/Shift.avars:tx", -6.0},
        {"/Asset/Rig/Controls/Shift.avars:ty", 7.0},
        {"/Asset/Rig/Controls/Shift.avars:tz", -1.0},
        {"/Asset/Rig/Controls/Turn.avars:rz", -57.0},
        {"/Asset/Rig/Controls/Turn.avars:tx", 2.0}};
    for (const auto &[input, value] : drags) {
        CHECK(scalar->SetInput(input, value, &error));
        CHECK(simd->SetInput(input, value, &error));
    }
    compare("dragged", "0");
    _SetEnv(variable, original);
    std::printf("%s: checked (%zu point(s) differ within the tolerance)\n",
                name, differing);
}

int
main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestComputedCurrentPhase();
    TestGeometryDomainArm();
    TestCurrentPhaseThroughCombine();
    TestCurrentPhaseFailure();
    TestVolumeMixCurrentPhase();
    TestEmptyPlaneAxis();
    TestAnimatedStaticPointsReported();
    TestComputedOpenRefusals();
    TestSparseSkinTopology();
    TestPathReadsFixture();
    TestPoseDrivenBlendWeights();
    TestBlendActivationDrag();
    TestBlendShapeLayouts();

    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }
    const std::string only = argc > 2 ? argv[2] : "";
    bool sawBaking = false;
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            continue;
        }
        if (!only.empty() &&
            std::string(fixture.stage).find(only) == std::string::npos) {
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
    // Last: it moves RIGEXEC_ENABLE_SIMD, which the evaluator reads once.
    TestSimdSettingPerReader();

    if (failures == 0) {
        std::printf("testRigExecRuntimeGeometry: all tests passed "
                    "(%d stage(s) compared over %d frame(s))\n",
                    comparedFixtures, comparedFrames);
        return 0;
    }
    std::printf("testRigExecRuntimeGeometry: %d failures "
                "(%d stage(s) compared over %d frame(s))\n", failures,
                comparedFixtures, comparedFrames);
    return 1;
}
