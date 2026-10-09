// rigExecRuntime geometry outputs: native program vs runtime
// over every baking fixture, comparing published chain points bit for
// bit. Each stage is baked at one time and the binary is played through
// its inputs: the input sampler hands it the stage's animated inputs at
// each frame, and a drag is an input set to an authored value, held to the
// evaluator with the same value authored in the session layer.
#include "rigExecBake/bake.h"
#include "rigExecBake/staticReport.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/format.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecRuntime/store.h"
#include "rigExecRuntime/stageArrayInputs.h"
#include "rigExecSampler/inputSampler.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <set>

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

// Bakes \p stage at the first of \p frames and plays the file through the
// input sampler beside a fresh baked evaluator, frame by frame: the moved
// points, ordered diagnostics and actual operation counts,
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

        // Ordered semantic diagnostics remain part of the native/runtime check.
        const auto &diagnostics = reader->GetDiagnostics();
        const auto &wantDiags = pose.diagnostics;
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

        // Accounting names actual shared operations and completion trace.
        const auto trace = reader->GetLastRunTraceForTesting();
        CHECK(reader->GetCounters().executedOpCount == trace.size());
        const std::set<int32_t> unique(trace.begin(), trace.end());
        CHECK(unique.size() == trace.size());
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
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, frames.front(), &bytes, &error));
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    const std::string step = "/PathReadAsset/Rig/Movers/Mush.inputs:step";
    const std::string held = "/PathReadAsset/Rig/Dials.held";
    // The scalar read rows are exactly those sites, each reading its head
    // when its walk yields nothing; the two that end on the valueless
    // double walk two hops, and their head holds the float the site reads.
    // (The array read rows read the input slots of the arrays.)
    size_t rows = 0;
    size_t twoHops = 0;
    for (const fb::RigExecWirePathRead &row : file->geometry->pathReads) {
        if (!row.read || RigExecFormatIsArrayTag(row.read->tag)) {
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
// agrees with the native evaluator): before, while and after the set, and
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
            stage, rigPath,
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
// native evaluator. At every frame the runtime's published field for
// \p weightObject -- the current-phase field its oracle measured against
// the points entering the revision -- must equal the native evaluator's
// weightFields entry bit for bit, and so must the points at \p pointsPath.
// \p fields receives the native fields, one per frame. False on any
// difference.
static bool
_FieldsMatchNative(const std::string &name, const UsdStageRefPtr &stage,
                    const std::vector<double> &frames,
                    const char *weightObject, const char *pointsPath,
                    std::vector<std::vector<float>> *fields)
{
    const SdfPath rigPath = _FindRig(stage);
    RigExecRigEvaluator baked(stage, rigPath);
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
    RigExecRigEvaluator native(stage, rigPath);

    bool same = true;
    for (double frame : frames) {
        const RigExecRigPose want = native.Evaluate(UsdTimeCode(frame));
        if (!want.valid) {
            std::printf("%s frame %g: native pose invalid\n", name.c_str(),
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
            std::printf("%s frame %g: no field for %s (native %d, runtime "
                        "%d)\n", name.c_str(), frame, weightObject,
                        int(wantField != want.weightFields.end()),
                        int(gotField != nullptr));
            return false;
        }
        const auto &w = wantField->second.weights;
        if (gotField->weights.size() != w.size() ||
            (!w.empty() &&
             std::memcmp(gotField->weights.data(), w.data(),
                         w.size() * sizeof(float)) != 0)) {
            std::printf("%s frame %g: the runtime's field for %s differs "
                        "from the native evaluator's\n", name.c_str(),
                        frame, weightObject);
            for (size_t i = 0; i < gotField->weights.size() || i < w.size();
                 ++i) {
                std::printf("  [%zu] runtime %.9g, native %.9g\n", i,
                            i < gotField->weights.size()
                                ? double(gotField->weights[i]) : -1.0,
                            i < w.size() ? double(w[i]) : -1.0);
            }
            same = false;
        }
        fields->emplace_back(w.begin(), w.end());
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
    {
        std::vector<RigExecRigPose> poses;
        CHECK(RigExecTestEditedPoses(stage, rigPath,
                                     {{SdfPath(falloffMax), VtValue(4.0f)}},
                                     {bakeTime}, &poses, &error));
        std::vector<std::string> diffs;
        const bool same =
            poses.size() == 1 &&
            RigExecCompareRuntimeOutputs(poses[0], player.Reader(), &diffs);
        CHECK(same);
        std::printf("%s, %s: %s\n", name,
                    "native",
                    same ? "binary == session edit" : "MISMATCH");
        for (const std::string &line : diffs) {
            std::printf("    %s\n", line.c_str());
        }
    }
}

// Three matrix movers on one points attribute: M0 on a control that moves at
// frames 1 to 3, M1 and M2 on one that stands still. \p firstWeight is M0's
// inputs:defaultWeight (zero leaves the points it moves where they
// entered); \p failMiddle puts M1's weight out of range at frame 2 only, so
// M1 passes through there and applies again at frame 3.
static UsdStageRefPtr
_StackedChainStage(float firstWeight, bool failMiddle)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim moving = stage->DefinePrim(
        SdfPath("/Asset/Rig/Moving"), TfToken("RigExecControl"));
    for (int frame = 1; frame <= 3; ++frame) {
        moving.GetAttribute(TfToken("avars:tx"))
            .Set(double(frame), UsdTimeCode(frame));
    }
    const UsdPrim still = stage->DefinePrim(
        SdfPath("/Asset/Rig/Still"), TfToken("RigExecControl"));
    still.GetAttribute(TfToken("avars:ty")).Set(1.0);
    const SdfPath target("/Asset/Shape.points");
    const UsdPrim shape =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Points"));
    shape.GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(0, 2, 0)});
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    for (int i = 0; i < 3; ++i) {
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M" + std::to_string(i)),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
        mover.GetRelationship(TfToken("rigExec:transform"))
            .SetTargets({i == 0 ? moving.GetPath() : still.GetPath()});
        UsdAttribute weight =
            mover.GetAttribute(TfToken("inputs:defaultWeight"));
        if (i == 0) {
            weight.Set(firstWeight);
        } else if (i == 1 && failMiddle) {
            weight.Set(1.0f, UsdTimeCode(1.0));
            weight.Set(2.0f, UsdTimeCode(2.0));
            weight.Set(1.0f, UsdTimeCode(3.0));
        } else {
            weight.Set(1.0f);
        }
    }
    return stage;
}

// The fuse flips its buffers rather than copying, and the chain's points key
// by content version: played against the native program -- points,
// diagnostics and operation counts bit for bit -- with
// RIGEXEC_VERIFY_CHAIN_VERSIONS on, read at Open, which fails any run whose
// versions and points' bytes disagree on a change. A revision that passes
// through and recovers, and an edit that leaves points where they were.
// RIGEXEC_VERIFY_EPILOGUE_LISTS fails a run whose epilogue, published from
// the steps holding lines, differs from a sweep of every step.
static void
TestChainPointVersions()
{
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "1");
    TfSetenv("RIGEXEC_VERIFY_EPILOGUE_LISTS", "1");
    _TestStage("chain revision passes through and recovers",
               _StackedChainStage(1.0f, true), {1, 2, 3, 2, 1, 3});
    _TestStage("chain revision keeps its unmoved points",
               _StackedChainStage(0.0f, false), {1, 2, 3, 3, 1});
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0");
    TfSetenv("RIGEXEC_VERIFY_EPILOGUE_LISTS", "0");
}

// One matrix mover on a control that moves every frame, weighted by a
// sparse RigExecDynamicWeight (clamp) over a sparse static base naming all
// three points with weight 1 and defaulting to 0.25. inputs:driver is 2 at
// frames 1 and 2, 3 at frame 3 and 0.5 at frame 4: the weight packet stands
// at frame 2, moves only its default at frame 3 (no point reads it, every
// value clamps to 1) and moves its values at frame 4. With \p unnamedLast
// the base and the weight name only points 0 and 1, so point 2 reads the
// default: at frame 3 the values and indices stay byte-equal ({1, 1}) while
// the field's last entry goes 0.5 -> 0.75.
static UsdStageRefPtr
_WeightOverlayStage(bool unnamedLast = false)
{
    const VtIntArray named =
        unnamedLast ? VtIntArray{0, 1} : VtIntArray{0, 1, 2};
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim moving = stage->DefinePrim(
        SdfPath("/Asset/Rig/Moving"), TfToken("RigExecControl"));
    for (int frame = 1; frame <= 4; ++frame) {
        moving.GetAttribute(TfToken("avars:tx"))
            .Set(double(frame), UsdTimeCode(frame));
    }
    const SdfPath target("/Asset/Shape.points");
    const UsdPrim shape =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Points"));
    shape.GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(0, 2, 0)});
    const UsdPrim base = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Base"), TfToken("RigExecStaticWeight"));
    base.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({target});
    base.CreateAttribute(TfToken("rigExec:representation"),
                         SdfValueTypeNames->Token, false)
        .Set(TfToken("sparse"));
    base.CreateAttribute(TfToken("rigExec:indices"),
                         SdfValueTypeNames->IntArray, false)
        .Set(named);
    base.CreateAttribute(TfToken("rigExec:values"),
                         SdfValueTypeNames->FloatArray, false)
        .Set(VtFloatArray(named.size(), 1.0f));
    base.CreateAttribute(TfToken("rigExec:defaultWeight"),
                         SdfValueTypeNames->Float, false)
        .Set(0.25f);
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/W"), TfToken("RigExecDynamicWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({target});
    weight.CreateRelationship(TfToken("rigExec:baseWeight"), false)
        .SetTargets({base.GetPath()});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("sparse"));
    weight.CreateAttribute(TfToken("rigExec:rangePolicy"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("clamp"));
    weight.CreateAttribute(TfToken("rigExec:indices"),
                           SdfValueTypeNames->IntArray, false)
        .Set(named);
    UsdAttribute driver = weight.CreateAttribute(
        TfToken("inputs:driver"), SdfValueTypeNames->Float, false);
    driver.Set(2.0f, UsdTimeCode(1.0));
    driver.Set(2.0f, UsdTimeCode(2.0));
    driver.Set(3.0f, UsdTimeCode(3.0));
    driver.Set(0.5f, UsdTimeCode(4.0));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/M0"), TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    mover.GetRelationship(TfToken("rigExec:transform"))
        .SetTargets({moving.GetPath()});
    mover.CreateRelationship(TfToken("rigExec:weightObject"), false)
        .SetTargets({weight.GetPath()});
    return stage;
}

// RevisionStatic reuses its weight field while the packet it was resolved
// from stands, and keys it and the envelope by content version: played
// against the native program, points, fields, diagnostics and operation
// counts bit for bit, with RIGEXEC_VERIFY_PACKET_VERSIONS on, read at Open,
// which fails any run whose versions and arrays' bytes disagree on a change.
static void
TestPacketArrayVersions()
{
    TfSetenv("RIGEXEC_VERIFY_PACKET_VERSIONS", "1");
    const char *const name = "weight field reused while its packet stands";
    const std::vector<double> frames = {1, 2, 3, 4, 1, 3};
    _TestStage(name, _WeightOverlayStage(), frames);
    std::vector<std::vector<float>> fields;
    CHECK(_FieldsMatchNative(name, _WeightOverlayStage(), frames,
                              "/Asset/Rig/Weights/W", "/Asset/Shape.points",
                              &fields));
    TfSetenv("RIGEXEC_VERIFY_PACKET_VERSIONS", "0");
    CHECK(fields.size() == frames.size());
    if (fields.size() == frames.size()) {
        const std::vector<float> full(3, 1.0f), half(3, 0.5f);
        CHECK(fields[0] == full && fields[1] == full && fields[2] == full);
        CHECK(fields[3] == half && fields[4] == full && fields[5] == full);
    }

    // Only the default moves at frame 3, and point 2 reads it: the arrays
    // the reuse is keyed by stay byte-equal, so a reuse that ignored the
    // rest of the packet would hold 0.5 where the field reads 0.75.
    TfSetenv("RIGEXEC_VERIFY_PACKET_VERSIONS", "1");
    const char *const defaultName = "weight field follows its packet's default";
    _TestStage(defaultName, _WeightOverlayStage(true), frames);
    std::vector<std::vector<float>> defaultFields;
    CHECK(_FieldsMatchNative(defaultName, _WeightOverlayStage(true), frames,
                              "/Asset/Rig/Weights/W", "/Asset/Shape.points",
                              &defaultFields));
    TfSetenv("RIGEXEC_VERIFY_PACKET_VERSIONS", "0");
    CHECK(defaultFields.size() == frames.size());
    if (defaultFields.size() == frames.size()) {
        const std::vector<float> two{1.0f, 1.0f, 0.5f},
            three{1.0f, 1.0f, 0.75f}, half{0.5f, 0.5f, 0.125f};
        CHECK(defaultFields[0] == two && defaultFields[1] == two &&
              defaultFields[2] == three);
        CHECK(defaultFields[3] == half && defaultFields[4] == two &&
              defaultFields[5] == three);
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
    CHECK(_FieldsMatchNative(name, _GeometryDomainArmStage(), frames,
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
    std::printf("%s: %zu field(s) equal the native evaluator's\n", name,
                fields.size());

    // The same arm with inputs:falloffMax halved by a float math mover: the
    // step-backed read's walk crosses the chain's target, so the oracle
    // reads 4 where 8 is authored (3 where 6 is).
    const char *const chained = "geometry-domain arm through a chain";
    rows.clear();
    _TestStage(chained, _GeometryDomainArmStage(true), frames, &rows);
    _CheckFramesDiffer(chained, rows);
    std::vector<std::vector<float>> narrowed;
    CHECK(_FieldsMatchNative(chained, _GeometryDomainArmStage(true), frames,
                              "/Asset/Rig/Weights/Sphere",
                              "/Asset/Geom/Slab.points", &narrowed));
    CHECK(narrowed.size() == frames.size());
    if (narrowed.size() == frames.size()) {
        CHECK(narrowed[0] == (std::vector<float>{1.0f, 0.0f, 0.0f}));
        CHECK(narrowed[2] == (std::vector<float>{0.0f, 0.0f, 1.0f}));
    }
    std::printf("%s: %zu field(s) equal the native evaluator's\n", chained,
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
    CHECK(_FieldsMatchNative(name, _CurrentPhaseThroughCombineStage(),
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
    std::printf("%s: %zu field(s) equal the native evaluator's\n", name,
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

struct _RangeChainOptions {
    // The base samples differ only in points [0, 100), and Moving stands
    // still.
    bool baseMovesInRangeZero = false;
    // M0 is weighted by a sparse static weight over points [0, 100).
    bool sparseFirst = false;
    // M1's inputs:defaultWeight leaves [0, 1] at frame 2 only.
    bool failMiddle = false;
    // The lattice's cage moves at frames 1 to 3.
    bool cageMoves = true;
};

// A range-pipelined chain: matrix movers M0 (on Moving), M1 and M2 (on
// Still) and a lattice blended in at 0.6 move the \p count points of
// /Asset/Shape.points, a box inside the 2 x 2 x 3 cage; above the default
// 4096 points a range (RIGEXEC_BAKED_CHUNK_VERTS) all four are cut into
// ranges, three for 10000 points. A matrix mover on the three points of
// /Asset/Small.points stays whole. The points hold a default, which Build
// counts, and samples at frames 1 to 3 that equal it unless
// \p options.baseMovesInRangeZero.
static UsdStageRefPtr
_RangeChainStage(size_t count, const _RangeChainOptions &options)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim moving = stage->DefinePrim(
        SdfPath("/Asset/Rig/Moving"), TfToken("RigExecControl"));
    for (int frame = 1; frame <= 3; ++frame) {
        moving.GetAttribute(TfToken("avars:tx"))
            .Set(options.baseMovesInRangeZero ? 1.0 : double(frame),
                 UsdTimeCode(frame));
    }
    const UsdPrim still = stage->DefinePrim(
        SdfPath("/Asset/Rig/Still"), TfToken("RigExecControl"));
    still.GetAttribute(TfToken("avars:ty")).Set(1.0);

    const SdfPath target("/Asset/Shape.points");
    VtVec3fArray points(count);
    for (size_t i = 0; i < count; ++i) {
        points[i] = GfVec3f(-0.9f + 0.075f * float(i % 25),
                            -0.9f + 0.09f * float((i / 25) % 20),
                            0.1f + 0.2f * float((i / 500) % 20));
    }
    UsdAttribute shapePoints =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Points"))
            .GetAttribute(TfToken("points"));
    shapePoints.Set(points);
    for (int frame = 1; frame <= 3; ++frame) {
        VtVec3fArray sample = points;
        if (options.baseMovesInRangeZero) {
            for (size_t i = 0; i < 100 && i < count; ++i) {
                sample[i][2] += 0.125f * float(frame);
            }
        }
        shapePoints.Set(sample, UsdTimeCode(frame));
    }

    // The cage at rest is its default; its middle layer widens per frame.
    const VtVec3fArray rest = {
        GfVec3f(-1, -1, 0), GfVec3f(1, -1, 0), GfVec3f(-1, 1, 0),
        GfVec3f(1, 1, 0),   GfVec3f(-1, -1, 2), GfVec3f(1, -1, 2),
        GfVec3f(-1, 1, 2),  GfVec3f(1, 1, 2),   GfVec3f(-1, -1, 4),
        GfVec3f(1, -1, 4),  GfVec3f(-1, 1, 4),  GfVec3f(1, 1, 4)};
    const UsdPrim cage =
        stage->DefinePrim(SdfPath("/Asset/Cage"), TfToken("Points"));
    UsdAttribute cagePoints = cage.GetAttribute(TfToken("points"));
    cagePoints.Set(rest);
    for (int frame = 1; frame <= 3; ++frame) {
        VtVec3fArray posed = rest;
        if (options.cageMoves) {
            for (size_t k = 4; k < 8; ++k) {
                posed[k][0] *= 1.0f + 0.25f * float(frame);
                posed[k][1] *= 1.0f + 0.25f * float(frame);
            }
        }
        cagePoints.Set(posed, UsdTimeCode(frame));
    }

    const SdfPath small("/Asset/Small.points");
    stage->DefinePrim(small.GetPrimPath(), TfToken("Points"))
        .GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(0, 2, 0)});

    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto matrixMover = [&](const std::string &name,
                                 const SdfPath &moves,
                                 const UsdPrim &control) {
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/" + name),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({moves});
        mover.GetRelationship(TfToken("rigExec:transform"))
            .SetTargets({control.GetPath()});
        return mover;
    };
    const UsdPrim m0 = matrixMover("M0", target, moving);
    if (options.sparseFirst) {
        const UsdPrim weight = stage->DefinePrim(
            SdfPath("/Asset/Rig/Weights/Face"),
            TfToken("RigExecStaticWeight"));
        weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
            .SetTargets({target});
        weight.CreateAttribute(TfToken("rigExec:representation"),
                               SdfValueTypeNames->Token, false)
            .Set(TfToken("sparse"));
        VtIntArray indices(100);
        VtFloatArray values(100);
        for (int i = 0; i < 100; ++i) {
            indices[i] = i;
            values[i] = 0.25f + 0.0075f * float(i);
        }
        weight.CreateAttribute(TfToken("rigExec:indices"),
                               SdfValueTypeNames->IntArray, false)
            .Set(indices);
        weight.CreateAttribute(TfToken("rigExec:values"),
                               SdfValueTypeNames->FloatArray, false)
            .Set(values);
        weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                               SdfValueTypeNames->Float, false)
            .Set(0.0f);
        m0.CreateRelationship(TfToken("rigExec:weightObject"), false)
            .SetTargets({weight.GetPath()});
    } else {
        m0.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    }
    UsdAttribute m1Weight = matrixMover("M1", target, still)
                                .GetAttribute(TfToken("inputs:defaultWeight"));
    if (options.failMiddle) {
        m1Weight.Set(1.0f, UsdTimeCode(1.0));
        m1Weight.Set(2.0f, UsdTimeCode(2.0));
        m1Weight.Set(1.0f, UsdTimeCode(3.0));
    } else {
        m1Weight.Set(1.0f);
    }
    matrixMover("M2", target, still)
        .GetAttribute(TfToken("inputs:defaultWeight"))
        .Set(1.0f);
    matrixMover("Small", small, still)
        .GetAttribute(TfToken("inputs:defaultWeight"))
        .Set(1.0f);

    const UsdPrim lattice = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Lattice"), TfToken("RigExecLatticeMover"));
    lattice.ApplyAPI(TfToken("RigExecMoverAPI"));
    lattice.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    lattice.GetRelationship(TfToken("rigExec:cage"))
        .SetTargets({cage.GetPath()});
    lattice.GetAttribute(TfToken("rigExec:basis")).Set(TfToken("bernstein"));
    lattice.GetAttribute(TfToken("rigExec:divisions")).Set(GfVec3i(2, 2, 3));
    lattice.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.6f);
    return stage;
}

// Range-pipelined chains play as the native program runs them -- points,
// ordered diagnostics and operation counts bit for bit -- with
// RIGEXEC_VERIFY_CHAIN_VERSIONS on, read at Open, which fails any run whose
// range or join versions and their points' bytes disagree on a change: a
// revision that passes through at frame 2 and recovers, base points that
// move in range 0 alone, and a sparse first mover whose other ranges pass
// through.
static void
TestRangeChainPlays()
{
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "1");
    _RangeChainOptions failing;
    failing.failMiddle = true;
    const std::vector<double> frames = {1, 2, 3, 2, 1, 3};
    _TestStage("range chain passes through and recovers",
               _RangeChainStage(10000, failing), frames);
    _RangeChainOptions base;
    base.baseMovesInRangeZero = true;
    _TestStage("range chain base moves in range 0",
               _RangeChainStage(10000, base), {1, 2, 3, 2, 1});
    _RangeChainOptions sparse;
    sparse.sparseFirst = true;
    _TestStage("range chain sparse first mover",
               _RangeChainStage(10000, sparse), {1, 2, 3, 1});
    TfSetenv("RIGEXEC_VERIFY_CHAIN_VERSIONS", "0");

    // M1's refusal is reported at frame 2 and nowhere else.
    std::vector<std::vector<std::string>> diagnostics;
    std::string error;
    CHECK(_RunDiagnostics(_RangeChainStage(10000, failing), frames,
                          &diagnostics, &error));
    if (!error.empty()) {
        std::printf("range chain diagnostics: %s\n", error.c_str());
    }
    const std::string failed = "MoverFailed /Asset/Rig/Movers/M1: execution "
                               "rejected its inputs; revision passed through";
    CHECK(diagnostics.size() == frames.size());
    for (size_t i = 0; i < diagnostics.size() && i < frames.size(); ++i) {
        CHECK(_HasLine(diagnostics[i], failed) == (frames[i] == 2.0));
    }
}

// Open sets the range role on exactly the file's range-pipelined revisions
// (unchunked, two or more chunks), each its own point source; and the
// ranges cut off apart. M0's sparse weight moves points in range 0 alone, so
// at frame 2 every range of M0 runs and every revision after it in the chain
// runs its range 0 alone, while nothing before it runs.
static void
TestRangeChainRolesAndCutoff()
{
    const char *const name = "range chain roles and cutoff";
    _RangeChainOptions options;
    options.sparseFirst = true;
    options.cageMoves = false;
    const UsdStageRefPtr stage = _RangeChainStage(10000, options);
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator baker(stage, SdfPath("/Asset/Rig"));
        CHECK(RigExecTestBakeAt(baker, 1.0, &bytes, &error));
    }
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    RigExecTestPlayer player;
    const bool opened = file && file->geometry && player.Open(bytes, stage, &error);
    CHECK(opened);
    if (!opened) {
        std::printf("%s: FAILED (%s)\n", name, error.c_str());
        return;
    }
    const fb::RigExecWireDomainGeometry &geometry = *file->geometry;
    // Each cut revision's mover, by its position in its chain.
    std::map<std::string, size_t> position;
    size_t ranges = 0, wholes = 0;
    for (const fb::RigExecWireChain &chain : geometry.chains) {
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const fb::RigExecWireRevision &revision = chain.revisions[r];
            const std::string mover =
                RigExecFormatPathText(*file, revision.moverPath);
            const bool expected =
                !revision.chunked && revision.chunks.size() >= 2;
            bool role = !expected, own = !expected;
            CHECK(player->GetRangeRoleForTesting(mover, &role, &own));
            CHECK(role == expected && own == expected);
            if (expected) {
                ++ranges;
                CHECK(revision.chunks.size() == 3);
                position[mover] = r;
            } else {
                ++wholes;
            }
        }
    }
    // M0, M1, M2 and the lattice are cut; Small's mover is whole.
    CHECK(ranges == 4 && wholes == 1);
    const auto first = position.find("/Asset/Rig/Movers/M0");
    CHECK(first != position.end());
    if (ranges != 4 || first == position.end()) {
        std::printf("%s: FAILED (%zu cut revision(s))\n", name, ranges);
        return;
    }

    CHECK(player.Play(1.0, &error));
    CHECK(player.Play(2.0, &error));
    size_t checked = 0;
    for (size_t s = 0; s < file->steps.size(); ++s) {
        const fb::RigExecWireStep &step = file->steps[s];
        if (step.kind != fb::StepKind::RevisionChunk || step.object < 0 ||
            size_t(step.object) >= geometry.revisionIndex.size()) {
            continue;
        }
        const auto &at = geometry.revisionIndex[size_t(step.object)];
        const std::string mover = RigExecFormatPathText(
            *file, geometry.chains[size_t(at.first)]
                       .revisions[size_t(at.second)]
                       .moverPath);
        const auto found = position.find(mover);
        if (found == position.end()) {
            continue;
        }
        const bool want =
            found->second == first->second ||
            (found->second > first->second && step.part == 0);
        const bool ran = player->GetStepRanForTesting(s);
        CHECK(ran == want);
        if (ran != want) {
            std::printf("%s: %s range %d %s at frame 2\n", name,
                        mover.c_str(), int(step.part),
                        ran ? "ran" : "did not run");
        }
        ++checked;
    }
    CHECK(checked == 12);
    std::printf("%s: %zu range step(s) checked\n", name, checked);
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
    /// The plane's rigExec:sampleSource names that Points prim itself, not
    /// its points attribute.
    bool samplerPrim = false;
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
            .SetTargets({options.samplerPrim ? sampler.GetPrimPath()
                                             : sampler.GetPath()});
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
// baked program and the native evaluator bit for bit.
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
        CHECK(_FieldsMatchNative(name, _VolumeMixStage(options), frames,
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
        std::printf("%s: %zu field(s) equal the native evaluator's, %zu "
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

// Animated oracle/gather arrays are omitted from static reporting only
// when original-file stage transport actually reaches their private slots.
static void
TestAnimatedStaticPointsReported()
{
    const auto report = [&](const _MixOptions &options,
                            const std::string &sampledPath) {
        const auto stage = _VolumeMixStage(options);
        RigExecRigEvaluator evaluator(stage, _FindRig(stage));
        RigExecBakeOpts opts;
        opts.time = 1.0;
        RigExecBakeResult result;
        std::string error;
        CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
        if (!sampledPath.empty()) {
            const auto file = RigExecTestUnpack(result.bytes);
            auto reader = RigExecRuntimeReader::Open(
                result.bytes.data(), result.bytes.size(), &error);
            CHECK(file && reader);
            if (file && reader) {
                const int64_t slot = RigExecTestSlotOf(*file, sampledPath);
                CHECK(slot >= int64_t(file->listedInputs) &&
                      slot < int64_t(file->inputs.size()));
                size_t publicIndex = 0;
                CHECK(!reader->FindInput(sampledPath, &publicIndex));
                const auto inputs = RigExecRuntimeStageArrayInputs::Enumerate(*reader);
                CHECK(std::count_if(inputs.begin(), inputs.end(),
                    [&](const auto &info) {
                        return info.name == sampledPath &&
                               info.slot == size_t(slot) &&
                               info.tag == RrInputTag::Vec3fArray;
                    }) == 1);
            }
        }
        std::vector<RigExecBakeStaticEntry> entries;
        CHECK(RigExecBakeStaticReport(evaluator, &entries, &error));
        for (const auto &entry : entries) {
            std::printf("  static %s: %s\n", entry.field.c_str(),
                        entry.source.c_str());
        }
        CHECK(entries.empty());
    };
    report(_MixOptions(), "");
    _MixOptions sampler;
    sampler.animatedSampler = true;
    report(sampler, "/Asset/Geom/Sampler.points");
    _MixOptions curve;
    curve.animatedCurve = true;
    report(curve, "/Asset/Geom/CurveSource.points");
    std::printf("animated private sample source and curve: transported, not static\n");
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
// weight +0 and -0, a trailing (0, -0) and a trailing (0, +0), a zero
// weight on another index, and in one variant a negative weight, under
// classicLinear and dualQuaternion: the file's sparse form keeps each row
// up to its trailing (0, +0) run, so it expands to the authored arrays bit
// for bit, and plays every frame bit for bit with the baked program.
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
            // The file's layout: the 16 entries less the third row's
            // trailing (0, +0), in order and bit for bit. A negative weight
            // fails the layout's validation, so the bake keeps no epoch
            // layout and both sides read the per-frame arrays.
            RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
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
                  (std::vector<uint8_t>{4, 4, 3, 4}));
            CHECK(topology->indexWidth == 1);
            CHECK(topology->indices8 ==
                  (std::vector<uint8_t>{0, 1, 0, 2,  //
                                        2, 1, 0, 0,  //
                                        1, 0, 2,     //
                                        0, 0, 1, 2}));
            const std::vector<float> kept = {0.0f,  0.5f, -0.0f, 0.5f,  //
                                             0.0f,  1.0f, 0.0f,  -0.0f, //
                                             0.25f, 0.75f, 0.0f,        //
                                             -0.0f, 0.0f, 0.6f,  0.4f};
            CHECK(topology->weights.size() == kept.size() &&
                  std::memcmp(topology->weights.data(), kept.data(),
                              sizeof(float) * kept.size()) == 0);
            // And the expansion is the authored arrays, bit for bit.
            std::vector<int32_t> denseIndices;
            std::vector<float> denseWeights;
            RigExecFormatExpandTopology(*topology, &denseIndices,
                                        &denseWeights);
            VtIntArray authoredIndices;
            VtFloatArray authoredWeights;
            const UsdPrim skin =
                stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Skin"));
            CHECK(skin.GetAttribute(TfToken("rigExec:jointIndices"))
                      .Get(&authoredIndices) &&
                  skin.GetAttribute(TfToken("rigExec:jointWeights"))
                      .Get(&authoredWeights));
            CHECK(denseIndices.size() == authoredIndices.size() &&
                  std::equal(denseIndices.begin(), denseIndices.end(),
                             authoredIndices.cbegin()));
            CHECK(denseWeights.size() == authoredWeights.size() &&
                  std::memcmp(denseWeights.data(), authoredWeights.cdata(),
                              sizeof(float) * denseWeights.size()) == 0);
        }
    }
    std::printf("sparse skin topology: checked\n");
}

static UsdStageRefPtr _TwoKeySkinStage();

// The layout rule the format and playback share
// (RigExecFormatSkinLayoutValidates) against the evaluator's own
// (RigExecBuildSkinTopology), layout by layout: valid rows, a -0 weight and
// no rows pass; element sizes 0 and -1, unequal lengths, ragged rows, no
// influences, an index below 0 or at the influence count, and NaN, inf and
// negative weights fail. The point count is the rows of an element size of
// at least 1, else 0.
static void
TestSharedLayoutRule()
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    struct Layout {
        const char *name;
        std::vector<int> indices;
        std::vector<float> weights;
        int elementSize;
        size_t influences;
    };
    const Layout layouts[] = {
        {"valid", {0, 1, 1, 0}, {0.5f, 0.5f, 1.0f, 0.0f}, 2, 2},
        {"-0 weight", {0, 1}, {-0.0f, 1.0f}, 2, 2},
        {"no rows", {}, {}, 2, 2},
        {"element size 0", {}, {}, 0, 2},
        {"element size 0 with entries", {0}, {1.0f}, 0, 2},
        {"element size -1", {0, 1}, {0.5f, 0.5f}, -1, 2},
        {"unequal lengths", {0, 1}, {1.0f}, 1, 2},
        {"ragged", {0, 1, 0}, {1.0f, 0.0f, 0.0f}, 2, 2},
        {"zero influences", {0, 0}, {1.0f, 0.0f}, 2, 0},
        {"no rows, no influences", {}, {}, 2, 0},
        {"index -1", {-1, 0}, {0.5f, 0.5f}, 2, 2},
        {"index at the influence count", {2, 0}, {0.5f, 0.5f}, 2, 2},
        {"NaN weight", {0, 1}, {nan, 0.5f}, 2, 2},
        {"inf weight", {0, 1}, {inf, 0.5f}, 2, 2},
        {"-1e-30 weight", {0, 1}, {-1e-30f, 0.5f}, 2, 2},
    };
    size_t passing = 0;
    for (const Layout &layout : layouts) {
        RigExecSkinTopology built;
        RigExecBuildSkinTopology(TfSpan<const int>(layout.indices),
                                 TfSpan<const float>(layout.weights),
                                 layout.elementSize, layout.influences,
                                 &built);
        const bool shared = RigExecFormatSkinLayoutValidates(
            layout.indices.data(), layout.indices.size(),
            layout.weights.data(), layout.weights.size(), layout.elementSize,
            layout.influences);
        CHECK(shared == built.validated);
        const bool rows = layout.elementSize >= 1 &&
                          layout.indices.size() == layout.weights.size() &&
                          layout.indices.size() %
                                  size_t(layout.elementSize) ==
                              0;
        CHECK(built.pointCount ==
              (rows ? layout.indices.size() / size_t(layout.elementSize)
                    : 0));
        if (shared != built.validated) {
            std::printf("shared layout rule, %s: format %d, evaluator %d\n",
                        layout.name, int(shared), int(built.validated));
        }
        passing += shared ? 1 : 0;
    }
    std::printf("shared layout rule: %zu layouts, %zu passing, every one as "
                "the evaluator judges it\n",
                sizeof(layouts) / sizeof(layouts[0]), passing);
    CHECK(passing == 3);
}

// A reader of \p bytes that ran once, at its defaults.
static std::unique_ptr<RigExecRuntimeReader>
_ArrayReader(const char *name, const std::vector<uint8_t> &bytes)
{
    std::string error;
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    if (!reader || !reader->Execute(&error)) {
        std::printf("FAILED: %s: %s\n", name, error.c_str());
        ++failures;
        return nullptr;
    }
    return reader;
}

// \p reader, after its first run, takes \p sets and runs again: equal to
// the dynamic and the baked evaluator at the bake time with the sets
// authored in the session layer (RigExecTestArrayReference), outputs,
// property values and diagnostics.
static bool
_ArraySetsMatch(const char *name, const std::string &leg,
                const UsdStageRefPtr &stage, const SdfPath &rigPath,
                RigExecRuntimeReader *reader,
                const std::vector<RigExecTestArraySet> &sets)
{
    std::string error;
    if (!RigExecTestApplyArraySets(reader, sets, &error) ||
        !reader->Execute(&error)) {
        std::printf("FAILED: %s, %s: %s\n", name, leg.c_str(),
                    error.c_str());
        ++failures;
        return false;
    }
    bool same = true;
    {
        RigExecRigPose pose;
        if (!RigExecTestArrayReference(stage, rigPath, sets,
                                       reader->GetBakeTime(), &pose,
                                       &error)) {
            std::printf("FAILED: %s, %s: %s\n", name, leg.c_str(),
                        error.c_str());
            ++failures;
            return false;
        }
        std::vector<std::string> diffs;
        if (!RigExecCompareRuntimeRun(pose, *reader, &diffs)) {
            same = false;
            std::printf("FAILED: %s, %s, %s mode:\n", name, leg.c_str(),
                        "native");
            for (const std::string &diff : diffs) {
                std::printf("    %s\n", diff.c_str());
            }
        }
    }
    CHECK(same);
    return same;
}

// Exec's per-element weight inputs are excluded from upstream admission.
// Assert the real file's private storage and public refusal before crafting
// a public slot for the independent runtime setter/diagnostic tests.
static std::unique_ptr<RigExecRuntimeReader>
_PrivateOracleSetterReader(const char *name, const UsdStageRefPtr &stage,
                           const SdfPath &rigPath,
                           const std::vector<uint8_t> &bytes,
                           const std::string &path,
                           const VtVec3fArray &points,
                           RigExecRuntimeReader *original)
{
    const auto file = RigExecTestUnpack(bytes);
    if (!file) {
        return nullptr;
    }
    const int64_t slot = RigExecTestSlotOf(*file, path);
    CHECK(slot >= 0 && slot >= int64_t(file->listedInputs) &&
          slot < int64_t(file->inputs.size()));
    if (slot < 0 || slot < int64_t(file->listedInputs) ||
        slot >= int64_t(file->inputs.size())) {
        return nullptr;
    }
    size_t index = 0;
    CHECK(!original->FindInput(path, &index));
    VtVec3fArray changed = points;
    if (!changed.empty()) {
        changed[0][0] += 1.0f;
    }
    const RigExecRuntimeArray value{
        RrInputTag::Vec3fArray, changed.cdata(), changed.size()};
    std::string error;
    CHECK(!original->SetInputArray(path, value, &error));
    CHECK(!original->SetInputArrayAt(size_t(slot), value, &error));
    CHECK(!original->SetSampledInputArrayAt(size_t(slot), value, &error));
    RigExecRuntimeArray privateView;
    CHECK(!original->GetInputArrayAt(size_t(slot), &privateView));
    CHECK(!original->ResetInput(path, &error));
    CHECK(original->Execute(&error));
    {
        RigExecRigPose pose;
        CHECK(RigExecTestArrayReference(stage, rigPath, {}, 2.0,
                                        &pose, &error));
        std::vector<std::string> diffs;
        CHECK(RigExecCompareRuntimeRun(pose, *original, &diffs));
        for (const auto &diff : diffs) {
            std::printf("  %s, refused private set: %s\n", name,
                        diff.c_str());
        }
    }
    return _ArrayReader(name, RigExecTestListPrivateArraySlots(bytes));
}

// The oracle's sample points as an input: the volume mix's plane samples
// a keyed Points prim through rigExec:sampleSource. Private storage is kept
// at the attribute the oracle resolves the relationship to, with its packet
// gather bound to the same slot. A crafted authored set of its count reaches the
// current-phase field; a sampled one of another count fails it with the
// oracle's words, as live fails it.
static void
TestOraclePointsSet()
{
    const char *const name = "oracle sample points";
    _MixOptions options;
    options.animatedSampler = true;
    const UsdStageRefPtr stage = _VolumeMixStage(options);
    const SdfPath rigPath("/Asset/Rig");
    const std::string sampler = "/Asset/Geom/Sampler.points";
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        CHECK(RigExecTestBakeAt(evaluator, 2.0, &bytes, &error));
    }
    // The plane's oracle and packet gather share private array storage.
    {
        const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes);
        if (!file) {
            return;
        }
        const int64_t slot = RigExecTestSlotOf(*file, sampler);
        int listed = 0;
        for (const fb::RigExecWireWeightObject &object :
             file->geometry->weightObjects) {
            if (RigExecFormatPathText(*file, object.path) ==
                "/Asset/Rig/Weights/Plane") {
                listed += object.oracleSamplesSlot == slot ? 1 : 0;
            }
        }
        const fb::RigExecWirePathRead *row =
            RigExecTestPathReadRow(file.get(), sampler, false);
        CHECK(slot >= 0 && slot >= int64_t(file->listedInputs) &&
              slot < int64_t(file->inputs.size()) && listed == 1 &&
              row && row->read &&
              (file->inputs[size_t(slot)].flags() &
               uint8_t(fb::InputSlotFlags::Listed)) == 0 &&
              !row->value &&
              row->read->walk == std::vector<uint32_t>{uint32_t(slot)});
    }
    // At its defaults the file plays as live does.
    std::unique_ptr<RigExecRuntimeReader> reader = _ArrayReader(name, bytes);
    if (!reader) {
        return;
    }
    {
        RigExecRigPose pose;
        CHECK(RigExecTestArrayReference(stage, rigPath, {}, 2.0,
                                        &pose, &error));
        std::vector<std::string> diffs;
        CHECK(RigExecCompareRuntimeRun(pose, *reader, &diffs));
        for (const std::string &diff : diffs) {
            std::printf("  %s, defaults: %s\n", name, diff.c_str());
        }
    }
    VtVec3fArray points;
    CHECK(stage->GetAttributeAtPath(SdfPath(sampler))
              .Get(&points, UsdTimeCode(2.0)));
    reader = _PrivateOracleSetterReader(name, stage, rigPath, bytes,
                                        sampler, points, reader.get());
    if (!reader) {
        return;
    }
    VtVec3fArray moved = points;
    for (GfVec3f &p : moved) {
        p[0] -= 0.75f;
        p[1] += 0.25f;
    }
    const auto defaults = reader->GetPoints();
    int matched = 0;
    matched += _ArraySetsMatch(name, "authored", stage, rigPath, reader.get(),
                               {{sampler, VtValue(moved), false}});
    CHECK(!_SamePoints({defaults}, {reader->GetPoints()}));
    std::unique_ptr<RigExecRuntimeReader> fewer =
        _ArrayReader(name, RigExecTestListPrivateArraySlots(bytes));
    if (fewer) {
        VtVec3fArray shorter(points.begin(), points.end() - 1);
        matched += _ArraySetsMatch(name, "sampled, one point fewer", stage,
                                   rigPath, fewer.get(),
                                   {{sampler, VtValue(shorter), true}});
        const std::string failed =
            "current-phase weight failed: /Asset/Rig/Weights/Plane: "
            "sampled point count does not match the target";
        const std::vector<std::string> &lines = fewer->GetDiagnostics();
        CHECK(std::find(lines.begin(), lines.end(), failed) != lines.end());
    }
    std::printf("%s: %d of 2 sets == session edit\n", name, matched);
    CHECK(matched == 2);
}

// The oracle's sample points are the attribute the oracle resolves its
// relationship to: a plane whose rigExec:sampleSource names a Points prim
// samples that prim's points, a private slot beside the packet's
// gathered weight target, which names its attribute. The packet gathers
// no sample points through a prim, so the samples have no row. An
// authored set in a crafted public file reaches the current-phase field as the
// session edit does.
static void
TestOracleSlotIsCanonical()
{
    const char *const name = "canonical oracle points";
    _MixOptions options;
    options.animatedSampler = true;
    options.samplerPrim = true;
    const UsdStageRefPtr stage = _VolumeMixStage(options);
    const SdfPath rigPath("/Asset/Rig");
    const std::string sampler = "/Asset/Geom/Sampler.points";
    const std::string target = "/Asset/Geom/M.points";
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        CHECK(RigExecTestBakeAt(evaluator, 2.0, &bytes, &error));
    }
    bool canonical = false;
    if (const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes)) {
        const int64_t samples = RigExecTestSlotOf(*file, sampler);
        const int64_t gathered = RigExecTestSlotOf(*file, target);
        for (const fb::RigExecWireWeightObject &object :
             file->geometry->weightObjects) {
            if (RigExecFormatPathText(*file, object.path) !=
                "/Asset/Rig/Weights/Plane") {
                continue;
            }
            const fb::RigExecWirePathRead *row =
                RigExecTestPathReadRow(file.get(), target, false);
            canonical =
                samples >= 0 && samples >= int64_t(file->listedInputs) &&
                samples < int64_t(file->inputs.size()) && gathered >= 0 &&
                (file->inputs[size_t(samples)].flags() &
                 uint8_t(fb::InputSlotFlags::Listed)) == 0 &&
                samples != gathered &&
                object.oracleSamplesSlot == samples &&
                object.samplePoints.empty() &&
                !RigExecTestPathReadRow(file.get(), sampler, false) && row &&
                row->read &&
                row->read->walk == std::vector<uint32_t>{uint32_t(gathered)};
        }
    }
    CHECK(canonical);
    std::unique_ptr<RigExecRuntimeReader> reader = _ArrayReader(name, bytes);
    if (!canonical || !reader) {
        return;
    }
    {
        RigExecRigPose pose;
        CHECK(RigExecTestArrayReference(stage, rigPath, {}, 2.0,
                                        &pose, &error));
        std::vector<std::string> diffs;
        CHECK(RigExecCompareRuntimeRun(pose, *reader, &diffs));
        for (const std::string &diff : diffs) {
            std::printf("  %s, defaults: %s\n", name, diff.c_str());
        }
    }
    VtVec3fArray points;
    CHECK(stage->GetAttributeAtPath(SdfPath(sampler))
              .Get(&points, UsdTimeCode(2.0)));
    reader = _PrivateOracleSetterReader(name, stage, rigPath, bytes,
                                        sampler, points, reader.get());
    if (!reader) {
        return;
    }
    for (GfVec3f &p : points) {
        p[0] -= 0.5f;
    }
    const auto defaults = reader->GetPoints();
    const bool matched =
        _ArraySetsMatch(name, "authored", stage, rigPath, reader.get(),
                        {{sampler, VtValue(points), false}});
    CHECK(!_SamePoints({defaults}, {reader->GetPoints()}));
    std::printf("%s: private %s apart from gathered %s; crafted set %s\n", name,
                sampler.c_str(), target.c_str(),
                matched ? "== session edit" : "differs");
}

// Real stage sampling uses the original private oracle slot, not a
// crafted public file. Empty arrays and failed reads differ in the oracle's
// diagnostics and raw target fallback; later valid samples recover. A private
// curve has no sampleSource fallback and preserves genuine failed-Get coverage.
static void
TestStageArrayOracleAndChain()
{
    for (const int kind : {0, 1, 2, 3}) {
        const bool privateOracle = kind != 1;
        const bool curveOracle = kind == 2;
        const bool blockedAtBake = kind == 3;
        const char *name = blockedAtBake ? "stage blocked-at-bake chain raw fallback"
            : curveOracle ? "stage private curve missing/recovery"
            : privateOracle ? "stage private oracle samples"
                            : "stage unweighted chain missing/empty base";
        _MixOptions options;
        options.animatedSampler = true;
        options.samplerPrim = true;
        options.animatedCurve = curveOracle;
        if (curveOracle) options.mode = "average";
        auto stage = _VolumeMixStage(options);
        const SdfPath rigPath("/Asset/Rig");
        const SdfPath path(curveOracle ? "/Asset/Geom/CurveSource.points"
            : privateOracle ? "/Asset/Geom/Sampler.points"
                            : "/Asset/Geom/M.points");
        if (!privateOracle) {
            // Changing weighted-domain cardinality is an epoch refusal.
            // This clean unweighted chain exercises legal sampled base changes.
            stage = UsdStage::CreateInMemory();
            stage->DefinePrim(rigPath, TfToken("RigExecRoot"));
            const auto mesh = stage->DefinePrim(path.GetPrimPath(), TfToken("Points"));
            mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
                .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                                 GfVec3f(0, 1, 0)});
            const auto joint = stage->DefinePrim(
                SdfPath("/Asset/Rig/Joints/J"), TfToken("RigExecJoint"));
            joint.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
                .Set(2.0);
            const auto mover = stage->DefinePrim(
                SdfPath("/Asset/Rig/Movers/Move"), TfToken("RigExecMatrixMover"));
            CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
            mover.CreateRelationship(TfToken("rigExec:moves")).SetTargets({path});
            mover.CreateRelationship(TfToken("rigExec:transform"))
                .SetTargets({joint.GetPath()});
        }
        const auto attribute = stage->GetAttributeAtPath(path);
        VtVec3fArray initial;
        CHECK(attribute.Get(&initial, UsdTimeCode(1)) && !initial.empty());
        if (initial.empty()) continue;
        VtVec3fArray changed = initial;
        for (auto &point : changed) {
            point[0] -= 0.75f;
            point[1] += 0.25f;
        }
        CHECK(attribute.Clear());
        CHECK(attribute.Set(initial));
        CHECK(attribute.Set(initial, UsdTimeCode(1)));
        CHECK(attribute.Set(changed, UsdTimeCode(2)));
        CHECK(attribute.Set(VtVec3fArray(initial.begin(), initial.end() - 1),
                            UsdTimeCode(3)));
        CHECK(attribute.Set(VtVec3fArray(), UsdTimeCode(4)));
        CHECK(attribute.Set(SdfValueBlock(), UsdTimeCode(5)));
        CHECK(attribute.Set(changed, UsdTimeCode(6)));
        const SdfPath fallbackPath("/Asset/Geom/M.points");
        if (blockedAtBake) {
            CHECK(attribute.Set(SdfValueBlock(), UsdTimeCode(1)));
            VtVec3fArray raw;
            CHECK(stage->GetAttributeAtPath(fallbackPath).Get(&raw) &&
                  raw == initial);
            double precedingLift = 0.0;
            CHECK(stage->GetAttributeAtPath(
                      SdfPath("/Asset/Rig/Joints/Lift.avars:ty"))
                      .Get(&precedingLift, UsdTimeCode(1)) &&
                  precedingLift != 0.0);
            // The Mix is current-phase through its preceding Sphere. Plane's
            // canonical fallback is the raw base, apart from First's lift.
            CHECK(stage->GetPrimAtPath(SdfPath("/Asset/Rig/Weights/Plane"))
                      .GetRelationship(TfToken("rigExec:weightTarget"))
                      .SetTargets({fallbackPath.GetPrimPath()}));
        }
        std::vector<uint8_t> bytes;
        std::string error;
        {
            RigExecRigEvaluator evaluator(stage, rigPath);
            CHECK(RigExecTestBakeAt(evaluator, 1, &bytes, &error));
        }
        const auto file = RigExecTestUnpack(bytes);
        CHECK(file);
        if (!file) continue;
        const int64_t slot = RigExecTestSlotOf(*file, path.GetString());
        CHECK(slot >= 0 && slot < int64_t(file->inputs.size()));
        if (slot < 0 || slot >= int64_t(file->inputs.size())) continue;
        auto reader = _ArrayReader(name, bytes);
        if (!reader) continue;
        const size_t publicCount = reader->GetInputCount();
        if (privateOracle) {
            CHECK(slot >= int64_t(file->listedInputs));
            size_t publicIndex = 0;
            CHECK(!reader->FindInput(path.GetString(), &publicIndex));
            RigExecRuntimeArray view;
            CHECK(!reader->GetInputArrayAt(size_t(slot), &view));
            const RigExecRuntimeArray value{
                RrInputTag::Vec3fArray, changed.cdata(), changed.size()};
            CHECK(!reader->SetInputArray(path.GetString(), value, &error));
            CHECK(!reader->SetInputArrayAt(size_t(slot), value, &error));
            CHECK(!reader->SetSampledInputArrayAt(size_t(slot), value, &error));
            CHECK(!reader->ResetInput(path.GetString(), &error));
            int bound = 0;
            for (const auto &object : file->geometry->weightObjects) {
                if (RigExecFormatPathText(*file, object.path) ==
                    (curveOracle ? "/Asset/Rig/Weights/Curve"
                                 : "/Asset/Rig/Weights/Plane")) {
                    bound += (curveOracle ? object.oracleCurveSlot
                                          : object.oracleSamplesSlot) == slot ? 1 : 0;
                }
            }
            CHECK(bound == 1);
            if (blockedAtBake) {
                CHECK((file->inputs[size_t(slot)].flags() &
                       uint8_t(fb::InputSlotFlags::HasValue)) == 0);
                const int64_t fallbackSlot =
                    RigExecTestSlotOf(*file, fallbackPath.GetString());
                CHECK(fallbackSlot >= 0 &&
                      fallbackSlot < int64_t(file->inputs.size()));
                if (fallbackSlot >= 0 &&
                    fallbackSlot < int64_t(file->inputs.size())) {
                    CHECK(file->inputs[size_t(fallbackSlot)].type() ==
                          fb::InputTag::Vec3fArray);
                    size_t publicIndex = 0;
                    const bool listed = fallbackSlot < int64_t(file->listedInputs);
                    CHECK(reader->FindInput(fallbackPath.GetString(), &publicIndex)
                          == listed);
                    if (listed) CHECK(publicIndex == size_t(fallbackSlot));
                }
                CHECK(std::any_of(file->geometry->chains.begin(),
                                  file->geometry->chains.end(),
                    [&](const auto &chain) {
                        return RigExecFormatPathText(*file, chain.target) ==
                                   fallbackPath.GetString() &&
                            std::any_of(chain.revisions.begin(), chain.revisions.end(),
                                [](const auto &revision) {
                                    return revision.weightCurrentPhase;
                                });
                    }));
                for (const auto &object : file->geometry->weightObjects) {
                    if (RigExecFormatPathText(*file, object.path) ==
                        "/Asset/Rig/Weights/Plane") {
                        CHECK(object.targetPoints.size() == 1 &&
                              RigExecFormatPathText(*file, object.targetPoints[0]) ==
                                  fallbackPath.GetString() &&
                              object.targetValid == std::vector<uint8_t>{0});
                    }
                }
            }
        }
        const auto transported = RigExecRuntimeStageArrayInputs::Enumerate(*reader);
        CHECK(std::count_if(transported.begin(), transported.end(),
                            [&](const auto &info) {
                                return info.slot == size_t(slot) &&
                                       info.name == path.GetString() &&
                                       info.tag == RrInputTag::Vec3fArray;
                            }) == 1);
        RigExecInputSampler sampler;
        CHECK(sampler.Bind(stage, *reader, &error));
        CHECK(sampler.GetWarnings().empty());
        const std::string countFailure =
            "current-phase weight failed: /Asset/Rig/Weights/Plane: "
            "sampled point count does not match the target";
        const std::string missingFailure =
            "current-phase weight failed: /Asset/Rig/Weights/Plane: "
            "could not read the points to sample";
        for (const double time : {1., 2., 3., 4., 5., 6., 2.}) {
            bool sampled = false;
            CHECK(sampler.Apply(UsdTimeCode(time), reader.get(), &error, &sampled));
            CHECK(reader->Execute(&error));
            CHECK(reader->GetInputCount() == publicCount);
            {
                RigExecRigPose pose;
                CHECK(RigExecTestArrayReference(stage, rigPath, {}, time,
                                                &pose, &error));
                std::vector<std::string> diffs;
                CHECK(RigExecCompareRuntimeRun(pose, *reader, &diffs));
                for (const auto &diff : diffs) {
                    std::printf("  %s at %.0f: %s\n", name, time, diff.c_str());
                }
            }
            if (privateOracle) {
                const auto &lines = reader->GetDiagnostics();
                if (curveOracle) {
                    const std::string curveFailure =
                        "current-phase weight failed: /Asset/Rig/Weights/Curve: "
                        "rigExec:curve must name exactly one points source";
                    CHECK(_HasLine(lines, curveFailure) == (time == 4 || time == 5));
                } else {
                    CHECK(_HasLine(lines, countFailure) == (time == 3 || time == 4));
                    // A blocked primary sampleSource falls back to raw
                    // weightTarget points. A successful empty primary does not.
                    CHECK(_HasLine(lines, missingFailure) ==
                          false);
                }
                size_t publicIndex = 0;
                CHECK(!reader->FindInput(path.GetString(), &publicIndex));
            }
            if (time == 2) {
                RigExecRigPose held;
                CHECK(RigExecTestArrayReference(
                    stage, rigPath,
                    {{path.GetString(), VtValue(initial), true}}, time,
                    &held, &error));
                std::vector<std::string> diffs;
                CHECK(!RigExecCompareRuntimeOutputs(held, *reader, &diffs));
            }
            // A repeated frame performs no new stage sampling, including
            // after failed reads. The reader still retains that read kind.
            sampled = true;
            CHECK(sampler.Apply(UsdTimeCode(time), reader.get(), &error, &sampled));
            CHECK(!sampled);
        }
        std::printf("%s: original slot, 7 frames incl short/empty/missing/recovery\n",
                    name);
    }
}

// docs/examples/skin_mover.usda's two skins, whose layouts hold a mid-row
// (0, 0) entry: a row that opens with (0, 0) before its other influence.
// The stored layouts keep it and expand to the stage's arrays bit for bit,
// which are their inputs' defaults, and every frame plays as live baked
// does, linear and dual quaternion.
static void
TestLosslessLayoutPlaysAsLive()
{
    const char *const name = "lossless skin layouts";
    const std::string path =
        std::string(RIGEXEC_EXAMPLES_DIR) + "/../docs/examples/skin_mover.usda";
    const UsdStageRefPtr stage = UsdStage::Open(path);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = _FindRig(stage);
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator baker(stage, rigPath);
        CHECK(RigExecTestBakeAt(baker, 1001.0, &bytes, &error));
    }
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    size_t layouts = 0, leadingZeros = 0;
    for (const fb::RigExecWireChain &chain : file->geometry->chains) {
        for (const fb::RigExecWireRevision &revision : chain.revisions) {
            if (!revision.topology) {
                continue;
            }
            const std::string mover =
                RigExecFormatPathText(*file, revision.moverPath);
            const UsdPrim prim = stage->GetPrimAtPath(SdfPath(mover));
            VtIntArray indices;
            VtFloatArray weights;
            CHECK(prim.GetAttribute(TfToken("rigExec:jointIndices"))
                      .Get(&indices) &&
                  prim.GetAttribute(TfToken("rigExec:jointWeights"))
                      .Get(&weights));
            std::vector<int32_t> expandedIndices;
            std::vector<float> expandedWeights;
            RigExecFormatExpandTopology(*revision.topology, &expandedIndices,
                                        &expandedWeights);
            const bool same =
                !revision.topology->raw &&
                expandedIndices.size() == indices.size() &&
                std::equal(expandedIndices.begin(), expandedIndices.end(),
                           indices.cbegin()) &&
                expandedWeights.size() == weights.size() &&
                std::memcmp(expandedWeights.data(), weights.cdata(),
                            sizeof(float) * weights.size()) == 0;
            CHECK(same);
            // Its inputs default to that layout.
            CHECK(revision.jointIndicesSlot ==
                      RigExecTestSlotOf(*file,
                                        mover + ".rigExec:jointIndices") &&
                  revision.jointWeightsSlot ==
                      RigExecTestSlotOf(*file,
                                        mover + ".rigExec:jointWeights"));
            for (size_t e = 0; e + 1 < indices.size(); e += 2) {
                leadingZeros += indices[e] == 0 && weights[e] == 0.0f ? 1 : 0;
            }
            // Restoring signed-zero source bits reruns the owning layout
            // producer; the next held run must do no work.
            for (size_t z = 0; z < weights.size(); ++z) {
                if (weights[z] != 0.0f || std::signbit(weights[z])) {
                    continue;
                }
                std::unique_ptr<RigExecRuntimeReader> reader =
                    _ArrayReader(name, bytes);
                if (!reader) {
                    break;
                }
                const std::string input = mover + ".rigExec:jointWeights";
                VtFloatArray moved = weights;
                moved[z] = 0.25f;
                CHECK(RigExecTestApplyArraySets(
                          reader.get(), {{input, VtValue(moved), false}},
                          &error) &&
                      reader->Execute(&error));
                VtFloatArray signedZero = weights;
                signedZero[z] = -0.0f;
                CHECK(RigExecTestApplyArraySets(
                          reader.get(),
                          {{input, VtValue(signedZero), false}}, &error) &&
                      reader->Execute(&error));
                CHECK(!reader->GetSkinLayoutIsOpenForTesting(mover));
                CHECK(reader->Execute(&error));
                const uint64_t unchanged =
                    reader->GetCounters().executedOpCount;
                CHECK(unchanged == 0 && reader->GetLastRunTraceForTesting().empty());
                CHECK(reader->ResetInput(input, &error) &&
                      reader->Execute(&error));
                CHECK(reader->GetSkinLayoutIsOpenForTesting(mover));
                bool ranLayout = false;
                for (int32_t id : reader->GetLastRunTraceForTesting())
                    ranLayout = ranLayout || (id >= 0 && size_t(id) < file->steps.size() &&
                        file->steps[size_t(id)].kind == fb::StepKind::SkinTopology &&
                        reader->GetStepLabelForTesting(size_t(id)).find(mover) != std::string::npos);
                CHECK(ranLayout);
                RigExecRigPose restored;
                CHECK(RigExecTestArrayReference(stage, rigPath, {}, 1001.0,
                                                &restored, &error));
                std::vector<std::string> restoredDiffs;
                CHECK(RigExecCompareRuntimeRun(restored, *reader,
                                               &restoredDiffs, true));
                for (const auto &diff : restoredDiffs)
                    std::printf("restored signed-zero layout: %s\n", diff.c_str());
                CHECK(reader->Execute(&error));
                CHECK(reader->GetCounters().executedOpCount == unchanged);
                CHECK(reader->GetLastRunTraceForTesting().empty());
                break;
            }
            layouts += same ? 1 : 0;
        }
    }
    CHECK(layouts == 2 && leadingZeros > 0);
    std::printf("%s: %zu layouts expand to their arrays, %zu rows opening "
                "with a kept (0, 0)\n",
                name, layouts, leadingZeros);
    std::vector<double> frames;
    for (double t = 1001.0; t <= 1012.0; t += 1.0) {
        frames.push_back(t);
    }
    _TestStage(name, stage, frames);
}

// A current-phase combine of a `preceding` sphere and a sparse dynamic
// weight that paints its own support, [0, 1], its sparse base's.
static UsdStageRefPtr
_DynamicSupportStage()
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
    const auto weight = [&](const char *path, const char *type) {
        const UsdPrim prim = stage->DefinePrim(SdfPath(path), TfToken(type));
        prim.CreateRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({target});
        return prim;
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

    const UsdPrim full = weight("/Asset/Rig/Weights/Full",
                                "RigExecStaticWeight");
    setToken(full, "rigExec:representation", "dense");
    full.CreateAttribute(TfToken("rigExec:values"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray(rest.size(), 1.0f));
    setFloat(full, "rigExec:defaultWeight", 0.0f);

    const UsdPrim sphere = weight("/Asset/Rig/Weights/Sphere",
                                  "RigExecSphereWeight");
    sphere.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetMetadata(TfToken("rigExecReadPhase"), std::string("preceding"));
    place(sphere, 0, 0, 0);
    setFloat(sphere, "inputs:falloffMin", 0.0f);
    setFloat(sphere, "inputs:falloffMax", 2.5f);
    setToken(sphere, "rigExec:falloffProfile", "linear");

    const UsdPrim base = weight("/Asset/Rig/Weights/Base",
                                "RigExecStaticWeight");
    setToken(base, "rigExec:representation", "sparse");
    setFloat(base, "rigExec:defaultWeight", 0.0f);
    base.CreateAttribute(TfToken("rigExec:indices"),
                         SdfValueTypeNames->IntArray)
        .Set(VtIntArray{0, 1});
    base.CreateAttribute(TfToken("rigExec:values"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{0.5f, 0.25f});
    const UsdPrim dynamic = weight("/Asset/Rig/Weights/Dyn",
                                   "RigExecDynamicWeight");
    setToken(dynamic, "rigExec:representation", "sparse");
    dynamic.CreateRelationship(TfToken("rigExec:baseWeight"))
        .SetTargets({base.GetPath()});
    dynamic.CreateAttribute(TfToken("rigExec:indices"),
                            SdfValueTypeNames->IntArray)
        .Set(VtIntArray{0, 1});

    const UsdPrim combine = weight("/Asset/Rig/Weights/Mix",
                                   "RigExecCombineWeight");
    combine.CreateRelationship(TfToken("rigExec:inputWeights"))
        .SetTargets({sphere.GetPath(), dynamic.GetPath()});
    setToken(combine, "rigExec:combineMode", "max");
    setToken(combine, "rigExec:rangePolicy", "clamp");

    const auto mover = [&](const char *path, const UsdPrim &transform,
                           const UsdPrim &envelope) {
        const UsdPrim m =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecMatrixMover"));
        CHECK(m.ApplyAPI(TfToken("RigExecMoverAPI")));
        m.CreateRelationship(TfToken("rigExec:moves")).SetTargets({target});
        m.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({transform.GetPath()});
        m.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({envelope.GetPath()});
    };
    mover("/Asset/Rig/Movers/Second", joint, combine);
    mover("/Asset/Rig/Movers/Second/First", lift, full);
    return stage;
}

// The dynamic support check over the painted indices, which the export
// lists as inputs: an indices set off the base's support fails the
// dynamic weight with the
// oracle's words on every run, where the evaluators' compile refuses the
// authored edit and skips its consumer, with the same pass-through; a set
// of the base's support in another order passes, as the session edit
// does.
static void
TestDynamicSupportFromSlots()
{
    const char *const name = "dynamic sparse support";
    const UsdStageRefPtr stage = _DynamicSupportStage();
    const SdfPath rigPath("/Asset/Rig");
    const std::string dynamic = "/Asset/Rig/Weights/Dyn";
    const std::string indices = dynamic + ".rigExec:indices";
    const std::string failed =
        "current-phase weight failed: dynamic/base sparse support mismatch "
        "on " + dynamic;
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        CHECK(RigExecTestBakeAt(evaluator, 2.0, &bytes, &error));
    }
    // Each weight's indices are its input.
    int listed = 0;
    if (const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes)) {
        for (const fb::RigExecWireWeightObject &object :
             file->geometry->weightObjects) {
            const std::string path = RigExecFormatPathText(*file, object.path);
            if ((path == dynamic || path == "/Asset/Rig/Weights/Base") &&
                object.indicesSlot >= 0 &&
                object.indicesSlot ==
                    RigExecTestSlotOf(*file, path + ".rigExec:indices")) {
                ++listed;
            }
        }
    }
    CHECK(listed == 2);
    const auto hasLine = [&](const RigExecRuntimeReader &reader) {
        const std::vector<std::string> &lines = reader.GetDiagnostics();
        return std::find(lines.begin(), lines.end(), failed) != lines.end();
    };
    std::unique_ptr<RigExecRuntimeReader> reader =
        listed == 2 ? _ArrayReader(name, bytes) : nullptr;
    if (!reader) {
        return;
    }
    // At its defaults the support matches: live's run.
    {
        RigExecRigPose pose;
        CHECK(RigExecTestArrayReference(stage, rigPath, {}, 2.0,
                                        &pose, &error));
        std::vector<std::string> diffs;
        CHECK(RigExecCompareRuntimeRun(pose, *reader, &diffs));
        for (const std::string &diff : diffs) {
            std::printf("  %s, defaults: %s\n", name, diff.c_str());
        }
        CHECK(!hasLine(*reader));
    }
    size_t publicIndex = 0;
    CHECK(!reader->FindInput(indices, &publicIndex));
    // A crafted file exercises dynamic support validation on a runtime
    // array set; real painted storage remains inaccessible to callers.
    reader = _ArrayReader(name, RigExecTestListPrivateArraySlots(bytes));
    CHECK(reader);
    if (!reader) {
        return;
    }
    // Off the base's support.
    const std::vector<RigExecTestArraySet> off = {
        {indices, VtValue(VtIntArray{0, 2}), false}};
    CHECK(RigExecTestApplyArraySets(reader.get(), off, &error) &&
          reader->Execute(&error));
    CHECK(hasLine(*reader));
    {
        RigExecRigPose pose;
        CHECK(RigExecTestArrayReference(stage, rigPath, off, 2.0,
                                        &pose, &error));
        std::vector<std::string> diffs;
        CHECK(RigExecCompareRuntimeRun(pose, *reader, &diffs, true));
        for (const std::string &diff : diffs) {
            std::printf("  %s, off the support: %s\n", name, diff.c_str());
        }
    }
    // The base's support in another order passes, as live passes it.
    CHECK(_ArraySetsMatch(name, "the base's support", stage, rigPath,
                          reader.get(),
                          {{indices, VtValue(VtIntArray{1, 0}), false}}));
    CHECK(!hasLine(*reader));
    std::printf("%s: checked\n", name);
}

// A chunked skin's layout as inputs (the two-key skin, chunked at two
// vertices): a repaint of its indices runs the revision whole and plays as
// the session edit does with J1 moved, a weights-only set keeps the chunks,
// and a reset returns the Open layout and the partition.
static void
TestLayoutSetOnChunkedRevision()
{
    const char *const name = "chunked layout inputs";
    const std::string always = TfGetenv("RIGEXEC_BAKED_CHUNK_ALWAYS");
    const std::string verts = TfGetenv("RIGEXEC_BAKED_CHUNK_VERTS");
    TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", "1");
    TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", "2");
    const std::string skin = "/Asset/Rig/Movers/Skin";
    const std::string indices = skin + ".rigExec:jointIndices";
    const std::string weights = skin + ".rigExec:jointWeights";
    const std::string j1 = "/Asset/Rig/J1.avars:ty";
    const UsdStageRefPtr stage = _TwoKeySkinStage();
    const SdfPath rigPath("/Asset/Rig");
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
    }
    // The chunked revision's layout arrays are its inputs, defaulting to
    // the stored layout.
    int listed = 0;
    if (const std::unique_ptr<fb::RigExecWireFile> file =
            RigExecTestUnpack(bytes)) {
        const auto &geometry = *file->geometry;
        for (size_t id = 0; id < geometry.revisionIndex.size(); ++id) {
            const auto &at = geometry.revisionIndex[id];
            const fb::RigExecWireRevision &revision =
                geometry.chains[size_t(at.first)]
                    .revisions[size_t(at.second)];
            const int64_t i = RigExecTestSlotOf(*file, indices);
            const int64_t w = RigExecTestSlotOf(*file, weights);
            if (RigExecFormatPathText(*file, revision.moverPath) != skin ||
                !revision.chunked || !revision.topologyResolved || i < 0 ||
                w < 0 || revision.jointIndicesSlot != i ||
                revision.jointWeightsSlot != w) {
                continue;
            }
            const fb::RigExecWireValue &a =
                file->values[file->inputs[size_t(i)].value()];
            const fb::RigExecWireValue &b =
                file->values[file->inputs[size_t(w)].value()];
            if (a.arraySource == fb::ArraySource::SkinIndices &&
                b.arraySource == fb::ArraySource::SkinWeights &&
                a.array == id && b.array == id) {
                ++listed;
            }
        }
    }
    CHECK(listed == 1);
    std::unique_ptr<RigExecRuntimeReader> reader =
        listed == 1 ? _ArrayReader(name, bytes) : nullptr;
    if (reader) {
        CHECK(!reader->GetPartitionStaleForTesting(skin) &&
              reader->GetSkinLayoutIsOpenForTesting(skin));
        // A repaint: the partition holds other indices, so the revision
        // runs whole, and only J1 moves the points.
        CHECK(_ArraySetsMatch(name, "repainted, J1 moved", stage, rigPath,
                              reader.get(),
                              {{indices, VtValue(VtIntArray{1, 1, 1, 1}),
                                false},
                               {j1, VtValue(25.0), false}}));
        CHECK(reader->GetPartitionStaleForTesting(skin) &&
              !reader->GetSkinLayoutIsOpenForTesting(skin));
        // Back to the file's layout: the Open one, and the chunks run.
        CHECK(reader->ResetInput(indices, &error) &&
              reader->Execute(&error));
        CHECK(!reader->GetPartitionStaleForTesting(skin) &&
              reader->GetSkinLayoutIsOpenForTesting(skin));
    }
    std::unique_ptr<RigExecRuntimeReader> weighted =
        listed == 1 ? _ArrayReader(name, bytes) : nullptr;
    if (weighted) {
        // Weights alone: the partition's indices stand, so the chunks run.
        CHECK(_ArraySetsMatch(
            name, "weights only", stage, rigPath, weighted.get(),
            {{weights, VtValue(VtFloatArray{0.5f, 1.0f, 1.0f, 0.25f}),
              false}}));
        CHECK(!weighted->GetPartitionStaleForTesting(skin) &&
              !weighted->GetSkinLayoutIsOpenForTesting(skin));
    }
    std::printf("%s: checked\n", name);
    if (always.empty()) {
        TfUnsetenv("RIGEXEC_BAKED_CHUNK_ALWAYS");
    } else {
        TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", always);
    }
    if (verts.empty()) {
        TfUnsetenv("RIGEXEC_BAKED_CHUNK_VERTS");
    } else {
        TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", verts);
    }
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

// A skin whose four points take one influence each, J0 for the first two
// and J1 for the last two: under RIGEXEC_BAKED_CHUNK_VERTS=2 its Build keys
// are {j0} and {j1}. J0 moves its points along x and J1 along y.
static UsdStageRefPtr
_TwoKeySkinStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim j0 = stage->DefinePrim(SdfPath("/Asset/Rig/J0"),
                                         TfToken("RigExecControl"));
    j0.GetAttribute(TfToken("avars:tx")).Set(10.0);
    const UsdPrim j1 = stage->DefinePrim(SdfPath("/Asset/Rig/J1"),
                                         TfToken("RigExecControl"));
    j1.GetAttribute(TfToken("avars:ty")).Set(20.0);
    const SdfPath target("/Asset/Geom/Mesh.points");
    const UsdPrim mesh =
        stage->DefinePrim(target.GetPrimPath(), TfToken("Mesh"));
    VtVec3fArray points(4);
    for (size_t i = 0; i < points.size(); ++i) {
        points[i] = GfVec3f(float(i), float(i) * 2.0f, float(i) * 3.0f);
    }
    mesh.GetAttribute(TfToken("points")).Set(points);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({j0.GetPath(), j1.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(1);
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray)
        .Set(VtIntArray{0, 0, 1, 1});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1.0f, 1.0f, 1.0f, 1.0f});
    return stage;
}

// A chunked skin repainted in place, so chunk 0's points bind J1. A bake
// compiles first and so cuts its keys from the repainted layout; the file
// a layout set at playback amounts to holds Build's keys, the repainted
// layout, and Build's layout as the partition. It is made here by editing
// the bake of the layout as built. Playback keeps the keys, finds the
// partition stale and runs the revision whole, so moving J1 alone moves
// chunk 0's points too, as live baked does after rebuilding for the same
// authored repaint. The file as baked keeps running its chunks.
static void
TestRepaintedChunkLayoutRunsWhole()
{
    const char *const name = "repainted chunk layout";
    const std::string always = TfGetenv("RIGEXEC_BAKED_CHUNK_ALWAYS");
    const std::string verts = TfGetenv("RIGEXEC_BAKED_CHUNK_VERTS");
    TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", "1");
    TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", "2");
    const std::string skinPath = "/Asset/Rig/Movers/Skin";
    const std::string j1 = "/Asset/Rig/J1.avars:ty";
    const UsdStageRefPtr stage = _TwoKeySkinStage();
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    const UsdTimeCode time(1.0);
    std::vector<uint8_t> asBuilt;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, 1.0, &asBuilt, &error));

    fb::RigExecWireSkinTopology repainted;
    CHECK(RigExecFormatTopology({1, 1, 1, 1}, {1.0f, 1.0f, 1.0f, 1.0f}, 1, 4,
                                2, true, &repainted, &error));
    int edited = 0;
    const std::vector<uint8_t> repaintedBytes =
        RigExecTestEdited(asBuilt, [&](fb::RigExecWireFile *file) {
            for (auto &chain : file->geometry->chains) {
                for (auto &revision : chain.revisions) {
                    if (RigExecFormatPathText(*file, revision.moverPath) !=
                            skinPath ||
                        !revision.chunked || !revision.topology) {
                        continue;
                    }
                    revision.partitionTopology =
                        std::make_unique<fb::RigExecWireSkinTopology>(
                            *revision.topology);
                    revision.partitionSameAsTopology = false;
                    *revision.topology = repainted;
                    ++edited;
                }
            }
        });
    CHECK(edited == 1);

    const auto play = [&](const char *leg, const std::vector<uint8_t> &bytes,
                          bool stale) {
        const std::unique_ptr<RigExecRuntimeReader> reader =
            RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
        if (!reader) {
            std::printf("FAILED: %s, %s: %s\n", name, leg, error.c_str());
            ++failures;
            return;
        }
        const auto same = [&](const char *what, const RigExecRigPose &live) {
            std::vector<std::string> diffs;
            CHECK(RigExecCompareRuntimeOutputs(live, *reader, &diffs));
            for (const std::string &diff : diffs) {
                std::printf("  %s, %s, %s: %s\n", name, leg, what,
                            diff.c_str());
            }
        };
        CHECK(reader->Execute(&error));
        same("defaults", evaluator.Evaluate(time));
        CHECK(reader->GetPartitionStaleForTesting(skinPath) == stale);
        // Only J1 moves: authored on the stage, set on the binary.
        const RigExecTestSessionEdit edit(
            stage->GetAttributeAtPath(SdfPath(j1)), 25.0);
        CHECK(edit.IsSet());
        CHECK(reader->SetInput(j1, 25.0, &error) && reader->Execute(&error));
        same("J1 moved", evaluator.Evaluate(time));
        CHECK(reader->GetPartitionStaleForTesting(skinPath) == stale);
    };
    play("as built", asBuilt, false);

    // Topology is epoch state: live baked rebuilds once for the authored
    // repaint and cuts fresh keys, and still plays as the file does.
    const size_t builds = evaluator.GetBakedProgramBuildCount();
    stage->GetAttributeAtPath(SdfPath(skinPath + ".rigExec:jointIndices"))
        .Set(VtIntArray{1, 1, 1, 1});
    play("repainted", repaintedBytes, true);
    CHECK(evaluator.GetBakedProgramBuildCount() == builds + 1);

    if (always.empty()) {
        TfUnsetenv("RIGEXEC_BAKED_CHUNK_ALWAYS");
    } else {
        TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", always);
    }
    if (verts.empty()) {
        TfUnsetenv("RIGEXEC_BAKED_CHUNK_VERTS");
    } else {
        TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", verts);
    }
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

// One revision of each class RevisionStatic decides: a matrix, a blend shape
// and a dense wire whose inputs:defaultWeight leaves [0, 1] at frame 2 (a
// refused packet) and returns at full and partial strength, and a
// classicLinear skin whose joint weights go negative at frame 2 (the layout
// half of its validation refuses). At a partial weight the wire blends with
// the envelope RevisionStatic resolved.
static UsdStageRefPtr
_AcceptanceStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const bool imported = stage->GetRootLayer()->ImportFromString(R"USD(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 4
)
def Scope "Asset"
{
    def RigExecRoot "Rig"
    {
        def RigExecControl "Shift"
        {
            double avars:tx.timeSamples = {1: 1, 2: 2, 3: 3, 4: 4}
        }
        def RigExecControl "A"
        {
            double avars:rz.timeSamples = {1: 0, 2: 35, 3: -60, 4: 20}
        }
        def RigExecControl "B"
        {
            double avars:tx.timeSamples = {1: 0, 2: 2.5, 3: -4, 4: 1}
        }
        def Scope "BlendInputs"
        {
            def RigExecBlendInput "Raise"
            {
                float inputs:weight = 1
                rel rigExec:samples = </Asset/Rig/BlendInputs/Raise/Full>

                def RigExecBlendSample "Full"
                {
                    float rigExec:activation = 1
                    rel rigExec:targetPoints = </Asset/Targets/Raised.points>
                }
            }
        }
        def Scope "Movers"
        {
            def RigExecMatrixMover "Lift" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Geom/Lift.points>
                rel rigExec:transform = </Asset/Rig/Shift>
                float inputs:defaultWeight.timeSamples = {1: 0.5, 2: 1.5, 3: 1, 4: 0.25}
            }
            def RigExecBlendShapeMover "Blend" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Geom/Blend.points>
                rel rigExec:blendInputs = </Asset/Rig/BlendInputs/Raise>
                float inputs:defaultWeight.timeSamples = {1: 0.5, 2: 1.5, 3: 1, 4: 0.25}
            }
            def RigExecCurveMover "Wire" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:mode = "wire"
                rel rigExec:moves = </Asset/Geom/Wire.points>
                rel rigExec:driverCurve = </Asset/Curve>
                rel rigExec:bindCoordinates = </Asset/Bind.st>
                float inputs:defaultWeight.timeSamples = {1: 0.5, 2: 1.5, 3: 1, 4: 0.25}
            }
            def RigExecSkinMover "Skin" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Geom/Skin.points>
                rel rigExec:influences = [</Asset/Rig/A>, </Asset/Rig/B>]
                uniform int rigExec:elementSize = 2
                uniform token rigExec:skinningMethod = "classicLinear"
                int[] rigExec:jointIndices = [0, 1, 0, 1, 0, 1, 0, 1]
                # Compile checks the layout against the default value.
                float[] rigExec:jointWeights = [1, 0, 0.5, 0.5, 0.25, 0.75, 0, 1]
                float[] rigExec:jointWeights.timeSamples = {
                    1: [1, 0, 0.5, 0.5, 0.25, 0.75, 0, 1],
                    2: [1, 0, 0.5, 0.5, -0.25, 0.75, 0, 1],
                    3: [1, 0, 0.5, 0.5, 0.25, 0.75, 0, 1],
                    4: [0.5, 0.5, 0.5, 0.5, 0.25, 0.75, 0, 1],
                }
            }
        }
    }
    def Scope "Geom"
    {
        def Mesh "Lift"
        {
            point3f[] points = [(1, 0, 0), (0, 2, 0.5), (-1, 1, 2), (3, -1, 1)]
        }
        def Mesh "Blend"
        {
            point3f[] points = [(1, 0, 0), (0, 2, 0.5), (-1, 1, 2), (3, -1, 1)]
        }
        def Mesh "Wire"
        {
            point3f[] points = [(0.25, 0, 0), (0.5, 0.1, 0), (0.75, 0, 0), (1, 0.2, 0)]
        }
        def Mesh "Skin"
        {
            point3f[] points = [(1, 0, 0), (0, 2, 0.5), (-1, 1, 2), (3, -1, 1)]
        }
    }
    def Scope "Targets"
    {
        def Points "Raised"
        {
            point3f[] points = [(1, 0, 2), (0, 2, 2.5), (-1, 1, 4), (3, -1, 3)]
        }
    }
    def Scope "Bind"
    {
        float2[] st = [(0.25, 0), (0.5, 0.1), (0.75, 0), (1, 0.2)]
    }
    def NurbsCurves "Curve"
    {
        int[] curveVertexCounts = [2]
        int[] order = [2]
        double[] knots = [0, 0, 1, 1]
        point3f[] points = [(0, 0, 0), (1, 0, 0)]
        point3f[] points.timeSamples = {
            1: [(0, 0, 0), (1, 0.5, 0)],
            4: [(0, 0, 0), (1, 2, 0)],
        }
    }
}
)USD");
    CHECK(imported);
    return imported ? stage : UsdStageRefPtr();
}

// The decision each revision's packet carries is what its chunks' kernels
// answered, on the native program and in the file alike, while a revision
// fails its validation at frame 2 and recovers at frames 3 and 4. Native
// and the file play every frame bit for bit (_TestStage), and both decide
// the same way.
static void
TestAcceptanceFailureRecovers()
{
    const char *const name = "acceptance failure recovers";
    const std::vector<double> frames = {1.0, 2.0, 3.0, 4.0};
    const UsdStageRefPtr stage = _AcceptanceStage();
    if (!stage) {
        std::printf("%s: FAILED (no stage)\n", name);
        return;
    }
    std::vector<std::vector<RigExecRuntimePoints>> rows;
    _TestStage(name, stage, frames, &rows);
    _CheckFramesDiffer(name, rows);

    const SdfPath rigPath("/Asset/Rig");
    std::vector<uint8_t> bytes;
    std::string error;
    {
        RigExecRigEvaluator baker(stage, rigPath);
        CHECK(RigExecTestBakeAt(baker, frames.front(), &bytes, &error));
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        std::printf("%s: FAILED (open: %s)\n", name, error.c_str());
        CHECK(false);
        return;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    const char *const movers[] = {
        "/Asset/Rig/Movers/Lift", "/Asset/Rig/Movers/Blend",
        "/Asset/Rig/Movers/Wire", "/Asset/Rig/Movers/Skin"};
    size_t decided = 0;
    for (const double frame : frames) {
        CHECK(evaluator.Evaluate(frame).valid);
        CHECK(player.Play(frame, &error));
        const RigExecBakedProgram *program = evaluator.GetBakedProgram();
        CHECK(program != nullptr);
        if (!program) {
            return;
        }
        const RigExecBakedProgramImpl &B = program->GetStepGraph();
        const RigExecRevisionAcceptance expected =
            frame == 2.0 ? RigExecRevisionAcceptance::Refuses
                         : RigExecRevisionAcceptance::Applies;
        for (const char *mover : movers) {
            const SdfPath moverPath(mover);
            const RigExecBakedProgramImpl::GeomRevision *revision = nullptr;
            for (const auto &chain : B.chains) {
                for (const auto &candidate : chain.revisions) {
                    if (candidate.moverPath == moverPath) {
                        revision = &candidate;
                    }
                }
            }
            CHECK(revision != nullptr);
            if (!revision) {
                continue;
            }
            bool chunksOk = !revision->chunks.empty();
            for (const auto &chunk : revision->chunks) {
                chunksOk = chunksOk && chunk.ok;
            }
            int played = -1;
            bool playedOk = false;
            const bool found = player.Reader().GetRevisionDecisionForTesting(
                mover, &played, &playedOk);
            const bool agree =
                revision->acceptance == expected &&
                chunksOk == (expected == RigExecRevisionAcceptance::Applies) &&
                found && played == int(revision->acceptance) &&
                playedOk == chunksOk;
            CHECK(agree);
            if (!agree) {
                std::printf("%s frame %g, %s: native %d (chunks %s), file %d "
                            "(chunks %s), expected %d\n",
                            name, frame, mover, int(revision->acceptance),
                            chunksOk ? "ok" : "refused", played,
                            playedOk ? "ok" : "refused", int(expected));
            }
            ++decided;
        }
    }
    std::printf("%s: %zu decision(s) equal their kernels' answers\n", name,
                decided);
}

static void TestRetainedResultOwnership()
{
    RrRetainedArray<RrVec3f> producer;
    std::vector<RrVec3f> spare(1,RrVec3f(1,2,3));
    producer.swap(spare);
    const auto retained=producer;
    CHECK(retained.data()==producer.data());
    spare.assign(1,RrVec3f(4,5,6));producer.swap(spare);
    CHECK(retained.Read()[0]==RrVec3f(1,2,3));
    CHECK(producer.Read()[0]==RrVec3f(4,5,6));
    producer.clear();CHECK(producer.empty());
    CHECK(retained.size()==1);
    RrRetainedArray<float> weights;weights.Write().assign(1,0.25f);
    const auto retainedWeights=weights;
    weights.Write()[0]=0.75f;
    CHECK(retainedWeights.Read()[0]==0.25f);
    CHECK(weights.Read()[0]==0.75f);
}

int
main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestComputedCurrentPhase();
    TestChainPointVersions();
    TestRangeChainPlays();
    TestRangeChainRolesAndCutoff();
    TestPacketArrayVersions();
    TestGeometryDomainArm();
    TestCurrentPhaseThroughCombine();
    TestCurrentPhaseFailure();
    TestVolumeMixCurrentPhase();
    TestOraclePointsSet();
    TestOracleSlotIsCanonical();
    TestStageArrayOracleAndChain();
    TestDynamicSupportFromSlots();
    TestEmptyPlaneAxis();
    TestAnimatedStaticPointsReported();
    TestComputedOpenRefusals();
    TestSparseSkinTopology();
    TestSharedLayoutRule();
    TestLosslessLayoutPlaysAsLive();
    TestPathReadsFixture();
    TestPoseDrivenBlendWeights();
    TestBlendActivationDrag();
    TestBlendShapeLayouts();
    TestAcceptanceFailureRecovers();

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
    TestRetainedResultOwnership();
    TestRepaintedChunkLayoutRunsWhole();
    TestLayoutSetOnChunkedRevision();
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
