// Multi-stage imaging (docs/multistage-imaging.md; usdOrchestrate SPEC 14.1).
// Several stages imaged in ONE process, each through the REAL UsdImaging
// chain (UsdImagingCreateSceneIndices, exactly as usdview builds it), each
// with its own imaging context:
//   (a) two stages opened from the SAME file (identical prim paths) plus a
//       third, different rig stage: SetTime on one changes only that
//       stage's generation and published control frames;
//   (b) each chain serves its own stage's results for the same path, and
//       every stage-scoped read (guide bounds, moved floats, frame states,
//       warming counts) and write (overlay, triggers, cache clear) answers
//       for its own stage only;
//   (c) a preview on one stage -- rig lane and xform lane -- moves nothing
//       in the other's chain;
//   (d) deactivating one leaves the others active and evaluated;
//   (e) a stage with no rig: its chain binds, and a plain Xform preview
//       reaches only its chain;
//   (f) the legacy surface routes to the current context, and
//       GetInstance().Activate with a stage that already has a context
//       forwards to it;
//   (g) a destroyed stage releases its context, and the chain left behind
//       dangles nowhere -- with no manual Deactivate: an AUTOMATIC
//       activation is released with the last chain bound to it, a later
//       engine on the same stage activates it again, and an explicit
//       activation outlives its engines until its host deactivates it;
//   (h) a stage replaced under the same chain rebinds it, and the
//       automatic activation of the stage it left is released;
//   (i) a TouchPose highlight opened on one stage lights only its chain;
//   (j) a stage its host deactivated is never revived by the automatic
//       path (a stray rig-root pull), only by an explicit activation;
//   (k) warming counters survive Deactivate and re-activation, and a
//       context owns a scheduler (threads) only while a rig is active.
// No GL: every assertion reads the terminal scene index or the C surface.
#include "rigExecImaging/registry.h"
#include "rigExecImaging/sceneIndices.h"
#include "rigExecImaging/touchPose.h"
#include "rigExec/backgroundScheduler.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/imaging/hd/filteringSceneIndex.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/retainedSceneIndex.h"
#include "pxr/imaging/hd/sceneIndexObserver.h"
#include "pxr/imaging/hd/xformSchema.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xform.h"
#include "pxr/usd/usdUtils/stageCache.h"
#include "pxr/usdImaging/usdImaging/sceneIndices.h"
#include "pxr/usdImaging/usdImaging/stageSceneIndex.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

const SdfPath kTurret("/TurretAsset/Geom/Turret");
const SdfPath kTurretGeom("/TurretAsset/Geom");
const SdfPath kBarrel("/TurretAsset/Geom/Turret/Barrel");
const char *kTrackTarget = "/TurretAsset/Rig/Controls/TrackTarget";
const char *kTurretRig = "/TurretAsset/Rig";
const char *kTail1 = "/TailAsset/Rig/Controls/Tail1";

std::string
_SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

std::string
_ImagingResourceDir()
{
#ifdef RIGEXEC_IMAGING_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_IMAGING_RESOURCE_DIR);
#else
    return std::string();
#endif
}

// Records forwarded notices for assertion.
class _RecordingObserver : public HdSceneIndexObserver {
public:
    void PrimsAdded(const HdSceneIndexBase &,
                    const AddedPrimEntries &entries) override {
        for (const auto &e : entries) added.push_back(e.primPath);
    }
    void PrimsRemoved(const HdSceneIndexBase &,
                      const RemovedPrimEntries &entries) override {
        for (const auto &e : entries) removed.push_back(e.primPath);
    }
    void PrimsDirtied(const HdSceneIndexBase &,
                      const DirtiedPrimEntries &entries) override {
        for (const auto &e : entries) dirtied.push_back(e.primPath);
    }
    void PrimsRenamed(const HdSceneIndexBase &,
                      const RenamedPrimEntries &) override {}

    bool DirtiedUnder(const SdfPath &root) const {
        for (const SdfPath &path : dirtied) {
            if (path.HasPrefix(root)) {
                return true;
            }
        }
        return false;
    }

    SdfPathVector added, removed, dirtied;
};

// The RigExec results index inside a real chain, found by walking the
// filtering inputs back from the terminal.
RigExecResultsSceneIndex *
_FindResults(const HdSceneIndexBaseRefPtr &index, int depth = 0)
{
    if (!index || depth > 128) {
        return nullptr;
    }
    HdSceneIndexBase *raw = get_pointer(index);
    if (auto *results = dynamic_cast<RigExecResultsSceneIndex *>(raw)) {
        return results;
    }
    if (auto *filtering = dynamic_cast<HdFilteringSceneIndexBase *>(raw)) {
        for (const HdSceneIndexBaseRefPtr &input :
             filtering->GetInputScenes()) {
            if (RigExecResultsSceneIndex *found =
                    _FindResults(input, depth + 1)) {
                return found;
            }
        }
    }
    return nullptr;
}

// One usdview-shaped viewport: a stage in the stage cache, and the real
// UsdImaging chain over it.
struct _Viewport {
    UsdStageRefPtr stage;
    long long id = 0;
    UsdImagingSceneIndices indices;
    TfWeakPtr<RigExecResultsSceneIndex> results;

    HdSceneIndexBaseRefPtr Terminal() const {
        return indices.finalSceneIndex;
    }
};

std::unique_ptr<_Viewport>
_Open(const UsdStageRefPtr &stage)
{
    auto viewport = std::make_unique<_Viewport>();
    viewport->stage = stage;
    if (!stage) {
        return viewport;
    }
    viewport->id = UsdUtilsStageCache::Get().Insert(stage).ToLongInt();
    UsdImagingCreateSceneIndicesInfo info;
    info.stage = stage;
    viewport->indices = UsdImagingCreateSceneIndices(info);
    viewport->results = TfWeakPtr<RigExecResultsSceneIndex>(
        _FindResults(viewport->indices.finalSceneIndex));
    return viewport;
}

void
_Close(std::unique_ptr<_Viewport> &viewport)
{
    if (!viewport) {
        return;
    }
    if (viewport->stage) {
        UsdUtilsStageCache::Get().Erase(viewport->stage);
    }
    viewport.reset();
}

GfMatrix4d
_WorldOf(const HdSceneIndexBaseRefPtr &terminal, const SdfPath &path)
{
    const HdSceneIndexPrim prim = terminal->GetPrim(path);
    const HdXformSchema xform = HdXformSchema::GetFromParent(prim.dataSource);
    if (!xform || !xform.GetMatrix()) {
        return GfMatrix4d(0.0);
    }
    return xform.GetMatrix()->GetTypedValue(0.0f);
}

bool
_Close(const GfMatrix4d &a, const GfMatrix4d &b)
{
    return GfIsClose(a, b, 1e-6);
}

uint64_t
_KeyOf(const UsdStageRefPtr &stage)
{
    const RigExecImagingRegistry::Ptr context =
        RigExecImagingRegistry::ForStage(stage, /* create = */ false);
    return context ? context->GetKey() : 0;
}

RigExecImagingRegistry::Ptr
_ContextOf(const UsdStageRefPtr &stage)
{
    return RigExecImagingRegistry::ForStage(stage, /* create = */ false);
}

// Whether evaluations memoize into the frame cache in this environment.
bool
_ReadsOn()
{
    return RigExecFrameCacheModeFromEnvironment() !=
        RigExecFrameCacheMode::Off;
}

// Whether warming runs at all in this environment (the frame-cache tests
// guard the same way): reads on, and the background pool enabled.
bool
_WarmingRuns()
{
    return RigExecFrameCacheModeFromEnvironment() !=
               RigExecFrameCacheMode::Off &&
        RigExecBackgroundWarmingEnabled();
}

// The real UsdImaging chain over \p stage (one engine's worth), and its
// results index.
UsdImagingSceneIndices
_Engine(const UsdStageRefPtr &stage, TfWeakPtr<RigExecResultsSceneIndex> *out)
{
    UsdImagingCreateSceneIndicesInfo info;
    info.stage = stage;
    UsdImagingSceneIndices indices = UsdImagingCreateSceneIndices(info);
    if (out) {
        *out = TfWeakPtr<RigExecResultsSceneIndex>(
            _FindResults(indices.finalSceneIndex));
    }
    return indices;
}

bool
_HasGuideChild(const HdSceneIndexBaseRefPtr &terminal, const SdfPath &path)
{
    for (const SdfPath &child : terminal->GetChildPrimPaths(path)) {
        if (child.GetName().rfind("rigGuide", 0) == 0) {
            return true;
        }
    }
    return false;
}

bool
_HasTouchPrimvar(const HdSceneIndexBaseRefPtr &terminal, const SdfPath &path)
{
    const HdPrimvarsSchema primvars =
        HdPrimvarsSchema::GetFromParent(terminal->GetPrim(path).dataSource);
    return bool(primvars.GetPrimvar(TfToken("rigExecTouchRegion")));
}

UsdStageRefPtr
_PlainStage()
{
    // No rig at all: one Xform with a translate op, one child mesh.
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const UsdGeomXform plain =
        UsdGeomXform::Define(stage, SdfPath("/Plain"));
    plain.AddTranslateOp().Set(GfVec3d(1, 2, 3));
    const UsdGeomMesh child =
        UsdGeomMesh::Define(stage, SdfPath("/Plain/Child"));
    child.CreatePointsAttr(VtValue(VtVec3fArray{
        GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0)}));
    child.CreateFaceVertexCountsAttr(VtValue(VtIntArray{3}));
    child.CreateFaceVertexIndicesAttr(VtValue(VtIntArray{0, 1, 2}));
    return stage;
}

// A rig that warms in the background (a skin mover the bake supports; the
// examples' aim and FK rigs carry no baked program): two controls animated
// over frames 1-4 driving one mesh. Built the same way twice, it gives two
// stages with identical paths.
UsdStageRefPtr
_WarmableRig()
{
    const size_t pointCount = 8;
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    const UsdPrim root =
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    // The rig asks for the baked program: only a baked program can be
    // sampled for background warming.
    root.CreateAttribute(TfToken("rigExec:baked"), SdfValueTypeNames->Bool,
                         /* custom = */ false, SdfVariabilityUniform)
        .Set(true);
    const UsdPrim alongX = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongX"), TfToken("RigExecControl"));
    const UsdPrim alongY = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongY"), TfToken("RigExecControl"));
    UsdAttribute tx = alongX.GetAttribute(TfToken("avars:tx"));
    UsdAttribute ty = alongY.GetAttribute(TfToken("avars:ty"));
    for (int t = 1; t <= 4; ++t) {
        tx.Set(10.0 + 0.1 * double(t), UsdTimeCode(double(t)));
        ty.Set(20.0 - 0.05 * double(t), UsdTimeCode(double(t)));
    }
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const SdfPath meshPath("/Asset/Geom/Mesh_0");
    const UsdPrim mesh = stage->DefinePrim(meshPath, TfToken("Mesh"));
    VtVec3fArray points(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        points[i] = GfVec3f(float(i) * 0.5f, float(i) * -0.25f,
                            -float(i) * 0.125f);
    }
    mesh.GetAttribute(TfToken("points")).Set(points);
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin_0"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({meshPath.AppendProperty(TfToken("points"))});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({alongX.GetPath(), alongY.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(2);
    VtIntArray indices(pointCount * 2);
    VtFloatArray weights(pointCount * 2);
    for (size_t i = 0; i < pointCount; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
        weights[i * 2] = 0.1f;
        weights[i * 2 + 1] = 0.9f;
    }
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(indices);
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray).Set(weights);
    return stage;
}

void
TestMultiStage(const std::string &examplesDir)
{
    const std::string turretFile = examplesDir + "/10_AimXformTurret.usda";
    const std::string tailFile = examplesDir + "/01_FkChainTail.usda";

    // Isolated from whatever an earlier activation left current.
    RigExecImaging_Deactivate();

    std::unique_ptr<_Viewport> a = _Open(UsdStage::Open(turretFile));
    std::unique_ptr<_Viewport> b = _Open(UsdStage::Open(turretFile));
    std::unique_ptr<_Viewport> c = _Open(UsdStage::Open(tailFile));
    CHECK(a->stage && b->stage && c->stage);
    if (!a->stage || !b->stage || !c->stage) {
        return;
    }
    CHECK(a->stage != b->stage);
    CHECK(a->stage->GetRootLayer() == b->stage->GetRootLayer());
    CHECK(a->results && b->results && c->results);
    if (!a->results || !b->results || !c->results) {
        return;
    }

    // Every chain bound to its OWN stage's context through rigExec/stageKey,
    // and the three contexts are distinct.
    const uint64_t keyA = _KeyOf(a->stage);
    const uint64_t keyB = _KeyOf(b->stage);
    const uint64_t keyC = _KeyOf(c->stage);
    CHECK(keyA != 0 && keyB != 0 && keyC != 0);
    CHECK(keyA != keyB && keyB != keyC && keyA != keyC);
    CHECK(a->results->GetContextKey() == keyA);
    CHECK(b->results->GetContextKey() == keyB);
    CHECK(c->results->GetContextKey() == keyC);
    CHECK(a->results->GetStore() ==
          RigExecImagingRegistry::ForKey(keyA)->GetStore());

    // Population activated all three: nobody refused a second stage.
    CHECK(RigExecImaging_IsActiveForStage(a->id) == 1);
    CHECK(RigExecImaging_IsActiveForStage(b->id) == 1);
    CHECK(RigExecImaging_IsActiveForStage(c->id) == 1);

    // ...and all three activate explicitly through the id-taking C surface.
    CHECK(RigExecImaging_Activate(a->id, "", 1001.0) == 0);
    CHECK(RigExecImaging_Activate(b->id, "", 1001.0) == 0);
    CHECK(RigExecImaging_Activate(c->id, "", 1001.0) == 0);
    CHECK(RigExecImaging_IsActiveForStage(a->id) == 1);
    CHECK(RigExecImaging_IsActiveForStage(b->id) == 1);
    CHECK(RigExecImaging_IsActiveForStage(c->id) == 1);

    const long long genA0 = RigExecImaging_GetGenerationForStage(a->id);
    const long long genB0 = RigExecImaging_GetGenerationForStage(b->id);
    const long long genC0 = RigExecImaging_GetGenerationForStage(c->id);
    CHECK(genA0 > 0 && genB0 > 0 && genC0 > 0);
    const GfMatrix4d turretA1001 = _WorldOf(a->Terminal(), kTurret);
    const GfMatrix4d turretB1001 = _WorldOf(b->Terminal(), kTurret);
    CHECK(_Close(turretA1001, turretB1001));

    CHECK(RigExecImaging_SetTimeForStage(a->id, 1024.0) == 0);
    CHECK(RigExecImaging_GetGenerationForStage(a->id) == genA0 + 1);
    CHECK(RigExecImaging_GetGenerationForStage(b->id) == genB0);
    CHECK(RigExecImaging_GetGenerationForStage(c->id) == genC0);

    // A's published control frame is A's frame 1024; B still publishes 1001.
    double frameA[16], frameB[16];
    CHECK(RigExecImaging_GetControlFrameAssetSpace(
              a->id, kTrackTarget, 1024.0, 0, frameA) == 1);
    CHECK(RigExecImaging_GetControlFrameAssetSpace(
              b->id, kTrackTarget, 1024.0, 0, frameB) == 0);
    CHECK(RigExecImaging_GetControlFrameAssetSpace(
              b->id, kTrackTarget, 1001.0, 0, frameB) == 1);
    CHECK(RigExecImaging_GetControlFrameAssetSpace(
              a->id, kTrackTarget, 1001.0, 0, frameA) == 0);
    CHECK(RigExecImaging_GetControlFrameAssetSpace(
              a->id, kTrackTarget, 1024.0, 0, frameA) == 1);
    // The spline swings TrackTarget from x=-8 at 1001 to x=+8 at 1024.
    CHECK(std::abs(frameA[12] - frameB[12]) > 1.0);
    // C, a different rig, still publishes its own frame 1001.
    double frameC[16];
    CHECK(RigExecImaging_GetControlFrameAssetSpace(
              c->id, kTail1, 1001.0, 0, frameC) == 1);
    CHECK(RigExecImaging_GetControlFrameAssetSpace(
              c->id, kTail1, 1024.0, 0, frameC) == 0);
    CHECK(RigExecImaging_GetControlFrameAssetSpace(
              c->id, kTrackTarget, 1001.0, 0, frameC) == 0);
    CHECK(RigExecImaging_ContextCount() ==
          int(RigExecImagingRegistry::ContextCount()));

    const GfMatrix4d turretA1024 = _WorldOf(a->Terminal(), kTurret);
    CHECK(!_Close(turretA1024, turretA1001));
    CHECK(_Close(_WorldOf(b->Terminal(), kTurret), turretB1001));
    CHECK(!_Close(_WorldOf(a->Terminal(), kTurret),
                  _WorldOf(b->Terminal(), kTurret)));

    // Guide bounds: the same control path, each stage's own frame (A at
    // 1024, x=+8; B at 1001, x=-8).
    {
        double boundsA[6] = {0}, boundsB[6] = {0}, all[6] = {0};
        CHECK(RigExecImaging_GetGuideBoundsAssetSpaceForStage(
                  a->id, kTrackTarget, boundsA) == 1);
        CHECK(RigExecImaging_GetGuideBoundsAssetSpaceForStage(
                  b->id, kTrackTarget, boundsB) == 1);
        CHECK(boundsA[0] > 0.0 && boundsB[3] < 0.0);
        CHECK(RigExecImaging_GetAllGuideBoundsAssetSpaceForStage(
                  a->id, all) == 1);
        CHECK(all[3] >= boundsA[3]);
        CHECK(RigExecImaging_GetAllGuideBoundsAssetSpaceForStage(
                  b->id, all) == 1);
        CHECK(all[0] <= boundsB[0]);
        // The legacy spelling answers for the current context (C, the
        // last legacy activation), which has no such control.
        CHECK(RigExecImaging_GetGuideBoundsAssetSpace(kTrackTarget,
                                                       boundsA) == 0);
    }
    // Moved floats: the turret publishes none; a stage with no context and
    // an unknown id answer as an inactive registry.
    {
        const std::string packed = std::string(kTrackTarget) + ".avars:tx";
        float moved[1] = {7.0f};
        CHECK(RigExecImaging_GetMovedFloatsForStage(
                  a->id, packed.c_str(), moved, 1) == 0);
        CHECK(moved[0] == 0.0f);
        CHECK(RigExecImaging_GetMovedFloatsForStage(
                  -12345, packed.c_str(), moved, 1) == 0);
    }
    // The weight overlay: set on A republishes A only.
    {
        const long long genA = RigExecImaging_GetGenerationForStage(a->id);
        const long long genB = RigExecImaging_GetGenerationForStage(b->id);
        CHECK(RigExecImaging_SetWeightOverlayForStage(a->id, kTurretRig) == 0);
        CHECK(RigExecImaging_GetGenerationForStage(a->id) > genA);
        CHECK(RigExecImaging_GetGenerationForStage(b->id) == genB);
        CHECK(RigExecImaging_SetWeightOverlayForStage(a->id, "") == 0);
        CHECK(RigExecImaging_GetGenerationForStage(b->id) == genB);
        CHECK(RigExecImaging_SetWeightOverlayForStage(a->id, "not a path")
              != 0);
    }
    // Frame states and cache clears stay on their stage. (The example rigs
    // carry no baked program, so they never warm in the background; the
    // warming counters are pinned per stage in (k), on rigs that do.) Both
    // pools drain first, so nothing in flight moves a state under the
    // assertions.
    {
        const RigExecImagingRegistry::Ptr contextA = _ContextOf(a->stage);
        const RigExecImagingRegistry::Ptr contextB = _ContextOf(b->stage);
        CHECK(contextA && contextB);
        CHECK(contextA->MutableSchedulerProfiler() != nullptr);
        contextA->WaitUntilBackgroundIdle();
        contextB->WaitUntilBackgroundIdle();
        // B's playhead is 1001, A's 1024: each memoized its own.
        const std::vector<double> frames{1001.0, 1024.0};
        int statesA[2] = {-1, -1};
        int statesB0[2] = {-1, -1};
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  a->id, kTurretRig, frames.data(), statesA, 2) == 2);
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  b->id, kTurretRig, frames.data(), statesB0, 2) == 2);
        if (_ReadsOn()) {
            CHECK(statesA[1] == 2);
            CHECK(statesB0[0] == 2 && statesB0[1] == 0);
        }
        const long long warmedB0 =
            RigExecImaging_GetWarmingCompletedCountForStage(b->id);
        const long long genB = RigExecImaging_GetGenerationForStage(b->id);

        // A's triggers, warm range and clear leave B alone.
        CHECK(RigExecImaging_OnEditCommittedForStage(a->id) == 0);
        CHECK(RigExecImaging_OnIdleForStage(a->id) == 0);
        const double range[3] = {1030.0, 1031.0, 1032.0};
        CHECK(RigExecImaging_WarmRangeForStage(a->id, kTurretRig, range, 3)
              == 0);
        CHECK(RigExecImaging_WarmRangeForStage(a->id, "/Nope", range, 3)
              == -1);
        contextA->WaitUntilBackgroundIdle();
        CHECK(RigExecImaging_ClearFrameCacheForStage(a->id, kTurretRig) == 0);
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  a->id, kTurretRig, frames.data(), statesA, 2) == 2);
        CHECK(statesA[0] == 0 && statesA[1] == 0);
        int statesB[2] = {-1, -1};
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  b->id, kTurretRig, frames.data(), statesB, 2) == 2);
        CHECK(statesB[0] == statesB0[0] && statesB[1] == statesB0[1]);
        CHECK(RigExecImaging_GetWarmingCompletedCountForStage(b->id) ==
              warmedB0);
        CHECK(RigExecImaging_GetGenerationForStage(b->id) == genB);
        // C's rig is not A's: unknown on A.
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  a->id, "/TailAsset/Rig", frames.data(), statesA, 2) == -1);

        // The profile summary is per stage too (header plus A's rows).
        const std::string profile =
            (std::filesystem::temp_directory_path() /
             "testRigExecImagingMultiStage_profile.txt").string();
        CHECK(RigExecImaging_WriteProfileSummaryForStage(
                  a->id, profile.c_str()) == 0);
        CHECK(std::filesystem::exists(profile));
        std::error_code ignored;
        std::filesystem::remove(profile, ignored);
    }

    {
        _RecordingObserver observerB;
        b->Terminal()->AddObserver(HdSceneIndexObserverPtr(&observerB));

        // Rig lane: the aim target moves, A's turret re-aims, B's does not.
        const std::string avar = std::string(kTrackTarget) + ".avars:tx";
        CHECK(RigExecImaging_BeginPreviewForStage(a->id, avar.c_str()) == 1);
        const double moved = 30.0;
        CHECK(RigExecImaging_UpdatePreviewForStage(a->id, &moved, 1) == 0);
        CHECK(!_Close(_WorldOf(a->Terminal(), kTurret), turretA1024));
        CHECK(_Close(_WorldOf(b->Terminal(), kTurret), turretB1001));
        CHECK(RigExecImaging_GetGenerationForStage(b->id) == genB0);
        CHECK(RigExecImaging_EndPreviewForStage(a->id) == 0);
        CHECK(_Close(_WorldOf(a->Terminal(), kTurret), turretA1024));

        // Xform lane: /TurretAsset's own transform, 5 along X.
        const GfMatrix4d geomA = _WorldOf(a->Terminal(), kTurretGeom);
        const GfMatrix4d geomB = _WorldOf(b->Terminal(), kTurretGeom);
        CHECK(RigExecImaging_BeginPreviewForStage(
                  a->id, "/TurretAsset.xformOp:transform") == 16);
        GfMatrix4d placed(1.0);
        placed.SetTranslate(GfVec3d(105, 0, 0));
        double values[16];
        for (int r = 0; r < 4; ++r) {
            for (int col = 0; col < 4; ++col) {
                values[r * 4 + col] = placed[r][col];
            }
        }
        CHECK(RigExecImaging_UpdatePreviewForStage(a->id, values, 16) == 0);
        const GfMatrix4d previewedA = _WorldOf(a->Terminal(), kTurretGeom);
        CHECK(std::abs(previewedA.ExtractTranslation()[0] -
                       geomA.ExtractTranslation()[0] - 5.0) < 1e-9);
        CHECK(_Close(_WorldOf(b->Terminal(), kTurretGeom), geomB));
        CHECK(RigExecImaging_EndPreviewForStage(a->id) == 0);
        CHECK(_Close(_WorldOf(a->Terminal(), kTurretGeom), geomA));

        // Nothing of A's previews was even announced to B's chain.
        CHECK(!observerB.DirtiedUnder(SdfPath("/TurretAsset")));
        b->Terminal()->RemoveObserver(HdSceneIndexObserverPtr(&observerB));
    }

    RigExecImaging_DeactivateForStage(a->id);
    CHECK(RigExecImaging_IsActiveForStage(a->id) == 0);
    // A context without an active rig owns no threads; B's pool runs on.
    CHECK(_ContextOf(a->stage)->MutableSchedulerProfiler() == nullptr);
    CHECK(_ContextOf(b->stage)->MutableSchedulerProfiler() != nullptr);
    CHECK(RigExecImaging_OnIdleForStage(a->id) == 1);
    CHECK(RigExecImaging_OnEditCommittedForStage(a->id) == 1);
    CHECK(RigExecImaging_IsActiveForStage(b->id) == 1);
    CHECK(RigExecImaging_IsActiveForStage(c->id) == 1);
    CHECK(RigExecImaging_GetGenerationForStage(a->id) == 0);
    CHECK(!_Close(_WorldOf(a->Terminal(), kTurret), turretA1024));
    {
        const long long genB = RigExecImaging_GetGenerationForStage(b->id);
        CHECK(RigExecImaging_SetTimeForStage(b->id, 1024.0) == 0);
        CHECK(RigExecImaging_GetGenerationForStage(b->id) == genB + 1);
        // Same file, same frame: B now shows exactly what A showed.
        CHECK(_Close(_WorldOf(b->Terminal(), kTurret), turretA1024));
        const long long genC = RigExecImaging_GetGenerationForStage(c->id);
        CHECK(RigExecImaging_SetTimeForStage(c->id, 1024.0) == 0);
        CHECK(RigExecImaging_GetGenerationForStage(c->id) == genC + 1);
        // A deactivated context answers as an inactive registry.
        CHECK(RigExecImaging_SetTimeForStage(a->id, 1030.0) == 1);
    }

    {
        std::unique_ptr<_Viewport> d = _Open(_PlainStage());
        std::unique_ptr<_Viewport> e = _Open(_PlainStage());
        CHECK(d->results && e->results);
        const uint64_t keyD = _KeyOf(d->stage);
        CHECK(keyD != 0 && keyD != _KeyOf(e->stage));
        CHECK(d->results && d->results->GetContextKey() == keyD);
        CHECK(e->results && e->results->GetContextKey() == _KeyOf(e->stage));
        CHECK(RigExecImaging_IsActiveForStage(d->id) == 0);
        CHECK(_ContextOf(d->stage)->MutableSchedulerProfiler() == nullptr);
        CHECK(RigExecImaging_GetWarmingCompletedCountForStage(d->id) == 0);
        const SdfPath childPath("/Plain/Child");
        const GfMatrix4d childD = _WorldOf(d->Terminal(), childPath);
        const GfMatrix4d childE = _WorldOf(e->Terminal(), childPath);
        CHECK(std::abs(childD.ExtractTranslation()[0] - 1.0) < 1e-9);
        CHECK(RigExecImaging_BeginPreviewForStage(
                  d->id, "/Plain.xformOp:translate") == 3);
        const double translate[3] = {11.0, 2.0, 3.0};
        CHECK(RigExecImaging_UpdatePreviewForStage(d->id, translate, 3) == 0);
        CHECK(std::abs(_WorldOf(d->Terminal(), childPath)
                           .ExtractTranslation()[0] - 11.0) < 1e-9);
        CHECK(_Close(_WorldOf(e->Terminal(), childPath), childE));
        CHECK(RigExecImaging_EndPreviewForStage(d->id) == 0);
        CHECK(_Close(_WorldOf(d->Terminal(), childPath), childD));
        // A preview on a stage with no rig never activates anything.
        CHECK(RigExecImaging_IsActiveForStage(d->id) == 0);
        _Close(d);
        _Close(e);
    }

    {
        // The last legacy activation (C) is current.
        CHECK(RigExecImagingRegistry::Current() ==
              RigExecImagingRegistry::ForStage(c->stage, false));
        CHECK(RigExecImaging_GetGeneration() ==
              RigExecImaging_GetGenerationForStage(c->id));
        const long long genB = RigExecImaging_GetGenerationForStage(b->id);
        const long long genC = RigExecImaging_GetGenerationForStage(c->id);
        CHECK(RigExecImaging_SetTime(1030.0) == 0);
        CHECK(RigExecImaging_GetGenerationForStage(c->id) == genC + 1);
        CHECK(RigExecImaging_GetGenerationForStage(b->id) == genB);

        // GetInstance().Activate with a stage that already has a context:
        // forwarded to it, no new context, and it becomes current.
        RigExecImagingRegistry &legacy = RigExecImagingRegistry::GetInstance();
        const size_t count = RigExecImagingRegistry::ContextCount();
        std::vector<std::string> errors;
        CHECK(legacy.Activate(b->stage, SdfPath(), UsdTimeCode(1001.0),
                              &errors));
        CHECK(RigExecImagingRegistry::ContextCount() == count);
        const RigExecImagingRegistry::Ptr contextB =
            RigExecImagingRegistry::ForStage(b->stage, false);
        CHECK(RigExecImagingRegistry::Current() == contextB);
        CHECK(legacy.GetStore() == contextB->GetStore());
        CHECK(legacy.IsActive());
        CHECK(RigExecImaging_GetGeneration() ==
              RigExecImaging_GetGenerationForStage(b->id));
        // Nothing was replaced: C keeps evaluating.
        CHECK(RigExecImaging_IsActiveForStage(c->id) == 1);

        // A stage with no context yet: the legacy activation creates it.
        const UsdStageRefPtr fresh = UsdStage::Open(turretFile);
        CHECK(!RigExecImagingRegistry::ForStage(fresh, false));
        CHECK(legacy.Activate(fresh, SdfPath(), UsdTimeCode(1001.0),
                              &errors));
        CHECK(RigExecImagingRegistry::ContextCount() == count + 1);
        CHECK(RigExecImagingRegistry::Current() ==
              RigExecImagingRegistry::ForStage(fresh, false));

        // The legacy Deactivate deactivates the current context only.
        RigExecImaging_Deactivate();
        CHECK(!RigExecImagingRegistry::ForStage(fresh, false)->IsActive());
        CHECK(RigExecImaging_IsActiveForStage(b->id) == 1);
        CHECK(RigExecImaging_IsActiveForStage(c->id) == 1);

        // A failed legacy activation of another stage leaves the current
        // context -- and everything else -- exactly as it was.
        CHECK(legacy.Activate(b->stage, SdfPath(), UsdTimeCode(1001.0),
                              &errors));
        const UsdStageRefPtr empty = UsdStage::CreateInMemory();
        errors.clear();
        CHECK(!legacy.Activate(empty, SdfPath(), UsdTimeCode(1.0), &errors));
        CHECK(!errors.empty());
        CHECK(RigExecImagingRegistry::Current() == contextB);
        CHECK(legacy.IsActive());
    }

    {
        // A live explicit activation stays current throughout, so the
        // automatic activation below never becomes the legacy current one.
        CHECK(RigExecImaging_Activate(c->id, "", 1001.0) == 0);
        const size_t before = RigExecImagingRegistry::ContextCount();
        UsdStageRefPtr stage = UsdStage::Open(turretFile);
        UsdImagingSceneIndices indices;
        {
            UsdImagingCreateSceneIndicesInfo info;
            info.stage = stage;
            indices = UsdImagingCreateSceneIndices(info);
        }
        TfWeakPtr<RigExecResultsSceneIndex> results(
            _FindResults(indices.finalSceneIndex));
        CHECK(RigExecImagingRegistry::ContextCount() == before + 1);
        std::weak_ptr<RigExecImagingRegistry> weak =
            RigExecImagingRegistry::ForStage(stage, false);
        const uint64_t key = _KeyOf(stage);
        CHECK(key != 0 && results && results->GetContextKey() == key);
        CHECK(!weak.expired() && weak.lock()->IsActive());
        CHECK(RigExecImagingRegistry::Current() !=
              RigExecImagingRegistry::ForStage(stage, false));

        // The adapter activated it: the library owns that activation.
        CHECK(weak.lock()->IsAutoActivated());

        // A session closing with NO host deactivation (usdrecord, a plain
        // Hydra host): its engine goes -- UsdImaging's own chain holds the
        // stage until then -- and with it the last chain bound to the
        // context, which releases the automatic activation, its stage and
        // its threads. The stage then dies with its last holder.
        indices = UsdImagingSceneIndices();
        CHECK(!results);
        CHECK(!weak.expired() && !weak.lock()->IsActive());
        CHECK(!weak.expired() &&
              weak.lock()->MutableSchedulerProfiler() == nullptr);
        stage.Reset();
        CHECK(RigExecImagingRegistry::ContextCount() == before);
        CHECK(weak.expired());
        CHECK(!RigExecImagingRegistry::ForKey(key));
    }
    {
        // Released, not host-deactivated: a later engine on the same stage
        // activates it again. Two engines on one stage: the context stays
        // active until the last one goes.
        UsdStageRefPtr stage = UsdStage::Open(turretFile);
        TfWeakPtr<RigExecResultsSceneIndex> first, second;
        UsdImagingSceneIndices one = _Engine(stage, &first);
        const RigExecImagingRegistry::Ptr context = _ContextOf(stage);
        CHECK(context && context->IsActive() && context->IsAutoActivated());
        one = UsdImagingSceneIndices();
        CHECK(!first && !context->IsActive());
        one = _Engine(stage, &first);
        CHECK(context->IsActive());
        UsdImagingSceneIndices two = _Engine(stage, &second);
        CHECK(first && second && second->GetContextKey() == context->GetKey());
        one = UsdImagingSceneIndices();
        CHECK(context->IsActive());
        two = UsdImagingSceneIndices();
        CHECK(!context->IsActive());

        // An explicit activation takes the lifetime over: it outlives its
        // engine until the host deactivates it. The stage-scoped activation
        // never moves the legacy current context.
        const long long id =
            UsdUtilsStageCache::Get().Insert(stage).ToLongInt();
        one = _Engine(stage, &first);
        const RigExecImagingRegistry::Ptr current =
            RigExecImagingRegistry::Current();
        CHECK(RigExecImaging_ActivateForStage(id, "", 1001.0) == 0);
        CHECK(RigExecImagingRegistry::Current() == current);
        CHECK(context->IsActive() && !context->IsAutoActivated());
        one = UsdImagingSceneIndices();
        CHECK(context->IsActive());
        CHECK(RigExecImaging_ActivateForStage(id, "/Nope", 1001.0) == 3);
        CHECK(RigExecImaging_ActivateForStage(id, "not a path", 1001.0) == 2);
        CHECK(RigExecImaging_ActivateForStage(-12345, "", 1001.0) == 1);
        RigExecImaging_DeactivateForStage(id);
        CHECK(!context->IsActive());
        UsdUtilsStageCache::Get().Erase(stage);
    }
    {
        // No dangling chain: a chain that OUTLIVES its context (bound, then
        // its stage destroyed) resolves the stale key to nothing, stays
        // pullable, and rebinds cleanly. Hand-built exactly as the plugin
        // builds one, over a retained upstream that holds no stage.
        UsdStageRefPtr stage = _PlainStage();
        RigExecImagingRegistry::Ptr context =
            RigExecImagingRegistry::ForStage(stage, true);
        const uint64_t key = context->GetKey();
        std::weak_ptr<RigExecImagingRegistry> weak = context;
        const size_t before = RigExecImagingRegistry::ContextCount();

        HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
        auto pruning = RigExecInternalPrimPruningSceneIndex::New(upstream);
        auto xforms = RigExecXformOverrideSceneIndex::New(pruning);
        auto binding = RigExecBindingResolvingSceneIndex::New(xforms);
        auto results = RigExecResultsSceneIndex::New(
            binding, std::make_shared<RigExecSnapshotStore>());
        RigExecImagingRegistry::RegisterUnboundChain(
            pruning, binding, results, xforms);
        CHECK(results->GetContextKey() == 0);
        RigExecImagingRegistry::BindChain(get_pointer(results), key);
        CHECK(results->GetContextKey() == key);
        CHECK(results->GetStore() == context->GetStore());

        context.reset();
        stage.Reset();
        CHECK(RigExecImagingRegistry::ContextCount() == before - 1);
        CHECK(weak.expired());
        CHECK(!RigExecImagingRegistry::ForKey(key));
        CHECK(results->GetContextKey() == key);
        CHECK(results->GetStore() && !results->GetStore()->Get());
        upstream->AddPrims({{SdfPath("/Late"), TfToken("mesh"),
                             HdRetainedContainerDataSource::New()}});
        CHECK(results->GetPrim(SdfPath("/Late")).dataSource);
        CHECK(results->GetContextKey() == key);

        // ...and rebinds to a live context as any chain does.
        RigExecImagingRegistry::BindChain(get_pointer(results), keyC);
        CHECK(results->GetContextKey() == keyC);
        CHECK(results->GetStore() ==
              RigExecImagingRegistry::ForKey(keyC)->GetStore());
        CHECK(results->GetStore()->Get());
    }

    {
        std::unique_ptr<_Viewport> h = _Open(UsdStage::Open(turretFile));
        CHECK(h->results && h->results->GetContextKey() == _KeyOf(h->stage));
        const UsdStageRefPtr other = UsdStage::Open(tailFile);
        const long long otherId =
            UsdUtilsStageCache::Get().Insert(other).ToLongInt();
        h->indices.stageSceneIndex->SetStage(other);
        const uint64_t keyOther = _KeyOf(other);
        CHECK(keyOther != 0);
        CHECK(h->results && h->results->GetContextKey() == keyOther);
        CHECK(h->results && h->results->GetStore() ==
              RigExecImagingRegistry::ForKey(keyOther)->GetStore());
        CHECK(RigExecImaging_IsActiveForStage(otherId) == 1);
        // The chain now serves the tail rig: its joints draw guides.
        CHECK(_HasGuideChild(h->Terminal(),
                             SdfPath("/TailAsset/Rig/Joints/Seg1")));
        // The stage it left had an automatic activation and no chain any
        // more: released, so it pins nothing.
        CHECK(RigExecImaging_IsActiveForStage(h->id) == 0);
        CHECK(RigExecImaging_SetTimeForStage(h->id, 1030.0) == 1);
        CHECK(_ContextOf(h->stage) &&
              _ContextOf(h->stage)->MutableSchedulerProfiler() == nullptr);
        // ...and, explicitly re-activated, it no longer reaches the chain.
        _RecordingObserver observer;
        h->Terminal()->AddObserver(HdSceneIndexObserverPtr(&observer));
        CHECK(RigExecImaging_ActivateForStage(h->id, "", 1001.0) == 0);
        CHECK(RigExecImaging_IsActiveForStage(h->id) == 1);
        CHECK(RigExecImaging_SetTimeForStage(h->id, 1030.0) == 0);
        CHECK(!observer.DirtiedUnder(SdfPath("/TurretAsset")));
        observer.dirtied.clear();
        CHECK(RigExecImaging_SetTimeForStage(otherId, 1030.0) == 0);
        CHECK(observer.DirtiedUnder(SdfPath("/TailAsset")));
        h->Terminal()->RemoveObserver(HdSceneIndexObserverPtr(&observer));
        RigExecImaging_DeactivateForStage(otherId);
        RigExecImaging_DeactivateForStage(h->id);
        UsdUtilsStageCache::Get().Erase(other);
        _Close(h);
    }

    {
        std::unique_ptr<_Viewport> j = _Open(UsdStage::Open(turretFile));
        CHECK(RigExecImaging_IsActiveForStage(j->id) == 1);
        RigExecImaging_DeactivateForStage(j->id);
        CHECK(RigExecImaging_IsActiveForStage(j->id) == 0);
        // Announcing a stage swap, UsdImaging traverses the OLD stage and
        // pulls its rig root: that pull must not re-activate it (the
        // context would then pin the stage for the life of the process).
        j->indices.stageSceneIndex->SetStage(nullptr);
        CHECK(RigExecImaging_IsActiveForStage(j->id) == 0);
        // Nor does a full re-population...
        j->indices.stageSceneIndex->SetStage(j->stage);
        CHECK(RigExecImaging_IsActiveForStage(j->id) == 0);
        CHECK(j->results && j->results->GetContextKey() == _KeyOf(j->stage));
        // ...while an explicit activation does, as always.
        CHECK(RigExecImaging_Activate(j->id, "", 1001.0) == 0);
        CHECK(RigExecImaging_IsActiveForStage(j->id) == 1);
        RigExecImaging_DeactivateForStage(j->id);
        _Close(j);
    }

    // Two stages built identically (same prim paths) from a rig that warms
    // in the background. Warming on P counts, caches and clears on P only;
    // P's counters survive its Deactivate and re-activation.
    {
        std::unique_ptr<_Viewport> p = _Open(_WarmableRig());
        std::unique_ptr<_Viewport> q = _Open(_WarmableRig());
        const RigExecImagingRegistry::Ptr contextP = _ContextOf(p->stage);
        const RigExecImagingRegistry::Ptr contextQ = _ContextOf(q->stage);
        CHECK(contextP && contextQ && contextP != contextQ);
        CHECK(RigExecImaging_ActivateForStage(p->id, "", 2.0) == 0);
        CHECK(RigExecImaging_ActivateForStage(q->id, "", 2.0) == 0);
        CHECK(contextP->MutableSchedulerProfiler() != nullptr);
        contextQ->WaitUntilBackgroundIdle();
        const long long warmedQ =
            RigExecImaging_GetWarmingCompletedCountForStage(q->id);
        const std::vector<double> frames{1.0, 3.0, 4.0};
        int statesQ0[3] = {-1, -1, -1};
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  q->id, "/Asset/Rig", frames.data(), statesQ0, 3) == 3);

        CHECK(RigExecImaging_OnIdleForStage(p->id) == 0);
        contextP->WaitUntilBackgroundIdle();
        CHECK(RigExecImaging_OnEditCommittedForStage(p->id) == 0);
        contextP->WaitUntilBackgroundIdle();
        const long long warmed =
            RigExecImaging_GetWarmingCompletedCountForStage(p->id);
        int states[3] = {-1, -1, -1};
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  p->id, "/Asset/Rig", frames.data(), states, 3) == 3);
        if (_WarmingRuns()) {
            CHECK(warmed > 0);
            CHECK(states[0] == 2 && states[1] == 2 && states[2] == 2);
        }
        // Nothing of P's warming reached Q, whose rig has the same path.
        CHECK(RigExecImaging_GetWarmingCompletedCountForStage(q->id) ==
              warmedQ);
        int statesQ[3] = {-1, -1, -1};
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  q->id, "/Asset/Rig", frames.data(), statesQ, 3) == 3);
        CHECK(statesQ[0] == statesQ0[0] && statesQ[1] == statesQ0[1] &&
              statesQ[2] == statesQ0[2]);

        // P's clear drops P's frames only.
        CHECK(RigExecImaging_ClearFrameCacheForStage(p->id, "/Asset/Rig")
              == 0);
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  p->id, "/Asset/Rig", frames.data(), states, 3) == 3);
        CHECK(states[0] == 0 && states[1] == 0 && states[2] == 0);
        CHECK(RigExecImaging_GetFrameStatesForStage(
                  q->id, "/Asset/Rig", frames.data(), statesQ, 3) == 3);
        CHECK(statesQ[0] == statesQ0[0] && statesQ[1] == statesQ0[1] &&
              statesQ[2] == statesQ0[2]);

        // Deactivate stops P's pool; its counters stay.
        RigExecImaging_DeactivateForStage(p->id);
        CHECK(contextP->MutableSchedulerProfiler() == nullptr);
        const RigExecBackgroundSchedulerStats retired =
            contextP->GetBackgroundStats();
        CHECK(retired.queuedDepth == 0 && retired.running == 0);
        CHECK(RigExecImaging_GetWarmingCompletedCountForStage(p->id) ==
              warmed);
        // ...and a re-activation counts on from there.
        CHECK(RigExecImaging_ActivateForStage(p->id, "", 2.0) == 0);
        CHECK(RigExecImaging_GetWarmingCompletedCountForStage(p->id) >=
              warmed);
        CHECK(RigExecImaging_OnEditCommittedForStage(p->id) == 0);
        contextP->WaitUntilBackgroundIdle();
        if (_WarmingRuns()) {
            CHECK(RigExecImaging_GetWarmingCompletedCountForStage(p->id) >
                  warmed);
        }
        CHECK(RigExecImaging_GetWarmingCompletedCountForStage(q->id) ==
              warmedQ);
        RigExecImaging_DeactivateForStage(p->id);
        RigExecImaging_DeactivateForStage(q->id);
        _Close(p);
        _Close(q);
    }

    {
        CHECK(!_HasTouchPrimvar(a->Terminal(), kBarrel));
        CHECK(!_HasTouchPrimvar(b->Terminal(), kBarrel));
        const long long handle =
            RigExecTouchPose_Open(a->id, kBarrel.GetText());
        CHECK(handle > 0);
        if (handle > 0) {
            const int faces = RigExecTouchPose_GetFaceCount(handle);
            CHECK(faces > 0);
            std::vector<int32_t> regions(size_t(std::max(faces, 0)), 0);
            CHECK(RigExecTouchPose_SetFaceRegions(
                      handle, regions.data(), faces, 1) >= 0);
            CHECK(RigExecTouchPose_SetHighlightEnabled(handle, 1) == 0);
            CHECK(_HasTouchPrimvar(a->Terminal(), kBarrel));
            CHECK(!_HasTouchPrimvar(b->Terminal(), kBarrel));
            RigExecTouchPose_Close(handle);
            CHECK(!_HasTouchPrimvar(a->Terminal(), kBarrel));
        }
    }

    RigExecImaging_DeactivateForStage(b->id);
    RigExecImaging_DeactivateForStage(c->id);
    CHECK(RigExecImaging_IsActiveForStage(b->id) == 0);
    CHECK(RigExecImaging_IsActiveForStage(c->id) == 0);
    _Close(a);
    _Close(b);
    _Close(c);
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecImagingMultiStage <examplesDir>\n");
        return 2;
    }
    // Unbuffered: a crash must not swallow the lines that locate it.
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const std::string examplesDir = argv[1];
    const std::string resources = _SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    // The rig adapter's keyless registration and the scene-index plugins,
    // before any chain is built: the registries read Plug metadata once.
    const std::string imagingResources = _ImagingResourceDir();
    if (imagingResources.empty()) {
        std::printf("FATAL: the build did not provide the imaging plugInfo\n");
        return 2;
    }
    if (PlugRegistry::GetInstance().RegisterPlugins(
            imagingResources).empty()) {
        std::printf("FATAL: no imaging plugin found at %s\n",
                    imagingResources.c_str());
        return 2;
    }

    TestMultiStage(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecImagingMultiStage: all tests passed\n");
    return 0;
}
