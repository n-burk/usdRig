// RigExec Hydra scene-index tests (spec §10, §14.5 Hydra matrix):
// construction/pull goldens (pre-population attachment, populated
// wrapping with add-again resync, late-consumer traversal, legacy
// render-index pickup), live notice-stream filtering with locator
// preservation, narrow dependency-derived locators with
// ComputeDirtyLocators() sentinel expansion, motionBlurSupport
// capability matrix with one-sample non-motion profile and preflight
// failure without evaluation, derivative blocking, terminal-enumeration
// audit, and the real evaluator publishing the ArmShotAnim rig.
#include "rigExecImaging/bridge.h"
#include "rigExecImaging/playback.h"
#include "rigExecImaging/registry.h"
#include "rigExecImaging/sceneIndices.h"
#include "rigExecImaging/upstreamTable.h"
#include "rigExecBake/bake.h"
#include "rigExecBinary/format.h"
#include "rigExecMath/avarScale.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/bakedTrace.h"
#include "rigExec/frozenContext.h"
#include "rigExecRuntimeDrive.h"

#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/references.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/matrix4f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/diagnosticMgr.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/imaging/hd/basisCurvesSchema.h"
#include "pxr/imaging/hd/basisCurvesTopologySchema.h"
#include "pxr/imaging/hd/coneSchema.h"
#include "pxr/imaging/hd/containerDataSourceEditor.h"
#include "pxr/imaging/hd/cubeSchema.h"
#include "pxr/imaging/hd/dataSourceLocator.h"
#include "pxr/imaging/hd/extentSchema.h"
#include "pxr/imaging/hd/legacyDisplayStyleSchema.h"
#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
#include "pxr/imaging/hd/overlayContainerDataSource.h"
#include "pxr/imaging/hd/sphereSchema.h"
#include "pxr/imaging/hd/flattenedDataSourceProviders.h"
#include "pxr/imaging/hd/flatteningSceneIndex.h"
#include "pxr/imaging/hd/purposeSchema.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/renderIndex.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/retainedSceneIndex.h"
#include "pxr/imaging/hd/sceneIndexObserver.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hd/types.h"
#include "pxr/imaging/hd/unitTestNullRenderDelegate.h"
#include "pxr/imaging/hd/primOriginSchema.h"
#include "pxr/imaging/hd/visibilitySchema.h"
#include "pxr/imaging/hd/xformSchema.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/bboxCache.h"
#include "pxr/usd/usdGeom/boundable.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/primvarsAPI.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdUtils/stageCache.h"
#include "pxr/usdImaging/usdImaging/sceneIndices.h"
#include "pxr/usdImaging/usdImaging/stageSceneIndex.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
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
        for (const auto &e : entries) {
            dirtied.push_back(e.primPath);
            dirtiedLocators.push_back(e.dirtyLocators);
        }
    }
    void PrimsRenamed(const HdSceneIndexBase &,
                      const RenamedPrimEntries &) override {}

    SdfPathVector added, removed, dirtied;
    std::vector<HdDataSourceLocatorSet> dirtiedLocators;
};

static VtVec3fArray
_GetPointsPrimvar(const HdSceneIndexPrim &prim)
{
    HdPrimvarsSchema primvars =
        HdPrimvarsSchema::GetFromParent(prim.dataSource);
    HdPrimvarSchema points = primvars.GetPrimvar(HdTokens->points);
    if (HdSampledDataSourceHandle value = points.GetPrimvarValue()) {
        const VtValue v = value->GetValue(0.0f);
        if (v.IsHolding<VtVec3fArray>()) {
            return v.UncheckedGet<VtVec3fArray>();
        }
    }
    return VtVec3fArray();
}

// curveVertexCounts for a wire guide, faceVertexCounts for a mesh one;
// empty for the implicits, which carry no topology of their own.
static VtIntArray
_GuideTopologyCounts(const HdSceneIndexPrim &prim)
{
    VtIntArray counts;
    if (!prim.dataSource) {
        return counts;
    }
    if (HdBasisCurvesSchema curves =
            HdBasisCurvesSchema::GetFromParent(prim.dataSource)) {
        if (HdIntArrayDataSourceHandle ds =
                curves.GetTopology().GetCurveVertexCounts()) {
            counts = ds->GetTypedValue(0.0f);
        }
    } else if (HdMeshSchema mesh =
                   HdMeshSchema::GetFromParent(prim.dataSource)) {
        if (HdIntArrayDataSourceHandle ds =
                mesh.GetTopology().GetFaceVertexCounts()) {
            counts = ds->GetTypedValue(0.0f);
        }
    }
    return counts;
}

// The standard three-filter chain over one upstream (spec §10.1).
struct _Chain {
    RigExecInternalPrimPruningSceneIndexRefPtr pruning;
    RigExecXformOverrideSceneIndexRefPtr xforms;
    RigExecBindingResolvingSceneIndexRefPtr binding;
    RigExecResultsSceneIndexRefPtr results;
};

static _Chain
_BuildChain(const HdSceneIndexBaseRefPtr &upstream,
            const std::shared_ptr<RigExecSnapshotStore> &store,
            const std::set<SdfPath> &ownedScopes)
{
    _Chain chain;
    chain.pruning = RigExecInternalPrimPruningSceneIndex::New(upstream);
    chain.pruning->SetOwnedScopes(ownedScopes);
    // Same order as the real chain (sceneIndexPlugin.cpp): the preview filter
    // sits between pruning and binding, so every existing assertion here is
    // also the assertion that it is transparent when nothing is previewed.
    chain.xforms = RigExecXformOverrideSceneIndex::New(chain.pruning);
    chain.binding = RigExecBindingResolvingSceneIndex::New(chain.xforms);
    chain.results = RigExecResultsSceneIndex::New(chain.binding, store);
    return chain;
}

static HdRetainedSceneIndex::AddedPrimEntry
_MeshShell(const SdfPath &path)
{
    return {path, HdPrimTypeTokens->mesh,
            HdRetainedContainerDataSource::New(0, nullptr, nullptr)};
}

static HdRetainedSceneIndex::AddedPrimEntry
_MeshShellWithXform(const SdfPath &path, const GfMatrix4d &world)
{
    // resetXformStack true: upstream of these filters the transform is already
    // flattened, and that flag is how a composed matrix says so.
    static const TfToken xformName = HdXformSchemaTokens->xform;
    const HdDataSourceBaseHandle xformSource =
        HdXformSchema::Builder()
            .SetMatrix(
                HdRetainedTypedSampledDataSource<GfMatrix4d>::New(world))
            .SetResetXformStack(
                HdRetainedTypedSampledDataSource<bool>::New(true))
            .Build();
    return {path, HdPrimTypeTokens->mesh,
            HdRetainedContainerDataSource::New(1, &xformName, &xformSource)};
}

static HdRetainedSceneIndex::AddedPrimEntry
_GeneratedShell(const SdfPath &path)
{
    return {path, TfToken("RigExecMatrixPoint3fArrayMoverApplication"),
            HdRetainedContainerDataSource::New(0, nullptr, nullptr)};
}

// Recursively collects every prim path reachable by traversal.
static void
_Traverse(const HdSceneIndexBaseRefPtr &index, const SdfPath &root,
          SdfPathVector *out)
{
    for (const SdfPath &child : index->GetChildPrimPaths(root)) {
        out->push_back(child);
        _Traverse(index, child, out);
    }
}

// Pre-population attachment (spec §10.3.2, §14.5): the chain and its
// consumer attach before the upstream populates; population arrives as
// filtered PrimsAdded and the first pull works without any resync.
static void
TestPrePopulationAttachment()
{
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    auto store = std::make_shared<RigExecSnapshotStore>();
    const SdfPath generatedScope("/Asset/Rig/__RigExecGenerated");
    _Chain chain = _BuildChain(upstream, store, {generatedScope});

    _RecordingObserver observer;
    chain.results->AddObserver(HdSceneIndexObserverPtr(&observer));

    const SdfPath meshPath("/Asset/Geom/Body");
    HdRetainedSceneIndex::AddedPrimEntries entries;
    entries.push_back(_MeshShell(meshPath));
    entries.push_back(
        _GeneratedShell(generatedScope.AppendChild(TfToken("App_0"))));
    upstream->AddPrims(entries);

    bool sawMesh = false, sawOwned = false;
    for (const SdfPath &p : observer.added) {
        if (p == meshPath) sawMesh = true;
        if (p.HasPrefix(generatedScope)) sawOwned = true;
    }
    CHECK(sawMesh);
    CHECK(!sawOwned);
    CHECK(chain.results->GetPrim(meshPath).dataSource);

    chain.results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// Already-populated wrapping (spec §10.3.2, §14.5): the filters wrap a
// populated upstream; full traversal from / discovers everything except
// owned scopes, and an add-again resync of an existing path passes
// through to consumers.
static void
TestPopulatedWrappingAndResync()
{
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    const SdfPath meshPath("/Asset/Geom/Body");
    const SdfPath generatedScope("/Asset/Rig/__RigExecGenerated");
    HdRetainedSceneIndex::AddedPrimEntries entries;
    entries.push_back(_MeshShell(meshPath));
    entries.push_back(
        _GeneratedShell(generatedScope.AppendChild(TfToken("App_0"))));
    upstream->AddPrims(entries);

    auto store = std::make_shared<RigExecSnapshotStore>();
    _Chain chain = _BuildChain(upstream, store, {generatedScope});

    SdfPathVector traversed;
    _Traverse(chain.results, SdfPath::AbsoluteRootPath(), &traversed);
    bool sawMesh = false, sawOwned = false;
    for (const SdfPath &p : traversed) {
        if (p == meshPath) sawMesh = true;
        if (p.HasPrefix(generatedScope)) sawOwned = true;
    }
    CHECK(sawMesh);
    CHECK(!sawOwned);

    _RecordingObserver observer;
    chain.results->AddObserver(HdSceneIndexObserverPtr(&observer));
    HdRetainedSceneIndex::AddedPrimEntries again;
    again.push_back(_MeshShell(meshPath));
    upstream->AddPrims(again);
    bool resynced = false;
    for (const SdfPath &p : observer.added) {
        if (p == meshPath) resynced = true;
    }
    CHECK(resynced);
    chain.results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// Late-consumer traversal (spec §10.3.2, §14.5): generations publish
// before any consumer exists; a late consumer discovers cached results
// purely by traversal plus GetPrim, with no notices required.
static void
TestLateConsumerTraversal()
{
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    const SdfPath meshPath("/Asset/Geom/Body");
    HdRetainedSceneIndex::AddedPrimEntries entries;
    entries.push_back(_MeshShell(meshPath));
    upstream->AddPrims(entries);

    auto store = std::make_shared<RigExecSnapshotStore>();
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    snapshot->generation = 1;
    RigExecPublishedPrim &published = snapshot->prims[meshPath];
    published.hasPoints = true;
    published.points = {GfVec3f(0, 7, 0)};
    store->Publish(snapshot);

    _Chain chain = _BuildChain(upstream, store, {});
    SdfPathVector traversed;
    _Traverse(chain.results, SdfPath::AbsoluteRootPath(), &traversed);
    bool sawMesh = false;
    for (const SdfPath &p : traversed) {
        if (p == meshPath) sawMesh = true;
    }
    CHECK(sawMesh);
    CHECK(_GetPointsPrimvar(chain.results->GetPrim(meshPath)) ==
          published.points);
}

// Live PrimsAdded/PrimsRemoved/PrimsDirtied streams (spec §14.5): owned
// paths never surface, while unrelated entries pass through with their
// exact locator sets preserved.
static void
TestNoticeStreamFiltering()
{
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    const SdfPath meshPath("/Asset/Geom/Body");
    const SdfPath generatedScope("/Asset/Rig/__RigExecGenerated");
    const SdfPath generatedApp =
        generatedScope.AppendChild(TfToken("App_0"));
    HdRetainedSceneIndex::AddedPrimEntries entries;
    entries.push_back(_MeshShell(meshPath));
    entries.push_back(_GeneratedShell(generatedApp));
    upstream->AddPrims(entries);

    auto store = std::make_shared<RigExecSnapshotStore>();
    _Chain chain = _BuildChain(upstream, store, {generatedScope});
    _RecordingObserver observer;
    chain.results->AddObserver(HdSceneIndexObserverPtr(&observer));

    // Dirtied: the owned entry is dropped; the unrelated entry keeps its
    // exact locators.
    const HdDataSourceLocator visibility(TfToken("visibility"));
    HdSceneIndexObserver::DirtiedPrimEntries dirtied;
    dirtied.emplace_back(
        generatedApp, HdDataSourceLocatorSet::UniversalSet());
    dirtied.emplace_back(meshPath, HdDataSourceLocatorSet{visibility});
    upstream->DirtyPrims(dirtied);
    CHECK(observer.dirtied.size() == 1);
    if (observer.dirtied.size() == 1) {
        CHECK(observer.dirtied[0] == meshPath);
        CHECK(observer.dirtiedLocators[0].Contains(visibility));
        CHECK(!observer.dirtiedLocators[0].Contains(HdDataSourceLocator()));
    }

    // Removed: the owned entry is dropped; the unrelated entry passes.
    HdSceneIndexObserver::RemovedPrimEntries removed;
    removed.emplace_back(generatedApp);
    removed.emplace_back(meshPath);
    upstream->RemovePrims(removed);
    CHECK(observer.removed.size() == 1);
    if (observer.removed.size() == 1) {
        CHECK(observer.removed[0] == meshPath);
    }
    chain.results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// Narrow dependency-derived locators (spec §10.4 locator table, §14.5):
// value edits produce exact leaf locators expanded through every rebuilt
// __containerDataSource ancestor; structural output-set changes use
// universal dirtiness; an identical republication produces no notice.
static void
TestNarrowLocators()
{
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    const SdfPath meshPath("/Asset/Geom/Body");
    HdRetainedSceneIndex::AddedPrimEntries entries;
    entries.push_back(_MeshShell(meshPath));
    upstream->AddPrims(entries);

    auto store = std::make_shared<RigExecSnapshotStore>();
    _Chain chain = _BuildChain(upstream, store, {});
    _RecordingObserver observer;
    chain.results->AddObserver(HdSceneIndexObserverPtr(&observer));

    auto makeSnapshot = [&](uint64_t generation, const GfMatrix4d &xform,
                            const VtVec3fArray &points,
                            const GfVec3d &extentMax) {
        auto snapshot = std::make_shared<RigExecImagingSnapshot>();
        snapshot->generation = generation;
        RigExecPublishedPrim &p = snapshot->prims[meshPath];
        p.hasXform = true;
        p.xform = xform;
        p.hasPoints = true;
        p.points = points;
        p.hasExtent = true;
        p.extentMin = GfVec3d(0);
        p.extentMax = extentMax;
        return snapshot;
    };
    // Non-const: Publish derives hasDrivenXforms into the snapshot, so that
    // the fast path can never disagree with the prims it is guarding.
    auto publish = [&](const std::shared_ptr<RigExecImagingSnapshot>
                           &snapshot) {
        observer.dirtied.clear();
        observer.dirtiedLocators.clear();
        chain.results->NotifyGenerationPublished(store->Publish(snapshot));
    };

    const GfMatrix4d identity(1.0);
    const VtVec3fArray basePoints = {GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)};
    const VtVec3fArray movedPoints = {GfVec3f(0, 1, 0), GfVec3f(1, 1, 0)};

    // First publication of the prim is structural: universal dirtiness.
    publish(makeSnapshot(1, identity, basePoints, GfVec3d(1)));
    CHECK(observer.dirtied.size() == 1);
    if (!observer.dirtiedLocators.empty()) {
        CHECK(observer.dirtiedLocators[0].Contains(HdDataSourceLocator()));
    }

    // Points-only value change: the exact points leaf, the epoch-owned
    // derivative block entries, and the rebuilt container sentinels —
    // nothing for extent or xform.
    publish(makeSnapshot(2, identity, movedPoints, GfVec3d(1)));
    CHECK(observer.dirtied.size() == 1);
    if (!observer.dirtiedLocators.empty()) {
        const HdDataSourceLocatorSet &locators = observer.dirtiedLocators[0];
        CHECK(locators.Contains(HdDataSourceLocator(
            HdPrimvarsSchemaTokens->primvars, HdTokens->points,
            HdPrimvarSchemaTokens->primvarValue)));
        CHECK(locators.Contains(HdDataSourceLocator(
            HdPrimvarsSchemaTokens->primvars, HdTokens->velocities)));
        CHECK(locators.Contains(HdDataSourceLocator(
            HdPrimvarsSchemaTokens->primvars, HdTokens->accelerations)));
        CHECK(locators.Contains(HdDataSourceLocator(
            HdDataSourceLocatorSentinelTokens->container)));
        CHECK(locators.Contains(HdDataSourceLocator(
            HdPrimvarsSchemaTokens->primvars,
            HdDataSourceLocatorSentinelTokens->container)));
        CHECK(!locators.Intersects(
            HdDataSourceLocator(HdExtentSchemaTokens->extent)));
        CHECK(!locators.Intersects(
            HdDataSourceLocator(HdXformSchemaTokens->xform)));
        CHECK(!locators.Contains(HdDataSourceLocator()));
    }

    // Extent-only change: extent/min and extent/max, no primvars.
    publish(makeSnapshot(3, identity, movedPoints, GfVec3d(2)));
    CHECK(observer.dirtied.size() == 1);
    if (!observer.dirtiedLocators.empty()) {
        const HdDataSourceLocatorSet &locators = observer.dirtiedLocators[0];
        CHECK(locators.Contains(HdDataSourceLocator(
            HdExtentSchemaTokens->extent, HdExtentSchemaTokens->min)));
        CHECK(locators.Contains(HdDataSourceLocator(
            HdExtentSchemaTokens->extent, HdExtentSchemaTokens->max)));
        CHECK(!locators.Intersects(
            HdDataSourceLocator(HdPrimvarsSchemaTokens->primvars)));
        CHECK(!locators.Contains(HdDataSourceLocator()));
    }

    // Xform-only change: xform/matrix, nothing else.
    GfMatrix4d translated(1.0);
    translated.SetTranslate(GfVec3d(0, 3, 0));
    publish(makeSnapshot(4, translated, movedPoints, GfVec3d(2)));
    CHECK(observer.dirtied.size() == 1);
    if (!observer.dirtiedLocators.empty()) {
        const HdDataSourceLocatorSet &locators = observer.dirtiedLocators[0];
        CHECK(locators.Contains(HdDataSourceLocator(
            HdXformSchemaTokens->xform, HdXformSchemaTokens->matrix)));
        CHECK(!locators.Intersects(
            HdDataSourceLocator(HdPrimvarsSchemaTokens->primvars)));
        CHECK(!locators.Intersects(
            HdDataSourceLocator(HdExtentSchemaTokens->extent)));
    }

    // An identical republication produces no notice at all.
    publish(makeSnapshot(5, translated, movedPoints, GfVec3d(2)));
    CHECK(observer.dirtied.empty());

    // The prim leaving the published set is structural: universal.
    auto empty = std::make_shared<RigExecImagingSnapshot>();
    empty->generation = 6;
    publish(empty);
    CHECK(observer.dirtied.size() == 1);
    if (!observer.dirtiedLocators.empty()) {
        CHECK(observer.dirtiedLocators[0].Contains(HdDataSourceLocator()));
    }
    chain.results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// motionBlurSupport capability matrix and the one-sample non-motion
// profile (spec §10.3.1, §10.5, §14.5 motion cases).
static void
TestMotionCapabilityMatrix(const std::string &examplesDir)
{
    using MotionBlurSupport = RigExecImagingBridge::MotionBlurSupport;

    // Static preflight: capability false permits exactly one sample; an
    // empty offset set never renders; true/absent allow the profile.
    std::string whyNot;
    CHECK(!RigExecImagingBridge::PreflightMotionProfile(
        {}, MotionBlurSupport::True, &whyNot));
    CHECK(!whyNot.empty());
    CHECK(!RigExecImagingBridge::PreflightMotionProfile(
        {-0.25f, 0.25f}, MotionBlurSupport::False));
    CHECK(RigExecImagingBridge::PreflightMotionProfile(
        {0.0f}, MotionBlurSupport::False));
    CHECK(RigExecImagingBridge::PreflightMotionProfile(
        {-0.25f, 0.0f, 0.25f}, MotionBlurSupport::True));
    CHECK(RigExecImagingBridge::PreflightMotionProfile(
        {-0.25f, 0.0f, 0.25f}, MotionBlurSupport::Absent));
    CHECK(!RigExecImagingBridge::PreflightMotionProfile(
        {0.0f, 0.0f}, MotionBlurSupport::True));
    CHECK(!RigExecImagingBridge::PreflightMotionProfile(
        {1.0f, -1.0f}, MotionBlurSupport::True));
    CHECK(!RigExecImagingBridge::PreflightMotionProfile(
        {std::numeric_limits<float>::quiet_NaN()}, MotionBlurSupport::True));

    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmShotAnim.usda");
    CHECK(stage);
    if (!stage) return;
    RigExecImagingBridge bridge(stage, SdfPath("/Shot/HeroArm/Rig"));
    CHECK(bridge.Compile());

    const SdfPath bodyPath("/Shot/HeroArm/Geom/ArmBody");
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    HdRetainedSceneIndex::AddedPrimEntries entries;
    entries.push_back(_MeshShell(bodyPath));
    upstream->AddPrims(entries);
    _Chain chain = _BuildChain(
        upstream, bridge.GetStore(), {bridge.GetGeneratedScope()});
    bridge.SetSceneIndices(chain.binding, chain.results);

    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
    const uint64_t baseline = bridge.GetStore()->Get()->generation;

    // Capability false + multi-sample profile: preflight fails before
    // evaluation; nothing publishes and the retained generation serves.
    const auto refused = bridge.EvaluateAndPublishSamples(
        UsdTimeCode(1013), {-0.5f, 0.0f, 0.5f}, MotionBlurSupport::False);
    CHECK(!refused.ok);
    CHECK(bridge.GetStore()->Get()->generation == baseline);

    // Capability true: the explicit render-preflight set publishes exact
    // retained frame-relative offsets under one generation fence.
    const auto motion = bridge.EvaluateAndPublishSamples(
        UsdTimeCode(1013), {-0.5f, 0.0f, 0.5f}, MotionBlurSupport::True);
    CHECK(motion.ok);
    chain.results->NotifyGenerationPublished(motion.dirtied);
    {
        HdPrimvarsSchema primvars = HdPrimvarsSchema::GetFromParent(
            chain.results->GetPrim(bodyPath).dataSource);
        HdSampledDataSourceHandle value =
            primvars.GetPrimvar(HdTokens->points).GetPrimvarValue();
        CHECK(value);
        if (value) {
            std::vector<HdSampledDataSource::Time> times;
            CHECK(value->GetContributingSampleTimesForInterval(
                -0.5f, 0.5f, &times));
            CHECK(times.size() == 3);
        }
    }

    // Capability false + one-sample profile: a single-sample non-motion
    // generation whose source reports no sample interval.
    const auto single = bridge.EvaluateAndPublishSamples(
        UsdTimeCode(1024), {0.0f}, MotionBlurSupport::False);
    CHECK(single.ok);
    chain.results->NotifyGenerationPublished(single.dirtied);
    {
        HdPrimvarsSchema primvars = HdPrimvarsSchema::GetFromParent(
            chain.results->GetPrim(bodyPath).dataSource);
        HdSampledDataSourceHandle value =
            primvars.GetPrimvar(HdTokens->points).GetPrimvarValue();
        CHECK(value);
        if (value) {
            std::vector<HdSampledDataSource::Time> times;
            CHECK(!value->GetContributingSampleTimesForInterval(
                -0.5f, 0.5f, &times));
        }
    }
}

// Recursive container-name audit: no RigExec name crosses the renderer
// boundary (spec §14.5 terminal enumeration).
static void
_AuditContainerNames(const HdContainerDataSourceHandle &container,
                     int depth)
{
    if (!container || depth > 4) {
        return;
    }
    for (const TfToken &name : container->GetNames()) {
        const std::string &s = name.GetString();
        CHECK(s.find("rigExec") == std::string::npos &&
              s.find("RigExec") == std::string::npos);
        if (HdContainerDataSourceHandle child =
                HdContainerDataSource::Cast(container->Get(name))) {
            _AuditContainerNames(child, depth + 1);
        }
    }
}

// Legacy adapter pickup (spec §10.3.2, §14.5): a render index built for
// backend emulation over the terminal chain picks up the published mesh
// with no RigExec code, while owned scopes stay invisible; terminal
// enumeration exposes only standard names.
static void
TestLegacyRenderIndexPickup(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmShotAnim.usda");
    CHECK(stage);
    if (!stage) return;
    RigExecImagingBridge bridge(stage, SdfPath("/Shot/HeroArm/Rig"));
    CHECK(bridge.Compile());

    const SdfPath bodyPath("/Shot/HeroArm/Geom/ArmBody");
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    HdRetainedSceneIndex::AddedPrimEntries entries;
    entries.push_back(_MeshShell(bodyPath));
    entries.push_back(_GeneratedShell(
        bridge.GetGeneratedScope().AppendChild(TfToken("App_0"))));
    upstream->AddPrims(entries);
    _Chain chain = _BuildChain(
        upstream, bridge.GetStore(), {bridge.GetGeneratedScope()});
    bridge.SetSceneIndices(chain.binding, chain.results);
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));

    // Terminal enumeration: only standard names cross the boundary.
    _AuditContainerNames(chain.results->GetPrim(bodyPath).dataSource, 0);

    // Legacy pickup through Hydra's render-index emulation: the render
    // index observes the inserted terminal chain and populates rprims;
    // RigExec supplies no renderer code.
    Hd_UnitTestNullRenderDelegate renderDelegate;
    HdRenderIndex *renderIndex =
        HdRenderIndex::New(&renderDelegate, HdDriverVector());
    CHECK(renderIndex);
    if (renderIndex) {
        renderIndex->InsertSceneIndex(
            chain.results, SdfPath::AbsoluteRootPath());
        const SdfPathVector rprims =
            renderIndex->GetRprimSubtree(SdfPath::AbsoluteRootPath());
        bool sawBody = false, sawOwned = false;
        for (const SdfPath &p : rprims) {
            if (p == bodyPath) sawBody = true;
            if (p.HasPrefix(bridge.GetGeneratedScope())) sawOwned = true;
        }
        CHECK(sawBody);
        CHECK(!sawOwned);
        delete renderIndex;
    }
}

// The filter chain over a synthetic retained upstream: overlay strength,
// derivative blocks, pruning of owned scopes from queries and notices.
static void
TestFilterChainOverRetainedScene()
{
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();

    // Upstream mesh shell with authored points and velocities.
    const SdfPath meshPath("/Asset/Geom/Body");
    const VtVec3fArray upstreamPoints = {
        GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)};
    const VtVec3fArray upstreamVelocities = {
        GfVec3f(0, 9, 0), GfVec3f(0, 9, 0)};
    HdRetainedSceneIndex::AddedPrimEntries entries;
    {
        TfTokenVector primvarNames = {HdTokens->points,
                                      HdTokens->velocities};
        std::vector<HdDataSourceBaseHandle> primvarValues = {
            HdPrimvarSchema::Builder()
                .SetPrimvarValue(
                    HdRetainedTypedSampledDataSource<VtVec3fArray>::New(
                        upstreamPoints))
                .Build(),
            HdPrimvarSchema::Builder()
                .SetPrimvarValue(
                    HdRetainedTypedSampledDataSource<VtVec3fArray>::New(
                        upstreamVelocities))
                .Build()};
        const TfToken names[] = {HdPrimvarsSchemaTokens->primvars};
        const HdDataSourceBaseHandle values[] = {
            HdRetainedContainerDataSource::New(
                primvarNames.size(), primvarNames.data(),
                primvarValues.data())};
        entries.push_back(
            {meshPath, HdPrimTypeTokens->mesh,
             HdRetainedContainerDataSource::New(1, names, values)});
    }
    // A generated scope that must be pruned.
    const SdfPath generatedScope("/Asset/Rig/__RigExecGenerated");
    entries.push_back(
        {generatedScope.AppendChild(TfToken("App_0")),
         TfToken("RigExecMatrixPoint3fArrayMoverApplication"),
         HdRetainedContainerDataSource::New(0, nullptr, nullptr)});
    upstream->AddPrims(entries);

    auto store = std::make_shared<RigExecSnapshotStore>();
    auto pruning = RigExecInternalPrimPruningSceneIndex::New(upstream);
    pruning->SetOwnedScopes({generatedScope});
    auto binding = RigExecBindingResolvingSceneIndex::New(pruning);
    auto results = RigExecResultsSceneIndex::New(binding, store);

    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));

    // Pruning: owned paths vanish from queries...
    CHECK(!results->GetPrim(generatedScope.AppendChild(TfToken("App_0")))
               .dataSource);
    SdfPathVector rigChildren =
        results->GetChildPrimPaths(SdfPath("/Asset/Rig"));
    for (const SdfPath &child : rigChildren) {
        CHECK(child != generatedScope);
    }
    // ...and from notices, while unrelated entries pass through.
    {
        HdRetainedSceneIndex::AddedPrimEntries more;
        more.push_back(
            {generatedScope.AppendChild(TfToken("App_1")),
             TfToken("RigExecMatrixPoint3fArrayMoverApplication"),
             HdRetainedContainerDataSource::New(0, nullptr, nullptr)});
        more.push_back(
            {SdfPath("/Asset/Geom/Other"), HdPrimTypeTokens->mesh,
             HdRetainedContainerDataSource::New(0, nullptr, nullptr)});
        upstream->AddPrims(more);
        bool sawOwned = false, sawOther = false;
        for (const SdfPath &p : observer.added) {
            if (p.HasPrefix(generatedScope)) sawOwned = true;
            if (p == SdfPath("/Asset/Geom/Other")) sawOther = true;
        }
        CHECK(!sawOwned);
        CHECK(sawOther);
    }

    // Before any publication, upstream values pass through untouched.
    CHECK(_GetPointsPrimvar(results->GetPrim(meshPath)) == upstreamPoints);

    // Publish a generation: the owned points leaf wins, and the complete
    // velocities entry is blocked (ownership, not value comparison).
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    snapshot->generation = 1;
    RigExecPublishedPrim &published = snapshot->prims[meshPath];
    published.hasPoints = true;
    published.points = {GfVec3f(0, 5, 0), GfVec3f(1, 5, 0)};
    published.hasExtent = true;
    published.extentMin = GfVec3d(0, 5, 0);
    published.extentMax = GfVec3d(1, 5, 0);
    observer.dirtied.clear();
    results->NotifyGenerationPublished(store->Publish(snapshot));

    const HdSceneIndexPrim prim = results->GetPrim(meshPath);
    CHECK(_GetPointsPrimvar(prim) == published.points);

    // Velocities: the stronger block masks the authored upstream entry —
    // the composed container must expose NO velocities value (the overlay
    // resolves a stronger HdBlockDataSource to null for consumers).
    {
        HdContainerDataSourceHandle primvars =
            HdContainerDataSource::Cast(
                prim.dataSource->Get(HdPrimvarsSchemaTokens->primvars));
        CHECK(primvars);
        if (primvars) {
            HdDataSourceBaseHandle velocities =
                primvars->Get(HdTokens->velocities);
            const bool masked =
                !velocities || HdBlockDataSource::Cast(velocities);
            CHECK(masked);
        }
    }

    // Extent overlays as GfVec3d min/max.
    {
        HdExtentSchema extent =
            HdExtentSchema::GetFromParent(prim.dataSource);
        CHECK(extent.GetMin() &&
              extent.GetMin()->GetTypedValue(0.0f) == GfVec3d(0, 5, 0));
        CHECK(extent.GetMax() &&
              extent.GetMax()->GetTypedValue(0.0f) == GfVec3d(1, 5, 0));
    }

    // Precise dirtied notice for the published prim.
    bool sawMeshDirty = false;
    for (size_t i = 0; i < observer.dirtied.size(); ++i) {
        if (observer.dirtied[i] == meshPath) {
            sawMeshDirty = true;
            CHECK(observer.dirtiedLocators[i].Intersects(
                HdDataSourceLocator(HdPrimvarsSchemaTokens->primvars)));
        }
    }
    CHECK(sawMeshDirty);

    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// The real evaluator publishing the animated shot through the chain:
// evaluation completes before Hydra pulls, and per-frame publication
// moves the published points.
static void
TestBridgeOverShotStage(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmShotAnim.usda");
    CHECK(stage);
    if (!stage) return;

    const SdfPath rigPath("/Shot/HeroArm/Rig");
    RigExecImagingBridge bridge(stage, rigPath);
    std::vector<std::string> errors;
    const bool compiled = bridge.Compile(&errors);
    for (const std::string &e : errors) {
        std::printf("compile error: %s\n", e.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;

    // Minimal upstream shell for the deforming mesh (the full UsdImaging
    // chain supplies this in the application; the filters only need the
    // prim shell, spec §10.3.1).
    const SdfPath bodyPath("/Shot/HeroArm/Geom/ArmBody");
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    HdRetainedSceneIndex::AddedPrimEntries entries;
    entries.push_back({bodyPath, HdPrimTypeTokens->mesh,
                       HdRetainedContainerDataSource::New(0, nullptr,
                                                          nullptr)});
    // Joint/solver prim shells: guides only synthesize beneath parents
    // that exist upstream (UsdImaging supplies these in the application).
    // The wrist is INVISIBLE upstream, as a flattened UsdImaging prim would
    // be if it or any ancestor were hidden. Its synthesized guides must
    // follow: they are created downstream of the flattening that resolves
    // inherited visibility, so nothing else can make them.
    // ...and it carries a primOrigin naming its own stage path, as every
    // UsdImaging prim does. A guide has no USD counterpart, so this is the
    // only thing that lets picking one resolve to anything.
    static const TfToken wristInheritedNames[] = {
        HdVisibilitySchema::GetSchemaToken(),
        HdPrimOriginSchema::GetSchemaToken()};
    const SdfPath wristStagePath(
        "/Shot/HeroArm/Rig/Joints/Shoulder/Elbow/Wrist");
    const HdDataSourceBaseHandle wristInherited[] = {
        HdVisibilitySchema::Builder()
            .SetVisibility(HdRetainedTypedSampledDataSource<bool>::New(false))
            .Build(),
        HdRetainedContainerDataSource::New(
            HdPrimOriginSchemaTokens->scenePath,
            HdRetainedTypedSampledDataSource<
                HdPrimOriginSchema::OriginPath>::New(
                    HdPrimOriginSchema::OriginPath(wristStagePath)))};
    entries.push_back({wristStagePath, TfToken(),
                       HdRetainedContainerDataSource::New(
                           2, wristInheritedNames, wristInherited)});
    entries.push_back({SdfPath("/Shot/HeroArm/Rig/Solvers/FK"), TfToken(),
                       HdRetainedContainerDataSource::New(0, nullptr,
                                                          nullptr)});
    upstream->AddPrims(entries);

    auto pruning = RigExecInternalPrimPruningSceneIndex::New(upstream);
    pruning->SetOwnedScopes({bridge.GetGeneratedScope()});
    auto binding = RigExecBindingResolvingSceneIndex::New(pruning);
    auto results = RigExecResultsSceneIndex::New(binding, bridge.GetStore());
    bridge.SetSceneIndices(binding, results);

    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));

    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
    const VtVec3fArray restPoints =
        _GetPointsPrimvar(results->GetPrim(bodyPath));
    CHECK(restPoints.size() == 4);

    // Frame 1024: full IK toward the animated goal moves the published
    // points; the epoch swap and value notices both arrived.
    observer.dirtied.clear();
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
    const VtVec3fArray posedPoints =
        _GetPointsPrimvar(results->GetPrim(bodyPath));
    CHECK(posedPoints.size() == 4);
    bool moved = false;
    for (size_t i = 0; i < posedPoints.size() && i < restPoints.size();
         ++i) {
        if ((GfVec3d(posedPoints[i]) - GfVec3d(restPoints[i])).GetLength() >
            1e-4) {
            moved = true;
        }
    }
    CHECK(moved);
    bool sawBodyDirty = false;
    for (const SdfPath &p : observer.dirtied) {
        if (p == bodyPath) sawBodyDirty = true;
    }
    CHECK(sawBodyDirty);

    // Guide drawing (IrJointScope-aligned): joints and aggregate solvers
    // grow synthesized sphere/cone guide children whose xforms follow the
    // posed frames per generation.
    {
        const SdfPath wristPath("/Shot/HeroArm/Rig/Joints/Shoulder/Elbow/Wrist");
        const SdfPath wristSphere =
            wristPath.AppendChild(TfToken("rigGuideSphere_0"));
        const SdfPathVector wristKids =
            results->GetChildPrimPaths(wristPath);
        bool sawSphere = false, sawCone = false;
        for (const SdfPath &p : wristKids) {
            if (p == wristSphere) sawSphere = true;
            if (p.GetName() == "rigGuideCone_0") sawCone = true;
        }
        // Wrist is a leaf: hierarchy-derived joint links stop there.
        CHECK(sawSphere && !sawCone);

        const HdSceneIndexPrim sphere = results->GetPrim(wristSphere);
        CHECK(sphere.primType == HdPrimTypeTokens->sphere);
        CHECK(sphere.dataSource);
        if (sphere.dataSource) {
            HdPurposeSchema purpose =
                HdPurposeSchema::GetFromParent(sphere.dataSource);
            CHECK(purpose.GetPurpose() &&
                  purpose.GetPurpose()->GetTypedValue(0.0f) ==
                      HdRenderTagTokens->guide);
        }
        auto guideMatrix = [&]() {
            HdXformSchema xf = HdXformSchema::GetFromParent(
                results->GetPrim(wristSphere).dataSource);
            return xf.GetMatrix() ? xf.GetMatrix()->GetTypedValue(0.0f)
                                  : GfMatrix4d(1.0);
        };
        const GfMatrix4d posed = guideMatrix();
        observer.dirtied.clear();
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        const GfMatrix4d rest = guideMatrix();
        CHECK(posed != rest);
        bool sawGuideDirty = false;
        for (const SdfPath &p : observer.dirtied) {
            if (p == wristSphere) sawGuideDirty = true;
        }
        CHECK(sawGuideDirty);

        // The guide inherits its parent's visibility. The wrist is hidden
        // upstream, so its guides must be hidden too -- a synthesized prim
        // with no visibility of its own falls back to VISIBLE, which left
        // guides floating over a hidden rig.
        {
            HdVisibilitySchema vis = HdVisibilitySchema::GetFromParent(
                results->GetPrim(wristSphere).dataSource);
            const bool visible = !vis || !vis.GetVisibility() ||
                                 vis.GetVisibility()->GetTypedValue(0.0f);
            if (visible) {
                std::printf("  guide under a hidden joint is still visible "
                            "-- inherited visibility not applied\n");
            }
            CHECK(!visible);
        }

        // ...and picking a guide must resolve to the JOINT. A synthesized
        // prim has no USD path of its own, so without the parent's
        // primOrigin a click on a guide selects nothing at all.
        {
            HdPrimOriginSchema origin = HdPrimOriginSchema::GetFromParent(
                results->GetPrim(wristSphere).dataSource);
            const SdfPath resolved =
                origin ? origin.GetOriginPath(
                             HdPrimOriginSchemaTokens->scenePath)
                       : SdfPath();
            if (resolved != wristPath) {
                std::printf("  guide has no primOrigin -- picking it "
                            "resolves to nothing selectable\n");
            }
            CHECK(resolved == wristPath);
        }

        // Aggregate solver guides: the FK chain publishes three frames.
        const SdfPath fkCone2 =
            SdfPath("/Shot/HeroArm/Rig/Solvers/FK")
                .AppendChild(TfToken("rigGuideCone_2"));
        CHECK(results->GetPrim(fkCone2).primType == HdPrimTypeTokens->cone);

        // Notice reconciliation: removing the joint shell upstream drops
        // its synthesized guides; re-adding it re-announces them.
        observer.added.clear();
        observer.removed.clear();
        upstream->RemovePrims({{wristPath}});
        CHECK(!results->GetPrim(wristSphere).dataSource);
        upstream->AddPrims(
            {{wristPath, TfToken(),
              HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});
        bool reAnnounced = false;
        for (const SdfPath &p : observer.added) {
            if (p == wristSphere) reAnnounced = true;
        }
        CHECK(reAnnounced);
        CHECK(results->GetPrim(wristSphere).dataSource);

        // Batched removal of multiple authored collisions re-announces
        // each revealed synthesized guide exactly once.
        const SdfPath wristCone =
            wristPath.AppendChild(TfToken("rigGuideCone_0"));
        upstream->AddPrims(
            {{wristSphere, HdPrimTypeTokens->mesh,
              HdRetainedContainerDataSource::New(0, nullptr, nullptr)},
             {wristCone, HdPrimTypeTokens->mesh,
              HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});
        observer.added.clear();
        upstream->RemovePrims({{wristSphere}, {wristCone}});
        int sphereAdds = 0, coneAdds = 0;
        for (const SdfPath &p : observer.added) {
            if (p == wristSphere) ++sphereAdds;
            if (p == wristCone) ++coneAdds;
        }
        CHECK(sphereAdds == 1);
        CHECK(coneAdds == 0);
        CHECK(results->GetPrim(wristSphere).dataSource);
    }

    // Motion samples (spec 10.5 subset): explicit shutter offsets around
    // the IK switch retain distinct point sets under one generation
    // fence, and the sampled source reports the retained offsets.
    {
        const auto sampled = bridge.EvaluateAndPublishSamples(
            UsdTimeCode(1013), {-0.5f, 0.0f, 0.5f});
        CHECK(sampled.ok);
        results->NotifyGenerationPublished(sampled.dirtied);
        const HdSceneIndexPrim prim = results->GetPrim(bodyPath);
        HdPrimvarsSchema primvars =
            HdPrimvarsSchema::GetFromParent(prim.dataSource);
        HdSampledDataSourceHandle value =
            primvars.GetPrimvar(HdTokens->points).GetPrimvarValue();
        CHECK(value);
        if (value) {
            std::vector<HdSampledDataSource::Time> times;
            CHECK(value->GetContributingSampleTimesForInterval(
                -0.5f, 0.5f, &times));
            CHECK(times.size() == 3);
            const VtValue before = value->GetValue(-0.5f);
            const VtValue after = value->GetValue(0.5f);
            CHECK(before.IsHolding<VtVec3fArray>() &&
                  after.IsHolding<VtVec3fArray>());
            if (before.IsHolding<VtVec3fArray>() &&
                after.IsHolding<VtVec3fArray>()) {
                // The IK weight steps 0 -> 1 at 1013 (held spline), so the
                // bracketing samples differ.
                CHECK(before.UncheckedGet<VtVec3fArray>() !=
                      after.UncheckedGet<VtVec3fArray>());
            }
        }
    }

    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// Control guides (spec §10.3 extension): every RigExecControl grows one
// synthesized `rigGuideCtrl` child drawing the authored guide:shape in the
// authored guide:drawMode, sized by evaluated control scale multiplied by
// guide:scaleX/Y/Z, and placed at the control's posed frame.
// The shape table is asserted exhaustively -- all six shapes in both draw
// modes -- because the prim type and the topology are the contract a
// renderer consumes, and a wrong count draws a shape that is merely
// plausible: a 32-gon missing its closing vertex still renders, as an arc.
static void
TestControlGuides(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmShotAnim.usda");
    CHECK(stage);
    if (!stage) return;

    const SdfPath rigPath("/Shot/HeroArm/Rig");
    const SdfPath controls("/Shot/HeroArm/Rig/Controls");
    const SdfPath shoulderFk = controls.AppendChild(TfToken("ShoulderFK"));
    const SdfPath elbowFk = controls.AppendChild(TfToken("ElbowFK"));
    const SdfPath wristFk = controls.AppendChild(TfToken("WristFK"));
    const SdfPath handIk = controls.AppendChild(TfToken("HandIK"));
    const SdfPath elbowPole = controls.AppendChild(TfToken("ElbowPole"));
    static const TfToken guideChild("rigGuideCtrl");

    RigExecImagingBridge bridge(stage, rigPath);
    std::vector<std::string> errors;
    const bool compiled = bridge.Compile(&errors);
    for (const std::string &e : errors) {
        std::printf("compile error: %s\n", e.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;

    // Upstream prim shells for the controls: a guide only synthesizes
    // beneath a parent that exists upstream. WristFK is hidden and carries
    // a primOrigin, exactly as a flattened UsdImaging prim would.
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    HdRetainedSceneIndex::AddedPrimEntries shells;
    for (const SdfPath &p :
         {shoulderFk, elbowFk, handIk, elbowPole}) {
        shells.push_back({p, TfToken(),
                          HdRetainedContainerDataSource::New(0, nullptr,
                                                             nullptr)});
    }
    static const TfToken wristNames[] = {
        HdVisibilitySchema::GetSchemaToken(),
        HdPrimOriginSchema::GetSchemaToken()};
    const HdDataSourceBaseHandle wristValues[] = {
        HdVisibilitySchema::Builder()
            .SetVisibility(HdRetainedTypedSampledDataSource<bool>::New(false))
            .Build(),
        HdRetainedContainerDataSource::New(
            HdPrimOriginSchemaTokens->scenePath,
            HdRetainedTypedSampledDataSource<
                HdPrimOriginSchema::OriginPath>::New(
                    HdPrimOriginSchema::OriginPath(wristFk)))};
    shells.push_back({wristFk, TfToken(),
                      HdRetainedContainerDataSource::New(2, wristNames,
                                                         wristValues)});
    upstream->AddPrims(shells);

    auto binding = RigExecBindingResolvingSceneIndex::New(upstream);
    auto results = RigExecResultsSceneIndex::New(binding, bridge.GetStore());
    bridge.SetSceneIndices(binding, results);
    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));

    // Authoring through the schema-declared attribute, not CreateAttribute:
    // an invalid handle here means the codeless schema lost the property,
    // and that must fail loudly rather than quietly author nothing.
    auto setToken = [&stage](const SdfPath &path, const char *name,
                             const char *value) {
        const UsdAttribute a =
            stage->GetPrimAtPath(path).GetAttribute(TfToken(name));
        return a && a.Set(TfToken(value));
    };
    auto setDouble = [&stage](const SdfPath &path, const char *name,
                              double value) {
        const UsdAttribute a =
            stage->GetPrimAtPath(path).GetAttribute(TfToken(name));
        return a && a.Set(value);
    };

    auto guidePath = [&](const SdfPath &control) {
        return control.AppendChild(guideChild);
    };
    auto hasGuideChild = [&](const SdfPath &control) {
        for (const SdfPath &p : results->GetChildPrimPaths(control)) {
            if (p == guidePath(control)) return true;
        }
        return false;
    };
    // curveVertexCounts for a wire guide, faceVertexCounts for a mesh one.
    auto topologyCounts = [](const HdSceneIndexPrim &prim) {
        VtIntArray counts;
        if (!prim.dataSource) return counts;
        if (HdBasisCurvesSchema curves =
                HdBasisCurvesSchema::GetFromParent(prim.dataSource)) {
            if (HdIntArrayDataSourceHandle ds =
                    curves.GetTopology().GetCurveVertexCounts()) {
                counts = ds->GetTypedValue(0.0f);
            }
        } else if (HdMeshSchema mesh =
                       HdMeshSchema::GetFromParent(prim.dataSource)) {
            if (HdIntArrayDataSourceHandle ds =
                    mesh.GetTopology().GetFaceVertexCounts()) {
                counts = ds->GetTypedValue(0.0f);
            }
        }
        return counts;
    };
    auto faceVertexIndices = [](const HdSceneIndexPrim &prim) {
        VtIntArray indices;
        if (prim.dataSource) {
            if (HdMeshSchema mesh =
                    HdMeshSchema::GetFromParent(prim.dataSource)) {
                if (HdIntArrayDataSourceHandle ds =
                        mesh.GetTopology().GetFaceVertexIndices()) {
                    indices = ds->GetTypedValue(0.0f);
                }
            }
        }
        return indices;
    };

    // circle. This runs FIRST, before any guide attribute is set, because
    // it is the only moment the unauthored state exists -- and it is the
    // state every control in every rig that predates this feature is in, so
    // it is the one that decides whether they all suddenly draw nothing.
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
    {
        CHECK(hasGuideChild(elbowFk));
        const HdSceneIndexPrim guide = results->GetPrim(guidePath(elbowFk));
        CHECK(guide.primType == HdPrimTypeTokens->basisCurves);
        const VtIntArray counts = topologyCounts(guide);
        CHECK(counts.size() == 1);
        if (counts.size() == 1) {
            CHECK(counts[0] == 33);
        }
        CHECK(_GetPointsPrimvar(guide).size() == 33);
    }
    // ...and the attributes really are schema-declared: an invalid handle
    // here means the codeless schema lost the property, which must fail
    // loudly rather than quietly author nothing.
    CHECK(setToken(shoulderFk, "guide:shape", "circle"));

    // Wire rings close by repeating their first point, so a ring of N
    // segments is N+1 points; the mesh forms drop the repeat because a face
    // closes itself.
    // faceVertexIndices are asserted in ORDER, not merely counted. The
    // order IS the winding, and a wrong winding is invisible to a count and
    // nearly invisible on screen: doubleSided keeps an inward-wound solid
    // from disappearing, so it survives a look at the viewport while
    // inverting every derived normal a normal AOV or a front/back-sensitive
    // material sees. The solid pyramid shipped inward-wound on all five
    // faces behind exactly that gap.
    struct _Expected {
        const char *shape;
        const char *drawMode;
        TfToken primType;
        std::vector<int> counts;    // empty for the implicits
        size_t points;              // 0 for the implicits
        std::vector<int> indices;   // faceVertexIndices; empty unless a mesh
    };
    // The circle mesh is one n-gon over consecutive vertices.
    std::vector<int> circleFan(32);
    for (int i = 0; i < 32; ++i) {
        circleFan[i] = i;
    }
    const std::vector<_Expected> expectations = {
        {"circle", "wire", HdPrimTypeTokens->basisCurves, {33}, 33, {}},
        {"sphere", "wire", HdPrimTypeTokens->basisCurves,
         {33, 33, 33}, 99, {}},
        {"box", "wire", HdPrimTypeTokens->basisCurves, {5}, 5, {}},
        {"cube", "wire", HdPrimTypeTokens->basisCurves,
         {5, 5, 2, 2, 2, 2}, 18, {}},
        {"diamond", "wire", HdPrimTypeTokens->basisCurves,
         {5, 5, 5}, 15, {}},
        {"pyramid", "wire", HdPrimTypeTokens->basisCurves,
         {5, 2, 2, 2, 2}, 13, {}},
        {"sphere", "geometry", HdPrimTypeTokens->sphere, {}, 0, {}},
        {"cube", "geometry", HdPrimTypeTokens->cube, {}, 0, {}},
        {"circle", "geometry", HdPrimTypeTokens->mesh, {32}, 32, circleFan},
        // The unit quad in the XZ plane, wound counter-clockwise seen from
        // +Y so its derived normal is +Y.
        {"box", "geometry", HdPrimTypeTokens->mesh, {4}, 4, {0, 1, 2, 3}},
        // Octahedron, vertices [+X, -X, +Y, -Y, +Z, -Z]: four faces around
        // +Y then four around -Y, every one wound counter-clockwise seen
        // from outside.
        {"diamond", "geometry", HdPrimTypeTokens->mesh,
         {3, 3, 3, 3, 3, 3, 3, 3}, 6,
         {0, 2, 4, 4, 2, 1, 1, 2, 5, 5, 2, 0,
          0, 4, 3, 4, 1, 3, 1, 5, 3, 5, 0, 3}},
        // Base corners [0..3] counter-clockwise seen from +Y, apex 4. Each
        // side takes two base corners in ring order then the apex; the base
        // quad takes the ring REVERSED, because it is the one face whose
        // outward direction is -Y.
        {"pyramid", "geometry", HdPrimTypeTokens->mesh,
         {3, 3, 3, 3, 4}, 5,
         {0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4, 3, 2, 1, 0}}};
    for (const _Expected &expected : expectations) {
        CHECK(setToken(shoulderFk, "guide:shape", expected.shape));
        CHECK(setToken(shoulderFk, "guide:drawMode", expected.drawMode));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));

        CHECK(hasGuideChild(shoulderFk));
        const HdSceneIndexPrim guide =
            results->GetPrim(guidePath(shoulderFk));
        if (guide.primType != expected.primType) {
            std::printf("  %s/%s drew a %s, expected %s\n", expected.shape,
                        expected.drawMode, guide.primType.GetText(),
                        expected.primType.GetText());
        }
        CHECK(guide.primType == expected.primType);
        CHECK(guide.dataSource);
        if (!guide.dataSource) continue;

        const VtIntArray counts = topologyCounts(guide);
        const bool countsMatch =
            counts.size() == expected.counts.size() &&
            std::equal(counts.begin(), counts.end(),
                       expected.counts.begin());
        if (!countsMatch) {
            std::printf("  %s/%s topology counts: got %zu entries, "
                        "expected %zu\n", expected.shape, expected.drawMode,
                        counts.size(), expected.counts.size());
        }
        CHECK(countsMatch);
        CHECK(_GetPointsPrimvar(guide).size() == expected.points);

        const VtIntArray indices = faceVertexIndices(guide);
        const bool indicesMatch =
            indices.size() == expected.indices.size() &&
            std::equal(indices.begin(), indices.end(),
                       expected.indices.begin());
        if (!indicesMatch) {
            std::printf("  %s/%s faceVertexIndices differ (got %zu, expected "
                        "%zu) -- a reordered face is a flipped winding\n",
                        expected.shape, expected.drawMode, indices.size(),
                        expected.indices.size());
            for (size_t i = 0; i < indices.size() &&
                               i < expected.indices.size(); ++i) {
                if (indices[i] != expected.indices[i]) {
                    std::printf("    first difference at %zu: %d != %d\n", i,
                                indices[i], expected.indices[i]);
                    break;
                }
            }
        }
        CHECK(indicesMatch);

        // The implicits carry their dimension in their own schema; the
        // explicit shapes carry a local unit extent instead.
        if (expected.primType == HdPrimTypeTokens->sphere) {
            HdSphereSchema sphere =
                HdSphereSchema::GetFromParent(guide.dataSource);
            CHECK(sphere.GetRadius() &&
                  sphere.GetRadius()->GetTypedValue(0.0f) == 1.0);
        } else if (expected.primType == HdPrimTypeTokens->cube) {
            HdCubeSchema cube =
                HdCubeSchema::GetFromParent(guide.dataSource);
            CHECK(cube.GetSize() &&
                  cube.GetSize()->GetTypedValue(0.0f) == 2.0);
        } else {
            HdExtentSchema extent =
                HdExtentSchema::GetFromParent(guide.dataSource);
            CHECK(extent.GetMin() && extent.GetMax());
            if (extent.GetMin() && extent.GetMax()) {
                const GfVec3d min = extent.GetMin()->GetTypedValue(0.0f);
                const GfVec3d max = extent.GetMax()->GetTypedValue(0.0f);
                // Unit shapes: no axis reaches past 1, and the bound is a
                // bound (the planar shapes are flat in Y, so min == max
                // there is expected).
                for (size_t i = 0; i < 3; ++i) {
                    CHECK(min[i] >= -1.0 - 1e-6 && max[i] <= 1.0 + 1e-6);
                    CHECK(min[i] <= max[i]);
                }
            }
        }
        // Meshes are double-sided: a guide is looked at from every side.
        if (expected.primType == HdPrimTypeTokens->mesh) {
            HdMeshSchema mesh =
                HdMeshSchema::GetFromParent(guide.dataSource);
            CHECK(mesh.GetDoubleSided() &&
                  mesh.GetDoubleSided()->GetTypedValue(0.0f));
        }
        // Wire guides carry a constant width from guide:wireWidth, which
        // is what makes them clickable: usdview picks with a single-pixel
        // window, so an unwidthed hairline is a target real clicks miss.
        // Nothing else authors widths -- a mesh or an implicit has no
        // curves to widen.
        HdPrimvarsSchema primvars =
            HdPrimvarsSchema::GetFromParent(guide.dataSource);
        HdPrimvarSchema widths = primvars.GetPrimvar(HdTokens->widths);
        if (expected.primType == HdPrimTypeTokens->basisCurves) {
            CHECK(widths.GetPrimvarValue());
            CHECK(widths.GetInterpolation() &&
                  widths.GetInterpolation()->GetTypedValue(0.0f) ==
                      HdPrimvarSchemaTokens->constant);
            if (HdSampledDataSourceHandle v = widths.GetPrimvarValue()) {
                const VtValue held = v->GetValue(0.0f);
                CHECK(held.IsHolding<VtFloatArray>());
                if (held.IsHolding<VtFloatArray>()) {
                    const VtFloatArray w =
                        held.UncheckedGet<VtFloatArray>();
                    CHECK(w.size() == 1);
                    // The schema default.
                    if (w.size() == 1) {
                        CHECK(std::abs(w[0] - 0.05f) < 1e-6f);
                    }
                }
            }
        } else {
            CHECK(!widths);
        }
    }

    CHECK(setToken(shoulderFk, "guide:shape", "circle"));
    CHECK(setToken(shoulderFk, "guide:drawMode", "wire"));
    CHECK(setToken(elbowFk, "guide:shape", "sphere"));
    CHECK(setToken(handIk, "guide:shape", "diamond"));
    CHECK(setToken(handIk, "guide:drawMode", "geometry"));
    CHECK(setToken(wristFk, "guide:shape", "cube"));
    CHECK(setToken(wristFk, "guide:drawMode", "geometry"));
    CHECK(setToken(elbowPole, "guide:shape", "pyramid"));
    CHECK(setDouble(elbowPole, "guide:scaleX", 2.0));
    CHECK(setDouble(elbowPole, "guide:scaleY", 3.0));
    CHECK(setDouble(elbowPole, "guide:scaleZ", 0.5));
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));

    {
        const HdSceneIndexPrim guide =
            results->GetPrim(guidePath(shoulderFk));
        CHECK(guide.dataSource);
        if (guide.dataSource) {
            // Controls draw as ordinary GEOMETRY, not as diagnostics: an
            // animator must not have to find a viewer setting before the
            // thing they are meant to click appears.
            HdPurposeSchema purpose =
                HdPurposeSchema::GetFromParent(guide.dataSource);
            CHECK(purpose.GetPurpose() &&
                  purpose.GetPurpose()->GetTypedValue(0.0f) ==
                      HdRenderTagTokens->geometry);

            HdPrimvarsSchema primvars =
                HdPrimvarsSchema::GetFromParent(guide.dataSource);
            HdPrimvarSchema color =
                primvars.GetPrimvar(HdTokens->displayColor);
            CHECK(color.GetInterpolation() &&
                  color.GetInterpolation()->GetTypedValue(0.0f) ==
                      HdPrimvarSchemaTokens->constant);
            HdPrimvarSchema opacity =
                primvars.GetPrimvar(HdTokens->displayOpacity);
            CHECK(opacity.GetInterpolation() &&
                  opacity.GetInterpolation()->GetTypedValue(0.0f) ==
                      HdPrimvarSchemaTokens->constant);
            // The control schema's own default, not the joint guide red.
            if (HdSampledDataSourceHandle v = color.GetPrimvarValue()) {
                const VtValue held = v->GetValue(0.0f);
                CHECK(held.IsHolding<VtVec3fArray>());
                if (held.IsHolding<VtVec3fArray>()) {
                    const VtVec3fArray c = held.UncheckedGet<VtVec3fArray>();
                    CHECK(c.size() == 1);
                    if (c.size() == 1) {
                        CHECK(std::abs(c[0][1] - 0.85f) < 1e-5f);
                    }
                }
            }
        }
    }

    // inactive control set. UsdAttribute::Get never follows a connection,
    // so a bridge reading the attribute plainly drew the local value and a
    // wired rig looked wired while nothing faded.
    {
        auto readOpacity = [&](const SdfPath &control) {
            HdPrimvarsSchema primvars = HdPrimvarsSchema::GetFromParent(
                results->GetPrim(guidePath(control)).dataSource);
            HdSampledDataSourceHandle v =
                primvars.GetPrimvar(HdTokens->displayOpacity)
                    .GetPrimvarValue();
            const VtValue held = v ? v->GetValue(0.0f) : VtValue();
            if (!held.IsHolding<VtFloatArray>() ||
                held.UncheckedGet<VtFloatArray>().size() != 1) {
                return -1.0f;
            }
            return held.UncheckedGet<VtFloatArray>()[0];
        };
        UsdPrim pole = stage->GetPrimAtPath(elbowPole);
        const UsdPrim shoulder = stage->GetPrimAtPath(shoulderFk);
        const UsdPrim elbow = stage->GetPrimAtPath(elbowFk);
        // A float dial, as the biped's avars:ikfk is, and a double twin,
        // as every schema avar is: both have to drive.
        const UsdAttribute dial = pole.CreateAttribute(
            TfToken("dial"), SdfValueTypeNames->Float, /* custom = */ true);
        const UsdAttribute dialD = pole.CreateAttribute(
            TfToken("dialD"), SdfValueTypeNames->Double, /* custom = */ true);
        CHECK(dial.Set(0.3f) && dialD.Set(0.6));
        const UsdAttribute shoulderOpacity =
            shoulder.GetAttribute(TfToken("guide:displayOpacity"));
        const UsdAttribute elbowOpacity =
            elbow.GetAttribute(TfToken("guide:displayOpacity"));
        CHECK(shoulderOpacity &&
              shoulderOpacity.SetConnections({dial.GetPath()}));
        CHECK(elbowOpacity && elbowOpacity.SetConnections({dialD.GetPath()}));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        CHECK(std::abs(readOpacity(shoulderFk) - 0.3f) < 1e-6f);
        CHECK(std::abs(readOpacity(elbowFk) - 0.6f) < 1e-6f);

        // A dial a math mover doubles: the guide reads it at the opacity
        // attribute's own read phase -- the dial as authored by default,
        // the doubled value once the attribute declares `final`.
        {
            const TfToken phaseField("rigExecReadPhase");
            const UsdPrim gain = stage->DefinePrim(
                SdfPath("/Shot/HeroArm/Rig/DialGain"),
                TfToken("RigExecFloatMathMover"));
            CHECK(gain.ApplyAPI(TfToken("RigExecMoverAPI")));
            gain.CreateAttribute(TfToken("rigExec:operation"),
                                 SdfValueTypeNames->Token)
                .Set(TfToken("multiply"));
            gain.CreateAttribute(TfToken("inputs:value"),
                                 SdfValueTypeNames->Float)
                .Set(2.0f);
            gain.CreateRelationship(TfToken("rigExec:moves"))
                .SetTargets({dial.GetPath()});
            CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
            CHECK(std::abs(readOpacity(shoulderFk) - 0.3f) < 1e-6f);
            CHECK(shoulderOpacity.SetMetadata(phaseField,
                                              std::string("final")));
            CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
            CHECK(std::abs(readOpacity(shoulderFk) - 0.6f) < 1e-6f);
            CHECK(shoulderOpacity.ClearMetadata(phaseField));
            CHECK(stage->RemovePrim(gain.GetPath()));
            CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
            CHECK(std::abs(readOpacity(shoulderFk) - 0.3f) < 1e-6f);
        }

        // Invert: the FK side of one switch draws the complement.
        const UsdAttribute invert =
            shoulder.GetAttribute(TfToken("guide:displayOpacityInvert"));
        CHECK(invert && invert.Set(true));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        CHECK(std::abs(readOpacity(shoulderFk) - 0.7f) < 1e-6f);

        // The floor: a dial at its end stop must not make the control
        // vanish. The schema default first, then an authored one, then
        // zero, which is how a true fade-out is asked for.
        CHECK(dial.Set(1.0f));
        observer.dirtied.clear();
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        CHECK(std::abs(readOpacity(shoulderFk) - 0.15f) < 1e-6f);
        // ...and an edit on the SOURCE prim dirtied the dependent guide,
        // which hangs off a different prim entirely.
        CHECK(std::find(observer.dirtied.begin(), observer.dirtied.end(),
                        guidePath(shoulderFk)) != observer.dirtied.end());
        const UsdAttribute floorAttr =
            shoulder.GetAttribute(TfToken("guide:displayOpacityMin"));
        CHECK(floorAttr && floorAttr.Set(0.4f));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        CHECK(std::abs(readOpacity(shoulderFk) - 0.4f) < 1e-6f);
        CHECK(floorAttr.Set(0.0f));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        CHECK(std::abs(readOpacity(shoulderFk)) < 1e-6f);
        // An out-of-range source clamps (inverted: 1 - (-2) = 3 -> 1).
        CHECK(dial.Set(-2.0f));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        CHECK(std::abs(readOpacity(shoulderFk) - 1.0f) < 1e-6f);

        // Disconnected, the local value draws again, invert and floor
        // ignored: an unconnected attribute means exactly what it says.
        CHECK(shoulderOpacity.ClearConnections() &&
              elbowOpacity.ClearConnections());
        CHECK(shoulderOpacity.Set(0.9f));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        CHECK(std::abs(readOpacity(shoulderFk) - 0.9f) < 1e-6f);
        CHECK(std::abs(readOpacity(elbowFk) - 1.0f) < 1e-6f);
        // Put the fixture back for the checks that follow.
        CHECK(invert.Clear() && floorAttr.Clear() && shoulderOpacity.Clear());
        CHECK(pole.RemoveProperty(TfToken("dial")) &&
              pole.RemoveProperty(TfToken("dialD")));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
    }

    // The guide sits at the control's posed frame, in ASSET space: nothing
    // upstream places /Shot/HeroArm here, so the asset root resolves to
    // identity and ShoulderFK's rest translate is the whole transform.
    auto guideXform = [&](const SdfPath &control) {
        HdXformSchema xf = HdXformSchema::GetFromParent(
            results->GetPrim(guidePath(control)).dataSource);
        return xf && xf.GetMatrix() ? xf.GetMatrix()->GetTypedValue(0.0f)
                                    : GfMatrix4d(1.0);
    };
    CHECK((guideXform(shoulderFk).ExtractTranslation() -
           GfVec3d(0, 10, 0)).GetLength() < 1e-6);

    // These fixture controls have identity scale avars, so the authored
    // guide:scaleX/Y/Z values are the effective basis-vector lengths. The
    // focused standalone test below covers their multiplication by evaluated
    // avar scale.
    {
        const GfMatrix4d xform = guideXform(elbowPole);
        const double expectedLengths[3] = {2.0, 3.0, 0.5};
        for (size_t i = 0; i < 3; ++i) {
            const double length = xform.GetRow3(i).GetLength();
            if (std::abs(length - expectedLengths[i]) > 1e-6) {
                std::printf("  basis %zu length %g, expected %g -- per-axis "
                            "guide scale not applied\n", i, length,
                            expectedLengths[i]);
            }
            CHECK(std::abs(length - expectedLengths[i]) < 1e-6);
        }
    }

    // Inherited visibility: WristFK is hidden upstream, so its guide is
    // hidden too. A synthesized prim with no visibility of its own falls
    // back to VISIBLE, which would float a guide over a hidden control.
    {
        const HdSceneIndexPrim guide = results->GetPrim(guidePath(wristFk));
        CHECK(guide.dataSource);
        HdVisibilitySchema vis =
            HdVisibilitySchema::GetFromParent(guide.dataSource);
        const bool visible = !vis || !vis.GetVisibility() ||
                             vis.GetVisibility()->GetTypedValue(0.0f);
        if (visible) {
            std::printf("  control guide under a hidden control is still "
                        "visible -- inherited visibility not applied\n");
        }
        CHECK(!visible);

        // ...and picking it must select the CONTROL, which is the whole
        // point of drawing a control guide.
        HdPrimOriginSchema origin =
            HdPrimOriginSchema::GetFromParent(guide.dataSource);
        const SdfPath resolved =
            origin ? origin.GetOriginPath(HdPrimOriginSchemaTokens->scenePath)
                   : SdfPath();
        if (resolved != wristFk) {
            std::printf("  control guide has no primOrigin -- picking it "
                        "resolves to nothing selectable\n");
        }
        CHECK(resolved == wristFk);
    }

    // The guide follows the pose per generation: HandIK is animated, so
    // its guide moves and the child is dirtied.
    {
        const GfMatrix4d rest = guideXform(handIk);
        observer.dirtied.clear();
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        const GfMatrix4d posed = guideXform(handIk);
        CHECK(rest != posed);
        bool sawGuideDirty = false;
        for (const SdfPath &p : observer.dirtied) {
            if (p == guidePath(handIk)) sawGuideDirty = true;
        }
        CHECK(sawGuideDirty);
    }

    // A shape edit WITHIN one draw mode keeps the prim type and changes the
    // topology under it -- wire circle and wire pyramid are both
    // basisCurves. There is no re-add to carry that (the type is what an
    // add announces), so the consumer learns it from the structural dirty
    // on the child, and a consumer that cached the old container has to be
    // told to drop it. Without that, a retyped guide keeps drawing its old
    // shape forever.
    {
        CHECK(setToken(elbowPole, "guide:shape", "circle"));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        const VtIntArray before =
            topologyCounts(results->GetPrim(guidePath(elbowPole)));
        CHECK(before.size() == 1);

        observer.dirtied.clear();
        CHECK(setToken(elbowPole, "guide:shape", "pyramid"));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        const HdSceneIndexPrim guide =
            results->GetPrim(guidePath(elbowPole));
        CHECK(guide.primType == HdPrimTypeTokens->basisCurves);
        const VtIntArray after = topologyCounts(guide);
        const bool resynced =
            after.size() == 5 && after[0] == 5 && after[1] == 2 &&
            after[2] == 2 && after[3] == 2 && after[4] == 2;
        if (!resynced) {
            std::printf("  a guide:shape change within one draw mode left "
                        "%zu curve(s), expected the pyramid's 5\n",
                        after.size());
        }
        CHECK(resynced);
        bool sawGuideDirty = false;
        for (const SdfPath &p : observer.dirtied) {
            if (p == guidePath(elbowPole)) sawGuideDirty = true;
        }
        if (!sawGuideDirty) {
            std::printf("  a guide:shape change did not dirty the guide -- "
                        "a cached consumer keeps the old topology\n");
        }
        CHECK(sawGuideDirty);
    }

    // A shape or draw-mode edit changes the child's PRIM TYPE, so it is
    // structural: the consumer is told through a fresh PrimsAdded carrying
    // the new type, not through a value dirty on a prim it still believes
    // is a basisCurves.
    {
        observer.added.clear();
        CHECK(setToken(handIk, "guide:drawMode", "wire"));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(results->GetPrim(guidePath(handIk)).primType ==
              HdPrimTypeTokens->basisCurves);
        bool reAnnounced = false;
        for (const SdfPath &p : observer.added) {
            if (p == guidePath(handIk)) reAnnounced = true;
        }
        if (!reAnnounced) {
            std::printf("  a draw-mode change re-typed the guide without "
                        "re-announcing it\n");
        }
        CHECK(reAnnounced);
    }

    // A non-positive scale on ANY axis draws nothing at all -- the same
    // rule the joint guide:radius follows. The child disappears from the
    // traversal, is removed, and stops resolving.
    {
        observer.removed.clear();
        CHECK(setDouble(elbowPole, "guide:scaleY", 0.0));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(!hasGuideChild(elbowPole));
        CHECK(!results->GetPrim(guidePath(elbowPole)).dataSource);
        bool sawRemoval = false;
        for (const SdfPath &p : observer.removed) {
            if (p == guidePath(elbowPole)) sawRemoval = true;
        }
        CHECK(sawRemoval);

        // ...and restoring it brings the guide back.
        observer.added.clear();
        CHECK(setDouble(elbowPole, "guide:scaleY", 1.0));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(hasGuideChild(elbowPole));
        bool sawAdd = false;
        for (const SdfPath &p : observer.added) {
            if (p == guidePath(elbowPole)) sawAdd = true;
        }
        CHECK(sawAdd);

        // A NEGATIVE scale is rejected on the same rule, and rejected
        // rather than mirrored: a negative axis would flip the shape's
        // winding, so accepting it would draw an inside-out guide instead
        // of no guide at all.
        CHECK(setDouble(elbowPole, "guide:scaleX", -1.0));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(!hasGuideChild(elbowPole));
        CHECK(!results->GetPrim(guidePath(elbowPole)).dataSource);
        CHECK(setDouble(elbowPole, "guide:scaleX", 2.0));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(hasGuideChild(elbowPole));
    }

    // token: BBoxCache buckets a prim's extent by this exact attribute, so
    // routing the drawn render tag through it is what keeps the bounds and
    // the drawing in the same bucket. Controls keep the inherited
    // `default`; authoring `guide` moves drawing AND bounds together.
    {
        auto guidePurpose = [&]() {
            HdPurposeSchema purpose = HdPurposeSchema::GetFromParent(
                results->GetPrim(guidePath(handIk)).dataSource);
            return purpose.GetPurpose()
                ? purpose.GetPurpose()->GetTypedValue(0.0f) : TfToken();
        };
        CHECK(setToken(handIk, "purpose", "guide"));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(guidePurpose() == HdRenderTagTokens->guide);

        // ...and back, which also proves the diff notices a purpose edit
        // rather than leaving the old render tag in place.
        observer.dirtied.clear();
        CHECK(setToken(handIk, "purpose", "default"));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(guidePurpose() == HdRenderTagTokens->geometry);
        bool sawPurposeDirty = false;
        for (const SdfPath &p : observer.dirtied) {
            if (p == guidePath(handIk)) sawPurposeDirty = true;
        }
        CHECK(sawPurposeDirty);
    }

    // usdview's interactive pick window is one physical pixel, so an
    // unwidthed hairline is a target a real click virtually never lands on
    // -- and an empty pick deselects to the pseudo-root, which is what the
    // user actually saw. The width is the fix, so it is pinned here.
    {
        CHECK(setToken(elbowPole, "guide:shape", "circle"));
        CHECK(setToken(elbowPole, "guide:drawMode", "wire"));
        // The authored width, or a negative sentinel when the primvar is
        // absent entirely (which is the hairline fallback).
        auto wireWidth = [&]() -> float {
            const HdSceneIndexPrim guide =
                results->GetPrim(guidePath(elbowPole));
            if (!guide.dataSource) return -1.0f;
            HdPrimvarSchema widths =
                HdPrimvarsSchema::GetFromParent(guide.dataSource)
                    .GetPrimvar(HdTokens->widths);
            if (!widths) return -1.0f;
            HdSampledDataSourceHandle value = widths.GetPrimvarValue();
            if (!value) return -1.0f;
            const VtValue held = value->GetValue(0.0f);
            if (!held.IsHolding<VtFloatArray>()) return -1.0f;
            const VtFloatArray w = held.UncheckedGet<VtFloatArray>();
            return w.size() == 1 ? w[0] : -1.0f;
        };

        // A widthed wire guide also asks to be REFINED, or the width is
        // decoration: Storm honours a curve's width only once the curve is
        // refined (HdStBasisCurves::_SupportsRefinement is refineLevel > 0),
        // and usdview's default complexity is refineLevel 0. Measured
        // against the real single-pixel pick path, this is the difference
        // between 8 and 89 hits out of 1681.
        auto refineLevel = [&]() -> int {
            const HdSceneIndexPrim guide =
                results->GetPrim(guidePath(elbowPole));
            if (!guide.dataSource) return -1;
            HdLegacyDisplayStyleSchema style =
                HdLegacyDisplayStyleSchema::GetFromParent(guide.dataSource);
            if (!style || !style.GetRefineLevel()) return -1;
            return style.GetRefineLevel()->GetTypedValue(0.0f);
        };

        // Unauthored: the schema default reaches the curves.
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(std::abs(wireWidth() - 0.05f) < 1e-6f);
        CHECK(refineLevel() == 1);

        // An authored width flows through...
        CHECK(setDouble(elbowPole, "guide:wireWidth", 0.25));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(std::abs(wireWidth() - 0.25f) < 1e-6f);

        // ...and changing it dirties the guide child, or a cached consumer
        // goes on drawing the old thickness forever.
        observer.dirtied.clear();
        CHECK(setDouble(elbowPole, "guide:wireWidth", 0.4));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(std::abs(wireWidth() - 0.4f) < 1e-6f);
        bool sawWidthDirty = false;
        for (const SdfPath &p : observer.dirtied) {
            if (p == guidePath(elbowPole)) sawWidthDirty = true;
        }
        if (!sawWidthDirty) {
            std::printf("  a guide:wireWidth change did not dirty the "
                        "guide -- the drawn width goes stale\n");
        }
        CHECK(sawWidthDirty);

        // Zero authors NO widths at all -- the hairline fallback, which is
        // exactly what wire guides drew before this attribute existed.
        // Unlike a bad scale, this still draws the guide.
        CHECK(setDouble(elbowPole, "guide:wireWidth", 0.0));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(results->GetPrim(guidePath(elbowPole)).dataSource);
        CHECK(wireWidth() < 0.0f);
        // ...and a hairline asks for no refinement either: there is no
        // width for refinement to make visible, so it would be pure
        // tessellation cost.
        CHECK(refineLevel() == -1);

        // ...and so does a negative one.
        CHECK(setDouble(elbowPole, "guide:wireWidth", -2.0));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(results->GetPrim(guidePath(elbowPole)).dataSource);
        CHECK(wireWidth() < 0.0f);
        CHECK(refineLevel() == -1);

        // The geometry draw mode ignores the width entirely: a mesh has no
        // curves to widen, and a stray widths primvar is one more thing a
        // renderer has to decide what to do with.
        CHECK(setDouble(elbowPole, "guide:wireWidth", 0.25));
        CHECK(setToken(elbowPole, "guide:drawMode", "geometry"));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));
        CHECK(results->GetPrim(guidePath(elbowPole)).primType ==
              HdPrimTypeTokens->mesh);
        CHECK(wireWidth() < 0.0f);
        // A mesh has nothing to refine here either.
        CHECK(refineLevel() == -1);

        // Back to a widthed wire for whatever runs after this.
        CHECK(setToken(elbowPole, "guide:drawMode", "wire"));
        CHECK(setToken(elbowPole, "guide:shape", "pyramid"));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1024)));

        // The diff arm directly: two generations differing ONLY in the
        // width report a guide VALUE change. Asserted as an exact equality
        // rather than a bit test, because the failure that matters in both
        // directions is silent -- no bit at all leaves the width stale,
        // and a structural bit would resync the child on every width
        // nudge.
        {
            RigExecSnapshotStore diffStore;
            const SdfPath probe("/DiffProbe/Ctrl");
            auto generation = [&probe](double width) {
                auto snap = std::make_shared<RigExecImagingSnapshot>();
                RigExecPublishedPrim &published = snap->prims[probe];
                published.hasControlGuide = true;
                published.controlGuideShape = TfToken("circle");
                published.controlGuideDrawMode = TfToken("wire");
                published.controlGuideWireWidth = width;
                return snap;
            };
            diffStore.Publish(generation(0.05));
            const RigExecPublishedDirtyVector dirty =
                diffStore.Publish(generation(0.2));
            CHECK(dirty.size() == 1);
            if (dirty.size() == 1) {
                CHECK(dirty[0].path == probe);
                CHECK(dirty[0].changes == RigExecChangeGuides);
            }
            // ...and an unchanged width reports nothing at all.
            CHECK(diffStore.Publish(generation(0.2)).empty());
        }
    }

    // An authored prim occupying the guide's name wins it, as everywhere
    // else in this index; removing it reveals the synthesized guide again.
    {
        upstream->AddPrims(
            {{guidePath(elbowFk), HdPrimTypeTokens->mesh,
              HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});
        CHECK(results->GetPrim(guidePath(elbowFk)).primType ==
              HdPrimTypeTokens->mesh);
        observer.added.clear();
        upstream->RemovePrims({{guidePath(elbowFk)}});
        int adds = 0;
        for (const SdfPath &p : observer.added) {
            if (p == guidePath(elbowFk)) ++adds;
        }
        CHECK(adds == 1);
        CHECK(results->GetPrim(guidePath(elbowFk)).primType ==
              HdPrimTypeTokens->basisCurves);
    }

    // A control whose shell is not upstream synthesizes nothing: the guide
    // hangs off a prim that must exist for a traversal to ever find it.
    {
        upstream->RemovePrims({{shoulderFk}});
        CHECK(!results->GetPrim(guidePath(shoulderFk)).dataSource);
        CHECK(!hasGuideChild(shoulderFk));
    }

    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// A control guide is an output in its own right.  Authoring tools commonly
// create and place controls before wiring joints or movers, and that isolated
// control must remain visible while the rig is being built.  Exercise the
// schema fallbacks too: the minimally authored typed prim should draw the
// default unit wire circle at its default frame.
static void
TestStandaloneControlGuide()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    const SdfPath rigPath("/Asset/Rig");
    stage->DefinePrim(rigPath, TfToken("RigExecRoot"));
    stage->DefinePrim(rigPath.AppendChild(TfToken("Controls")),
                      TfToken("Scope"));
    const SdfPath control =
        rigPath.AppendPath(SdfPath("Controls/Control"));
    const UsdPrim controlPrim =
        stage->DefinePrim(control, TfToken("RigExecControl"));

    RigExecImagingBridge bridge(stage, rigPath);
    std::vector<std::string> errors;
    const bool compiled = bridge.Compile(&errors);
    for (const std::string &e : errors) {
        std::printf("standalone-control compile error: %s\n", e.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;

    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    const RigExecImagingSnapshotConstPtr snapshot = bridge.GetStore()->Get();
    CHECK(snapshot);
    if (!snapshot) return;
    const auto published = snapshot->prims.find(control);
    CHECK(published != snapshot->prims.end());
    if (published == snapshot->prims.end()) return;
    CHECK(published->second.hasControlGuide);
    CHECK(published->second.controlGuideShape == TfToken("circle"));
    CHECK(published->second.controlGuideDrawMode == TfToken("wire"));
    CHECK(published->second.controlGuideScale == GfVec3d(1.0));

    // Imported controls carry local wire shapes without authored geometry.
    {
        const VtVec3fArray vertices{{1,2,3},{4,2,3},{0,0,0},{0,5,0}};
        CHECK(controlPrim.GetAttribute(TfToken("guide:shape")).Set(TfToken("custom")));
        CHECK(controlPrim.GetAttribute(TfToken("guide:points")).Set(vertices));
        CHECK(controlPrim.GetAttribute(TfToken("guide:curveVertexCounts")).Set(VtIntArray{2,2}));
        auto upstream = HdRetainedSceneIndex::New();
        upstream->AddPrims({{control, TfToken(), HdRetainedContainerDataSource::New(
            HdPrimOriginSchema::GetSchemaToken(), HdRetainedContainerDataSource::New(
                HdPrimOriginSchemaTokens->scenePath,
                HdRetainedTypedSampledDataSource<HdPrimOriginSchema::OriginPath>::New(control)))}});
        auto binding = RigExecBindingResolvingSceneIndex::New(upstream);
        auto results = RigExecResultsSceneIndex::New(binding, bridge.GetStore());
        bridge.SetSceneIndices(binding, results);
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        const SdfPath child=control.AppendChild(TfToken("rigGuideCtrl"));
        auto guide = results->GetPrim(child);
        CHECK(guide.primType == HdPrimTypeTokens->basisCurves);
        CHECK(HdBasisCurvesSchema::GetFromParent(guide.dataSource).GetTopology()
            .GetCurveVertexCounts()->GetTypedValue(0) == VtIntArray({2,2}));
        CHECK(HdPrimvarsSchema::GetFromParent(guide.dataSource).GetPrimvar(TfToken("points"))
            .GetPrimvarValue()->GetValue(0).Get<VtVec3fArray>() == vertices);
        CHECK(controlPrim.GetAttribute(TfToken("avars:tx")).Set(7.0));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        guide = results->GetPrim(child);
        CHECK(HdXformSchema::GetFromParent(guide.dataSource).GetMatrix()
            ->GetTypedValue(0).ExtractTranslation() == GfVec3d(7,0,0));
        CHECK(controlPrim.GetAttribute(TfToken("avars:sx")).Set(-2.0));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        guide = results->GetPrim(child);
        CHECK(HdXformSchema::GetFromParent(guide.dataSource).GetMatrix()
            ->GetTypedValue(0).Transform(GfVec3d(1,2,3)) == GfVec3d(5,2,3));
        CHECK(controlPrim.GetAttribute(TfToken("guide:orient")).Set(
            GfQuatf(GfRotation(GfVec3d(0,0,1),90).GetQuat())));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        guide = results->GetPrim(child);
        CHECK(GfIsClose(HdXformSchema::GetFromParent(guide.dataSource).GetMatrix()
            ->GetTypedValue(0).Transform(GfVec3d(1,2,3)),GfVec3d(11,1,3),1e-5));
        CHECK(controlPrim.GetAttribute(TfToken("guide:orient")).Clear());
        CHECK(controlPrim.GetAttribute(TfToken("guide:points")).Set(VtVec3fArray{{0,0,0},{2,0,0}}));
        CHECK(controlPrim.GetAttribute(TfToken("guide:curveVertexCounts")).Set(VtIntArray{2}));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        guide = results->GetPrim(child);
        CHECK(HdBasisCurvesSchema::GetFromParent(guide.dataSource).GetTopology()
            .GetCurveVertexCounts()->GetTypedValue(0) == VtIntArray({2}));
        CHECK(controlPrim.GetAttribute(TfToken("guide:curveVertexCounts")).Set(VtIntArray{3}));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        CHECK(!results->GetPrim(child).dataSource);
        CHECK(controlPrim.GetAttribute(TfToken("guide:curveVertexCounts")).Set(VtIntArray{2}));
        CHECK(controlPrim.GetAttribute(TfToken("guide:displayOpacity")).Set(0.0f));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        CHECK(!results->GetPrim(child).dataSource);
        CHECK(controlPrim.GetAttribute(TfToken("guide:displayOpacity")).Clear());
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        CHECK(results->GetPrim(child).dataSource);
        CHECK(controlPrim.GetAttribute(TfToken("guide:shape")).Clear());
        CHECK(controlPrim.GetAttribute(TfToken("avars:tx")).Clear());
        CHECK(controlPrim.GetAttribute(TfToken("avars:sx")).Clear());
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    }

    // guide:offset moves the drawn shape in the control's local frame and
    // leaves the control itself where it is.
    {
        const GfMatrix4d placement = published->second.controlGuideFrame;
        const GfMatrix4d pivot = published->second.controlFrame;
        const GfVec3d offset(1.0, 2.0, 3.0);
        UsdAttribute attr = controlPrim.CreateAttribute(
            TfToken("guide:offset"), SdfValueTypeNames->Double3);
        CHECK(attr && attr.Set(offset));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        const RigExecImagingSnapshotConstPtr moved = bridge.GetStore()->Get();
        const auto m = moved ? moved->prims.find(control)
                             : snapshot->prims.end();
        CHECK(moved && m != moved->prims.end());
        if (moved && m != moved->prims.end()) {
            const GfMatrix4d want =
                GfMatrix4d(1.0).SetTranslate(offset) * placement;
            CHECK(GfIsClose(m->second.controlGuideFrame, want, 1e-9));
            CHECK(!GfIsClose(m->second.controlGuideFrame, placement, 1e-6));
            CHECK(GfIsClose(m->second.controlFrame, pivot, 1e-12));
        }
        CHECK(attr.Clear());
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    }

    // The live guide gets its size from the evaluated frame, not a second raw
    // read of the avars. Authored guide scale remains a positive shape-size
    // multiplier. A negative transform scale mirrors deformation but keeps
    // the guide visible through its evaluated magnitude.
    const GfVec3d avarScale(-2.0, 3.0, 0.5);
    const GfVec3d authoredGuideScale(1.5, 0.5, 4.0);
    static const char *avarNames[3] = {
        "avars:sx", "avars:sy", "avars:sz"};
    static const char *guideScaleNames[3] = {
        "guide:scaleX", "guide:scaleY", "guide:scaleZ"};
    for (int axis = 0; axis < 3; ++axis) {
        const UsdAttribute avar =
            controlPrim.GetAttribute(TfToken(avarNames[axis]));
        const UsdAttribute guideScale =
            controlPrim.GetAttribute(TfToken(guideScaleNames[axis]));
        CHECK(avar && avar.Set(1.0, UsdTimeCode(0.0)));
        CHECK(avar && avar.Set(avarScale[axis], UsdTimeCode(10.0)));
        CHECK(guideScale && guideScale.Set(authoredGuideScale[axis]));
    }
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(10.0)));
    const RigExecImagingSnapshotConstPtr scaledSnapshot =
        bridge.GetStore()->Get();
    CHECK(scaledSnapshot);
    if (!scaledSnapshot) return;
    const auto scaled = scaledSnapshot->prims.find(control);
    CHECK(scaled != scaledSnapshot->prims.end());
    if (scaled == scaledSnapshot->prims.end()) return;
    const GfVec3d expectedScale(3.0, 1.5, 2.0);
    CHECK(scaled->second.controlGuideScale == expectedScale);
    for (int axis = 0; axis < 3; ++axis) {
        CHECK(std::abs(
                  scaled->second.controlGuideFrame.GetRow3(axis).GetLength() -
                  1.0) < 1e-9);
    }

    // Zero and signed sub-floor avars are supported transform channels. They
    // normalize to signed 1e-4 in evaluation; guide sizing uses magnitudes, so
    // none of these axes may make the guide disappear.
    const GfVec3d subfloorScale(
        0.0, -0.0, -0.5 * RigExecAvarScaleFloor);
    for (int axis = 0; axis < 3; ++axis) {
        const UsdAttribute avar =
            controlPrim.GetAttribute(TfToken(avarNames[axis]));
        CHECK(avar && avar.Set(subfloorScale[axis], UsdTimeCode(20.0)));
    }
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(20.0)));
    const RigExecImagingSnapshotConstPtr floorSnapshot =
        bridge.GetStore()->Get();
    CHECK(floorSnapshot);
    if (!floorSnapshot) return;
    const auto floored = floorSnapshot->prims.find(control);
    CHECK(floored != floorSnapshot->prims.end());
    if (floored == floorSnapshot->prims.end()) return;
    CHECK(floored->second.hasControlGuide);
    const GfVec3d floorExpected =
        authoredGuideScale * RigExecAvarScaleFloor;
    for (int axis = 0; axis < 3; ++axis) {
        CHECK(std::abs(floored->second.controlGuideScale[axis] -
                       floorExpected[axis]) < 1e-12);
        CHECK(std::abs(
                  floored->second.controlGuideFrame.GetRow3(axis).GetLength() -
                  1.0) < 1e-9);
    }

    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    const HdContainerDataSourceHandle origin =
        HdRetainedContainerDataSource::New(
            HdPrimOriginSchemaTokens->scenePath,
            HdRetainedTypedSampledDataSource<
                HdPrimOriginSchema::OriginPath>::New(
                    HdPrimOriginSchema::OriginPath(control)));
    upstream->AddPrims(
        {{control, TfToken(),
          HdRetainedContainerDataSource::New(
              HdPrimOriginSchema::GetSchemaToken(), origin)}});
    const RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(upstream, bridge.GetStore());
    const HdSceneIndexPrim guide = results->GetPrim(
        control.AppendChild(TfToken("rigGuideCtrl")));
    CHECK(guide.primType == HdPrimTypeTokens->basisCurves);
    CHECK(!_GetPointsPrimvar(guide).empty());
    const HdXformSchema guideXform =
        HdXformSchema::GetFromParent(guide.dataSource);
    CHECK(guideXform && guideXform.GetMatrix());
    if (guideXform && guideXform.GetMatrix()) {
        const GfMatrix4d matrix =
            guideXform.GetMatrix()->GetTypedValue(0.0f);
        for (int axis = 0; axis < 3; ++axis) {
            CHECK(std::abs(matrix.GetRow3(axis).GetLength() -
                           floorExpected[axis]) < 1e-12);
        }
    }
}

// Joint guides are hierarchy edges, not authored dimensions: one sphere per
// joint and one cone from the evaluated parent origin to every nested child.
// The shipped spider component is the user-facing regression for an oblique
// link whose parent +X axis does not already point at the child.
static void
TestJointHierarchyGuides(const std::string &examplesDir)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/components/spider_leg.usd");
    CHECK(stage);
    if (!stage) return;

    const SdfPath rigPath("/RigRoot");
    const SdfPath jointPath("/RigRoot/Joints/Shoulder");
    const SdfPath childPath("/RigRoot/Joints/Shoulder/ankle");
    CHECK(stage->GetPrimAtPath(jointPath));
    CHECK(stage->GetPrimAtPath(childPath));
    CHECK(!stage->GetPrimAtPath(jointPath).HasProperty(
        TfToken("guide:length")));

    RigExecImagingBridge bridge(stage, rigPath);
    std::vector<std::string> errors;
    const bool compiled = bridge.Compile(&errors);
    for (const std::string &error : errors) {
        std::printf("joint-hierarchy compile error: %s\n", error.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));

    const RigExecImagingSnapshotConstPtr snapshot = bridge.GetStore()->Get();
    CHECK(snapshot);
    if (!snapshot) return;
    const auto published = snapshot->prims.find(jointPath);
    CHECK(published != snapshot->prims.end());
    if (published == snapshot->prims.end()) return;
    CHECK(published->second.hasGuides);
    CHECK(published->second.guideFrames.size() == 1);
    CHECK(published->second.guideLengths.size() == 1);
    CHECK(published->second.guideRadii.size() == 1);
    CHECK(published->second.guideDrawSpheres.size() == 1);
    const GfVec3d expectedDelta(
        4.476721406958012, -2.6550437955052333, 0.0);
    const double expectedLength = expectedDelta.GetLength();
    CHECK(std::abs(published->second.guideLengths[0] - expectedLength) <
          1e-9);
    CHECK(published->second.guideRadii[0] == 1.0);
    CHECK(published->second.guideDrawSpheres[0]);
    CHECK((published->second.guideFrames[0].GetRow3(0) -
           expectedDelta / expectedLength).GetLength() < 1e-9);
    CHECK(published->second.guidePurpose == UsdGeomTokens->guide);

    const auto childPublished = snapshot->prims.find(childPath);
    CHECK(childPublished != snapshot->prims.end());
    if (childPublished != snapshot->prims.end()) {
        CHECK(childPublished->second.guideFrames.size() == 1);
        CHECK(childPublished->second.guideLengths.size() == 1);
        CHECK(childPublished->second.guideLengths[0] == 0.0);
        CHECK(childPublished->second.guideDrawSpheres[0]);
    }

    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    auto shell = [](const SdfPath &path) {
        const HdContainerDataSourceHandle origin =
            HdRetainedContainerDataSource::New(
                HdPrimOriginSchemaTokens->scenePath,
                HdRetainedTypedSampledDataSource<
                    HdPrimOriginSchema::OriginPath>::New(
                        HdPrimOriginSchema::OriginPath(path)));
        return HdRetainedSceneIndex::AddedPrimEntry{
            path, TfToken(),
            HdRetainedContainerDataSource::New(
                HdPrimOriginSchema::GetSchemaToken(), origin)};
    };
    upstream->AddPrims({shell(jointPath), shell(childPath)});
    const RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(upstream, bridge.GetStore());
    const SdfPath spherePath =
        jointPath.AppendChild(TfToken("rigGuideSphere_0"));
    const SdfPath conePath =
        jointPath.AppendChild(TfToken("rigGuideCone_0"));
    const SdfPathVector children = results->GetChildPrimPaths(jointPath);
    CHECK(children.size() == 3);  // authored ankle plus sphere and cone
    CHECK(std::find(children.begin(), children.end(), spherePath) !=
          children.end());
    CHECK(std::find(children.begin(), children.end(), conePath) !=
          children.end());
    const HdSceneIndexPrim cone = results->GetPrim(conePath);
    CHECK(cone.primType == HdPrimTypeTokens->cone);
    const HdConeSchema coneSchema =
        HdConeSchema::GetFromParent(cone.dataSource);
    CHECK(coneSchema.GetHeight());
    if (coneSchema.GetHeight()) {
        CHECK(std::abs(coneSchema.GetHeight()->GetTypedValue(0.0f) -
                       expectedLength) < 1e-9);
    }
    const HdSceneIndexPrim sphere = results->GetPrim(spherePath);
    CHECK(sphere.primType == HdPrimTypeTokens->sphere);
    CHECK(sphere.dataSource);
    const HdSphereSchema sphereSchema =
        HdSphereSchema::GetFromParent(sphere.dataSource);
    CHECK(sphereSchema.GetRadius());
    if (sphereSchema.GetRadius()) {
        CHECK(sphereSchema.GetRadius()->GetTypedValue(0.0f) == 1.0);
    }
    const HdPurposeSchema purpose =
        HdPurposeSchema::GetFromParent(sphere.dataSource);
    CHECK(purpose.GetPurpose());
    if (purpose.GetPurpose()) {
        CHECK(purpose.GetPurpose()->GetTypedValue(0.0f) ==
              HdRenderTagTokens->guide);
    }
    const HdXformSchema xform = HdXformSchema::GetFromParent(
        sphere.dataSource);
    CHECK(xform.GetMatrix());
    CHECK(xform.GetResetXformStack());
    if (xform.GetResetXformStack()) {
        CHECK(xform.GetResetXformStack()->GetTypedValue(0.0f));
    }

    const SdfPath childSphere =
        childPath.AppendChild(TfToken("rigGuideSphere_0"));
    const SdfPath childCone =
        childPath.AppendChild(TfToken("rigGuideCone_0"));
    CHECK(results->GetPrim(childSphere).primType == HdPrimTypeTokens->sphere);
    CHECK(!results->GetPrim(childCone).dataSource);

    // Moving the child changes the derived link without changing topology.
    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));
    const UsdAttribute childTx =
        stage->GetPrimAtPath(childPath).GetAttribute(TfToken("rest:tx"));
    CHECK(childTx && childTx.Set(6.0));
    RigExecImagingBridge::PublishResult update =
        bridge.EvaluateAndPublishResult(UsdTimeCode::Default());
    CHECK(update.ok);
    results->NotifyGenerationPublished(update.dirtied);
    CHECK(results->GetPrim(spherePath).dataSource);
    CHECK(results->GetPrim(conePath).primType == HdPrimTypeTokens->cone);
    CHECK(results->GetChildPrimPaths(jointPath).size() == 3);
    CHECK(observer.added.empty());
    CHECK(observer.removed.empty());
    const RigExecImagingSnapshotConstPtr movedSnapshot =
        bridge.GetStore()->Get();
    const auto moved = movedSnapshot->prims.find(jointPath);
    CHECK(moved != movedSnapshot->prims.end());
    if (moved != movedSnapshot->prims.end()) {
        const GfVec3d movedDelta(6.0, -2.6550437955052333, 0.0);
        CHECK(std::abs(moved->second.guideLengths[0] -
                       movedDelta.GetLength()) < 1e-9);
    }

    // Branching adds a second cone but not a duplicate parent sphere.
    const SdfPath branchPath = jointPath.AppendChild(TfToken("knee"));
    const UsdPrim branch = stage->DefinePrim(branchPath,
                                              TfToken("RigExecJoint"));
    CHECK(branch.GetAttribute(TfToken("rest:tx")).Set(-3.0));
    CHECK(branch.GetAttribute(TfToken("rest:ty")).Set(1.0));
    errors.clear();
    CHECK(bridge.Compile(&errors));
    update = bridge.EvaluateAndPublishResult(UsdTimeCode::Default());
    CHECK(update.ok);
    results->NotifyGenerationPublished(update.dirtied);
    const SdfPath branchCone =
        jointPath.AppendChild(TfToken("rigGuideCone_1"));
    const SdfPath duplicateSphere =
        jointPath.AppendChild(TfToken("rigGuideSphere_1"));
    CHECK(results->GetPrim(branchCone).primType == HdPrimTypeTokens->cone);
    CHECK(!results->GetPrim(duplicateSphere).dataSource);
    CHECK(results->GetChildPrimPaths(jointPath).size() == 4);
    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// The shipped minimal rig is the user-facing integration case: its lone
// control both publishes a wire guide and drives a MatrixMover without a
// boilerplate weight object. Rotating the control must revise the real mesh
// points, not merely rotate the guide.
static void
TestSimpleRigControlAndDeformation(const std::string &examplesDir)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/simple_rig.usd");
    CHECK(stage);
    if (!stage) return;

    const SdfPath rigPath("/World/RigExecRoot");
    const SdfPath controlPath(
        "/World/RigExecRoot/Controllers/Root");
    const SdfPath moverPath(
        "/World/RigExecRoot/Movers/RigExecMatrixMover1");
    const SdfPath spherePath("/World/Geom/Sphere");
    const SdfPath guidePath =
        controlPath.AppendChild(TfToken("rigGuideCtrl"));

    const UsdPrim mover = stage->GetPrimAtPath(moverPath);
    CHECK(mover);
    SdfPathVector weightObjects;
    if (const UsdRelationship relationship =
            mover.GetRelationship(TfToken("rigExec:weightObject"))) {
        relationship.GetTargets(&weightObjects);
    }
    CHECK(weightObjects.empty());

    RigExecImagingBridge bridge(stage, rigPath);
    std::vector<std::string> errors;
    const bool compiled = bridge.Compile(&errors);
    for (const std::string &error : errors) {
        std::printf("simple-rig compile error: %s\n", error.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;

    const UsdPrim control = stage->GetPrimAtPath(controlPath);
    CHECK(control);
    const UsdAttribute ry =
        control.GetAttribute(TfToken("avars:ry"));
    CHECK(ry);
    if (!control || !ry) return;

    CHECK(ry.Set(0.0));
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    const RigExecImagingSnapshotConstPtr restSnapshot =
        bridge.GetStore()->Get();
    CHECK(restSnapshot);
    if (!restSnapshot) return;
    const auto restSphere = restSnapshot->prims.find(spherePath);
    CHECK(restSphere != restSnapshot->prims.end());
    CHECK(restSphere != restSnapshot->prims.end() &&
          restSphere->second.hasPoints);
    if (restSphere == restSnapshot->prims.end() ||
        !restSphere->second.hasPoints) {
        return;
    }
    const VtVec3fArray restPoints = restSphere->second.points;
    CHECK(!restPoints.empty());

    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    const HdContainerDataSourceHandle origin =
        HdRetainedContainerDataSource::New(
            HdPrimOriginSchemaTokens->scenePath,
            HdRetainedTypedSampledDataSource<
                HdPrimOriginSchema::OriginPath>::New(
                    HdPrimOriginSchema::OriginPath(controlPath)));
    upstream->AddPrims(
        {{controlPath, TfToken(),
          HdRetainedContainerDataSource::New(
              HdPrimOriginSchema::GetSchemaToken(), origin)}});
    const RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(upstream, bridge.GetStore());
    const HdSceneIndexPrim guide = results->GetPrim(guidePath);
    CHECK(guide.primType == HdPrimTypeTokens->basisCurves);
    CHECK(!_GetPointsPrimvar(guide).empty());

    CHECK(ry.Set(45.0));
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    const RigExecImagingSnapshotConstPtr posedSnapshot =
        bridge.GetStore()->Get();
    CHECK(posedSnapshot);
    if (!posedSnapshot) return;
    const auto posedSphere = posedSnapshot->prims.find(spherePath);
    CHECK(posedSphere != posedSnapshot->prims.end());
    CHECK(posedSphere != posedSnapshot->prims.end() &&
          posedSphere->second.hasPoints);
    if (posedSphere == posedSnapshot->prims.end() ||
        !posedSphere->second.hasPoints) {
        return;
    }
    const VtVec3fArray &posedPoints = posedSphere->second.points;
    CHECK(posedPoints.size() == restPoints.size());
    double maxDisplacement = 0.0;
    for (size_t i = 0;
         i < posedPoints.size() && i < restPoints.size(); ++i) {
        maxDisplacement = std::max(
            maxDisplacement,
            (GfVec3d(posedPoints[i]) - GfVec3d(restPoints[i])).GetLength());
    }
    CHECK(maxDisplacement > 1e-4);
}

// Derived shader matrices cross the GPU boundary as float4x4, while the
// evaluated and authored matrices retain their double-precision contract.
static void
TestShaderMatrixPrimvars()
{
    const SdfPath eyePath("/Eye");
    const TfToken projector("projectorFrame");
    const TfToken dials("packedDials");
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const UsdPrim eye = stage->DefinePrim(eyePath, TfToken("Mesh"));
    UsdGeomPrimvarsAPI primvars(eye);
    GfMatrix4d authored(1.0);
    authored.SetTranslateOnly(GfVec3d(3.125, 164.301019281, -2.75));
    const UsdGeomPrimvar projectorAttr = primvars.CreatePrimvar(
        projector, SdfValueTypeNames->Matrix4d, UsdGeomTokens->constant);
    const UsdGeomPrimvar dialsAttr = primvars.CreatePrimvar(
        dials, SdfValueTypeNames->Matrix4d, UsdGeomTokens->constant);
    CHECK(projectorAttr.Set(authored));
    CHECK(dialsAttr.Set(GfMatrix4d(0.0)));
    std::string sourceBefore, sessionBefore;
    CHECK(stage->GetRootLayer()->ExportToString(&sourceBefore));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionBefore));

    auto upstream = UsdImagingStageSceneIndex::New();
    upstream->SetStage(stage);
    upstream->ApplyPendingUpdates();
    auto store = std::make_shared<RigExecSnapshotStore>();
    auto results = RigExecResultsSceneIndex::New(upstream, store);
    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));

    RigExecPublishedPrim published;
    published.shaderMatrices[projector] = authored;
    // Distinct values in all sixteen positions catch accidental transpose
    // or dial-slot repacking as well as the float/double storage mismatch.
    GfMatrix4d packed(0.0);
    for (size_t row = 0; row < 4; ++row) {
        for (size_t col = 0; col < 4; ++col) {
            packed[row][col] = 100.0 * row + 10.0 * col + 0.1234567890123;
        }
    }
    published.shaderMatrices[dials] = packed;
    auto generation = [&]() {
        auto snapshot = std::make_shared<RigExecImagingSnapshot>();
        snapshot->stage = stage;
        snapshot->prims[eyePath] = published;
        return snapshot;
    };
    auto checkMatrices = [&]() {
        const HdPrimvarsSchema drawn = HdPrimvarsSchema::GetFromParent(
            results->GetPrim(eyePath).dataSource);
        for (const auto &[name, matrix] : published.shaderMatrices) {
            const HdPrimvarSchema pv = drawn.GetPrimvar(name);
            CHECK(pv.GetInterpolation() &&
                  pv.GetInterpolation()->GetTypedValue(0.0f) ==
                      HdPrimvarSchemaTokens->constant);
            const HdSampledDataSourceHandle value = pv.GetPrimvarValue();
            CHECK(value);
            if (!value) continue;
            const VtValue v = value->GetValue(0.0f);
            CHECK(v.IsHolding<GfMatrix4f>());
            CHECK(!v.IsHolding<GfMatrix4d>());
            const HdTupleType type = HdGetValueTupleType(v);
            CHECK(type.type == HdTypeFloatMat4 && type.count == 1);
            CHECK(HdDataSizeOfTupleType(type) == 16 * sizeof(float));
            if (v.IsHolding<GfMatrix4f>()) {
                CHECK(v.UncheckedGet<GfMatrix4f>() == GfMatrix4f(matrix));
            }
            // Only the GPU-facing value is narrowed. The authoritative
            // evaluated snapshot and the upstream authored value stay double.
            CHECK(store->Get()->prims.at(eyePath).shaderMatrices.at(name) == matrix);
            const VtValue upstreamValue = HdPrimvarsSchema::GetFromParent(
                upstream->GetPrim(eyePath).dataSource).GetPrimvar(name)
                    .GetPrimvarValue()->GetValue(0.0f);
            CHECK(upstreamValue.IsHolding<GfMatrix4d>());
        }
    };
    results->NotifyGenerationPublished(store->Publish(generation()));
    checkMatrices();

    observer.added.clear();
    observer.removed.clear();
    observer.dirtied.clear();
    observer.dirtiedLocators.clear();
    published.shaderMatrices[projector].SetRotateOnly(
        GfRotation(GfVec3d(0, 1, 0), 27.0));
    const RigExecPublishedDirtyVector dirty = store->Publish(generation());
    CHECK(dirty.size() == 1);
    if (dirty.size() == 1) {
        CHECK(dirty[0].path == eyePath);
        CHECK(dirty[0].changes == RigExecChangeShaderMatrix);
    }
    results->NotifyGenerationPublished(dirty);
    checkMatrices();
    CHECK(observer.added.empty() && observer.removed.empty());
    CHECK(observer.dirtied.size() == 1);
    if (observer.dirtied.size() == 1) {
        CHECK(observer.dirtied[0] == eyePath);
        CHECK(observer.dirtiedLocators[0] ==
              HdContainerDataSourceEditor::ComputeDirtyLocators(
                  HdDataSourceLocatorSet{HdPrimvarsSchema::GetDefaultLocator()}));
    }
    CHECK(store->Publish(generation()).empty());
    GfMatrix4d authoredAfter(0.0);
    CHECK(projectorAttr.Get(&authoredAfter));
    CHECK(authoredAfter == authored);
    CHECK(projectorAttr.GetTypeName() == SdfValueTypeNames->Matrix4d);
    std::string sourceAfter, sessionAfter;
    CHECK(stage->GetRootLayer()->ExportToString(&sourceAfter));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionAfter));
    CHECK(sourceAfter == sourceBefore && sessionAfter == sessionBefore);
    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// A control drag changes the final transform of a cached unit shape, not
// its geometry. Rebuilding that geometry on every sample is unnecessary
// and can force a renderer through curve topology/material preparation.
static void
TestControlGuidePoseDirtiness()
{
    const SdfPath assetRoot("/Asset");
    const SdfPath control("/Asset/Ctrl");
    const SdfPath guide = control.AppendChild(TfToken("rigGuideCtrl"));
    auto upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims(
        {{assetRoot, TfToken(), HdRetainedContainerDataSource::New()},
         {control, TfToken(), HdRetainedContainerDataSource::New()}});
    auto store = std::make_shared<RigExecSnapshotStore>();
    auto results = RigExecResultsSceneIndex::New(upstream, store);
    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));

    RigExecPublishedPrim published;
    published.assetRoot = assetRoot;
    published.hasControlGuide = true;
    published.hasControlFrame = true;
    published.controlGuideShape = TfToken("circle");
    published.controlGuideDrawMode = TfToken("wire");
    published.controlGuidePlaneNormal = TfToken("Y");
    auto generation = [&]() {
        auto snapshot = std::make_shared<RigExecImagingSnapshot>();
        snapshot->assetRoot = assetRoot;
        snapshot->prims[control] = published;
        return snapshot;
    };
    results->NotifyGenerationPublished(store->Publish(generation()));
    const HdSceneIndexPrim initial = results->GetPrim(guide);
    CHECK(initial.dataSource);
    const VtVec3fArray initialPoints = _GetPointsPrimvar(initial);
    const VtIntArray initialCounts = _GuideTopologyCounts(initial);
    const auto initialExtent = HdExtentSchema::GetFromParent(initial.dataSource);
    CHECK(initialExtent.GetMin() && initialExtent.GetMax());
    const GfVec3d extentMin = initialExtent.GetMin()->GetTypedValue(0.0f);
    const GfVec3d extentMax = initialExtent.GetMax()->GetTypedValue(0.0f);
    const HdDataSourceLocatorSet xformLocators =
        HdContainerDataSourceEditor::ComputeDirtyLocators(
            HdDataSourceLocatorSet{HdDataSourceLocator(
                HdXformSchemaTokens->xform, HdXformSchemaTokens->matrix)});

    auto publish = [&](uint16_t expectedChanges, bool universal) {
        observer.added.clear();
        observer.removed.clear();
        observer.dirtied.clear();
        observer.dirtiedLocators.clear();
        const RigExecPublishedDirtyVector dirty = store->Publish(generation());
        CHECK(dirty.size() == 1);
        if (dirty.size() == 1) {
            CHECK(dirty[0].path == control);
            CHECK(dirty[0].changes == expectedChanges);
        }
        results->NotifyGenerationPublished(dirty);
        CHECK(observer.added.empty());
        CHECK(observer.removed.empty());
        size_t guideDirties = 0;
        for (size_t i = 0; i < observer.dirtied.size(); ++i) {
            if (observer.dirtied[i] != guide) continue;
            ++guideDirties;
            const HdDataSourceLocatorSet &locators = observer.dirtiedLocators[i];
            CHECK(locators == (universal
                ? HdDataSourceLocatorSet::UniversalSet() : xformLocators));
            if (!universal) {
                CHECK(locators.Intersects(HdXformSchema::GetDefaultLocator()));
                CHECK(!locators.Intersects(HdPrimvarsSchema::GetDefaultLocator()));
                CHECK(!locators.Intersects(HdBasisCurvesSchema::GetDefaultLocator()));
                CHECK(!locators.Intersects(HdExtentSchema::GetDefaultLocator()));
                CHECK(!locators.Intersects(HdDataSourceLocator(TfToken("materialBindings"))));
                CHECK(!locators.Intersects(HdLegacyDisplayStyleSchema::GetDefaultLocator()));
                const HdSceneIndexPrim current = results->GetPrim(guide);
                CHECK(_GetPointsPrimvar(current) == initialPoints);
                CHECK(_GuideTopologyCounts(current) == initialCounts);
                const auto extent = HdExtentSchema::GetFromParent(current.dataSource);
                CHECK(extent.GetMin()->GetTypedValue(0.0f) == extentMin);
                CHECK(extent.GetMax()->GetTypedValue(0.0f) == extentMax);
            }
        }
        CHECK(guideDirties == 1);
    };
    auto guideMatrix = [&]() {
        return HdXformSchema::GetFromParent(results->GetPrim(guide).dataSource)
            .GetMatrix()->GetTypedValue(0.0f);
    };

    published.controlFrame.SetTranslate(GfVec3d(3, 0, 0));
    published.controlGuideFrame = published.controlFrame;
    publish(RigExecChangeControlGuideXform, false);
    CHECK(guideMatrix() == published.controlGuideFrame);

    published.controlGuideFrame.SetRotateOnly(
        GfRotation(GfVec3d(0, 0, 1), 30.0));
    publish(RigExecChangeControlGuideXform, false);
    CHECK(guideMatrix() == published.controlGuideFrame);

    published.controlGuideScale = GfVec3d(2, 3, 4);
    publish(RigExecChangeControlGuideXform, false);
    GfMatrix4d scale(1.0);
    scale.SetScale(published.controlGuideScale);
    CHECK(guideMatrix() == scale * published.controlGuideFrame);

    // The native manipulator frame is published separately from the
    // rigidized drawing frame. Its change must not revive universal
    // curve dirtiness when the drawing frame itself stays unchanged.
    published.controlFrame.SetTranslate(GfVec3d(4, 0, 0));
    publish(RigExecChangeControlGuideXform, false);
    CHECK(store->Get()->prims.at(control).controlFrame == published.controlFrame);
    CHECK(guideMatrix() == scale * published.controlGuideFrame);

    published.guideOpacity = 0.25f;
    publish(RigExecChangeGuides, true);
    published.controlGuideFrame.SetTranslateOnly(GfVec3d(5, 0, 0));
    published.guideColor = GfVec3f(0, 1, 0);
    publish(RigExecChangeControlGuideXform | RigExecChangeGuides, true);
    published.guidePurpose = TfToken("guide");
    publish(RigExecChangeGuides, true);
    published.controlGuideWireWidth = 0.2;
    publish(RigExecChangeGuides, true);
    published.controlGuideWireWidth = 0.0;
    publish(RigExecChangeGuides, true);
    CHECK(!HdPrimvarsSchema::GetFromParent(results->GetPrim(guide).dataSource)
        .GetPrimvar(HdTokens->widths));

    // A same-type shape change is still structural: its points and local
    // extent differ, even though the existing child needs no add/remove.
    published.controlGuideShape = TfToken("box");
    publish(RigExecChangeStructural, true);
    CHECK(store->Publish(generation()).empty());
    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// A constraint-driven ASSET ROOT carries the synthesized guides with it,
// and says so.
// A guide's matrix is not inherited, it is BAKED: the builder composes the
// asset root's world transform into the guide's own matrix and declares the
// stack reset. So when the rig drives its own root, every guide's transform
// changes while its published payload -- frame, scale, styling -- stays
// byte-identical. RigExecChangeGuides never fires, and Hydra dirtiness is
// not hierarchical, so no ancestor notice reaches a reset-stack prim
// either. Nothing but this walk can tell a cached renderer to pull again;
// without it the asset walks off and its guides stay where they were.
static void
TestGuidesFollowDrivenAssetRoot()
{
    const SdfPath assetRoot("/Asset");
    const SdfPath rig("/Asset/Rig");
    const SdfPath controls("/Asset/Rig/Controls");
    const SdfPath control("/Asset/Rig/Controls/Ctrl");
    const SdfPath guide = control.AppendChild(TfToken("rigGuideCtrl"));

    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    static const TfToken xformName = HdXformSchemaTokens->xform;
    const HdDataSourceBaseHandle identityXform =
        HdXformSchema::Builder()
            .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(
                GfMatrix4d(1.0)))
            .SetResetXformStack(
                HdRetainedTypedSampledDataSource<bool>::New(true))
            .Build();
    upstream->AddPrims(
        {{assetRoot, TfToken(),
          HdRetainedContainerDataSource::New(1, &xformName, &identityXform)},
         {rig, TfToken(),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)},
         {controls, TfToken(),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)},
         {control, TfToken(),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});

    auto store = std::make_shared<RigExecSnapshotStore>();
    auto results = RigExecResultsSceneIndex::New(upstream, store);
    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));

    // The control's guide payload is IDENTICAL across both generations --
    // only the asset root's driven transform moves. That is the whole point:
    // the diff produces no guide change for the control at all.
    auto publish = [&](double rootX) {
        auto snapshot = std::make_shared<RigExecImagingSnapshot>();
        snapshot->assetRoot = assetRoot;
        RigExecPublishedPrim &root = snapshot->prims[assetRoot];
        root.hasXform = true;
        root.xformBase = GfMatrix4d(1.0);
        GfMatrix4d revised(1.0);
        revised.SetTranslateOnly(GfVec3d(rootX, 0, 0));
        root.xform = revised;
        RigExecPublishedPrim &published = snapshot->prims[control];
        published.hasControlGuide = true;
        published.controlGuideFrame = GfMatrix4d(1.0);
        published.controlGuideShape = TfToken("circle");
        published.controlGuideDrawMode = TfToken("wire");
        published.controlGuideScale = GfVec3d(1, 1, 1);
        results->NotifyGenerationPublished(store->Publish(snapshot));
    };
    auto guideTranslation = [&]() {
        HdXformSchema xf = HdXformSchema::GetFromParent(
            results->GetPrim(guide).dataSource);
        return xf && xf.GetMatrix()
            ? xf.GetMatrix()->GetTypedValue(0.0f).ExtractTranslation()
            : GfVec3d(0);
    };

    publish(0.0);
    CHECK(results->GetPrim(guide).dataSource);
    CHECK((guideTranslation() - GfVec3d(0, 0, 0)).GetLength() < 1e-6);

    observer.dirtied.clear();
    publish(5.0);

    // The pull is right...
    const GfVec3d moved = guideTranslation();
    if ((moved - GfVec3d(5, 0, 0)).GetLength() >= 1e-6) {
        std::printf("  guide at (%g, %g, %g), expected the asset root's "
                    "(5, 0, 0)\n", moved[0], moved[1], moved[2]);
    }
    CHECK((moved - GfVec3d(5, 0, 0)).GetLength() < 1e-6);

    // ...and the renderer was told to take it.
    bool sawGuideDirty = false;
    for (const SdfPath &p : observer.dirtied) {
        if (p == guide) sawGuideDirty = true;
    }
    if (!sawGuideDirty) {
        std::printf("  a driven asset root moved the guide without dirtying "
                    "it -- a cached viewport leaves it behind\n");
    }
    CHECK(sawGuideDirty);

    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// An aggregate solver's guides are sized by guide:radius on the SOLVER,
// the same way a joint's are sized by guide:radius on the joint.
// The elements have no prim of their own, which is why they were pinned at
// Hydra's fallback radius of 1.0 long after the joints stopped being. On an
// asset authored at one unit -- chars/puppetA is 0.59 units tall -- that is
// several times the whole character, so turning guide display on buried the
// rig in solver spheres and cones. The solver prim already carries the
// guide colour and opacity; the radius is read from the same place.
static void
TestSolverGuideRadius(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
    CHECK(stage);
    if (!stage) return;

    const SdfPath rig("/TailAsset/Rig");
    const SdfPath solver("/TailAsset/Rig/Solvers/TailFK");
    RigExecImagingBridge bridge(stage, rig);
    std::vector<std::string> errors;
    const bool compiled = bridge.Compile(&errors);
    for (const std::string &e : errors) {
        std::printf("  compile error: %s\n", e.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;

    // Guides only synthesize under a parent the upstream index knows.
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims({{solver, TfToken(), nullptr}});
    auto results = RigExecResultsSceneIndex::New(upstream, bridge.GetStore());

    const SdfPath sphere = solver.AppendChild(TfToken("rigGuideSphere_0"));
    auto radius = [&]() {
        HdSphereSchema s =
            HdSphereSchema::GetFromParent(results->GetPrim(sphere).dataSource);
        return s.GetRadius() ? s.GetRadius()->GetTypedValue(0.0f) : -1.0;
    };
    auto announced = [&]() {
        for (const SdfPath &p : results->GetChildPrimPaths(solver)) {
            if (p == sphere) return true;
        }
        return false;
    };

    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
    CHECK(announced());
    CHECK(radius() == 1.0);  // the schema default: unchanged for every
                             // rig that authors nothing

    UsdPrim solverPrim = stage->GetPrimAtPath(solver);
    CHECK(solverPrim);
    CHECK(solverPrim.GetAttribute(TfToken("guide:radius")).Set(0.25));
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
    CHECK(radius() == 0.25);

    // Zero draws nothing at all, exactly as it does on a joint -- which is
    // how an author turns a solver's diagnostics off.
    CHECK(solverPrim.GetAttribute(TfToken("guide:radius")).Set(0.0));
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
    CHECK(!announced());
}

// The shipped examples' authored control guides actually draw.
// An unrecognized guide:shape token draws nothing, silently and by design
// (allowedTokens is documentation, not enforcement). That makes a typo in
// an example a defect no other test can see: the rig still compiles, still
// evaluates, and still publishes -- it just stops showing the thing the
// example exists to show.
static void
TestExampleControlGuides(const std::string &examplesDir)
{
    struct _Case {
        const char *control;
        TfToken primType;
        std::vector<int> counts;    // empty for the implicits
        GfVec3d scale;              // authored guide:scaleX/Y/Z
    };
    const GfVec3d unit(1, 1, 1);
    const struct {
        const char *file;
        const char *rig;
        const char *controls;
        std::vector<_Case> cases;
    } examples[] = {
        // Every scale here is above 1 on purpose: at rest the control, its
        // joint, and its solver element coincide, and the unit-radius
        // joint/solver guides would otherwise swallow every click aimed at
        // the control. The values are load-bearing for selectability, not
        // decoration, so they are asserted rather than left to drift.
        // The example draws only 3D shapes: the planar circle and box lie
        // in the local XZ plane and are edge-on from the default front
        // view, so they read as bare lines there. Their topology is
        // asserted exhaustively in TestControlGuides, and again in memory
        // at the end of this function.
        {"/01_FkChainTail.usda", "/TailAsset/Rig", "/TailAsset/Rig/Controls",
         {// diamond wire: three orthogonal rings through the ±axis vertices
          {"Tail1", HdPrimTypeTokens->basisCurves, {5, 5, 5},
           GfVec3d(1.6, 1.6, 1.6)},
          // cube geometry: Hydra's implicit, sized by the xform alone
          {"Tail2", HdPrimTypeTokens->cube, {}, GfVec3d(1.2, 1.2, 1.2)},
          // sphere wire: three orthogonal 33-point rings, squashed in Y
          {"Tail3", HdPrimTypeTokens->basisCurves, {33, 33, 33},
           GfVec3d(1.8, 0.6, 1.8)},
          // pyramid wire: base ring plus four apex edges
          {"Tail4", HdPrimTypeTokens->basisCurves, {5, 2, 2, 2, 2},
           GfVec3d(1.5, 1.5, 1.5)}}},
        {"/02_TwoBoneIkLeg.usda", "/LegAsset/Rig", "/LegAsset/Rig/Controls",
         {{"HipRoot", HdPrimTypeTokens->cube, {}, GfVec3d(0.6, 0.6, 0.6)},
          {"FootIK", HdPrimTypeTokens->basisCurves, {33, 33, 33}, unit},
          {"KneePole", HdPrimTypeTokens->sphere, {},
           GfVec3d(0.4, 0.4, 0.4)}}}};

    for (const auto &example : examples) {
        UsdStageRefPtr stage =
            UsdStage::Open(examplesDir + example.file);
        CHECK(stage);
        if (!stage) continue;
        RigExecImagingBridge bridge(stage, SdfPath(example.rig));
        std::vector<std::string> errors;
        const bool compiled = bridge.Compile(&errors);
        for (const std::string &e : errors) {
            std::printf("%s compile error: %s\n", example.file, e.c_str());
        }
        CHECK(compiled);
        if (!compiled) continue;

        // Shells carrying a primOrigin, as flattened UsdImaging prims do:
        // picking a guide has to resolve back to the control it draws.
        HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
        HdRetainedSceneIndex::AddedPrimEntries shells;
        for (const _Case &c : example.cases) {
            const SdfPath control =
                SdfPath(example.controls).AppendChild(TfToken(c.control));
            shells.push_back(
                {control, TfToken(),
                 HdRetainedContainerDataSource::New(
                     HdPrimOriginSchema::GetSchemaToken(),
                     HdRetainedContainerDataSource::New(
                         HdPrimOriginSchemaTokens->scenePath,
                         HdRetainedTypedSampledDataSource<
                             HdPrimOriginSchema::OriginPath>::New(
                                 HdPrimOriginSchema::OriginPath(control))))});
        }
        upstream->AddPrims(shells);
        auto results =
            RigExecResultsSceneIndex::New(upstream, bridge.GetStore());
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));

        for (const _Case &c : example.cases) {
            const SdfPath control =
                SdfPath(example.controls).AppendChild(TfToken(c.control));
            const SdfPath guide =
                control.AppendChild(TfToken("rigGuideCtrl"));

            bool announced = false;
            for (const SdfPath &p : results->GetChildPrimPaths(control)) {
                if (p == guide) announced = true;
            }
            CHECK(announced);

            const HdSceneIndexPrim prim = results->GetPrim(guide);
            if (prim.primType != c.primType) {
                std::printf("  %s %s: guide is a '%s', expected '%s' -- an "
                            "unrecognized guide:shape/drawMode draws "
                            "nothing\n", example.file, c.control,
                            prim.primType.GetText(), c.primType.GetText());
            }
            CHECK(prim.primType == c.primType);
            CHECK(prim.dataSource);
            if (!prim.dataSource) continue;

            // Topology, for the shapes that carry their own.
            const VtIntArray counts = _GuideTopologyCounts(prim);
            const bool countsMatch =
                counts.size() == c.counts.size() &&
                std::equal(counts.begin(), counts.end(), c.counts.begin());
            if (!countsMatch) {
                std::printf("  %s %s: %zu topology entries, expected %zu\n",
                            example.file, c.control, counts.size(),
                            c.counts.size());
            }
            CHECK(countsMatch);

            // The implicits carry no topology at all: their dimension is
            // the unit one, and the authored scale reaches them only
            // through the xform asserted below. A cube that shipped at
            // Hydra's fallback size would draw at half the authored size
            // with nothing else to show for it.
            if (c.primType == HdPrimTypeTokens->cube) {
                HdCubeSchema cube =
                    HdCubeSchema::GetFromParent(prim.dataSource);
                CHECK(cube.GetSize() &&
                      cube.GetSize()->GetTypedValue(0.0f) == 2.0);
            } else if (c.primType == HdPrimTypeTokens->sphere) {
                HdSphereSchema sphere =
                    HdSphereSchema::GetFromParent(prim.dataSource);
                CHECK(sphere.GetRadius() &&
                      sphere.GetRadius()->GetTypedValue(0.0f) == 1.0);
            }

            // The authored per-axis scale, as the xform's basis lengths.
            // Nothing upstream places the asset root, so it resolves to
            // identity and the guide's transform is scale * posed frame.
            HdXformSchema xf =
                HdXformSchema::GetFromParent(prim.dataSource);
            CHECK(xf && xf.GetMatrix());
            if (xf && xf.GetMatrix()) {
                const GfMatrix4d xform = xf.GetMatrix()->GetTypedValue(0.0f);
                for (size_t i = 0; i < 3; ++i) {
                    const double length = xform.GetRow3(i).GetLength();
                    if (std::abs(length - c.scale[i]) > 1e-6) {
                        std::printf("  %s %s: basis %zu length %g, expected "
                                    "%g\n", example.file, c.control, i,
                                    length, c.scale[i]);
                    }
                    CHECK(std::abs(length - c.scale[i]) < 1e-6);
                }
            }

            // Purpose, styling, and picking.
            HdPurposeSchema purpose =
                HdPurposeSchema::GetFromParent(prim.dataSource);
            CHECK(purpose.GetPurpose() &&
                  purpose.GetPurpose()->GetTypedValue(0.0f) ==
                      HdRenderTagTokens->geometry);

            HdPrimvarsSchema primvars =
                HdPrimvarsSchema::GetFromParent(prim.dataSource);
            for (const TfToken &name :
                 {HdTokens->displayColor, HdTokens->displayOpacity}) {
                HdPrimvarSchema primvar = primvars.GetPrimvar(name);
                CHECK(primvar.GetPrimvarValue());
                CHECK(primvar.GetInterpolation() &&
                      primvar.GetInterpolation()->GetTypedValue(0.0f) ==
                          HdPrimvarSchemaTokens->constant);
            }

            HdPrimOriginSchema origin =
                HdPrimOriginSchema::GetFromParent(prim.dataSource);
            const SdfPath resolved =
                origin ? origin.GetOriginPath(
                             HdPrimOriginSchemaTokens->scenePath)
                       : SdfPath();
            if (resolved != control) {
                std::printf("  %s %s: guide primOrigin resolves to '%s' -- "
                            "picking it does not select the control\n",
                            example.file, c.control,
                            resolved.GetText());
            }
            CHECK(resolved == control);
        }

        // No example draws circle or box any more: both lie in the local
        // XZ plane, so they are edge-on from the default front view and
        // read as bare lines there. They are still shapes a real rig must
        // be able to draw, and dropping them from the examples must not
        // quietly drop them from the coverage -- so they are authored here
        // instead, keeping the planar topology asserted against a composed
        // example stage rather than only against the synthetic sweep in
        // TestControlGuides.
        const SdfPath probe =
            SdfPath(example.controls)
                .AppendChild(TfToken(example.cases.front().control));
        const SdfPath probeGuide =
            probe.AppendChild(TfToken("rigGuideCtrl"));
        struct _Planar {
            const char *shape;
            const char *drawMode;
            TfToken primType;
            std::vector<int> counts;
            size_t points;
        };
        const _Planar planars[] = {
            // One 33-point closed ring: 32 segments plus the repeat that
            // closes it.
            {"circle", "wire", HdPrimTypeTokens->basisCurves, {33}, 33},
            // One quad face.
            {"box", "geometry", HdPrimTypeTokens->mesh, {4}, 4}};
        for (const _Planar &planar : planars) {
            const UsdPrim probePrim = stage->GetPrimAtPath(probe);
            CHECK(probePrim.GetAttribute(TfToken("guide:shape"))
                      .Set(TfToken(planar.shape)));
            CHECK(probePrim.GetAttribute(TfToken("guide:drawMode"))
                      .Set(TfToken(planar.drawMode)));
            CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));

            const HdSceneIndexPrim guide = results->GetPrim(probeGuide);
            if (guide.primType != planar.primType) {
                std::printf("  %s in-memory %s/%s drew a '%s', expected "
                            "'%s'\n", example.file, planar.shape,
                            planar.drawMode, guide.primType.GetText(),
                            planar.primType.GetText());
            }
            CHECK(guide.primType == planar.primType);
            CHECK(guide.dataSource);
            if (!guide.dataSource) continue;
            const VtIntArray counts = _GuideTopologyCounts(guide);
            const bool countsMatch =
                counts.size() == planar.counts.size() &&
                std::equal(counts.begin(), counts.end(),
                           planar.counts.begin());
            if (!countsMatch) {
                std::printf("  %s in-memory %s/%s: %zu topology entries, "
                            "expected %zu\n", example.file, planar.shape,
                            planar.drawMode, counts.size(),
                            planar.counts.size());
            }
            CHECK(countsMatch);
            CHECK(_GetPointsPrimvar(guide).size() == planar.points);
        }
    }
}

// Edit-driven re-evaluation (registry): any authored edit beneath the
// rig's asset re-evaluates at the last-set time and republishes;
// unrelated edits do not.
static void
TestMultiRigAtomicActivation(const std::string &examplesDir)
{
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    registry.Deactivate();

    // Two independent references exercise different character roots while
    // keeping the fixture small and identical to the single-rig goldens.
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    CHECK(stage);
    if (!stage) return;
    const std::string asset = TfAbsPath(examplesDir + "/ArmRig.usda");
    const UsdPrim armA = stage->DefinePrim(SdfPath("/ArmA"), TfToken("Xform"));
    const UsdPrim armB = stage->DefinePrim(SdfPath("/ArmB"), TfToken("Xform"));
    CHECK(armA.GetReferences().AddReference(asset, SdfPath("/ArmAsset")));
    CHECK(armB.GetReferences().AddReference(asset, SdfPath("/ArmAsset")));
    CHECK(stage->GetPrimAtPath(SdfPath("/ArmA/Rig")));
    CHECK(stage->GetPrimAtPath(SdfPath("/ArmB/Rig")));
    // Create the local property opinion before observing edits: first-time
    // authoring may send separate property-creation and value notices.
    CHECK(stage->GetAttributeAtPath(
              SdfPath("/ArmA/Rig/Controls/ShoulderFK.avars:tx")).Set(0.0));

    std::vector<std::string> errors;
    CHECK(registry.Activate(
        stage, SdfPath(), UsdTimeCode(1001.0), &errors));
    for (const std::string &error : errors) {
        std::printf("  multi-rig activation: %s\n", error.c_str());
    }
    RigExecImagingSnapshotConstPtr snapshot = registry.GetStore()->Get();
    CHECK(snapshot);
    if (!snapshot) {
        registry.Deactivate();
        return;
    }
    CHECK(snapshot->Describes(stage, UsdTimeCode(1001.0)));
    CHECK(snapshot->prims.count(SdfPath("/ArmA/Geom/ArmBody")) != 0);
    CHECK(snapshot->prims.count(SdfPath("/ArmB/Geom/ArmBody")) != 0);
    const auto guideA = snapshot->prims.find(
        SdfPath("/ArmA/Rig/Controls/ShoulderFK"));
    const auto guideB = snapshot->prims.find(
        SdfPath("/ArmB/Rig/Controls/ShoulderFK"));
    CHECK(guideA != snapshot->prims.end());
    CHECK(guideB != snapshot->prims.end());
    if (guideA != snapshot->prims.end()) {
        CHECK(guideA->second.assetRoot == SdfPath("/ArmA"));
    }
    if (guideB != snapshot->prims.end()) {
        CHECK(guideB->second.assetRoot == SdfPath("/ArmB"));
    }
    double bounds[6];
    CHECK(RigExecImaging_GetGuideBoundsAssetSpace(
              "/ArmA/Rig/Controls/ShoulderFK", bounds) == 1);
    CHECK(RigExecImaging_GetAllGuideBoundsAssetSpace(bounds) == 0);

    const uint64_t generation = snapshot->generation;
    CHECK(registry.SetTime(UsdTimeCode(1024.0)));
    snapshot = registry.GetStore()->Get();
    CHECK(snapshot && snapshot->generation == generation + 1);
    CHECK(snapshot && snapshot->prims.count(
                           SdfPath("/ArmA/Geom/ArmBody")) != 0);
    CHECK(snapshot && snapshot->prims.count(
                           SdfPath("/ArmB/Geom/ArmBody")) != 0);

    // Same-frame edits evaluate only the affected character session; the
    // atomic combined publication reuses the other character's snapshot.
    const SdfPath rigA("/ArmA/Rig"), rigB("/ArmB/Rig");
    const size_t pullsA = registry.GetSessionEvaluationCount(rigA);
    const size_t pullsB = registry.GetSessionEvaluationCount(rigB);
    CHECK(stage->GetAttributeAtPath(
              SdfPath("/ArmA/Rig/Controls/ShoulderFK.avars:tx")).Set(1.0));
    CHECK(registry.GetSessionEvaluationCount(rigA) == pullsA + 1);
    CHECK(registry.GetSessionEvaluationCount(rigB) == pullsB);
    CHECK(registry.SetTime(UsdTimeCode(1024.0)));
    CHECK(registry.GetSessionEvaluationCount(rigA) == pullsA + 1);
    CHECK(registry.GetSessionEvaluationCount(rigB) == pullsB);
    CHECK(registry.SetTime(UsdTimeCode(1025.0)));
    CHECK(registry.GetSessionEvaluationCount(rigA) == pullsA + 2);
    CHECK(registry.GetSessionEvaluationCount(rigB) == pullsB + 1);
    snapshot = registry.GetStore()->Get();

    // A replacement that cannot compile/evaluate must not tear down or
    // overwrite the coherent stage that is already active.
    const RigExecImagingSnapshotConstPtr beforeFailure = snapshot;
    const UsdStageRefPtr noRig = UsdStage::CreateInMemory();
    errors.clear();
    CHECK(!registry.Activate(
        noRig, SdfPath(), UsdTimeCode(1.0), &errors));
    CHECK(!errors.empty());
    CHECK(registry.IsActive());
    CHECK(registry.GetStore()->Get() == beforeFailure);

    registry.Deactivate();
    CHECK(!registry.GetStore()->Get());
}

static void
TestEditTriggeredReevaluation(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmRig.usda");
    CHECK(stage);
    if (!stage) return;
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    const bool activated = registry.Activate(
        stage, SdfPath("/ArmAsset/Rig"), UsdTimeCode(1.0), &errors);
    for (const std::string &e : errors) {
        std::printf("activate error: %s\n", e.c_str());
    }
    CHECK(activated);
    if (!activated) return;
    const uint64_t generation0 = registry.GetStore()->Get()->generation;

    // A value edit on a solver input that factors into the final frame
    // republishes without any timeline change.
    stage->GetAttributeAtPath(
             SdfPath("/ArmAsset/Rig/Solvers/IKFKBlend.inputs:weight"))
        .Set(0.75f);
    RigExecImagingSnapshotConstPtr snapshot = registry.GetStore()->Get();
    CHECK(snapshot && snapshot->generation > generation0);

    // A joint guide-styling edit republishes too.
    const uint64_t generation1 = snapshot->generation;
    stage->GetPrimAtPath(
             SdfPath("/ArmAsset/Rig/Joints/Shoulder/Elbow/Wrist"))
        .GetAttribute(TfToken("guide:radius"))
        .Set(0.75);
    snapshot = registry.GetStore()->Get();
    CHECK(snapshot && snapshot->generation > generation1);

    // ...as does a control guide edit, which is how an animator retyping a
    // guide sees it change in the viewport rather than on the next frame
    // change. The republished payload carries the new shape.
    const uint64_t generationGuide = snapshot->generation;
    const SdfPath shoulderFk("/ArmAsset/Rig/Controls/ShoulderFK");
    CHECK(stage->GetPrimAtPath(shoulderFk)
              .GetAttribute(TfToken("guide:shape"))
              .Set(TfToken("pyramid")));
    snapshot = registry.GetStore()->Get();
    CHECK(snapshot && snapshot->generation > generationGuide);
    if (snapshot) {
        const auto it = snapshot->prims.find(shoulderFk);
        CHECK(it != snapshot->prims.end());
        if (it != snapshot->prims.end()) {
            CHECK(it->second.hasControlGuide);
            CHECK(it->second.controlGuideShape == TfToken("pyramid"));
        }
    }

    // Unrelated edits outside the asset do not re-evaluate.
    const uint64_t generation2 = snapshot->generation;
    stage->DefinePrim(SdfPath("/Elsewhere"), TfToken("Scope"));
    stage->GetPrimAtPath(SdfPath("/Elsewhere"))
        .CreateAttribute(TfToken("unrelated"), SdfValueTypeNames->Float)
        .Set(1.0f);
    snapshot = registry.GetStore()->Get();
    CHECK(snapshot && snapshot->generation == generation2);

    registry.Deactivate();
}

class _TestMotionMatrix final : public HdTypedSampledDataSource<GfMatrix4d> {
public:
    HD_DECLARE_DATASOURCE(_TestMotionMatrix);
    GfMatrix4d GetTypedValue(Time offset) override { return offset < 0 ? _a : _b; }
    VtValue GetValue(Time offset) override { return VtValue(GetTypedValue(offset)); }
    bool GetContributingSampleTimesForInterval(Time, Time, std::vector<Time> *out) override {
        *out = {-11.5f, 11.5f};
        return true;
    }
private:
    _TestMotionMatrix(GfMatrix4d a, GfMatrix4d b) : _a(a), _b(b) {}
    GfMatrix4d _a, _b;
};

// Spatial projector matrices are target-local in the snapshot, world-space
// at the shader boundary. Packed dial matrices must never be transformed.
static void
TestShaderMatrixWorldSpace()
{
    const SdfPath assetPath("/Asset");
    const SdfPath eyePath("/Asset/Eye");
    const TfToken projector("spatialFrame");
    const TfToken dials("packedParameters");
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const UsdPrim asset = stage->DefinePrim(assetPath, TfToken("Xform"));
    const UsdPrim eye = stage->DefinePrim(eyePath, TfToken("Mesh"));
    const UsdGeomXformOp assetOp = UsdGeomXformable(asset).AddTransformOp();
    const UsdGeomXformOp eyeOp = UsdGeomXformable(eye).AddTransformOp();
    GfMatrix4d scale(1), rotate(1), translate(1);
    scale.SetScale(GfVec3d(2.0, 1.5, 0.75));
    rotate.SetRotate(GfRotation(GfVec3d(0, 1, 0), 31));
    translate.SetTranslate(GfVec3d(20, -3, 5));
    GfMatrix4d assetWorld = scale * rotate * translate;
    GfMatrix4d eyeLocal(1);
    eyeLocal.SetRotate(GfRotation(GfVec3d(1, 0, 0), 13));
    eyeLocal.SetTranslateOnly(GfVec3d(1, 4, 2));
    CHECK(assetOp.Set(assetWorld));
    CHECK(eyeOp.Set(eyeLocal));
    UsdGeomPrimvarsAPI primvars(eye);
    CHECK(primvars.CreatePrimvar(projector, SdfValueTypeNames->Matrix4d,
                                UsdGeomTokens->constant).Set(GfMatrix4d(1)));
    CHECK(primvars.CreatePrimvar(dials, SdfValueTypeNames->Matrix4d,
                                UsdGeomTokens->constant).Set(GfMatrix4d(0.0)));

    auto stageIndex = UsdImagingStageSceneIndex::New();
    stageIndex->SetStage(stage);
    stageIndex->ApplyPendingUpdates();
    auto flattened = HdFlatteningSceneIndex::New(
        stageIndex, HdFlattenedDataSourceProviders());
    auto store = std::make_shared<RigExecSnapshotStore>();
    auto results = RigExecResultsSceneIndex::New(flattened, store);
    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));

    RigExecPublishedPrim published;
    GfMatrix4d localProjector(1);
    localProjector.SetRotate(GfRotation(GfVec3d(0, 0, 1), -17));
    localProjector.SetTranslateOnly(GfVec3d(0.25, 0.5, -0.75));
    const GfMatrix4d packed(0.375, 0.51, 0.73, 1.1,
                            2.2, 3.3, 4.4, 5.5,
                            6.6, 7.7, 8.8, 9.9,
                            10.1, 11.2, 12.3, 13.4);
    published.shaderMatrices[projector] = localProjector;
    published.shaderMatrices[dials] = packed;
    published.targetLocalShaderMatrices.insert(projector);
    RigExecPublishedPrim drivenAsset;
    bool publishAsset = false;
    bool resetEyeStack = false;
    auto generation = [&]() {
        auto snapshot = std::make_shared<RigExecImagingSnapshot>();
        snapshot->stage = stage;
        snapshot->prims[eyePath] = published;
        if (publishAsset) snapshot->prims[assetPath] = drivenAsset;
        if (resetEyeStack) snapshot->xformResetPaths.insert(eyePath);
        return snapshot;
    };
    auto clearNotices = [&]() {
        observer.added.clear();
        observer.removed.clear();
        observer.dirtied.clear();
        observer.dirtiedLocators.clear();
    };
    auto checkDrawn = [&](const GfMatrix4d &targetWorld) {
        const HdPrimvarsSchema drawn = HdPrimvarsSchema::GetFromParent(
            results->GetPrim(eyePath).dataSource);
        const VtValue value = drawn.GetPrimvar(projector).GetPrimvarValue()->GetValue(0);
        CHECK(value.IsHolding<GfMatrix4f>());
        if (value.IsHolding<GfMatrix4f>()) {
            CHECK(GfIsClose(GfMatrix4d(value.UncheckedGet<GfMatrix4f>()),
                            GfMatrix4d(GfMatrix4f(localProjector * targetWorld)),
                            1e-5));
        }
        CHECK(drawn.GetPrimvar(dials).GetPrimvarValue()->GetValue(0) ==
              VtValue(GfMatrix4f(packed)));
        CHECK(store->Get()->prims.at(eyePath).shaderMatrices.at(projector) ==
              localProjector);
    };
    auto checkSpatialDirty = [&]() {
        bool primvarsDirty = false, xformDirty = false;
        for (size_t i = 0; i < observer.dirtied.size(); ++i) {
            if (observer.dirtied[i] != eyePath) continue;
            primvarsDirty |= observer.dirtiedLocators[i].Intersects(
                HdPrimvarsSchema::GetDefaultLocator());
            xformDirty |= observer.dirtiedLocators[i].Intersects(
                HdXformSchema::GetDefaultLocator());
        }
        CHECK(primvarsDirty && xformDirty);
        CHECK(observer.added.empty() && observer.removed.empty());
    };
    std::string sourceBefore, sessionBefore;
    CHECK(stage->GetRootLayer()->ExportToString(&sourceBefore));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionBefore));
    results->NotifyGenerationPublished(store->Publish(generation()));
    checkDrawn(eyeLocal * assetWorld);
    std::string sourceInitialAfter, sessionInitialAfter;
    CHECK(stage->GetRootLayer()->ExportToString(&sourceInitialAfter));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionInitialAfter));
    CHECK(sourceInitialAfter == sourceBefore && sessionInitialAfter == sessionBefore);

    // An ordinary authored ancestor edit changes the world frame, while the
    // rig's local projector and its snapshot generation stay identical.
    clearNotices();
    rotate.SetRotate(GfRotation(GfVec3d(0, 0, 1), -29));
    translate.SetTranslate(GfVec3d(-8, 12, 4));
    assetWorld = scale * rotate * translate;
    CHECK(assetOp.Set(assetWorld));
    stageIndex->ApplyPendingUpdates();
    checkDrawn(eyeLocal * assetWorld);
    checkSpatialDirty();
    CHECK(store->Publish(generation()).empty());
    // Save the intentionally edited source, then verify all subsequent
    // derived publication and pulls leave both USD layers byte-identical.
    CHECK(stage->GetRootLayer()->ExportToString(&sourceBefore));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionBefore));

    published.hasXform = true;
    published.xformBase = eyeLocal;
    published.xform = eyeLocal;
    results->NotifyGenerationPublished(store->Publish(generation()));
    clearNotices();
    published.xform.SetTranslateOnly(GfVec3d(3, 6, -2));
    results->NotifyGenerationPublished(store->Publish(generation()));
    checkDrawn(published.xform * assetWorld);
    checkSpatialDirty();

    publishAsset = true;
    drivenAsset.hasXform = true;
    drivenAsset.xformBase = assetWorld;
    drivenAsset.xform = assetWorld;
    results->NotifyGenerationPublished(store->Publish(generation()));
    clearNotices();
    drivenAsset.xform.SetTranslateOnly(GfVec3d(9, -2, 7));
    results->NotifyGenerationPublished(store->Publish(generation()));
    checkDrawn(published.xform * drivenAsset.xform);
    checkSpatialDirty();

    std::string sourceAfter, sessionAfter;
    CHECK(stage->GetRootLayer()->ExportToString(&sourceAfter));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionAfter));
    CHECK(sourceAfter == sourceBefore && sessionAfter == sessionBefore);

    // A target that resets its authored stack must not gain the driven
    // ancestor's placement through the shader-only composition path.
    CHECK(UsdGeomXformable(eye).SetResetXformStack(true));
    stageIndex->ApplyPendingUpdates();
    resetEyeStack = true;
    CHECK(stage->GetRootLayer()->ExportToString(&sourceBefore));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionBefore));
    results->NotifyGenerationPublished(store->Publish(generation()));
    checkDrawn(published.xform);
    drivenAsset.xform.SetTranslateOnly(GfVec3d(100, 200, 300));
    results->NotifyGenerationPublished(store->Publish(generation()));
    checkDrawn(published.xform);
    CHECK(stage->GetRootLayer()->ExportToString(&sourceAfter));
    CHECK(stage->GetSessionLayer()->ExportToString(&sessionAfter));
    CHECK(sourceAfter == sourceBefore && sessionAfter == sessionBefore);
    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));

    // Preserve supported upstream and driven-world motion samples at the
    // same float shader boundary, without applying them to packed dials.
    auto motionInput = HdRetainedSceneIndex::New();
    GfMatrix4d worldA(1), worldB(1);
    worldA.SetTranslate(GfVec3d(2, 3, 4));
    worldB.SetTranslate(GfVec3d(-5, 6, 7));
    const TfToken xformName = HdXformSchemaTokens->xform;
    const HdDataSourceBaseHandle xformSource = HdXformSchema::Builder()
        .SetMatrix(_TestMotionMatrix::New(worldA, worldB))
        .SetResetXformStack(HdRetainedTypedSampledDataSource<bool>::New(true))
        .Build();
    motionInput->AddPrims({{eyePath, HdPrimTypeTokens->mesh,
        HdRetainedContainerDataSource::New(1, &xformName, &xformSource)}});
    auto motionStore = std::make_shared<RigExecSnapshotStore>();
    auto motionResults = RigExecResultsSceneIndex::New(motionInput, motionStore);
    auto motionSnapshot = std::make_shared<RigExecImagingSnapshot>();
    published.hasXform = false;
    motionSnapshot->prims[eyePath] = published;
    motionStore->Publish(motionSnapshot);
    auto checkMotion = [&](const GfMatrix4d &a, const GfMatrix4d &b) {
        const HdPrimvarsSchema drawn = HdPrimvarsSchema::GetFromParent(
            motionResults->GetPrim(eyePath).dataSource);
        const auto value = drawn.GetPrimvar(projector).GetPrimvarValue();
        std::vector<float> samples;
        CHECK(value->GetContributingSampleTimesForInterval(-11.5f, 11.5f, &samples));
        CHECK(samples == std::vector<float>({-11.5f, 11.5f}));
        CHECK(value->GetValue(-11.5f) == VtValue(GfMatrix4f(localProjector * a)));
        CHECK(value->GetValue(11.5f) == VtValue(GfMatrix4f(localProjector * b)));
        const auto packedValue = drawn.GetPrimvar(dials).GetPrimvarValue();
        CHECK(!packedValue->GetContributingSampleTimesForInterval(-11.5f, 11.5f, &samples));
        CHECK(packedValue->GetValue(-11.5f) == VtValue(GfMatrix4f(packed)));
        CHECK(packedValue->GetValue(11.5f) == VtValue(GfMatrix4f(packed)));
    };
    checkMotion(worldA, worldB);
    auto drivenMotion = std::make_shared<RigExecImagingSnapshot>(*motionStore->Get());
    auto &motionPrim = drivenMotion->prims[eyePath];
    motionPrim.hasXform = true;
    motionPrim.xformBase = worldB;
    motionPrim.xform = GfMatrix4d(1).SetTranslate(GfVec3d(11, 12, 13));
    motionPrim.sampleOffsets = {-11.5f, 11.5f};
    motionPrim.xformBaseSamples = {worldA, worldB};
    motionPrim.xformSamples = {
        GfMatrix4d(1).SetTranslate(GfVec3d(8, 9, 10)), motionPrim.xform};
    motionStore->Publish(drivenMotion);
    checkMotion(motionPrim.xformSamples[0], motionPrim.xformSamples[1]);
}

static void
TestCompleteMotionPublication(const std::string &examplesDir)
{
    const std::vector<float> offsets{-0.5f, 0.0f, 0.5f};
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/ArmShotAnim.usda");
    RigExecImagingBridge bridge(stage, SdfPath("/Shot/HeroArm/Rig"));
    CHECK(bridge.Compile());
    std::vector<RigExecImagingSnapshotConstPtr> reference;
    for (float offset : offsets) {
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1013.0 + offset)));
        reference.push_back(bridge.GetStore()->Get());
    }
    CHECK(bridge.EvaluateAndPublishSamples(UsdTimeCode(1013), offsets).ok);
    const auto motion = bridge.GetStore()->Get();
    const SdfPath body("/Shot/HeroArm/Geom/ArmBody");
    auto upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims({_MeshShell(body)});
    auto results = RigExecResultsSceneIndex::New(upstream, bridge.GetStore());
    const auto data = results->GetPrim(body).dataSource;
    const auto normals = HdPrimvarsSchema::GetFromParent(data)
        .GetPrimvar(HdTokens->normals).GetPrimvarValue();
    const auto extent = HdExtentSchema::GetFromParent(data);
    CHECK(normals && extent.GetMin() && extent.GetMax());
    if (normals && extent.GetMin() && extent.GetMax()) {
        std::vector<float> samples;
        CHECK(normals->GetContributingSampleTimesForInterval(-0.5f, 0.5f, &samples));
        CHECK(samples == offsets);
        for (size_t i = 0; i < offsets.size(); ++i) {
            const auto &expected = reference[i]->prims.at(body);
            CHECK(normals->GetValue(offsets[i]) == VtValue(expected.normals));
            CHECK(extent.GetMin()->GetTypedValue(offsets[i]) == expected.extentMin);
            CHECK(extent.GetMax()->GetTypedValue(offsets[i]) == expected.extentMax);
        }
    }

    // A driven Xform and its un-published descendant must compose against
    // upstream world transforms at each sample, not against the center frame.
    stage = UsdStage::Open(examplesDir + "/10_AimXformTurret.usda");
    RigExecImagingBridge turretBridge(stage, SdfPath("/TurretAsset/Rig"));
    CHECK(turretBridge.Compile());
    CHECK(turretBridge.EvaluateAndPublishSamples(
        UsdTimeCode(1012.5), {-11.5f, 11.5f}).ok);
    const SdfPath turret("/TurretAsset/Geom/Turret");
    const SdfPath child = turret.AppendChild(TfToken("Barrel"));
    const auto &published = turretBridge.GetStore()->Get()->prims.at(turret);
    CHECK(published.xformSamples.size() == 2);
    if (published.xformSamples.size() != 2) return;
    CHECK(published.xformSamples[0] != published.xformSamples[1]);
    const GfMatrix4d parentA = GfMatrix4d(1).SetTranslate(GfVec3d(100, 0, 0));
    const GfMatrix4d parentB = GfMatrix4d(1).SetTranslate(GfVec3d(200, 0, 0));
    const GfMatrix4d childLocal = GfMatrix4d(1).SetTranslate(GfVec3d(0, 0, 4));
    const auto shell = [](const SdfPath &path, const GfMatrix4d &a, const GfMatrix4d &b) {
        return HdRetainedSceneIndex::AddedPrimEntry{
            path, HdPrimTypeTokens->mesh,
            HdRetainedContainerDataSource::New(HdXformSchemaTokens->xform,
                HdXformSchema::Builder().SetMatrix(_TestMotionMatrix::New(a, b)).Build())};
    };
    upstream = HdRetainedSceneIndex::New();
    const GfMatrix4d oldA = published.xformBaseSamples[0] * parentA;
    const GfMatrix4d oldB = published.xformBaseSamples[1] * parentB;
    upstream->AddPrims({shell(turret, oldA, oldB),
                       shell(child, childLocal * oldA, childLocal * oldB)});
    results = RigExecResultsSceneIndex::New(upstream, turretBridge.GetStore());
    const auto transform = HdXformSchema::GetFromParent(results->GetPrim(child).dataSource)
        .GetMatrix();
    CHECK(transform);
    if (transform) {
        std::vector<float> samples;
        CHECK(transform->GetContributingSampleTimesForInterval(-11.5f, 11.5f, &samples));
        CHECK(samples == std::vector<float>({-11.5f, 11.5f}));
        for (size_t i = 0; i < 2; ++i) {
            const GfMatrix4d expected = childLocal * published.xformSamples[i] *
                (i ? parentB : parentA);
            const GfMatrix4d actual = transform->GetTypedValue(i ? 11.5f : -11.5f);
            for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
                CHECK(std::abs(actual[r][c] - expected[r][c]) < 1e-8);
        }
    }
}

static void
TestDrivenXformResetBoundaries(const std::string &examplesDir)
{
    const auto stage = UsdStage::Open(examplesDir + "/10_AimXformTurret.usda");
    const SdfPath turret("/TurretAsset/Geom/Turret");
    const SdfPath detached = turret.AppendChild(TfToken("Detached"));
    const SdfPath leaf = detached.AppendChild(TfToken("Leaf"));
    UsdGeomXformable boundary(stage->DefinePrim(detached, TfToken("Xform")));
    CHECK(boundary.AddTranslateOp().Set(GfVec3d(7, 8, 9)));
    CHECK(boundary.SetResetXformStack(true));
    UsdGeomXformable child(stage->DefinePrim(leaf, TfToken("Xform")));
    CHECK(child.AddTranslateOp().Set(GfVec3d(1, 2, 3)));
    RigExecImagingBridge bridge(stage, SdfPath("/TurretAsset/Rig"));
    CHECK(bridge.Compile());
    auto upstream = HdRetainedSceneIndex::New();
    auto results = RigExecResultsSceneIndex::New(upstream, bridge.GetStore());
    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));
    const auto matrix = [&](const SdfPath &path) {
        return HdXformSchema::GetFromParent(results->GetPrim(path).dataSource)
            .GetMatrix()->GetTypedValue(0);
    };
    const auto close = [](const GfMatrix4d &a, const GfMatrix4d &b) {
        for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
            if (std::abs(a[r][c] - b[r][c]) > 1e-8) return false;
        return true;
    };
    for (double time : {1001.0, 1024.0}) {
        const auto update = bridge.EvaluateAndPublishResult(UsdTimeCode(time));
        CHECK(update.ok);
        CHECK(bridge.GetStore()->Get()->xformResetPaths.count(detached));
        UsdGeomXformCache cache{UsdTimeCode(time)};
        HdRetainedSceneIndex::AddedPrimEntries entries;
        for (const auto &path : {turret, detached, leaf}) {
            entries.push_back({path, TfToken("xform"),
                HdRetainedContainerDataSource::New(HdXformSchemaTokens->xform,
                    HdXformSchema::Builder()
                        .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(
                            cache.GetLocalToWorldTransform(stage->GetPrimAtPath(path))))
                        // Like Hydra flattening, EVERY prim advertises reset.
                        .SetResetXformStack(HdRetainedTypedSampledDataSource<bool>::New(true))
                        .Build())});
        }
        upstream->AddPrims(entries);
        results->NotifyGenerationPublished(update.dirtied);
        CHECK(close(matrix(detached), cache.GetLocalToWorldTransform(boundary.GetPrim())));
        CHECK(close(matrix(leaf), cache.GetLocalToWorldTransform(child.GetPrim())));
    }
    const auto resetWorld = matrix(leaf);
    for (bool reset : {false, true}) {
        CHECK(boundary.SetResetXformStack(reset));
        const auto update = bridge.EvaluateAndPublishResult(UsdTimeCode(1024));
        CHECK(update.ok);
        CHECK(bool(bridge.GetStore()->Get()->xformResetPaths.count(detached)) == reset);
        observer.dirtied.clear();
        results->NotifyGenerationPublished(update.dirtied);
        CHECK(std::find(observer.dirtied.begin(), observer.dirtied.end(), leaf) != observer.dirtied.end());
        const auto &published = bridge.GetStore()->Get()->prims.at(turret);
        // The incoming world matrix is intentionally unchanged: the only
        // changing input to this pull is the captured reset boundary.
        UsdGeomXformCache cache{UsdTimeCode(1024)};
        const auto oldWorld = cache.GetLocalToWorldTransform(stage->GetPrimAtPath(turret));
        const auto expected = reset ? resetWorld
            : resetWorld * oldWorld.GetInverse() * published.xform *
                published.xformBase.GetInverse() * oldWorld;
        CHECK(close(matrix(leaf), expected));
        if (!reset) CHECK(!close(matrix(leaf), resetWorld));
    }
    // A boundary can itself be driven. Apply its own revision, then stop
    // before applying any driven namespace ancestor.
    auto snapshot = std::make_shared<RigExecImagingSnapshot>(*bridge.GetStore()->Get());
    auto &own = snapshot->prims[detached];
    own.hasXform = true;
    own.xformBase = GfMatrix4d(1).SetTranslate(GfVec3d(7, 8, 9));
    own.xform = GfMatrix4d(1).SetTranslate(GfVec3d(7, 12, 9));
    results->NotifyGenerationPublished(bridge.GetStore()->Publish(snapshot));
    CHECK(close(matrix(leaf), GfMatrix4d(1).SetTranslate(GfVec3d(8, 14, 12))));
    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// Failed replacement must preserve notice ordering as well as the published
// snapshot: exec invalidation must still run before the retained registry
// listener evaluates an edit at the same time.
static void
TestFailedActivationKeepsLiveNoticeOrdering()
{
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    registry.Deactivate();
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    const SdfPath rigPath("/Asset/Rig");
    stage->DefinePrim(rigPath, TfToken("RigExecRoot"));
    const SdfPath controlPath("/Asset/Rig/Control");
    const UsdPrim control =
        stage->DefinePrim(controlPath, TfToken("RigExecControl"));
    const UsdAttribute tx = control.GetAttribute(TfToken("avars:tx"));
    CHECK(tx && tx.Set(0.0));
    CHECK(control.GetAttribute(TfToken("avars:sx")).Set(3.0));
    const auto stageId = UsdUtilsStageCache::Get().Insert(stage);

    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rigPath, UsdTimeCode(1.0), &errors));
    const RigExecImagingSnapshotConstPtr before = registry.GetStore()->Get();
    CHECK(before);
    if (!before) {
        registry.Deactivate();
        return;
    }
    double fullFrame[16];
    std::fill(std::begin(fullFrame), std::end(fullFrame), -99.0);
    CHECK(RigExecImaging_GetControlFrameAssetSpace(stageId.ToLongInt(),
        controlPath.GetText(), 1.0, 0, fullFrame) == 1);
    CHECK(std::abs(fullFrame[0] - 3.0) < 1e-9); // full scale, not the rigid guide
    CHECK(RigExecImaging_GetControlFrameAssetSpace(stageId.ToLongInt(),
        controlPath.GetText(), 2.0, 0, fullFrame) == 0);
    const auto otherStage = UsdStage::CreateInMemory();
    const auto otherId = UsdUtilsStageCache::Get().Insert(otherStage);
    CHECK(RigExecImaging_GetControlFrameAssetSpace(otherId.ToLongInt(),
        controlPath.GetText(), 1.0, 0, fullFrame) == 0);
    UsdUtilsStageCache::Get().Erase(otherId);

    // An explicit invalid path reaches the rollback after its candidate
    // notice registration; an empty stage without a rig returns earlier.
    const UsdStageRefPtr replacement = UsdStage::CreateInMemory();
    CHECK(!registry.Activate(replacement, SdfPath("/MissingRig"),
                             UsdTimeCode(1.0), &errors));
    CHECK(registry.GetStore()->Get() == before);
    CHECK(registry.IsActive());
    for (double value : {3.0, 7.0}) {
        CHECK(tx.Set(value));
        const RigExecImagingSnapshotConstPtr snapshot =
            registry.GetStore()->Get();
        CHECK(snapshot && snapshot->generation > before->generation);
        if (!snapshot) continue;
        const auto it = snapshot->prims.find(controlPath);
        CHECK(it != snapshot->prims.end());
        if (it != snapshot->prims.end()) {
            CHECK(it->second.hasControlGuide);
            CHECK(std::abs(it->second.controlGuideFrame
                               .ExtractTranslation()[0] - value) < 1e-9);
        }
    }
    registry.Deactivate();
    UsdUtilsStageCache::Get().Erase(stageId);
}

static void
TestExternalReadEditsRepublish()
{
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    registry.Deactivate();
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Xform"));
    const SdfPath rigPath("/Asset/Rig");
    stage->DefinePrim(rigPath, TfToken("RigExecRoot"));
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Points"));
    CHECK(mesh.GetAttribute(TfToken("points"))
              .Set(VtVec3fArray{GfVec3f(0)}));
    const UsdPrim driver = stage->DefinePrim(
        SdfPath("/External/Driver"), TfToken("RigExecControl"));
    const UsdAttribute tx = driver.GetAttribute(TfToken("avars:tx"));
    CHECK(tx.Set(0.0));
    // Dependency collection must terminate even for an unrelated authored
    // relationship cycle in an external provider's subtree.
    CHECK(driver.CreateRelationship(TfToken("dependencyLoop"))
              .SetTargets({driver.GetPath()}));
    const UsdAttribute source = stage->DefinePrim(
        SdfPath("/Inputs/Source"), TfToken("Scope"))
        .CreateAttribute(TfToken("value"), SdfValueTypeNames->Double);
    CHECK(source.Set(8.0));
    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    CHECK(mover.GetRelationship(TfToken("rigExec:moves"))
              .SetTargets({SdfPath("/Asset/Geom.points")}));
    CHECK(mover.GetRelationship(TfToken("rigExec:transform"))
              .SetTargets({driver.GetPath()}));
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rigPath, UsdTimeCode(1), &errors));
    const auto expect = [&](float x) {
        const RigExecImagingSnapshotConstPtr snapshot = registry.GetStore()->Get();
        CHECK(snapshot);
        if (!snapshot) return;
        const auto it = snapshot->prims.find(mesh.GetPath());
        CHECK(it != snapshot->prims.end());
        if (it == snapshot->prims.end()) return;
        CHECK(it->second.points.size() == 1);
        if (it->second.points.size() == 1) {
            CHECK(std::abs(it->second.points[0][0] - x) < 1e-5f);
        }
    };
    expect(0.0f);
    CHECK(tx.Set(5.0));
    expect(5.0f);
    // Retarget an external provider's connection after activation. The new
    // transitive source must join the cached regions before its next edit.
    CHECK(tx.SetConnections({source.GetPath()}));
    expect(8.0f);
    CHECK(source.Set(9.0));
    expect(9.0f);
    CHECK(tx.ClearConnections());
    expect(5.0f);
    const RigExecImagingSnapshotConstPtr disconnected = registry.GetStore()->Get();
    CHECK(source.Set(11.0));
    CHECK(registry.GetStore()->Get() == disconnected);
    registry.Deactivate();
}

// A constraint-driven Xform reaches the snapshot as a transform, and it
// changes with time.
// The evaluator side is asserted in testRigExecArm; this is the imaging leg.
// hasXform was plumbed to HdXformSchema and had no writer at all, so the
// failure mode here is a published-once-then-frozen transform, which looks
// exactly like a static prop in the viewer.
// A rig with no joints publishes through the bridge like any other.
// The evaluator gate is only half of it: the bridge's guide fills all iterate
// joint frames, the binding epoch is built from the published prim set, and a
// rig whose entire output is one driven Xform exercises every one of those
// with an empty joint set.
// Disconnecting a constraint's target must un-drive the subtree.
// The user-visible report: remove the Xform from the aim constraint and the
// mesh parented under it stays exactly where the constraint had put it. The
// scene index handles a driven transform DISAPPEARING (see the
// _announcedDrivenXforms path), and the store diffs a prim that left the
// generation as structural -- so if the stale pose survives, the failure is
// upstream of both: nothing was published at all.
static void
TestDisconnectingAConstraintClearsTheDrivenXform(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/10_AimXformTurret.usda");
    CHECK(stage);
    if (!stage) return;

    const SdfPath turret("/TurretAsset/Geom/Turret");
    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(stage, SdfPath("/TurretAsset/Rig"), store);
    CHECK(bridge.Compile(nullptr));
    CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(1024)).ok);

    RigExecImagingSnapshotConstPtr snap = store->Get();
    CHECK(snap != nullptr);
    if (!snap) return;
    const auto driven = snap->prims.find(turret);
    CHECK(driven != snap->prims.end());
    if (driven == snap->prims.end()) return;
    CHECK(driven->second.hasXform);

    // Disconnect: exactly what removing the link in the node editor does.
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdPrim mover =
        stage->GetPrimAtPath(SdfPath("/TurretAsset/Rig/Movers/AimBarrel"));
    CHECK(mover);
    if (!mover) return;
    mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({});

    const RigExecImagingBridge::PublishResult after =
        bridge.EvaluateAndPublishResult(UsdTimeCode(1024));

    // Whatever the rig now thinks of itself, the previously driven prim must
    // not still be published as driven -- and the change must be announced,
    // or a cached renderer keeps the last delta it was given.
    snap = store->Get();
    const bool stillDriven =
        snap && snap->prims.count(turret) && snap->prims.at(turret).hasXform;
    if (stillDriven) {
        std::printf("  the turret is STILL published as driven after its "
                    "target was disconnected (publish ok=%d, %zu dirtied) -- "
                    "the stale pose survives\n",
                    int(after.ok), after.dirtied.size());
    }
    CHECK(!stillDriven);

    bool announced = false;
    for (const auto &entry : after.dirtied) {
        if (entry.path == turret) {
            announced = true;
        }
    }
    if (!announced) {
        std::printf("  nothing dirtied the turret; the mesh under it keeps "
                    "the transform it was last given\n");
    }
    CHECK(announced);
}

static void
TestJointFreeRigPublishes(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/rigexec_flat.usda");
    CHECK(stage);
    if (!stage) return;

    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(stage, SdfPath("/World/RigRoot"), store);
    std::vector<std::string> errors;
    const bool compiled = bridge.Compile(&errors);
    for (const std::string &e : errors) {
        std::printf("  compile error: %s\n", e.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;

    const SdfPath sphere("/World/Geom/Sphere");
    CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(0)).ok);
    RigExecImagingSnapshotConstPtr snap = store->Get();
    CHECK(snap != nullptr);
    if (!snap) return;
    const auto it = snap->prims.find(sphere);
    CHECK(it != snap->prims.end());
    if (it == snap->prims.end()) {
        std::printf("  joint-free rig published nothing for its target\n");
        return;
    }
    CHECK(it->second.hasXform);
    const GfMatrix4d first = it->second.xform;

    // The aim target swings a half turn across the shot, so a frozen
    // publication is distinguishable from a live one.
    CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(50)).ok);
    snap = store->Get();
    CHECK(snap != nullptr);
    if (!snap) return;
    const auto later = snap->prims.find(sphere);
    CHECK(later != snap->prims.end());
    if (later != snap->prims.end()) {
        CHECK(later->second.hasXform);
        CHECK(later->second.xform != first);
    }
}

// Property-domain results stay out of the imaging snapshot.
// They share movedProperties with the point chains, and the publish loop used
// to index snapshot->prims before deciding whether it had anything to store —
// so a float dial on a Scope MINTED an empty published prim for that Scope,
// which then entered the binding epoch and made the scene index resolve a
// prim RigExec publishes nothing for.
static void
TestPropertyResultsDoNotPublishPrims(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/09_PropertyMathMovers.usda");
    CHECK(stage);
    if (!stage) return;

    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(stage, SdfPath("/PropMathAsset/Rig"), store);
    CHECK(bridge.Compile(nullptr));
    CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode::Default()).ok);
    RigExecImagingSnapshotConstPtr snap = store->Get();
    CHECK(snap != nullptr);
    if (!snap) return;

    // The Scope owning the three dials owns no geometry and no guide.
    const SdfPath dials("/PropMathAsset/Rig/Channels/Dials");
    if (snap->prims.count(dials)) {
        std::printf("  the dial Scope was published as a prim; a property "
                    "result minted an empty entry\n");
    }
    CHECK(snap->prims.count(dials) == 0);

    // ...while the rig's actual geometry still is.
    CHECK(snap->prims.count(SdfPath("/PropMathAsset/Geom/Card")) == 1);
}

// Removing a rigExec:moves RELATIONSHIP re-evaluates, through the same
// notice path usdview uses.
// The bridge-level test above drives EvaluateAndPublishResult by hand. This
// one edits the stage and touches nothing else, so it covers the half that
// test cannot: that a relationship edit is noticed at all. Attribute edits
// were already covered (TestEditTriggeredReevaluation); a relationship edit
// arrives as a RESYNC rather than as changed-info, which is a different arm
// of _OnObjectsChanged.
static void
TestRelationshipEditTriggersReevaluation(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/10_AimXformTurret.usda");
    CHECK(stage);
    if (!stage) return;

    const SdfPath turret("/TurretAsset/Geom/Turret");
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    const bool activated = registry.Activate(
        stage, SdfPath("/TurretAsset/Rig"), UsdTimeCode(1024), &errors);
    for (const std::string &e : errors) {
        std::printf("  activate error: %s\n", e.c_str());
    }
    CHECK(activated);
    if (!activated) return;

    RigExecImagingSnapshotConstPtr snapshot = registry.GetStore()->Get();
    CHECK(snapshot != nullptr);
    CHECK(snapshot && snapshot->prims.count(turret) &&
          snapshot->prims.at(turret).hasXform);

    // The edit, and nothing else: no SetTime, no manual republish.
    stage->SetEditTarget(stage->GetSessionLayer());
    stage->GetPrimAtPath(SdfPath("/TurretAsset/Rig/Movers/AimBarrel"))
        .GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({});

    snapshot = registry.GetStore()->Get();
    const bool stillDriven = snapshot && snapshot->prims.count(turret) &&
                             snapshot->prims.at(turret).hasXform;
    if (stillDriven) {
        std::printf("  removing the moves relationship did not re-evaluate; "
                    "the turret is still driven\n");
    }
    CHECK(!stillDriven);

    // Reconnecting brings it back, so the clear is a state the rig recovers
    // from rather than a one-way trip.
    stage->GetPrimAtPath(SdfPath("/TurretAsset/Rig/Movers/AimBarrel"))
        .GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({turret});

    snapshot = registry.GetStore()->Get();
    const bool drivenAgain = snapshot && snapshot->prims.count(turret) &&
                             snapshot->prims.at(turret).hasXform;
    if (!drivenAgain) {
        std::printf("  reconnecting the moves relationship did not restore "
                    "the driven transform\n");
    }
    CHECK(drivenAgain);

    registry.Deactivate();
}

static void
TestConstraintDrivenXformPublishes(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/10_AimXformTurret.usda");
    CHECK(stage);
    if (!stage) return;

    auto store = std::make_shared<RigExecSnapshotStore>();
    RigExecImagingBridge bridge(stage, SdfPath("/TurretAsset/Rig"), store);
    std::vector<std::string> errors;
    const bool compiled = bridge.Compile(&errors);
    for (const std::string &e : errors) {
        std::printf("  compile error: %s\n", e.c_str());
    }
    CHECK(compiled);
    if (!compiled) return;

    const SdfPath turret("/TurretAsset/Geom/Turret");

    CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(1001)).ok);
    RigExecImagingSnapshotConstPtr snap = store->Get();
    CHECK(snap != nullptr);
    if (!snap) return;
    const auto it = snap->prims.find(turret);
    CHECK(it != snap->prims.end());
    if (it == snap->prims.end()) {
        std::printf("  turret absent from the published snapshot\n");
        return;
    }
    CHECK(it->second.hasXform);
    const GfMatrix4d first = it->second.xform;

    // ...and again at a time where the aim target has swung across.
    const RigExecImagingBridge::PublishResult second =
        bridge.EvaluateAndPublishResult(UsdTimeCode(1024));
    CHECK(second.ok);
    snap = store->Get();
    CHECK(snap);
    if (!snap) return;
    const auto it2 = snap->prims.find(turret);
    CHECK(it2 != snap->prims.end());
    if (it2 == snap->prims.end()) return;
    CHECK(it2->second.hasXform);
    if (it2->second.xform == first) {
        std::printf("  turret xform identical at 1001 and 1024 -- "
                    "published but frozen\n");
    }
    CHECK(it2->second.xform != first);

    // The change must be announced, or the viewer keeps the stale matrix.
    bool announced = false;
    for (const auto &entry : second.dirtied) {
        if (entry.path == turret &&
            (entry.changes & RigExecChangeXform)) {
            announced = true;
        }
    }
    if (!announced) {
        std::printf("  turret xform changed but was not dirtied\n");
    }
    CHECK(announced);

    // THE CHILD SIDE. Everything above proves the parent's transform is
    // published and dirtied; none of it proves a mesh parented underneath
    // actually moves, which is the entire point of driving an Xform. Compose
    // the flattening index the render index applies downstream and read the
    // child's flattened transform at both times.
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    const SdfPath barrel("/TurretAsset/Geom/Turret/Barrel");
    const SdfPath sight("/TurretAsset/Geom/Turret/Sight");
    const SdfPath scope("/TurretAsset/Geom/Turret/Sight/Scope");
    GfMatrix4d assetRootOffset(1.0);
    GfMatrix4d siblingLocal(1.0);
    GfMatrix4d grandchildLocal(1.0);
    {
        static const TfToken xformName("xform");
        auto xformDs = [](const GfMatrix4d &m) -> HdDataSourceBaseHandle {
            return HdXformSchema::Builder()
                .SetMatrix(
                    HdRetainedTypedSampledDataSource<GfMatrix4d>::New(m))
                .Build();
        };
        auto shell = [&](const SdfPath &p, const TfToken &type,
                         const HdDataSourceBaseHandle &xf)
            -> HdRetainedSceneIndex::AddedPrimEntry {
            return {p, type,
                    HdRetainedContainerDataSource::New(1, &xformName, &xf)};
        };

        // The asset root carries a NON-IDENTITY transform on purpose. With an
        // identity ancestor a local matrix and a world matrix are the same
        // thing, and publishing the wrong one is undetectable -- which is how
        // a space error survives a green test.
        assetRootOffset = GfMatrix4d(1.0).SetTranslate(GfVec3d(100, 0, 0));

        HdRetainedSceneIndex::AddedPrimEntries shells;
        // Ancestors, as UsdImaging emits them. Flattening composes through the
        // prim table, so a hierarchy with holes is not the hierarchy the
        // render index sees.
        shells.push_back(shell(SdfPath("/TurretAsset"), TfToken("xform"),
                               xformDs(assetRootOffset)));
        shells.push_back(shell(SdfPath("/TurretAsset/Geom"), TfToken("scope"),
                               xformDs(GfMatrix4d(1.0))));
        // The turret's own authored local transform, matching the example.
        shells.push_back(shell(turret, TfToken("xform"),
                               xformDs(GfMatrix4d(1.0).SetTranslate(
                                   GfVec3d(0, 2, 0)))));
        // The child carries a real identity xform, as UsdImaging emits for
        // any Xformable. An empty container has nothing to flatten into and
        // would make this assertion measure nothing.
        shells.push_back(shell(barrel, HdPrimTypeTokens->mesh,
                               xformDs(GfMatrix4d(1.0))));
        // A sibling with a NON-identity local, and a grandchild beneath it.
        // Identity-local children make "just copy the parent's matrix onto
        // every descendant" indistinguishable from composing correctly, and
        // a single level makes a broken recursion indistinguishable from a
        // working one. These two make both mistakes fail.
        siblingLocal = GfMatrix4d(1.0).SetTranslate(GfVec3d(0, 0, 7));
        grandchildLocal = GfMatrix4d(1.0).SetTranslate(GfVec3d(3, 0, 0));
        shells.push_back(shell(sight, HdPrimTypeTokens->mesh,
                               xformDs(siblingLocal)));
        // A LOCAL, like every other entry here -- the flattening index below
        // is what composes it with the chain above.
        shells.push_back(shell(scope, HdPrimTypeTokens->mesh,
                               xformDs(grandchildLocal)));
        upstream->AddPrims(shells);
    }
    // ORDER MATTERS, and it is the opposite of what one would choose.
    // Flattening comes FIRST because that is where UsdImaging puts it: the
    // chain's only HdFlatteningSceneIndex is built inside
    // UsdImagingNiPrototypePropagatingSceneIndex, which
    // UsdImagingCreateSceneIndices constructs before it appends plugin scene
    // indices -- and UsdImagingSceneIndexPlugin::AppendSceneIndex is the only
    // hook RigExec has. So RigExec sees world-space transforms and is solely
    // responsible for carrying a driven Xform's subtree along with it.
    // (Null inputArgs would mean ZERO flattened data source providers -- an
    // index that flattens nothing, where every child reads back identity
    // forever, which looks exactly like the bug this test exists to catch.)
    auto flattened = HdFlatteningSceneIndex::New(
        upstream, HdFlattenedDataSourceProviders());
    auto results = RigExecResultsSceneIndex::New(flattened, store);

    auto flatOf = [&](const SdfPath &p) -> GfMatrix4d {
        const HdSceneIndexPrim prim = results->GetPrim(p);
        if (!prim.dataSource) return GfMatrix4d(1.0);
        HdXformSchema xf = HdXformSchema::GetFromParent(prim.dataSource);
        if (!xf || !xf.GetMatrix()) return GfMatrix4d(1.0);
        return xf.GetMatrix()->GetTypedValue(0.0);
    };
    auto flatMatrix = [&](const SdfPath &p) -> GfMatrix4d {
        const HdSceneIndexPrim prim = results->GetPrim(p);
        if (!prim.dataSource) return GfMatrix4d(1.0);
        HdXformSchema xf = HdXformSchema::GetFromParent(prim.dataSource);
        if (!xf || !xf.GetMatrix()) return GfMatrix4d(1.0);
        return xf.GetMatrix()->GetTypedValue(0.0);
    };
    auto childMatrix = [&]() -> GfMatrix4d {
        const HdSceneIndexPrim prim = results->GetPrim(barrel);
        if (!prim.dataSource) return GfMatrix4d(1.0);
        HdXformSchema xf = HdXformSchema::GetFromParent(prim.dataSource);
        if (!xf || !xf.GetMatrix()) return GfMatrix4d(1.0);
        return xf.GetMatrix()->GetTypedValue(0.0);
    };

    // Mirror RigExecImagingRegistry::_Broadcast: publishing to the store is
    // only half of it -- the results index must be told, or every downstream
    // cache (flattening included) keeps serving the previous generation.
    auto publish = [&](double frame) {
        const RigExecImagingBridge::PublishResult r =
            bridge.EvaluateAndPublishResult(UsdTimeCode(frame));
        if (r.ok) {
            results->NotifyGenerationPublished(r.dirtied);
        }
    };
    // The upstream (UNDRIVEN) world transform, for the revert check below.
    // Read through `flattened`, not `results` -- the store already holds a
    // driven snapshot from the pulls above, so `results` would hand back the
    // driven value and the check would assert nothing.
    const GfMatrix4d turretRestWorld = [&] {
        const HdSceneIndexPrim p = flattened->GetPrim(barrel);
        HdXformSchema xf = HdXformSchema::GetFromParent(p.dataSource);
        return xf && xf.GetMatrix() ? xf.GetMatrix()->GetTypedValue(0.0)
                                    : GfMatrix4d(1.0);
    }();
    publish(1001);
    const GfMatrix4d parentAt1001 = flatMatrix(turret);
    const GfMatrix4d childAt1001 = childMatrix();
    publish(1024);
    const GfMatrix4d parentAt1024 = flatMatrix(turret);
    const GfMatrix4d childAt1024 = childMatrix();
    std::printf("  flattened parent changed: %s | child changed: %s\n",
                parentAt1001 != parentAt1024 ? "yes" : "no",
                childAt1001 != childAt1024 ? "yes" : "no");

    if (childAt1001 == childAt1024) {
        std::printf("  parented mesh flattened xform is IDENTICAL at 1001 and "
                    "1024 -- the parent moves, the child does not\n");
    }
    CHECK(childAt1001 != childAt1024);

    // The child's own local transform is identity, so its world transform
    // must equal its parent's exactly -- at both times. This is what catches
    // a local matrix published where a world matrix belongs: with a
    // non-identity asset root the two differ by that offset.
    auto close = [](const GfMatrix4d &a, const GfMatrix4d &b) {
        return GfIsClose(a, b, 1e-9);
    };
    if (!close(childAt1001, parentAt1001) ||
        !close(childAt1024, parentAt1024)) {
        std::printf("  child world != parent world (child local is identity)"
                    " -- transform published in the wrong space\n");
    }
    CHECK(close(childAt1001, parentAt1001));
    CHECK(close(childAt1024, parentAt1024));

    // The non-identity sibling and the grandchild beneath it must land on
    // exactly local * parentWorld -- the composition rule itself, not merely
    // "something changed". Copying the parent matrix down, or failing to
    // recurse past the first level, breaks these and nothing else.
    const GfMatrix4d sightAt1024 = flatOf(sight);
    const GfMatrix4d scopeAt1024 = flatOf(scope);
    if (!close(sightAt1024, siblingLocal * parentAt1024)) {
        std::printf("  sibling with non-identity local is not "
                    "local * parentWorld\n");
    }
    CHECK(close(sightAt1024, siblingLocal * parentAt1024));
    if (!close(scopeAt1024, grandchildLocal * siblingLocal * parentAt1024)) {
        std::printf("  grandchild is not composed through two levels\n");
    }
    CHECK(close(scopeAt1024, grandchildLocal * siblingLocal * parentAt1024));

    // ...and the driven prim must still sit under its asset root. Overwriting
    // the flattened matrix with the raw local one would silently drop this.
    const GfVec3d rootTranslate = assetRootOffset.ExtractTranslation();
    const GfVec3d parentTranslate = parentAt1024.ExtractTranslation();
    if (std::abs(parentTranslate[0] - rootTranslate[0]) > 1.0) {
        std::printf("  driven xform lost its asset-root offset (%g vs %g)\n",
                    parentTranslate[0], rootTranslate[0]);
    }
    CHECK(std::abs(parentTranslate[0] - rootTranslate[0]) <= 1.0);

    // INVALIDATION, which the pulls above cannot prove. Every read so far
    // called GetPrim directly, and GetPrim recomputes -- so the child would
    // resolve correctly even if nothing ever announced that it changed. A
    // render index does not poll; it redraws what it was told is dirty.
    // Nothing downstream of us flattens, so nothing downstream will infer
    // that a mesh moved because its parent Xform did.
    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));
    publish(1001);
    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));

    const HdDataSourceLocator xformMatrix(HdXformSchemaTokens->xform,
                                          HdXformSchemaTokens->matrix);
    // The descendant's set must be the EXPANDED one, not the bare leaf:
    // GetPrim hands out a freshly built retained container each generation,
    // so a consumer caching container handles needs the rebuilt chain
    // invalidated too. ComputeDirtyLocators expresses that by inserting a
    // container sentinel per level, and requiring the top-level one is what
    // distinguishes the expansion from a bare xform/matrix. (A universal set
    // also satisfies this, which is correct -- it is merely less precise.)
    const HdDataSourceLocator containerSentinel(
        HdDataSourceLocatorSentinelTokens->container);
    bool childDirtied = false;
    bool childDirtiedExpanded = false;
    for (size_t i = 0; i < observer.dirtied.size(); ++i) {
        if (observer.dirtied[i] == barrel &&
            observer.dirtiedLocators[i].Intersects(xformMatrix)) {
            childDirtied = true;
            if (observer.dirtiedLocators[i].Contains(containerSentinel)) {
                childDirtiedExpanded = true;
            }
        }
    }
    if (!childDirtied) {
        std::printf("  parented mesh was never dirtied -- it resolves "
                    "correctly on pull but a viewer is never told to pull\n");
    }
    CHECK(childDirtied);
    if (!childDirtiedExpanded) {
        std::printf("  descendant dirty set is the bare leaf, not the "
                    "rebuilt-container expansion\n");
    }
    CHECK(childDirtiedExpanded);

    // The published matrix is fully composed, so it must say so. A false
    // resetXformStack invites a later flattener to compose the parent in a
    // second time and makes consumers that key off the flag skip it.
    {
        const HdSceneIndexPrim p = results->GetPrim(turret);
        HdXformSchema xf = HdXformSchema::GetFromParent(p.dataSource);
        const bool resets = xf && xf.GetResetXformStack() &&
                            xf.GetResetXformStack()->GetTypedValue(0.0);
        if (!resets) {
            std::printf("  driven xform does not declare resetXformStack -- "
                        "a composed matrix advertised as composable\n");
        }
        CHECK(resets);
    }

    // STRUCTURAL transitions, which are a different code path entirely.
    // Every hasXform transition -- first publication, a provider gaining or
    // losing its constraint, recompilation, Deactivate() -- is reported as
    // RigExecChangeStructural, never as RigExecChangeXform. The structural
    // branch used to dirty only the provider and move on, so a driven
    // subtree went stale on exactly the transitions that matter most. Drive
    // the removal case: publish an empty snapshot and require the
    // descendants to be told.
    _RecordingObserver onClear;
    results->AddObserver(HdSceneIndexObserverPtr(&onClear));
    results->NotifyGenerationPublished(store->Publish(nullptr));
    results->RemoveObserver(HdSceneIndexObserverPtr(&onClear));

    bool childDirtiedOnClear = false;
    for (const SdfPath &p : onClear.dirtied) {
        if (p == barrel) childDirtiedOnClear = true;
    }
    if (!childDirtiedOnClear) {
        std::printf("  clearing the driven xform left its subtree "
                    "un-dirtied -- descendants keep the stale delta\n");
    }
    CHECK(childDirtiedOnClear);

    // ...and the value must actually revert to the upstream flattened one.
    CHECK(close(flatOf(barrel), turretRestWorld));

    // THE LATE CONSUMER, which everything above quietly avoids testing.
    // Every assertion so far ran on an index that watched the driven
    // transform appear, so its history was populated as a side effect. A
    // scene index chain built AFTER activation -- which is the normal case,
    // since usdview builds its viewport when it likes and the plugin
    // activates on stage open -- sees an already-driven store and no history
    // at all. Unless the constructor seeds it, the first loss finds
    // "wasDriven == false" and silently leaves the subtree stale.
    {
        publish(1024);  // store holds a driven generation again
        auto lateResults = RigExecResultsSceneIndex::New(flattened, store);
        // Pull once, as a real consumer would, so there is cached state to
        // go stale.
        const HdSceneIndexPrim warmed = lateResults->GetPrim(barrel);
        CHECK(warmed.dataSource != nullptr);

        _RecordingObserver lateObserver;
        lateResults->AddObserver(HdSceneIndexObserverPtr(&lateObserver));
        lateResults->NotifyGenerationPublished(store->Publish(nullptr));
        lateResults->RemoveObserver(HdSceneIndexObserverPtr(&lateObserver));

        bool lateChildDirtied = false;
        for (const SdfPath &p : lateObserver.dirtied) {
            if (p == barrel) lateChildDirtied = true;
        }
        if (!lateChildDirtied) {
            std::printf("  late consumer never told its driven subtree was "
                        "cleared -- history not seeded at construction\n");
        }
        CHECK(lateChildDirtied);
    }
}

// Singular and ill-conditioned transforms in the driven-Xform resolution.
// GfMatrix4d::GetInverse() does not report failure -- it returns
// FLT_MAX * identity -- so an unchecked inverse turns a zero-scale prim
// (an ordinary way to hide geometry) into components around 1e77. This
// exercises the guard, and the two behaviours that are easy to get wrong:
// a determinant threshold is NOT a conditioning test, and an unusable
// ancestor must be SKIPPED rather than abandoning the whole walk.
static void
TestDrivenXformConditioning()
{
    std::printf("TestDrivenXformConditioning\n");

    const SdfPath outer("/A");
    const SdfPath inner("/A/B");
    const SdfPath leaf("/A/B/C");

    static const TfToken xformName("xform");
    auto xformDs = [](const GfMatrix4d &m) -> HdDataSourceBaseHandle {
        return HdXformSchema::Builder()
            .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(m))
            .Build();
    };
    auto shell = [&](const SdfPath &p, const HdDataSourceBaseHandle &xf) {
        return HdRetainedSceneIndex::AddedPrimEntry{
            p, HdPrimTypeTokens->mesh,
            HdRetainedContainerDataSource::New(1, &xformName, &xf)};
    };

    // Publishes a driven transform for `path`, revising `base` to `revised`.
    auto publishDriven = [&](const std::shared_ptr<RigExecSnapshotStore> &store,
                             const std::vector<std::tuple<SdfPath, GfMatrix4d,
                                                          GfMatrix4d>> &driven) {
        auto snapshot = std::make_shared<RigExecImagingSnapshot>();
        for (const auto &[path, base, revised] : driven) {
            RigExecPublishedPrim &p = snapshot->prims[path];
            p.hasXform = true;
            p.xformBase = base;
            p.xform = revised;
        }
        store->Publish(snapshot);
    };

    // --- 1. A uniform scale of 1e-5 has determinant 1e-15 but condition
    // number 1. It is perfectly invertible and MUST still resolve; a
    // determinant threshold rejects it, which is exactly why the guard
    // measures ||A||*||A^-1|| instead. (1e-4 would sit ON a 1e-12
    // determinant threshold and decide nothing.)
    {
        auto store = std::make_shared<RigExecSnapshotStore>();
        HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
        GfMatrix4d tiny(1.0);
        tiny.SetScale(1e-5);
        upstream->AddPrims({shell(outer, xformDs(tiny))});
        auto results = RigExecResultsSceneIndex::New(upstream, store);

        GfMatrix4d revised(1.0);
        revised.SetScale(1e-5);
        revised.SetTranslateOnly(GfVec3d(5, 0, 0));
        publishDriven(store, {{outer, tiny, revised}});

        HdXformSchema xf = HdXformSchema::GetFromParent(
            results->GetPrim(outer).dataSource);
        const GfMatrix4d out = xf && xf.GetMatrix()
            ? xf.GetMatrix()->GetTypedValue(0.0) : GfMatrix4d(1.0);
        const bool moved = out.ExtractTranslation()[0] > 1.0;
        if (!moved) {
            std::printf("  a uniform 1e-5 scale was rejected -- the guard is "
                        "testing determinant, not conditioning\n");
        }
        CHECK(moved);
    }

    // --- 1b. Nonsingular but ILL-CONDITIONED must be rejected. diag(1e7,
    // 1e-7, 1) has determinant 1, so every determinant-based test accepts
    // it, but its condition number is ~1e14 and an inverse is numerically
    // worthless. Without this case, "det != 0" passes the whole suite.
    {
        auto store = std::make_shared<RigExecSnapshotStore>();
        HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
        GfMatrix4d skewed(1.0);
        skewed.SetScale(GfVec3d(1e7, 1e-7, 1));
        upstream->AddPrims({shell(outer, xformDs(skewed))});
        auto results = RigExecResultsSceneIndex::New(upstream, store);

        GfMatrix4d revised = skewed;
        revised.SetTranslateOnly(GfVec3d(5, 0, 0));
        publishDriven(store, {{outer, skewed, revised}});

        HdXformSchema xf = HdXformSchema::GetFromParent(
            results->GetPrim(outer).dataSource);
        const GfMatrix4d out = xf && xf.GetMatrix()
            ? xf.GetMatrix()->GetTypedValue(0.0) : GfMatrix4d(1.0);
        if (out != skewed) {
            std::printf("  an ill-conditioned base (det 1, cond ~1e14) was "
                        "accepted -- the guard is testing determinant\n");
        }
        CHECK(out == skewed);
    }

    // --- 2. A genuinely singular base must publish NOTHING, not 1e77.
    {
        auto store = std::make_shared<RigExecSnapshotStore>();
        HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
        GfMatrix4d flat(1.0);
        flat.SetScale(GfVec3d(0, 1, 1));
        upstream->AddPrims({shell(outer, xformDs(flat))});
        auto results = RigExecResultsSceneIndex::New(upstream, store);

        publishDriven(store, {{outer, flat, flat}});

        HdXformSchema xf = HdXformSchema::GetFromParent(
            results->GetPrim(outer).dataSource);
        const GfMatrix4d out = xf && xf.GetMatrix()
            ? xf.GetMatrix()->GetTypedValue(0.0) : GfMatrix4d(1.0);
        bool finite = true;
        double biggest = 0.0;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                finite = finite && std::isfinite(out[r][c]);
                biggest = std::max(biggest, std::abs(out[r][c]));
            }
        }
        if (!finite || biggest > 1e6) {
            std::printf("  singular base produced a matrix with components up "
                        "to %g -- unchecked GetInverse()\n", biggest);
        }
        CHECK(finite);
        CHECK(biggest <= 1e6);
    }

    // --- 2b. A TINY singular base: the sentinel case.
    // GetInverse() signals failure with SetScale(FLT_MAX), which is finite.
    // When the source is tiny the two magnitudes multiply to something
    // small -- diag(0, 1e-30, 1e-30) gives a proxy of only ~3.4e8, under the
    // conditioning bound -- so neither a finite check nor the proxy rejects
    // it, and the corruption is small enough to look plausible. Only
    // verifying that the inverse actually inverts catches this.
    {
        auto store = std::make_shared<RigExecSnapshotStore>();
        HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
        GfMatrix4d tinyFlat(1.0);
        tinyFlat.SetScale(GfVec3d(0, 1e-30, 1e-30));
        upstream->AddPrims({shell(outer, xformDs(tinyFlat))});
        auto results = RigExecResultsSceneIndex::New(upstream, store);

        publishDriven(store, {{outer, tinyFlat, tinyFlat}});

        HdXformSchema xf = HdXformSchema::GetFromParent(
            results->GetPrim(outer).dataSource);
        const GfMatrix4d out = xf && xf.GetMatrix()
            ? xf.GetMatrix()->GetTypedValue(0.0) : GfMatrix4d(1.0);
        if (out != tinyFlat) {
            std::printf("  a tiny singular base was accepted -- the FLT_MAX "
                        "failure sentinel slipped under the magnitude "
                        "bound\n");
        }
        CHECK(out == tinyFlat);
    }

    // --- 3. A singular INNER driven prim must not detach its subtree from a
    // valid OUTER one. Bailing out of the whole ancestor walk on the first
    // bad level leaves this leaf behind while its cousins move.
    {
        auto store = std::make_shared<RigExecSnapshotStore>();
        HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
        GfMatrix4d flat(1.0);
        flat.SetScale(GfVec3d(0, 1, 1));
        upstream->AddPrims({shell(outer, xformDs(GfMatrix4d(1.0))),
                            shell(inner, xformDs(flat)),
                            shell(leaf, xformDs(flat))});
        auto results = RigExecResultsSceneIndex::New(upstream, store);

        GfMatrix4d outerRevised(1.0);
        outerRevised.SetTranslateOnly(GfVec3d(9, 0, 0));
        publishDriven(store, {{outer, GfMatrix4d(1.0), outerRevised},
                              {inner, flat, flat}});

        HdXformSchema xf = HdXformSchema::GetFromParent(
            results->GetPrim(leaf).dataSource);
        const GfMatrix4d out = xf && xf.GetMatrix()
            ? xf.GetMatrix()->GetTypedValue(0.0) : GfMatrix4d(1.0);
        const bool followedOuter =
            std::abs(out.ExtractTranslation()[0] - 9.0) < 1e-6;
        if (!followedOuter) {
            std::printf("  a singular inner ancestor detached the leaf from a "
                        "valid outer one (x=%g, expected 9)\n",
                        out.ExtractTranslation()[0]);
        }
        CHECK(followedOuter);
    }
}

// The guide-bounds C export (registry.h), which is what gives usdview a
// box to frame.
// Nothing a rig draws is reachable by UsdGeomBBoxCache: the guides are
// synthesized inside the imaging chain and never authored, and the RigExec
// prim types are not UsdGeomImageable. So the published snapshot is the
// only place the drawn extent exists, and these bounds are asserted
// EXACTLY -- a framing box that is quietly half the right size still frames
// something, which is how a wrong one survives being looked at.
static void
TestGuideBoundsExport()
{
    // Publishing straight into the process-global store the export reads.
    // Deactivating first so a bridge left over from an earlier test cannot
    // republish over the fixture and make this order-dependent.
    RigExecImaging_Deactivate();
    const std::shared_ptr<RigExecSnapshotStore> &store =
        RigExecImagingRegistry::GetInstance().GetStore();

    const SdfPath joint("/R/Joint");
    const SdfPath control("/R/Ctrl");
    const SdfPath turned("/R/Turned");
    const SdfPath silent("/R/Silent");

    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    {
        // Sphere r=2 at (10,0,0) plus a cone reaching 4 along +X: the box
        // spans the sphere at the origin and the sphere at the tip.
        RigExecPublishedPrim &published = snapshot->prims[joint];
        published.hasGuides = true;
        GfMatrix4d frame(1.0);
        frame.SetTranslateOnly(GfVec3d(10, 0, 0));
        published.guideFrames.push_back(frame);
        published.guideLengths.push_back(4.0);
        published.guideRadii.push_back(2.0);
    }
    {
        // Unit shape scaled per axis, then placed by the frame.
        RigExecPublishedPrim &published = snapshot->prims[control];
        published.hasControlGuide = true;
        GfMatrix4d frame(1.0);
        frame.SetTranslateOnly(GfVec3d(0, 5, 0));
        published.controlGuideFrame = frame;
        published.controlGuideScale = GfVec3d(2, 3, 4);
        published.controlGuideShape = TfToken("circle");
        published.controlGuideDrawMode = TfToken("wire");
        // Hairline, so the box is the shape's own and the arithmetic below
        // stays about placement rather than about wire width.
        published.controlGuideWireWidth = 0.0;
    }
    {
        // ...and rotated, which is what proves the frame is applied rather
        // than just its translation: a quarter turn about Z swaps the X and
        // Y half-extents of the aligned box.
        RigExecPublishedPrim &published = snapshot->prims[turned];
        published.hasControlGuide = true;
        published.controlGuideFrame =
            GfMatrix4d(1.0).SetRotate(GfRotation(GfVec3d(0, 0, 1), 90.0));
        published.controlGuideScale = GfVec3d(2, 3, 4);
        published.controlGuideShape = TfToken("box");
        published.controlGuideDrawMode = TfToken("geometry");
    }
    {
        // Published, but draws nothing.
        RigExecPublishedPrim &published = snapshot->prims[silent];
        published.hasPoints = true;
        published.points = VtVec3fArray{GfVec3f(0, 0, 0)};
    }
    store->Publish(snapshot);

    double b[6] = {0, 0, 0, 0, 0, 0};
    auto bounds = [&b](const SdfPath &path) {
        return RigExecImaging_GetGuideBoundsAssetSpace(
            path.GetString().c_str(), b);
    };
    auto matches = [&b](const GfVec3d &min, const GfVec3d &max) {
        for (size_t i = 0; i < 3; ++i) {
            if (std::abs(b[i] - min[i]) > 1e-9 ||
                std::abs(b[i + 3] - max[i]) > 1e-9) {
                std::printf("  bounds (%g %g %g)..(%g %g %g), expected "
                            "(%g %g %g)..(%g %g %g)\n", b[0], b[1], b[2],
                            b[3], b[4], b[5], min[0], min[1], min[2],
                            max[0], max[1], max[2]);
                return false;
            }
        }
        return true;
    };

    CHECK(bounds(joint) == 1);
    CHECK(matches(GfVec3d(8, -2, -2), GfVec3d(16, 2, 2)));

    CHECK(bounds(control) == 1);
    CHECK(matches(GfVec3d(-2, 2, -4), GfVec3d(2, 8, 4)));

    CHECK(bounds(turned) == 1);
    CHECK(matches(GfVec3d(-3, -2, -4), GfVec3d(3, 2, 4)));

    // A prim that draws nothing, an unpublished prim, and a malformed path
    // all decline rather than reporting a degenerate box at the origin --
    // which would frame the camera on empty space.
    CHECK(bounds(silent) == 0);
    CHECK(bounds(SdfPath("/R/Absent")) == 0);
    CHECK(RigExecImaging_GetGuideBoundsAssetSpace("not a path", b) == 0);
    CHECK(RigExecImaging_GetGuideBoundsAssetSpace(nullptr, b) == 0);

    // The whole-rig union, which is what framing the RigExecRoot uses.
    CHECK(RigExecImaging_GetAllGuideBoundsAssetSpace(b) == 1);
    CHECK(matches(GfVec3d(-3, -2, -4), GfVec3d(16, 8, 4)));

    // Imported shapes have arbitrary local vertices, including asymmetric
    // shapes mirrored by the full evaluated control matrix.
    RigExecPublishedPrim &custom = snapshot->prims[SdfPath("/R/Custom")];
    custom.hasControlGuide = true;
    custom.controlGuideShape = TfToken("custom");
    custom.controlGuideDrawMode = TfToken("wire");
    custom.controlGuideWireWidth = 0.0;
    custom.controlGuidePoints = VtVec3fArray{{2,3,-1},{4,5,1}};
    custom.controlGuideCounts = VtIntArray{2};
    custom.controlGuideScale = GfVec3d(2,1,1);
    custom.controlGuideFrame = GfMatrix4d(1.0).SetScale(GfVec3d(-1,1,1));
    custom.controlGuideFrame.SetTranslateOnly(GfVec3d(10,0,0));
    store->Publish(snapshot);
    CHECK(bounds(SdfPath("/R/Custom")) == 1);
    CHECK(matches(GfVec3d(2,3,-1), GfVec3d(6,5,1)));
    custom.guideOpacity = 0.0f;
    store->Publish(snapshot);
    CHECK(bounds(SdfPath("/R/Custom")) == 0);
    custom.guideOpacity = 0.5f;
    custom.controlGuideCounts = VtIntArray{3};
    store->Publish(snapshot);
    CHECK(bounds(SdfPath("/R/Custom")) == 0);

    // An empty generation draws nothing at all.
    store->Publish(nullptr);
    CHECK(RigExecImaging_GetAllGuideBoundsAssetSpace(b) == 0);
    CHECK(bounds(joint) == 0);
}

static void
TestControlGuideSource()
{
    const auto stage=UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Rig"),TfToken("RigExecRoot"));
    const auto control=stage->DefinePrim(SdfPath("/Rig/Control"),TfToken("RigExecControl"));
    const auto source=stage->DefinePrim(SdfPath("/Rig/Source"),TfToken("RigExecJoint"));
    CHECK(control.GetAttribute(TfToken("guide:shape")).Set(TfToken("custom")));
    CHECK(control.GetAttribute(TfToken("guide:points")).Set(VtVec3fArray{{0,0,0},{1,0,0}}));
    CHECK(control.GetAttribute(TfToken("guide:curveVertexCounts")).Set(VtIntArray{2}));
    CHECK(control.GetRelationship(TfToken("guide:source")).SetTargets({source.GetPath()}));
    RigExecImagingBridge bridge(stage,SdfPath("/Rig"));
    std::vector<std::string> errors; CHECK(bridge.Compile(&errors));
    for(double x:{2.0,5.0,-1.0}) {
        CHECK(source.GetAttribute(TfToken("avars:tx")).Set(x));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
        const auto snapshot=bridge.GetStore()->Get();
        const auto &published=snapshot->prims.at(control.GetPath());
        CHECK(published.hasControlGuide);
        CHECK(published.controlGuideFrame.ExtractTranslation()==GfVec3d(x,0,0));
        CHECK(published.controlFrame.ExtractTranslation()==GfVec3d(0));
        // The guide stays under the editable owner in the scene index.
        CHECK(snapshot->prims.count(control.GetPath())==1);
    }
}

static void
TestCustomControlRestExtent()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Rig"), TfToken("RigExecRoot"));
    const UsdPrim control = stage->DefinePrim(SdfPath("/Rig/Ctrl"), TfToken("RigExecControl"));
    CHECK(control.GetAttribute(TfToken("guide:shape")).Set(TfToken("custom")));
    CHECK(control.GetAttribute(TfToken("guide:points")).Set(VtVec3fArray{{2,3,-1},{4,5,1}}));
    CHECK(control.GetAttribute(TfToken("guide:curveVertexCounts")).Set(VtIntArray{2}));
    CHECK(control.GetAttribute(TfToken("guide:wireWidth")).Set(0.0));
    GfMatrix4d rest(1.0);
    rest.SetScale(GfVec3d(-2,1,1));
    rest.SetTranslateOnly(GfVec3d(10,0,0));
    CHECK(control.GetAttribute(TfToken("rest:space")).Set(rest));
    VtVec3fArray extent;
    const UsdGeomBoundable boundable(control);
    CHECK(UsdGeomBoundable::ComputeExtentFromPlugins(boundable, UsdTimeCode::Default(), &extent));
    // Rest spaces are rigidized by evaluation, preserving the reflection.
    CHECK(extent == VtVec3fArray({GfVec3f(6,3,-1),GfVec3f(8,5,1)}));
    RigExecImagingBridge bridge(stage, SdfPath("/Rig"));
    std::vector<std::string> errors;
    CHECK(bridge.Compile(&errors));
    CHECK(bridge.EvaluateAndPublish(UsdTimeCode::Default()));
    const auto published = bridge.GetStore()->Get();
    CHECK(published->prims.at(control.GetPath()).controlGuideFrame.Transform(GfVec3d(2,3,-1)) == GfVec3d(8,3,-1));
    CHECK(control.GetAttribute(TfToken("guide:offset")).Set(GfVec3d(1,0,0)));
    CHECK(control.GetAttribute(TfToken("guide:scaleX")).Set(3.0));
    CHECK(control.GetAttribute(TfToken("avars:sx")).Set(2.0));
    CHECK(UsdGeomBoundable::ComputeExtentFromPlugins(boundable, UsdTimeCode::Default(), &extent));
    CHECK(extent == VtVec3fArray({GfVec3f(-16,3,-1),GfVec3f(-4,5,1)}));
    CHECK(control.GetAttribute(TfToken("guide:displayOpacity")).Set(0.0f));
    CHECK(!UsdGeomBoundable::ComputeExtentFromPlugins(boundable, UsdTimeCode::Default(), &extent));
    CHECK(control.GetAttribute(TfToken("guide:displayOpacity")).Set(0.5f));
    CHECK(control.GetAttribute(TfToken("guide:curveVertexCounts")).Set(VtIntArray{3}));
    CHECK(!UsdGeomBoundable::ComputeExtentFromPlugins(boundable, UsdTimeCode::Default(), &extent));
}

// The codeless schema's resource directory.
// The GENERATED one when the build supplied it: only that copy carries the
// LibraryPath that lets Plug load the compute-extent registration on demand,
// which is what makes UsdGeomBBoxCache answer for RigExec prims. The source
// tree's copy is data-only and is the fallback for an ad hoc build.
static std::string
_SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

// The generated imaging plugInfo, which carries the rig adapter's keyless
// registration (it has no source-tree fallback: the checked-in file is a
// template with an unexpanded library filename). Empty when the build did
// not provide one, in which case the live-evaluation test skips itself.
static std::string
_ImagingResourceDir()
{
#ifdef RIGEXEC_IMAGING_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_IMAGING_RESOURCE_DIR);
#else
    return std::string();
#endif
}

// Transform-authority validation (host-durability redesign): the compiler
// warns about the two things that leave a provider's computed extent
// placing its guide somewhere the rig is not, without failing a compile
// over what is only a framing inaccuracy.
static void
TestTransformAuthorityWarnings(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
    CHECK(stage);
    if (!stage) return;

    const SdfPath rigPath("/TailAsset/Rig");
    const SdfPath tail1("/TailAsset/Rig/Controls/Tail1");

    // A clean rig warns about nothing -- the check has to be quiet on the
    // ordinary shape of a rig, where joints nest under joints and every
    // one of them is Xformable now.
    {
        RigExecImagingBridge bridge(stage, rigPath);
        std::vector<std::string> errors;
        CHECK(bridge.Compile(&errors));
        for (const std::string &e : errors) {
            std::printf("  unexpected compile diagnostic: %s\n", e.c_str());
        }
        CHECK(errors.empty());
    }

    auto warnedAbout = [](const std::vector<std::string> &errors,
                          const char *needle) {
        for (const std::string &e : errors) {
            if (e.rfind("warning: ", 0) == 0 &&
                e.find(needle) != std::string::npos) {
                return true;
            }
        }
        return false;
    };

    // xformOps on a provider: a second transform authority that nothing
    // reads. The compile still succeeds -- evaluation is unaffected.
    {
        UsdGeomXformable(stage->GetPrimAtPath(tail1))
            .AddTranslateOp()
            .Set(GfVec3d(3, 0, 0));
        RigExecImagingBridge bridge(stage, rigPath);
        std::vector<std::string> errors;
        const bool compiled = bridge.Compile(&errors);
        CHECK(compiled);
        if (!warnedAbout(errors, "authors xformOps")) {
            std::printf("  no xformOps warning for %s\n",
                        tail1.GetString().c_str());
        }
        CHECK(warnedAbout(errors, "authors xformOps"));
    }

    // A solver is a provider too, now that it inherits Boundable: an
    // authored op there is applied by BBoxCache to an already-baked extent
    // while the guide it draws ignores it.
    {
        UsdGeomXformable(
            stage->GetPrimAtPath(SdfPath("/TailAsset/Rig/Solvers/TailFK")))
            .AddTranslateOp()
            .Set(GfVec3d(0, 4, 0));
        RigExecImagingBridge bridge(stage, rigPath);
        std::vector<std::string> errors;
        CHECK(bridge.Compile(&errors));
        bool solverWarned = false;
        for (const std::string &e : errors) {
            if (e.find("TailFK") != std::string::npos &&
                e.find("authors xformOps") != std::string::npos) {
                solverWarned = true;
            }
        }
        if (!solverWarned) {
            std::printf("  no xformOps warning for the solver\n");
        }
        CHECK(solverWarned);
    }

    // Geometry parented under a provider falls outside its extent, and
    // bounds stop descending at a Boundable, so it vanishes from every
    // ancestor's box too.
    {
        stage->DefinePrim(SdfPath("/TailAsset/Rig/Controls/Tail1/Extra"),
                          TfToken("Sphere"));
        RigExecImagingBridge bridge(stage, rigPath);
        std::vector<std::string> errors;
        CHECK(bridge.Compile(&errors));
        if (!warnedAbout(errors, "is parented under RigExec provider")) {
            std::printf("  no nested-gprim warning\n");
        }
        CHECK(warnedAbout(errors, "is parented under RigExec provider"));
    }

    // An Xformable between the asset root and a provider is COMPOSED into
    // the provider's frames as of 2026-09-10, so it is no longer a defect to
    // warn about -- placing a rig, or one leg of an assembly, under an Xform
    // inside the asset is a supported shape.
    // The assertion is that the walk stays quiet about it. That the
    // transform actually lands is a frame question, tested where the frames
    // are (tests/python/test_intervening_xform.py).
    {
        const SdfPath intervening("/TailAsset/Rig/Extra");
        stage->DefinePrim(intervening, TfToken("Xform"));
        stage->DefinePrim(intervening.AppendChild(TfToken("Ctrl")),
                          TfToken("RigExecControl"));
        UsdGeomXformable(stage->GetPrimAtPath(intervening))
            .AddTranslateOp()
            .Set(GfVec3d(0, 7, 0));
        RigExecImagingBridge bridge(stage, rigPath);
        std::vector<std::string> errors;
        CHECK(bridge.Compile(&errors));
        if (warnedAbout(errors, "sits between the asset root")) {
            std::printf("  still warning about a composed intervening "
                        "Xformable\n");
        }
        CHECK(!warnedAbout(errors, "sits between the asset root"));
    }
}

// An intervening Xformable moves the guide and must NOT move the extent.
// The extent is LOCAL and UsdGeomBBoxCache multiplies it by the prim's own
// local-to-world, which already contains that Xform. The published frames
// contain it too since 2026-09-10, so the snapshot branch has to divide it
// back out; if it does not, the box lands at the Xform applied twice --
// guide in one place, bounding box in another.
// Asserted with a pure TRANSLATION, where the local extent must come back
// bit-for-bit unchanged. A rotation would also grow the box through two
// axis-realignments, which is legal (bounds may be conservative) and would
// blunt the assertion.
static void
TestInterveningXformLeavesTheLocalExtentAlone(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
    CHECK(stage);
    if (!stage) return;

    const SdfPath rigPath("/TailAsset/Rig");
    const SdfPath control("/TailAsset/Rig/Controls/Tail4");
    UsdGeomBoundable boundable(stage->GetPrimAtPath(control));
    CHECK(boundable);
    if (!boundable) return;

    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    CHECK(registry.Activate(stage, rigPath, UsdTimeCode(1024), &errors));
    VtVec3fArray before;
    CHECK(boundable.ComputeExtent(UsdTimeCode(1024), &before));
    registry.Deactivate();

    // Reparent nothing: put the op on the Controls scope's own Xform-able
    // ancestor by making one. The rig root stays where it is, so the asset
    // root does too, and the only thing that changes is a transform BETWEEN
    // them.
    UsdGeomXformable intervening(
        stage->DefinePrim(SdfPath("/TailAsset/Rig/Controls"),
                          TfToken("Xform")));
    CHECK(intervening);
    intervening.AddTranslateOp().Set(GfVec3d(0, 13, 0));

    errors.clear();
    CHECK(registry.Activate(stage, rigPath, UsdTimeCode(1024), &errors));
    VtVec3fArray after;
    CHECK(boundable.ComputeExtent(UsdTimeCode(1024), &after));
    registry.Deactivate();

    CHECK(before.size() == 2 && after.size() == 2);
    if (before.size() == 2 && after.size() == 2) {
        const bool same =
            GfIsClose(GfVec3d(before[0]), GfVec3d(after[0]), 1e-4) &&
            GfIsClose(GfVec3d(before[1]), GfVec3d(after[1]), 1e-4);
        if (!same) {
            std::printf("  the local extent moved with the intervening "
                        "Xform: (%g %g %g)-(%g %g %g) became "
                        "(%g %g %g)-(%g %g %g)\n",
                        before[0][0], before[0][1], before[0][2],
                        before[1][0], before[1][1], before[1][2],
                        after[0][0], after[0][1], after[0][2],
                        after[1][0], after[1][1], after[1][2]);
        }
        CHECK(same);
    }
}

// The computed extent follows the POSE when a generation is published, and
// falls back to the rest pose when none is.
// Both halves matter: the rest answer is what an un-evaluated stage frames
// on, and the posed answer is what has to agree with the thing on screen.
static void
TestSnapshotBackedExtent(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath tail4("/TailAsset/Rig/Controls/Tail4");
    UsdGeomBoundable boundable(stage->GetPrimAtPath(tail4));
    CHECK(boundable);

    auto extentAt = [&boundable](double frame, VtVec3fArray *out) {
        return boundable.ComputeExtent(UsdTimeCode(frame), out);
    };

    // Nothing activated: the rest fallback answers, from authored data.
    RigExecImaging_Deactivate();
    VtVec3fArray rest;
    CHECK(extentAt(1024, &rest));
    CHECK(rest.size() == 2);

    // Now publish a real generation. Frame 1024 rotates the tail, so the
    // control's posed frame is not its rest frame.
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    const bool activated = registry.Activate(
        stage, SdfPath("/TailAsset/Rig"), UsdTimeCode(1024), &errors);
    CHECK(activated);
    if (!activated) return;

    VtVec3fArray posed;
    CHECK(extentAt(1024, &posed));
    CHECK(posed.size() == 2);
    if (posed.size() == 2 && rest.size() == 2) {
        const bool moved = !GfIsClose(GfVec3d(posed[0]), GfVec3d(rest[0]),
                                      1e-4) ||
                           !GfIsClose(GfVec3d(posed[1]), GfVec3d(rest[1]),
                                      1e-4);
        if (!moved) {
            std::printf("  posed extent equals the rest extent -- the "
                        "snapshot is not reaching ComputeExtent\n");
        }
        CHECK(moved);
    }
    registry.Deactivate();
}

// The extent callback is a PURE FUNCTION of (stage, time).
// The snapshot store is a process-global singleton and USD calls this
// callback with a stage and a time of its own choosing, so a lookup by
// path alone answers a query about one stage/frame with another's pose --
// plausible, wrong, and invisible downstream. Both halves are pinned here.
// Also pins the documented CACHING contract. UsdGeomBBoxCache caches a
// plugin-computed extent as constant: nothing about it is time-varying,
// because RigExec authors no extent attribute (and must not -- see
// testRigExecNoAuthoring). Measured: after SetTime(1024) a long-lived
// cache still returns the 1001 box while a fresh cache returns 1024's.
// So the contract is per-query correctness; a holder of a long-lived
// cache must Clear() it, and SetTime alone is NOT enough.
static void
TestExtentIsPureFunctionOfStageAndTime(const std::string &examplesDir)
{
    const std::string file = examplesDir + "/01_FkChainTail.usda";
    UsdStageRefPtr stage = UsdStage::Open(file);
    CHECK(stage);
    if (!stage) return;
    const SdfPath control("/TailAsset/Rig/Controls/Tail4");
    UsdGeomBoundable boundable(stage->GetPrimAtPath(control));
    CHECK(boundable);

    auto extent = [](const UsdGeomBoundable &b, double frame) {
        VtVec3fArray out;
        return b.ComputeExtent(UsdTimeCode(frame), &out) && out.size() == 2
            ? GfVec3d(out[0]) : GfVec3d(1e9);
    };

    RigExecImaging_Deactivate();
    const GfVec3d rest = extent(boundable, 1024);

    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    const bool activated = registry.Activate(
        stage, SdfPath("/TailAsset/Rig"), UsdTimeCode(1024), &errors);
    CHECK(activated);
    if (!activated) return;

    // The generation describes frame 1024, so 1024 is answered from it...
    const GfVec3d posed = extent(boundable, 1024);
    CHECK(!GfIsClose(posed, rest, 1e-4));

    // ...and 1001 is NOT. Before the time gate this returned the 1024
    // pose for a 1001 query; now it falls back to the rest pose, which is
    // the only answer this callback can give for a frame nothing has
    // evaluated.
    const GfVec3d otherFrame = extent(boundable, 1001);
    if (GfIsClose(otherFrame, posed, 1e-4)) {
        std::printf("  a 1001 query was answered from the 1024 "
                    "generation\n");
    }
    CHECK(!GfIsClose(otherFrame, posed, 1e-4));
    CHECK(GfIsClose(otherFrame, rest, 1e-4));

    // A DIFFERENT stage must not be answered from this stage's generation
    // -- and the case that matters is two stages sharing the SAME ROOT
    // LAYER, differing only by session layer. That is the ordinary way a
    // host opens a second view of one asset, and it is exactly what a
    // root-layer-identifier comparison cannot tell apart.
    {
        SdfLayerRefPtr root = SdfLayer::FindOrOpen(file);
        CHECK(root);
        UsdStageRefPtr other =
            UsdStage::Open(root, SdfLayer::CreateAnonymous());
        CHECK(other);
        if (other) {
            CHECK(other != stage);
            CHECK(other->GetRootLayer() == stage->GetRootLayer());
            UsdGeomBoundable otherBoundable(other->GetPrimAtPath(control));
            CHECK(otherBoundable);
            if (otherBoundable) {
                const GfVec3d fromOtherStage = extent(otherBoundable, 1024);
                if (GfIsClose(fromOtherStage, posed, 1e-4)) {
                    std::printf("  a second stage over the same root layer "
                                "was answered from this stage's "
                                "generation\n");
                }
                CHECK(!GfIsClose(fromOtherStage, posed, 1e-4));
                CHECK(GfIsClose(fromOtherStage, rest, 1e-4));
            }
        }
    }

    // The caching contract, measured rather than assumed.
    {
        UsdGeomBBoxCache longLived(UsdTimeCode(1024),
                                   {UsdGeomTokens->default_});
        const GfRange3d atPosed =
            longLived.ComputeWorldBound(stage->GetPrimAtPath(control))
                .ComputeAlignedRange();
        registry.SetTime(UsdTimeCode(1001));
        longLived.SetTime(UsdTimeCode(1001));
        const GfRange3d afterSetTime =
            longLived.ComputeWorldBound(stage->GetPrimAtPath(control))
                .ComputeAlignedRange();
        UsdGeomBBoxCache fresh(UsdTimeCode(1001), {UsdGeomTokens->default_});
        const GfRange3d freshRange =
            fresh.ComputeWorldBound(stage->GetPrimAtPath(control))
                .ComputeAlignedRange();

        // A fresh cache is correct for its time...
        CHECK(!freshRange.IsEmpty());
        // ...and the long-lived one is documented-stale after SetTime
        // alone. Asserted so the day USD starts re-querying, this test
        // fails and the documentation gets corrected rather than quietly
        // becoming wrong.
        CHECK(afterSetTime.GetMin() == atPosed.GetMin());
        CHECK(freshRange.GetMin() != atPosed.GetMin());
    }
    registry.Deactivate();
}

// One extent carries ONE purpose: a descendant whose resolved
// purpose differs from the boundable's is excluded from its bounds, and
// the compiler says so rather than leaving it a silent hole.
static void
TestPurposeScopedBounds(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
    CHECK(stage);
    if (!stage) return;

    // A default-purpose control parented under a guide-purpose joint.
    const SdfPath joint("/TailAsset/Rig/Joints/Seg1");
    const SdfPath nested = joint.AppendChild(TfToken("NestedCtrl"));
    UsdPrim control = stage->DefinePrim(nested, TfToken("RigExecControl"));
    CHECK(control);
    // Not `far`: WinDef.h still defines that as an empty legacy macro, so
    // the declaration silently loses its name and the file stops compiling.
    GfMatrix4d wayOut(1.0);
    wayOut.SetTranslateOnly(GfVec3d(0, 60, 0));
    CHECK(control.GetAttribute(TfToken("rest:space")).Set(wayOut));

    CHECK(UsdGeomImageable(stage->GetPrimAtPath(joint)).ComputePurpose() ==
          UsdGeomTokens->guide);
    CHECK(UsdGeomImageable(control).ComputePurpose() ==
          UsdGeomTokens->default_);

    // The joint's guide-purpose bound must not swallow it: at y=60 the
    // control is nowhere near the joint chain, so inclusion is obvious.
    RigExecImaging_Deactivate();
    UsdGeomBBoxCache guideOnly(UsdTimeCode(1001), {UsdGeomTokens->guide});
    const GfRange3d jointRange =
        guideOnly.ComputeWorldBound(stage->GetPrimAtPath(joint))
            .ComputeAlignedRange();
    CHECK(!jointRange.IsEmpty());
    if (!jointRange.IsEmpty() && jointRange.GetMax()[1] > 30.0) {
        std::printf("  a default-purpose control was folded into a "
                    "guide-purpose bound (maxY %g)\n",
                    jointRange.GetMax()[1]);
    }
    CHECK(jointRange.GetMax()[1] < 30.0);

    // ...and the author is told, because nothing in the namespace hints at
    // it.
    {
        RigExecImagingBridge bridge(stage, SdfPath("/TailAsset/Rig"));
        std::vector<std::string> errors;
        CHECK(bridge.Compile(&errors));
        bool warned = false;
        for (const std::string &e : errors) {
            if (e.find("one extent carries one purpose") !=
                std::string::npos) {
                warned = true;
            }
        }
        if (!warned) {
            std::printf("  no mixed-purpose nesting warning\n");
        }
        CHECK(warned);
    }
}

// Every stock purpose maps to its own Hydra render tag, so drawing and
// bounds classification agree for all four allowed tokens.
static void
TestAllPurposeRenderTags(const std::string &examplesDir)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath control("/TailAsset/Rig/Controls/Tail1");
    const SdfPath guidePath = control.AppendChild(TfToken("rigGuideCtrl"));

    RigExecImagingBridge bridge(stage, SdfPath("/TailAsset/Rig"));
    std::vector<std::string> errors;
    CHECK(bridge.Compile(&errors));
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims(
        {{control, TfToken(),
          HdRetainedContainerDataSource::New(0, nullptr, nullptr)}});
    auto results =
        RigExecResultsSceneIndex::New(upstream, bridge.GetStore());

    const struct {
        TfToken purpose;
        TfToken renderTag;
    } cases[] = {{UsdGeomTokens->default_, HdRenderTagTokens->geometry},
                 {UsdGeomTokens->guide, HdRenderTagTokens->guide},
                 {UsdGeomTokens->render, HdRenderTagTokens->render},
                 {UsdGeomTokens->proxy, HdRenderTagTokens->proxy}};
    for (const auto &c : cases) {
        CHECK(UsdGeomImageable(stage->GetPrimAtPath(control))
                  .GetPurposeAttr()
                  .Set(c.purpose));
        CHECK(bridge.EvaluateAndPublish(UsdTimeCode(1001)));
        HdPurposeSchema purpose = HdPurposeSchema::GetFromParent(
            results->GetPrim(guidePath).dataSource);
        const TfToken drawn = purpose.GetPurpose()
            ? purpose.GetPurpose()->GetTypedValue(0.0f) : TfToken();
        if (drawn != c.renderTag) {
            std::printf("  purpose '%s' drew render tag '%s', expected "
                        "'%s'\n", c.purpose.GetText(), drawn.GetText(),
                        c.renderTag.GetText());
        }
        CHECK(drawn == c.renderTag);
    }
}

// A manipulation preview of a prim no rig drives reaches Hydra as a
// post-multiplied world delta, and carries the prim's descendants with it
// (docs/superpowers/specs/2026-09-10-hydra-preview-manipulation-design.md).
// The delta form is what a FLATTENED chain needs. By the time a prim reaches
// these filters its xform is the composed world one -- which is why the
// results index marks its own overrides resetXformStack=true -- so a local
// matrix would drop the ancestors, and overriding only the dragged prim would
// leave its children at the parent's old place. Post-multiplying answers both,
// and this test is the assertion that it does: the child moves by exactly the
// same amount as the parent, while keeping its own offset.
static void
TestXformPreviewDelta()
{
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    auto store = std::make_shared<RigExecSnapshotStore>();
    _Chain chain = _BuildChain(upstream, store, {});

    // Flattened world matrices, as upstream would hand them over: the hand
    // sits 2 along X from the body, and the body 1 up in Y.
    const SdfPath bodyPath("/Asset/Geom/Body");
    const SdfPath handPath("/Asset/Geom/Body/Hand");
    const SdfPath otherPath("/Asset/Geom/Prop");
    GfMatrix4d bodyWorld(1.0), handWorld(1.0), otherWorld(1.0);
    bodyWorld.SetTranslate(GfVec3d(0, 1, 0));
    handWorld.SetTranslate(GfVec3d(2, 1, 0));
    otherWorld.SetTranslate(GfVec3d(-5, 0, 0));

    HdRetainedSceneIndex::AddedPrimEntries entries;
    entries.push_back(_MeshShellWithXform(bodyPath, bodyWorld));
    entries.push_back(_MeshShellWithXform(handPath, handWorld));
    entries.push_back(_MeshShellWithXform(otherPath, otherWorld));
    upstream->AddPrims(entries);

    _RecordingObserver observer;
    chain.results->AddObserver(HdSceneIndexObserverPtr(&observer));

    auto worldOf = [&](const SdfPath &path) {
        const HdSceneIndexPrim prim = chain.results->GetPrim(path);
        HdXformSchema schema = HdXformSchema::GetFromParent(prim.dataSource);
        if (!schema || !schema.GetMatrix()) {
            return GfMatrix4d(0.0);
        }
        return schema.GetMatrix()->GetTypedValue(0.0f);
    };
    auto composed = [&](const SdfPath &path) {
        const HdSceneIndexPrim prim = chain.results->GetPrim(path);
        HdXformSchema schema = HdXformSchema::GetFromParent(prim.dataSource);
        return schema && schema.GetResetXformStack() &&
               schema.GetResetXformStack()->GetTypedValue(0.0f);
    };

    CHECK(worldOf(bodyPath) == bodyWorld);
    CHECK(worldOf(handPath) == handWorld);

    // One mouse sample: the body moves 10 along X.
    GfMatrix4d delta(1.0);
    delta.SetTranslate(GfVec3d(10, 0, 0));
    observer.dirtied.clear();
    chain.xforms->SetWorldDeltas({{bodyPath, delta}});

    GfMatrix4d expectedBody(1.0), expectedHand(1.0);
    expectedBody.SetTranslate(GfVec3d(10, 1, 0));
    expectedHand.SetTranslate(GfVec3d(12, 1, 0));
    CHECK(worldOf(bodyPath) == expectedBody);
    // The child kept its 2 along X from the body and moved with it. A local
    // override would have put it at (2,1,0) -- its own offset, with the body's
    // transform lost -- and an override that skipped descendants would have
    // left it at (2,1,0) too, so this number distinguishes all three.
    CHECK(worldOf(handPath) == expectedHand);
    CHECK(worldOf(otherPath) == otherWorld);
    CHECK(composed(bodyPath) && composed(handPath));

    // The notice names the previewed prim AND its descendant: the child's
    // matrix changed without the child being mentioned in the call.
    bool dirtiedBody = false, dirtiedHand = false, dirtiedOther = false;
    for (const SdfPath &path : observer.dirtied) {
        if (path == bodyPath) dirtiedBody = true;
        if (path == handPath) dirtiedHand = true;
        if (path == otherPath) dirtiedOther = true;
    }
    CHECK(dirtiedBody);
    CHECK(dirtiedHand);
    CHECK(!dirtiedOther);

    // A second sample at the same value is not a change, and says nothing.
    observer.dirtied.clear();
    chain.xforms->SetWorldDeltas({{bodyPath, delta}});
    CHECK(observer.dirtied.empty());

    // Release: the authored transforms are what is drawn again, and the
    // subtree is dirtied on the way out.
    observer.dirtied.clear();
    chain.xforms->ClearWorldDeltas();
    CHECK(!chain.xforms->HasWorldDeltas());
    CHECK(worldOf(bodyPath) == bodyWorld);
    CHECK(worldOf(handPath) == handWorld);
    CHECK(!observer.dirtied.empty());

    chain.results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
}

// Live evaluation through the stage scene index's own clock -- the usdrecord
// path (docs/specs/imaging-datasource-redesign.md §3.2). No explicit
// activation anywhere in this test: the rig adapter notes the root during
// populate, the results index's _PrimsAdded forces the activation, and every
// SetTime after that re-evaluates through the rigExec/time trigger.
static void
TestSetTimeDrivenEvaluation(const std::string &examplesDir)
{
    if (_ImagingResourceDir().empty()) {
        std::printf("  (no imaging plugInfo; skipping live-evaluation test)\n");
        return;
    }
    const std::string path = examplesDir + "/10_AimXformTurret.usda";
    UsdStageRefPtr stage = UsdStage::Open(path);
    CHECK(stage != nullptr);
    if (!stage) {
        return;
    }

    SdfPath rigPath;
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            rigPath = prim.GetPath();
        }
    }
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }

    // Isolated from whatever earlier tests published: the store is cleared,
    // and the per-session evaluation count below starts from each new
    // session's own zero, so earlier publications cannot move these asserts.
    RigExecImaging_Deactivate();

    // The REAL chain, exactly as usdview/usdrecord build it: the RigExec
    // scene index plugin inserts itself, and the rig adapter discovers the
    // root. Nothing here activates explicitly.
    UsdImagingCreateSceneIndicesInfo info;
    info.stage = stage;
    const UsdImagingSceneIndices sceneIndices =
        UsdImagingCreateSceneIndices(info);
    const HdSceneIndexBaseRefPtr terminal = sceneIndices.finalSceneIndex;
    CHECK(terminal != nullptr);
    if (!terminal) {
        return;
    }

    // Populate alone activated the rig and evaluated it exactly once: one
    // generation, no pulls, no SetTime.
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    const long long genPopulate = RigExecImaging_GetGeneration();
    CHECK(genPopulate > 0);
    CHECK(registry.GetSessionEvaluationCount(rigPath) == 1);

    const SdfPath turret("/TurretAsset/Geom/Turret");
    auto worldOf = [&](const SdfPath &p) {
        HdXformSchema xf = HdXformSchema::GetFromParent(
            terminal->GetPrim(p).dataSource);
        return xf && xf.GetMatrix() ? xf.GetMatrix()->GetTypedValue(0.0f)
                                   : GfMatrix4d(1.0);
    };
    _RecordingObserver observer;
    terminal->AddObserver(HdSceneIndexObserverPtr(&observer));

    // SetTime alone drives one fresh evaluated generation per frame...
    sceneIndices.stageSceneIndex->SetTime(UsdTimeCode(1001));
    CHECK(RigExecImaging_GetGeneration() == genPopulate + 1);
    CHECK(registry.GetSessionEvaluationCount(rigPath) == 2);
    const GfMatrix4d at1001 = worldOf(turret);
    observer.dirtied.clear();
    observer.dirtiedLocators.clear();
    sceneIndices.stageSceneIndex->SetTime(UsdTimeCode(1024));
    CHECK(RigExecImaging_GetGeneration() == genPopulate + 2);
    CHECK(registry.GetSessionEvaluationCount(rigPath) == 3);
    const GfMatrix4d at1024 = worldOf(turret);
    CHECK(at1001 != at1024);

    // ...announced through dirtied notices, not just visible to pulls (a
    // pull-only test would pass on a frozen viewport).
    bool sawTurretDirty = false;
    for (const SdfPath &p : observer.dirtied) {
        if (p == turret) {
            sawTurretDirty = true;
        }
    }
    CHECK(sawTurretDirty);

    // ...and identical to the explicit path: the same frames driven through
    // the C entry point publish the same values, and re-driving an already
    // published frame is free through the redundant-call guard.
    CHECK(RigExecImaging_SetTime(1001.0) == 0);
    CHECK(worldOf(turret) == at1001);
    CHECK(RigExecImaging_SetTime(1024.0) == 0);
    CHECK(worldOf(turret) == at1024);
    const long long genExplicit = RigExecImaging_GetGeneration();
    const size_t countExplicit =
        registry.GetSessionEvaluationCount(rigPath);
    sceneIndices.stageSceneIndex->SetTime(UsdTimeCode(1024));
    CHECK(RigExecImaging_GetGeneration() == genExplicit);
    CHECK(registry.GetSessionEvaluationCount(rigPath) == countExplicit);
    CHECK(worldOf(turret) == at1024);

    terminal->RemoveObserver(HdSceneIndexObserverPtr(&observer));
    RigExecImaging_Deactivate();
}

// A policy-carrying rig activates without taking the registry down.
// The compile derives rigExec:startFrame targets into the session layer,
// and the registry registers its ObjectsChanged listener BEFORE Compile
// and holds a non-recursive mutex across the whole call -- so a notice
// fired by that derivation re-enters the registry on its own held mutex.
// Measured as an access violation in _OnObjectsChanged on the first
// usdview activation of Biped.usda; the derivation now swallows its own
// notices (TfNotice::Block) and activation survives. This pins that:
// both policy-carrying biped files activate, publish, retime, and
// deactivate. (The derived values themselves are pinned by
// test_rigexec_biped_hand.py; this test pins the activation.)
static void
TestPolicyRigActivationSurvivesItsOwnDerivation(
    const std::string &examplesDir)
{
    const char *files[] = {
        "/biped/Biped.usda",
        "/biped/Biped_body.usda",
    };
    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    for (const char *file : files) {
        UsdStageRefPtr stage = UsdStage::Open(examplesDir + file);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        std::vector<std::string> errors;
        CHECK(registry.Activate(stage, SdfPath("/Biped/Rig"),
                                UsdTimeCode(1.0), &errors));
        CHECK(registry.GetStore()->Get());
        CHECK(registry.SetTime(UsdTimeCode(2.0)));
        CHECK(registry.GetStore()->Get());
        registry.Deactivate();
    }
}

// --- Upstream scene-index inputs ------------------------------------------
//
// An upstream scene index publishes `rigExecInputs` containers; the results
// index pulls them (with no registry lock held) into a table its context
// stores, hands each live evaluation the values at its time, and warms other
// frames from the table alone. The frame cache keys every frame on its own
// values.

namespace {
static std::vector<RigExecOpGraphNode>
HeadGraphForTesting(const RigExecBakedProgramImpl &B)
{
    auto graph = RigExecBakedOpGraph(B);
    graph.erase(std::remove_if(graph.begin(),graph.end(),
        [&B](const auto &node) { return !B.steps[node.step].isHead; }),graph.end());
    return graph;
}


// Head entries are part of the ordinary execution trace.
static std::vector<RigExecOpTraceEntry>
ExecutedHeads(const RigExecBakedProgramImpl &B)
{
    auto trace = RigExecBakedLastRunTrace(B);
    trace.erase(std::remove_if(trace.begin(),trace.end(),
        [&B](const auto &entry) { return !B.steps[entry.step].isHead; }),trace.end());
    return trace;
}


// Calls into the test sources: how many, how many off a sample time, and
// how many from a thread other than the one that built the scene index.
struct _UpstreamCalls {
    std::atomic<size_t> getValue{0};
    std::atomic<size_t> offSample{0};
    std::atomic<size_t> offThread{0};
    std::thread::id owner = std::this_thread::get_id();
};

// One `rigExecInputs` child. Uniform: GetContributingSampleTimesForInterval
// says false. Sampled: true with the window's integer frames, answering
// f(T + dt) only there. Sparse: true with the given frames only, answering
// only there. T is the scene index's time at the call.
class _UpstreamSource : public HdSampledDataSource {
public:
    HD_DECLARE_DATASOURCE(_UpstreamSource);

    enum class Kind { Uniform, Sampled, Sparse };

    VtValue GetValue(Time offset) override
    {
        _Count();
        if (_kind == Kind::Uniform) {
            return _uniform;
        }
        const double at = _time->load() + double(offset);
        if (_kind == Kind::Sampled) {
            if (std::floor(at) == at) {
                return _f(at);
            }
        } else {
            const auto found = _samples.find(at);
            if (found != _samples.end()) {
                return found->second;
            }
        }
        _calls->offSample.fetch_add(1);
        return VtValue();
    }

    bool GetContributingSampleTimesForInterval(
        Time startTime, Time endTime, std::vector<Time> *out) override
    {
        if (_kind == Kind::Uniform) {
            return false;
        }
        const double t = _time->load();
        out->clear();
        if (_kind == Kind::Sampled) {
            for (double f = std::ceil(t + startTime);
                 f <= std::floor(t + endTime); f += 1.0) {
                out->push_back(Time(f - t));
            }
        } else {
            for (const auto &[at, value] : _samples) {
                out->push_back(Time(at - t));
            }
        }
        return true;
    }

private:
    _UpstreamSource(Kind kind, VtValue uniform,
                    std::function<VtValue(double)> f,
                    std::map<double, VtValue> samples,
                    std::shared_ptr<std::atomic<double>> time,
                    std::shared_ptr<_UpstreamCalls> calls)
        : _kind(kind), _uniform(std::move(uniform)), _f(std::move(f)),
          _samples(std::move(samples)), _time(std::move(time)),
          _calls(std::move(calls))
    {
    }

    void _Count()
    {
        _calls->getValue.fetch_add(1);
        if (std::this_thread::get_id() != _calls->owner) {
            _calls->offThread.fetch_add(1);
        }
    }

    Kind _kind;
    VtValue _uniform;
    std::function<VtValue(double)> _f;
    std::map<double, VtValue> _samples;
    std::shared_ptr<std::atomic<double>> _time;
    std::shared_ptr<_UpstreamCalls> _calls;
};

HD_DECLARE_DATASOURCE_HANDLES(_UpstreamSource);

// The sources by prim, then attribute name.
using _UpstreamSourceMap =
    std::map<SdfPath, std::map<TfToken, HdDataSourceBaseHandle>>;

// A prim's data source with `rigExecInputs` read from the live source map
// at each Get. The handle a downstream index caches (the flattening index
// keeps the prim's input data source until a resync) therefore stays
// valid while sources come and go, and a dirty is all a change needs.
class _UpstreamPrimSource : public HdContainerDataSource {
public:
    HD_DECLARE_DATASOURCE(_UpstreamPrimSource);

    TfTokenVector GetNames() override
    {
        TfTokenVector names =
            _input ? _input->GetNames() : TfTokenVector();
        if (_Inputs()) {
            names.push_back(RigExecUpstreamInputsToken());
        }
        return names;
    }

    HdDataSourceBaseHandle Get(const TfToken &name) override
    {
        if (name == RigExecUpstreamInputsToken()) {
            return _Inputs();
        }
        return _input ? _input->Get(name) : nullptr;
    }

private:
    _UpstreamPrimSource(HdContainerDataSourceHandle input,
                        std::shared_ptr<const _UpstreamSourceMap> sources,
                        SdfPath prim)
        : _input(std::move(input)), _sources(std::move(sources)),
          _prim(std::move(prim))
    {
    }

    HdContainerDataSourceHandle _Inputs() const
    {
        const auto found = _sources->find(_prim);
        if (found == _sources->end() || found->second.empty()) {
            return nullptr;
        }
        std::vector<TfToken> names;
        std::vector<HdDataSourceBaseHandle> values;
        for (const auto &[name, source] : found->second) {
            names.push_back(name);
            values.push_back(source);
        }
        return HdRetainedContainerDataSource::New(names.size(), names.data(),
                                                  values.data());
    }

    HdContainerDataSourceHandle _input;
    std::shared_ptr<const _UpstreamSourceMap> _sources;
    SdfPath _prim;
};

// Publishes `rigExecInputs` on the prims it was given values for, and sends
// the dirty a value change owes.
class _UpstreamInputsSceneIndex : public HdSingleInputFilteringSceneIndexBase {
public:
    static TfRefPtr<_UpstreamInputsSceneIndex> New(
        const HdSceneIndexBaseRefPtr &input)
    {
        return TfCreateRefPtr(new _UpstreamInputsSceneIndex(input));
    }

    void Set(const SdfPath &attribute, const VtValue &value)
    {
        _Put(attribute, _UpstreamSource::New(
                            _UpstreamSource::Kind::Uniform, value, nullptr,
                            std::map<double, VtValue>(), _time, calls));
    }

    void SetSampled(const SdfPath &attribute,
                    std::function<VtValue(double)> f)
    {
        _Put(attribute, _UpstreamSource::New(
                            _UpstreamSource::Kind::Sampled, VtValue(),
                            std::move(f), std::map<double, VtValue>(), _time,
                            calls));
    }

    void SetSparse(const SdfPath &attribute,
                   std::map<double, VtValue> samples)
    {
        _Put(attribute, _UpstreamSource::New(
                            _UpstreamSource::Kind::Sparse, VtValue(), nullptr,
                            std::move(samples), _time, calls));
    }

    void Clear(const SdfPath &attribute)
    {
        _Put(attribute, nullptr);
    }

    void SetTime(double t) { _time->store(t); }

    HdSceneIndexPrim GetPrim(const SdfPath &primPath) const override
    {
        HdSceneIndexPrim prim = _GetInputSceneIndex()->GetPrim(primPath);
        if (prim.dataSource) {
            prim.dataSource = _UpstreamPrimSource::New(prim.dataSource,
                                                       _sources, primPath);
        }
        return prim;
    }

    SdfPathVector GetChildPrimPaths(const SdfPath &primPath) const override
    {
        return _GetInputSceneIndex()->GetChildPrimPaths(primPath);
    }

    std::shared_ptr<_UpstreamCalls> calls =
        std::make_shared<_UpstreamCalls>();

protected:
    void _PrimsAdded(const HdSceneIndexBase &,
                     const HdSceneIndexObserver::AddedPrimEntries &entries)
        override
    {
        _SendPrimsAdded(entries);
    }
    void _PrimsRemoved(
        const HdSceneIndexBase &,
        const HdSceneIndexObserver::RemovedPrimEntries &entries) override
    {
        _SendPrimsRemoved(entries);
    }
    void _PrimsDirtied(
        const HdSceneIndexBase &,
        const HdSceneIndexObserver::DirtiedPrimEntries &entries) override
    {
        _SendPrimsDirtied(entries);
    }

private:
    explicit _UpstreamInputsSceneIndex(const HdSceneIndexBaseRefPtr &input)
        : HdSingleInputFilteringSceneIndexBase(input)
    {
    }

    void _Put(const SdfPath &attribute, const HdDataSourceBaseHandle &source)
    {
        const SdfPath prim = attribute.GetPrimPath();
        if (source) {
            (*_sources)[prim][attribute.GetNameToken()] = source;
        } else {
            (*_sources)[prim].erase(attribute.GetNameToken());
        }
        // A value change at any time is announced by dirtying the child.
        HdSceneIndexObserver::DirtiedPrimEntries dirtied;
        dirtied.emplace_back(
            prim, HdDataSourceLocatorSet{HdDataSourceLocator(
                      RigExecUpstreamInputsToken(),
                      attribute.GetNameToken())});
        _SendPrimsDirtied(dirtied);
    }

    std::shared_ptr<_UpstreamSourceMap> _sources =
        std::make_shared<_UpstreamSourceMap>();
    std::shared_ptr<std::atomic<double>> _time =
        std::make_shared<std::atomic<double>>(0.0);
};

// What a generation publishes: points, driven transforms and guide frames
// (joint matrices), by prim.
struct _UpstreamPose {
    std::map<SdfPath, VtVec3fArray> points;
    std::map<SdfPath, GfMatrix4d> xforms;
    std::map<SdfPath, std::vector<GfMatrix4d>> guides;

    bool Empty() const { return points.empty() && guides.empty(); }
};

_UpstreamPose
_CaptureUpstreamPose(const RigExecImagingSnapshotConstPtr &snapshot)
{
    _UpstreamPose pose;
    if (!snapshot) {
        return pose;
    }
    for (const auto &[path, prim] : snapshot->prims) {
        if (prim.hasPoints) {
            pose.points[path] = prim.points;
        }
        if (prim.hasXform) {
            pose.xforms[path] = prim.xform;
        }
        if (prim.hasGuides) {
            pose.guides[path] = prim.guideFrames;
        }
    }
    return pose;
}

// The entries whose path names any of \p names.
_UpstreamPose
_UpstreamPart(const _UpstreamPose &pose,
              const std::vector<std::string> &names)
{
    const auto named = [&](const SdfPath &path) {
        for (const std::string &name : names) {
            if (path.GetString().find(name) != std::string::npos) {
                return true;
            }
        }
        return false;
    };
    _UpstreamPose part;
    for (const auto &[path, points] : pose.points) {
        if (named(path)) {
            part.points[path] = points;
        }
    }
    for (const auto &[path, xform] : pose.xforms) {
        if (named(path)) {
            part.xforms[path] = xform;
        }
    }
    for (const auto &[path, guides] : pose.guides) {
        if (named(path)) {
            part.guides[path] = guides;
        }
    }
    return part;
}

// Bit-identical, printing the first difference under \p what.
bool
_SameUpstreamPose(const _UpstreamPose &a, const _UpstreamPose &b,
                  const std::string &what, bool report = true)
{
    std::string why;
    if (a.points.size() != b.points.size() ||
        a.xforms.size() != b.xforms.size() ||
        a.guides.size() != b.guides.size()) {
        why = "different prim sets";
    }
    for (const auto &[path, points] : a.points) {
        const auto found = b.points.find(path);
        if (why.empty() &&
            (found == b.points.end() || found->second != points)) {
            why = "points differ at " + path.GetString();
        }
    }
    for (const auto &[path, xform] : a.xforms) {
        const auto found = b.xforms.find(path);
        if (why.empty() &&
            (found == b.xforms.end() || found->second != xform)) {
            why = "xform differs at " + path.GetString();
        }
    }
    for (const auto &[path, guides] : a.guides) {
        const auto found = b.guides.find(path);
        if (why.empty() &&
            (found == b.guides.end() || found->second != guides)) {
            why = "guides differ at " + path.GetString();
        }
    }
    if (!why.empty() && report) {
        std::printf("    %s: %s\n", what.c_str(), why.c_str());
    }
    return why.empty();
}

// A value a reference stage authors in its session layer: a default, or
// time samples.
struct _UpstreamAuthored {
    SdfPath path;
    VtValue value;
    std::map<double, VtValue> samples;
};

// What a bridge publishes at \p t over \p stagePath with \p edits authored.
_UpstreamPose
_UpstreamReference(const std::string &stagePath, const SdfPath &rig,
                   const std::vector<_UpstreamAuthored> &edits, double t)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return _UpstreamPose();
    }
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        for (const _UpstreamAuthored &edit : edits) {
            const UsdAttribute attribute =
                stage->GetAttributeAtPath(edit.path);
            CHECK(attribute);
            if (!attribute) {
                continue;
            }
            if (edit.samples.empty()) {
                CHECK(attribute.Set(edit.value));
            }
            for (const auto &[at, value] : edit.samples) {
                CHECK(attribute.Set(value, UsdTimeCode(at)));
            }
        }
    }
    RigExecImagingBridge bridge(stage, rig);
    std::vector<std::string> errors;
    const bool compiled = bridge.Compile(&errors);
    CHECK(compiled);
    if (!compiled) {
        return _UpstreamPose();
    }
    CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(t)).ok);
    return _CaptureUpstreamPose(bridge.GetStore()->Get());
}

// One stage imaged through the real UsdImaging chain with the upstream
// scene index installed after the stage scene index; the rig adapter
// activates it.
struct _UpstreamImaging {
    UsdStageRefPtr stage;
    TfRefPtr<_UpstreamInputsSceneIndex> upstream;
    UsdImagingSceneIndices indices;
    RigExecImagingRegistry::Ptr context;
    SdfPath rig;

    ~_UpstreamImaging()
    {
        if (context) {
            context->Deactivate();
        }
    }

    // The input scene index's time and the stage scene index's, together:
    // the trigger the stage scene index sends carries T0.
    void SetTime(double t)
    {
        upstream->SetTime(t);
        indices.stageSceneIndex->SetTime(UsdTimeCode(t));
    }

    _UpstreamPose Pose() const
    {
        return _CaptureUpstreamPose(context->GetStore()->Get());
    }

    size_t Evaluations() const
    {
        return context->GetSessionEvaluationCount(rig);
    }

    const RigExecRigEvaluator *Evaluator() const
    {
        RigExecImagingBridge *bridge = context->GetBridge(rig);
        return bridge ? &bridge->GetEvaluator() : nullptr;
    }

    const RigExecBakedProgram *Program() const
    {
        const RigExecRigEvaluator *evaluator = Evaluator();
        return evaluator ? evaluator->GetBakedProgram() : nullptr;
    }
};

std::unique_ptr<_UpstreamImaging>
_OpenUpstreamImaging(
    const std::string &stagePath, const SdfPath &rig, double t,
    const std::function<void(_UpstreamInputsSceneIndex &)> &seed = nullptr,
    const std::function<void(const UsdStageRefPtr &)> &prepare = nullptr)
{
    auto imaging = std::make_unique<_UpstreamImaging>();
    imaging->stage = UsdStage::Open(stagePath);
    imaging->rig = rig;
    CHECK(imaging->stage);
    if (!imaging->stage) {
        return nullptr;
    }
    // Edits the stage needs before the chain activates the rig.
    if (prepare) {
        prepare(imaging->stage);
    }
    _UpstreamImaging *raw = imaging.get();
    UsdImagingCreateSceneIndicesInfo info;
    info.stage = imaging->stage;
    info.overridesSceneIndexCallback =
        [raw, &seed, t](const HdSceneIndexBaseRefPtr &input)
        -> HdSceneIndexBaseRefPtr {
        raw->upstream = _UpstreamInputsSceneIndex::New(input);
        // Sources standing before population reach RigExec as adds.
        if (seed) {
            raw->upstream->SetTime(t);
            seed(*raw->upstream);
        }
        return raw->upstream;
    };
    imaging->indices = UsdImagingCreateSceneIndices(info);
    CHECK(imaging->upstream);
    imaging->context =
        RigExecImagingRegistry::ForStage(imaging->stage, false);
    CHECK(imaging->context && imaging->context->IsActive());
    if (!imaging->upstream || !imaging->context ||
        !imaging->context->IsActive()) {
        return nullptr;
    }
    imaging->SetTime(t);
    return imaging;
}

// The frame cache mode for a span of the test, restored after.
class _UpstreamCacheMode {
public:
    explicit _UpstreamCacheMode(const char *mode)
        : _saved(TfGetenv("RIGEXEC_FRAME_CACHE", ""))
    {
        TfSetenv("RIGEXEC_FRAME_CACHE", mode);
    }
    ~_UpstreamCacheMode()
    {
        if (_saved.empty()) {
            TfUnsetenv("RIGEXEC_FRAME_CACHE");
        } else {
            TfSetenv("RIGEXEC_FRAME_CACHE", _saved);
        }
    }

private:
    std::string _saved;
};

const SdfPath kUpLimbsRig("/LimbsAsset/Rig");
const SdfPath kUpA0Rz("/LimbsAsset/Rig/Controls/A0.avars:rz");
const SdfPath kUpSpace("/LimbsAsset/Upstream.inputs:space");
const SdfPath kUpBRootRestSpace("/LimbsAsset/Rig/Controls/BRoot.rest:space");
const SdfPath kUpJointWeights(
    "/LimbsAsset/Rig/Movers/MeshASkin.rigExec:jointWeights");
const std::vector<std::string> kUpLimbA{"LimbA", "MeshA", "/A0", "/A1"};
const std::vector<std::string> kUpLimbB{"LimbB", "MeshB", "BRoot",
                                        "BEffector", "BPole"};

GfMatrix4d
_UpTranslate(double x, double y, double z)
{
    GfMatrix4d m(1.0);
    m.SetTranslateOnly(GfVec3d(x, y, z));
    return m;
}

bool
_Names(const std::string &label, const std::vector<std::string> &names)
{
    for (const std::string &name : names) {
        if (label.find(name) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// Steps forward of the steps whose label names \p seed, over \p graph.
std::set<size_t>
_ForwardClosure(const std::vector<RigExecOpGraphNode> &graph,
                const std::string &seed)
{
    std::set<size_t> closure;
    std::vector<size_t> pending;
    for (const RigExecOpGraphNode &node : graph) {
        if (node.label.find(seed) != std::string::npos) {
            pending.push_back(node.step);
        }
    }
    while (!pending.empty()) {
        const size_t step = pending.back();
        pending.pop_back();
        if (!closure.insert(step).second) {
            continue;
        }
        for (const RigExecOpGraphNode &node : graph) {
            if (node.step == step) {
                pending.insert(pending.end(), node.succs.begin(),
                               node.succs.end());
                break;
            }
        }
    }
    return closure;
}

// Step 3: only the forward cone of the moved value re-ran. The fixture has
// no always-dirty step. Region steps (sources aside) lie in the forward
// closure of \p regionSeed's steps when given, head steps in that of
// \p headSeed's when given, and no executed step, head or region, names
// only the limb the value does not reach (the slot grain the compose-group
// budget leaves machine-independent).
void
_CheckUpstreamCone(const _UpstreamImaging &imaging,
                   const std::vector<std::string> &own,
                   const std::vector<std::string> &other,
                   const std::string &regionSeed,
                   const std::string &headSeed, const std::string &what)
{
    const RigExecRigEvaluator *evaluator = imaging.Evaluator();
    const RigExecBakedProgram *program = imaging.Program();
    if (!evaluator || !program) {
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    for (size_t i = 0; i < B.steps.size(); ++i) {
        CHECK(!B.cones.alwaysSteps.Test(int(i)));
    }
    const std::vector<RigExecOpTraceEntry> region =
        evaluator->GetLastOpTrace();
    const std::vector<RigExecOpTraceEntry> head =
        ExecutedHeads(B);
    const std::set<size_t> regionClosure =
        regionSeed.empty() ? std::set<size_t>()
                           : _ForwardClosure(evaluator->GetOpGraph(),
                                             regionSeed);
    const std::set<size_t> headClosure =
        headSeed.empty() ? std::set<size_t>()
                         : _ForwardClosure(HeadGraphForTesting(B),
                                           headSeed);
    size_t outside = 0;
    size_t ran = 0;
    for (const RigExecOpTraceEntry &entry : region) {
        if (B.steps[entry.step].isHead) continue;
        if (entry.step < B.steps.size() && B.steps[entry.step].isSource) {
            continue;
        }
        ++ran;
        const bool foreign =
            _Names(entry.label, other) && !_Names(entry.label, own);
        const bool unreached =
            !regionSeed.empty() && !regionClosure.count(entry.step);
        if (foreign || unreached) {
            std::printf("    %s: region step %zu (%s) %s\n", what.c_str(),
                        entry.step, entry.label.c_str(),
                        foreign ? "names only the other limb"
                                : "is outside the forward closure");
            ++outside;
        }
    }
    for (const RigExecOpTraceEntry &entry : head) {
        ++ran;
        const bool foreign =
            _Names(entry.label, other) && !_Names(entry.label, own);
        const bool unreached =
            !headSeed.empty() && !headClosure.count(entry.step);
        if (foreign || unreached) {
            std::printf("    %s: head step %zu (%s) %s\n", what.c_str(),
                        entry.step, entry.label.c_str(),
                        foreign ? "names only the other limb"
                                : "is outside the forward closure");
            ++outside;
        }
    }
    CHECK(ran > 0);
    CHECK(outside == 0);
}

// The region steps (sources aside) and head steps the last generation ran.
size_t
_UpstreamWork(const _UpstreamImaging &imaging)
{
    const RigExecRigEvaluator *evaluator = imaging.Evaluator();
    const RigExecBakedProgram *program = imaging.Program();
    if (!evaluator || !program) {
        return 0;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    size_t work = ExecutedHeads(B).size();
    for (const RigExecOpTraceEntry &entry : evaluator->GetLastOpTrace()) {
        if (entry.step >= B.steps.size() || !B.steps[entry.step].isSource) {
            ++work;
        }
    }
    return work;
}

// A pose minus what a generation BUILT and minus its diagnostics, for
// comparing a frozen job with an evaluator.
size_t
_UpstreamPoseMismatches(const RigExecRigPose &reference,
                        const RigExecRigPose &pose)
{
    RigExecRigPose a = reference, b = pose;
    for (RigExecRigPose *p : {&a, &b}) {
        p->executedOpCount = 0;
        p->diagnostics.clear();
    }
    RigExecRigPose diff;
    RigExecComparePoses(a, b, &diff);
    for (size_t i = 0; i < diff.diagnostics.size() && i < 6; ++i) {
        std::printf("    %s\n", diff.diagnostics[i].c_str());
    }
    return diff.comparisonMismatches;
}

// Step 5: a job from a snapshot frozen now, sampled with the bridge's
// upstream values, equals a baked evaluator over a stage authoring them,
// and runs nothing outside live's cone of the last generation.
void
_CheckUpstreamFrozen(const _UpstreamImaging &imaging,
                     const std::string &stagePath,
                     const std::vector<_UpstreamAuthored> &edits, double t,
                     const std::string &what)
{
    const RigExecRigEvaluator *evaluator = imaging.Evaluator();
    const RigExecBakedProgram *program = imaging.Program();
    if (!evaluator || !program) {
        return;
    }
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(*evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("    %s: freeze refused: %s\n", what.c_str(),
                    error.c_str());
        return;
    }
    const std::vector<RigExecUpstreamValue> &upstream =
        imaging.context->GetBridge(imaging.rig)->GetUpstreamInputs();
    RigExecFrameInputs inputs;
    CHECK(RigExecSampleFrameInputs(*evaluator, UsdTimeCode(t), {}, upstream,
                                   &inputs, &error));
    CHECK(!inputs.HasChainResolvedInputs());
    CHECK(inputs.upstream.size() == upstream.size());
    RigExecFrozenEvalContext context;
    context.epochDigest = evaluator->GetBindingEpochDigest();
    context.slotCount = program->GetProviderCount();
    context.varyingInputCount = inputs.values.size();
    if (evaluator->GetPublishWeightFields()) {
        context.flags |= kRigExecFrozenPublishWeightFields;
    }
    if (evaluator->GetSolverGuidesEnabled()) {
        context.flags |= kRigExecFrozenSolverGuidesEnabled;
    }
    context.frozen = frozen.get();
    RigExecFrozenRunReport report;
    const RigExecRigPose job = RigExecEvaluateFrozen(
        context, inputs, RigExecMakeProductionStepRunner(), nullptr,
        SdfPath(), &report);
    CHECK(job.valid);
    CHECK(report.ran);
    // The reference: a baked evaluator over a stage authoring the values.
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    {
        UsdEditContext editContext(stage, stage->GetSessionLayer());
        for (const _UpstreamAuthored &edit : edits) {
            CHECK(stage->GetAttributeAtPath(edit.path).Set(edit.value));
        }
    }
    RigExecRigEvaluator reference(stage, imaging.rig);
    CHECK(reference.Compile());

    reference.SetSolverGuidesEnabled(evaluator->GetSolverGuidesEnabled());
    reference.SetPublishWeightFields(evaluator->GetPublishWeightFields());
    const size_t mismatches =
        _UpstreamPoseMismatches(reference.Evaluate(UsdTimeCode(t)), job);
    if (mismatches != 0) {
        std::printf("    %s: frozen job differs from the authored stage\n",
                    what.c_str());
    }
    CHECK(mismatches == 0);
    // The job's cone: no region step outside the clusters live ran (a
    // snapshot frozen after live's run holds the values, so it runs what
    // the job's own history moved, which is nothing beyond live's).
    std::set<int> clusters;
    for (const RigExecOpTraceEntry &entry : evaluator->GetLastOpTrace()) {
        clusters.insert(entry.cluster);
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    size_t outside = 0;
    for (const RigExecOpTraceEntry &entry : report.region) {
        if (entry.step < B.steps.size() && B.steps[entry.step].isSource) {
            continue;
        }
        if (!clusters.count(entry.cluster)) {
            ++outside;
        }
    }
    CHECK(outside == 0);
}

bool
_WarmRange(_UpstreamImaging &imaging, const std::vector<double> &frames)
{
    for (size_t tick = 0; tick < frames.size() + 6; ++tick) {
        bool allCached = true;
        for (const RigExecWarmFrameState state :
             imaging.context->GetFrameStates(imaging.rig, frames)) {
            allCached = allCached && state == RigExecWarmFrameState::Cached;
        }
        if (allCached) {
            return true;
        }
        imaging.context->OnIdle();
        imaging.context->WaitUntilBackgroundIdle();
    }
    bool allCached = true;
    for (const RigExecWarmFrameState state :
         imaging.context->GetFrameStates(imaging.rig, frames)) {
        allCached = allCached && state == RigExecWarmFrameState::Cached;
    }
    return allCached;
}

size_t
_ScopeCount(RigExecProfiler *profiler, const char *name)
{
    if (!profiler) {
        return 0;
    }
    for (const RigExecProfileSummaryRow &row : profiler->Summarize()) {
        if (row.name == name) {
            return row.count;
        }
    }
    return 0;
}

}  // namespace

// Steps 1-5 and 7 for one value, at time 1, with the frame cache off.
static void
_UpstreamValueSteps(const std::string &fixture, const SdfPath &attribute,
                    const VtValue &value,
                    const std::vector<std::string> &own,
                    const std::vector<std::string> &other,
                    const std::string &regionSeed,
                    const std::string &headSeed, const std::string &what)
{
    std::printf("  upstream steps: %s\n", what.c_str());
    std::unique_ptr<_UpstreamImaging> imaging =
        _OpenUpstreamImaging(fixture, kUpLimbsRig, 1.0);
    if (!imaging) {
        return;
    }
    const _UpstreamPose authored = imaging->Pose();
    CHECK(!authored.Empty());
    CHECK(_SameUpstreamPose(
        _UpstreamReference(fixture, kUpLimbsRig, {}, 1.0), authored,
        what + ": authored"));
    const size_t before = imaging->Evaluations();

    // 1. The Set evaluates once.
    imaging->upstream->Set(attribute, value);
    CHECK(imaging->Evaluations() == before + 1);
    CHECK(imaging->context->GetUpstreamTable(kUpLimbsRig));

    // 2. Live follows: equal to a stage authoring the value; the other limb
    // is unchanged.
    const _UpstreamPose standing = imaging->Pose();
    CHECK(_SameUpstreamPose(
        _UpstreamReference(fixture, kUpLimbsRig, {{attribute, value, {}}},
                           1.0),
        standing, what + ": standing"));
    CHECK(!_SameUpstreamPose(authored, standing, what, false));
    CHECK(_SameUpstreamPose(_UpstreamPart(authored, other),
                            _UpstreamPart(standing, other),
                            what + ": other limb"));
    CHECK(imaging->Evaluator()->GetUpstreamInputPaths() ==
          std::vector<SdfPath>{attribute});

    // 3. Only the forward cone re-ran.
    _CheckUpstreamCone(*imaging, own, other, regionSeed, headSeed,
                       what + ": placed");

    // 4. No change, no work: a second identical Set evaluates nothing, and
    // a generation forced with the value standing runs no step.
    imaging->upstream->Set(attribute, value);
    CHECK(imaging->Evaluations() == before + 1);
    if (imaging->Program()) {
        RigExecImagingBridge *bridge = imaging->context->GetBridge(kUpLimbsRig);
        CHECK(bridge->EvaluateAndPublish(UsdTimeCode(1.0)));
        CHECK(_UpstreamWork(*imaging) == 0);
    }

    // 5. Frozen follows.
    _CheckUpstreamFrozen(*imaging, fixture, {{attribute, value, {}}}, 1.0,
                         what);

    // 7. Remove: back to the authored pose, through the same cone.
    imaging->upstream->Clear(attribute);
    CHECK(_SameUpstreamPose(authored, imaging->Pose(), what + ": lifted"));
    CHECK(imaging->Evaluator()->GetUpstreamInputPaths().empty());
    CHECK(!imaging->context->GetUpstreamTable(kUpLimbsRig));
    _CheckUpstreamCone(*imaging, own, other, regionSeed, headSeed,
                       what + ": lifted");
}

// The geometry a playback generation owns: points and driven transforms
// (playback draws no guides).
static _UpstreamPose
_UpstreamGeometry(_UpstreamPose pose)
{
    pose.guides.clear();
    return pose;
}

// Step 6 playback integration: a file of the authored stage receives upstream values.
//  - Both keys are listed inputs whose defaults are the authored values.
//  - Execute with no input set equals the authored pose.
//  - SetSampledInputAt of both values, then Execute, equals live with both
//    standing.
//  - Through imaging, a playback session over a rigExec:asset rig follows
//    the same upstream->Set: equal to a stage authoring the value, lifted
//    by Clear, a dropped key reported with live's line.
static void
_UpstreamPlaybackSteps(const std::string &fixture)
{
    std::printf("  upstream step 6 (.rigexec)\n");
    const double t = 1.0;
    const GfMatrix4d space = _UpTranslate(1, 0, 10);
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, kUpLimbsRig);
    CHECK(evaluator.Compile());

    const RigExecRigPose authored = evaluator.Evaluate(UsdTimeCode(t));
    std::vector<uint8_t> bytes;
    std::string error;
    const bool baked = RigExecTestBakeAt(evaluator, t, &bytes, &error);
    if (!baked) {
        std::printf("    bake refused: %s\n", error.c_str());
    }
    CHECK(baked);
    if (!baked) {
        return;
    }
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        return;
    }
    size_t rz = 0;
    size_t spaceInput = 0;
    CHECK(reader->FindInput(kUpA0Rz.GetString(), &rz));
    CHECK(reader->FindInput(kUpSpace.GetString(), &spaceInput));
    const RigExecRuntimeInputInfo &rzInfo = reader->GetInputInfo(rz);
    const RigExecRuntimeInputInfo &spaceInfo =
        reader->GetInputInfo(spaceInput);
    CHECK(rzInfo.type == RrInputTag::Double && !rzInfo.animated);
    CHECK(rzInfo.defaultValue.f64 == 0.0);
    CHECK(spaceInfo.type == RrInputTag::Matrix4d && !spaceInfo.animated);
    CHECK(spaceInfo.defaultValue.matrix[3][2] == 10.0 &&
          spaceInfo.defaultValue.matrix[3][0] == 0.0);
    std::vector<std::string> diffs;
    CHECK(reader->Execute(&error));
    CHECK(RigExecCompareRuntimeOutputs(authored, *reader, &diffs));
    // Live with both values standing.
    evaluator.SetUpstreamInputs(
        {RigExecValueOverride{kUpA0Rz.GetPrimPath(), TfToken(),
                              kUpA0Rz.GetNameToken(), VtValue(30.0)},
         RigExecValueOverride{kUpSpace.GetPrimPath(), TfToken(),
                              kUpSpace.GetNameToken(), VtValue(space)}});
    const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(t));
    CHECK(evaluator.GetUpstreamInputPaths().size() == 2);
    RrInputValue value;
    CHECK(RigExecInputValueFrom(VtValue(30.0), RrInputTag::Double, &value));
    CHECK(reader->SetSampledInputAt(rz, value, &error));
    CHECK(RigExecInputValueFrom(VtValue(space), RrInputTag::Matrix4d,
                                &value));
    CHECK(reader->SetSampledInputAt(spaceInput, value, &error));
    CHECK(reader->Execute(&error));
    const bool same = RigExecCompareRuntimeOutputs(live, *reader, &diffs);
    for (const std::string &line : diffs) {
        std::printf("    %s\n", line.c_str());
    }
    CHECK(same);

    // Through imaging: the same fixture with rigExec:asset naming the bake.
    const std::filesystem::path binary =
        std::filesystem::temp_directory_path() /
        ("rigexec-upstream-step6-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()) +
         ".rigexec");
    {
        std::ofstream stream(binary, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(bytes.data()),
                     std::streamsize(bytes.size()));
    }
    {
        std::unique_ptr<_UpstreamImaging> imaging = _OpenUpstreamImaging(
            fixture, kUpLimbsRig, t, nullptr,
            [&binary](const UsdStageRefPtr &opened) {
                UsdEditContext context(opened, opened->GetSessionLayer());
                const UsdAttribute asset =
                    opened->GetPrimAtPath(kUpLimbsRig)
                        .CreateAttribute(TfToken("rigExec:asset"),
                                         SdfValueTypeNames->Asset);
                CHECK(asset.Set(SdfAssetPath(binary.generic_string())));
            });
        if (imaging) {
            RigExecBakedPlayback *playback =
                imaging->context->GetPlayback(kUpLimbsRig);
            CHECK(playback);
            CHECK(!imaging->context->GetBridge(kUpLimbsRig));
            const _UpstreamPose playAuthored = imaging->Pose();
            CHECK(!playAuthored.points.empty());
            CHECK(_SameUpstreamPose(
                _UpstreamGeometry(
                    _UpstreamReference(fixture, kUpLimbsRig, {}, t)),
                playAuthored, "playback: authored"));
            for (const auto &[attribute, value] :
                 std::vector<std::pair<SdfPath, VtValue>>{
                     {kUpA0Rz, VtValue(30.0)}, {kUpSpace, VtValue(space)}}) {
                const std::string what =
                    "playback " + attribute.GetString();
                const size_t before = imaging->Evaluations();
                imaging->upstream->Set(attribute, value);
                CHECK(imaging->Evaluations() == before + 1);
                CHECK(playback &&
                      playback->GetUpstreamInputPaths() ==
                          std::vector<SdfPath>{attribute});
                CHECK(_SameUpstreamPose(
                    _UpstreamGeometry(_UpstreamReference(
                        fixture, kUpLimbsRig, {{attribute, value, {}}}, t)),
                    imaging->Pose(), what + ": standing"));
                // A second identical Set hands nothing over.
                imaging->upstream->Set(attribute, value);
                CHECK(imaging->Evaluations() == before + 1);
                imaging->upstream->Clear(attribute);
                CHECK(playback && playback->GetUpstreamInputPaths().empty());
                CHECK(_SameUpstreamPose(playAuthored, imaging->Pose(),
                                        what + ": lifted"));
            }
            // Both at once, as the binary leg above.
            imaging->upstream->Set(kUpA0Rz, VtValue(30.0));
            imaging->upstream->Set(kUpSpace, VtValue(space));
            CHECK(_SameUpstreamPose(
                _UpstreamGeometry(_UpstreamReference(
                    fixture, kUpLimbsRig,
                    {{kUpA0Rz, VtValue(30.0), {}},
                     {kUpSpace, VtValue(space), {}}},
                    t)),
                imaging->Pose(), "playback: both standing"));
            imaging->upstream->Clear(kUpA0Rz);
            imaging->upstream->Clear(kUpSpace);
            // A dropped key: live's line, the authored pose.
            imaging->upstream->Set(kUpBRootRestSpace, VtValue(space));
            CHECK(playback && playback->GetUpstreamInputPaths().empty());
            CHECK(playback &&
                  playback->GetUpstreamDropLines() ==
                      std::vector<std::string>{
                          "upstream input " + kUpBRootRestSpace.GetString() +
                          ": the attribute is connected; ignored"});
            CHECK(_SameUpstreamPose(playAuthored, imaging->Pose(),
                                    "playback: dropped"));
        }
    }
    std::error_code removed;
    std::filesystem::remove(binary, removed);
}

// Step 6's full bake form: standing inputs remain integration values.
static void
TestABakeExposesUpstreamAsInputs(const std::string &fixture)
{
    const auto stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, kUpLimbsRig);

    CHECK(evaluator.Compile());
    const UsdTimeCode time(1.0);
    const auto authored = evaluator.Evaluate(time);
    RigExecBakeOpts opts;
    opts.time = 1.0;
    RigExecBakeResult baseline, result;
    std::string error;
    CHECK(RigExecBakeToBinary(evaluator, opts, &baseline, &error));
    const GfMatrix4d space = _UpTranslate(1, 0, 10);
    const std::vector<RigExecValueOverride> inputs = {
        {kUpA0Rz.GetPrimPath(), TfToken(), kUpA0Rz.GetNameToken(), VtValue(30.0)},
        {kUpSpace.GetPrimPath(), TfToken(), kUpSpace.GetNameToken(), VtValue(space)}};
    evaluator.SetUpstreamInputs(inputs);
    const auto standing = evaluator.Evaluate(time);
    CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(result.bytes == baseline.bytes);
    std::vector<std::string> names = {kUpA0Rz.GetString(), kUpSpace.GetString()};
    std::sort(names.begin(), names.end());
    CHECK(result.upstreamInputs == names);
    CHECK(evaluator.GetUpstreamInputs() == inputs);
    const auto restored = evaluator.Evaluate(time);
    auto reader = RigExecRuntimeReader::Open(result.bytes.data(),
                                             result.bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        return;
    }
    size_t rz = 0, spaceInput = 0;
    CHECK(reader->FindInput(kUpA0Rz.GetString(), &rz));
    CHECK(reader->FindInput(kUpSpace.GetString(), &spaceInput));
    CHECK(reader->GetInputInfo(rz).defaultValue.f64 == 0.0);
    const auto &m = reader->GetInputInfo(spaceInput).defaultValue.matrix;
    CHECK(m[3][0] == 0.0 && m[3][2] == 10.0);
    CHECK(reader->Execute(&error));
    std::vector<std::string> diffs;
    CHECK(RigExecCompareRuntimeOutputs(authored, *reader, &diffs));
    RrInputValue value;
    CHECK(RigExecInputValueFrom(VtValue(30.0), RrInputTag::Double, &value));
    CHECK(reader->SetSampledInputAt(rz, value, &error));
    CHECK(RigExecInputValueFrom(VtValue(space), RrInputTag::Matrix4d, &value));
    CHECK(reader->SetSampledInputAt(spaceInput, value, &error));
    CHECK(reader->Execute(&error));
    CHECK(RigExecCompareRuntimeOutputs(standing, *reader, &diffs));
    CHECK(RigExecCompareRuntimeOutputs(restored, *reader, &diffs));
}

// D2: actual imaging playback, with cones derived from the original export.
static void
TestTheRuntimeUpstreamCone(const std::string &fixture)
{
    const auto stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, kUpLimbsRig);
    CHECK(evaluator.Compile());

    evaluator.Evaluate(UsdTimeCode(1.0));
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
    std::unique_ptr<fb::RigExecWireFile> file;
    CHECK(RigExecFormatOpen(bytes.data(), bytes.size(), &file, &error));
    CHECK(file);
    if (!file) return;
    const bool fine = file->clustering->grainUs == 0.0;
    if (TfGetenv("RIGEXEC_BAKED_GRAIN_US", "") == "0") {
        CHECK(fine);
    }
    if (fine) {
        CHECK(file->clustering->clusters.size() == file->steps.size());
        for (size_t c = 0; c < file->clustering->clusters.size(); ++c) {
            const auto &members = file->clustering->clusters[c].members;
            CHECK(members.size() == 1);
            for (int index : members) {
                CHECK(index >= 0 && size_t(index) < file->steps.size());
                if (index >= 0 && size_t(index) < file->steps.size()) {
                    CHECK(file->steps[size_t(index)].cluster == int(c));
                    CHECK(file->clustering->clusterOf[size_t(index)] == int(c));
                }
            }
        }
    }
    // Canonical producer order can interleave heads and region operations.
    // Serialization must retain every actual head's identity and body mapping.
    const auto *nativeProgram = evaluator.GetBakedProgram();
    CHECK(nativeProgram);
    if (!nativeProgram) return;
    const auto &native = nativeProgram->GetStepGraph();
    CHECK(file->steps.size() == native.steps.size());
    CHECK(file->commonGraph);
    if (!file->commonGraph || file->steps.size() != native.steps.size()) return;
    CHECK(file->commonGraph->ops.size() == file->steps.size());
    if (file->commonGraph->ops.size() != file->steps.size()) return;
    std::set<size_t> nativeHeads, exportedHeads;
    for (size_t i = 0; i < file->steps.size(); ++i) {
        const auto &step = file->steps[i];
        const auto &body = native.steps[i];
        if (body.isHead) nativeHeads.insert(i);
        if (step.isHead) {
            exportedHeads.insert(i);
            CHECK(!step.isSource);
            CHECK(file->commonGraph->ops[i].originalIndex == i);
            CHECK(uint32_t(step.kind) == uint32_t(body.kind));
            CHECK(step.object == body.object);
            CHECK(step.part == body.part);
        }
    }
    CHECK(!nativeHeads.empty());
    CHECK(exportedHeads == nativeHeads);
    // The actual static fixture owes no volatile ordinary work.
    CHECK(std::all_of(file->cones->always->words.begin(), file->cones->always->words.end(),
        [](uint64_t word){return word == 0;}));
    const auto binary = std::filesystem::temp_directory_path() /
        ("rigexec-d2-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".rigexec");
    { std::ofstream out(binary, std::ios::binary); out.write(reinterpret_cast<const char*>(bytes.data()),
        std::streamsize(bytes.size())); }
    {
        auto imaging = _OpenUpstreamImaging(fixture, kUpLimbsRig, 1.0, nullptr, [&](const UsdStageRefPtr &opened) {
            UsdEditContext context(opened, opened->GetSessionLayer());
            CHECK(opened->GetPrimAtPath(kUpLimbsRig).CreateAttribute(TfToken("rigExec:asset"),
                SdfValueTypeNames->Asset).Set(SdfAssetPath(binary.generic_string())));
        });
        CHECK(imaging);
        if (imaging) {
            auto *playback = imaging->context->GetPlayback(kUpLimbsRig);
            CHECK(playback && !imaging->context->GetBridge(kUpLimbsRig));
            if (playback) {
                const auto *reader = playback->GetReaderForTesting();
                CHECK(reader);
                if (!reader) return;
                const auto authored = imaging->Pose();
                const auto compareNativeGraph = [&](const std::vector<_UpstreamAuthored> &edits) {
                    { // Native graph versus independently serialized runtime outputs.
                        const auto reference = UsdStage::Open(fixture);
                        CHECK(reference);
                        UsdEditContext editContext(reference, reference->GetSessionLayer());
                        for (const auto &edit:edits) CHECK(reference->GetAttributeAtPath(edit.path).Set(edit.value));
                        RigExecRigEvaluator live(reference, kUpLimbsRig);

                        CHECK(live.Compile());
                        const auto pose = live.Evaluate(UsdTimeCode(1.0));
                        std::vector<std::string> diffs;
                        CHECK(RigExecCompareRuntimeOutputs(pose, *reader, &diffs));
                    }
                };
                compareNativeGraph({});
                const auto coneFor = [&](const SdfPath &path) {
                    size_t slot = 0;
                    CHECK(reader->FindInput(path.GetString(), &slot));
                    CHECK(slot<file->listedInputs && (file->inputs[slot].flags() & 4));
                    const auto &info = reader->GetInputInfo(slot);
                    CHECK(!info.animated);
                    if (path == kUpA0Rz) CHECK(info.type == RrInputTag::Double && info.defaultValue.f64 == 0.0);
                    else CHECK(info.type == RrInputTag::Matrix4d &&
                               info.defaultValue.matrix[3][0] == 0.0 &&
                               info.defaultValue.matrix[3][2] == 10.0);
                    std::vector<char> cone(file->steps.size(), 0);
                    for (size_t i = 0;i<file->steps.size();++i) {
                        const auto &step = file->steps[i];
                        if (std::find(step.headInputSlots.begin(), step.headInputSlots.end(),
                            uint32_t(slot)) != step.headInputSlots.end()) cone[i] = 1;
                        for (const auto &read : step.headInputReads) {
                            const auto reaches = [&](const auto &candidates) {
                                return std::any_of(candidates.begin(), candidates.end(),
                                    [&](const auto &candidate) { return candidate.slot == slot; });
                            };
                            if (std::find(read.walk.begin(), read.walk.end(), uint32_t(slot)) != read.walk.end() ||
                                read.rawFallbackSlot == int32_t(slot) ||
                                reaches(read.propertyCandidates) || reaches(read.doubleCandidates)) {
                                cone[i] = 1;
                            }
                        }
                        // A scalar binding's flat index contains 11 channels
                        // per provider. AvarInputs consumes the raw read and
                        // publishes that provider's typed Avars value; its
                        // successors then cover Compose and geometry readers.
                        for (const auto &avar : file->pose->avarBindings) {
                            if (std::find(avar.read->walk.begin(), avar.read->walk.end(),
                                          uint32_t(slot)) == avar.read->walk.end()) continue;
                            const uint32_t provider = avar.flat / 11;
                            if (step.kind != fb::StepKind::AvarInputs ||
                                step.object != int32_t(provider)) continue;
                            CHECK(std::any_of(step.writes.begin(), step.writes.end(),
                                [&](const auto &write) {
                                    return write.domain() == fb::SlotDomain::Avars &&
                                           write.begin() == provider && write.end() == provider + 1;
                                }));
                            cone[i] = 1;
                        }
                    }
                    for (const auto &avar : file->pose->avarBindings) {
                        if (std::find(avar.read->walk.begin(), avar.read->walk.end(),
                                      uint32_t(slot)) == avar.read->walk.end()) continue;
                        const uint32_t provider = avar.flat / 11;
                        CHECK(std::count_if(file->steps.begin(), file->steps.end(),
                            [&](const auto &step) {
                                return step.kind == fb::StepKind::AvarInputs &&
                                       step.object == int32_t(provider);
                            }) == 1);
                    }
                    // Provider raw sampling enters through a typed SpaceLeaf,
                    // rather than a head input memo or an avar binding.
                    CHECK(file->providerProgram);
                    bool providerSource = false;
                    if (file->providerProgram) {
                        for (size_t leaf = 0; leaf < file->providerProgram->sampled.size(); ++leaf) {
                            if (file->providerProgram->sampled[leaf].inputSlot != int32_t(slot)) continue;
                            providerSource = true;
                            size_t readers = 0;
                            for (size_t i = 0; i < file->steps.size(); ++i) {
                                const auto &step = file->steps[i];
                                for (const auto &read : step.reads) {
                                    if (read.domain() != fb::SlotDomain::SpaceLeaf ||
                                        read.begin() > leaf || leaf >= read.end()) continue;
                                    CHECK(step.kind == fb::StepKind::SpaceExpression);
                                    CHECK(step.part == 2 && step.object == int32_t(leaf));
                                    cone[i] = 1;
                                    ++readers;
                                }
                            }
                            CHECK(readers == 1);
                        }
                    }
                    // Build prunes an avar's provider leaf, which no step
                    // reads: the ladder and AvarInputs bind avars directly.
                    CHECK(providerSource == (path != kUpA0Rz));
                    CHECK(std::find(cone.begin(), cone.end(), char(1)) != cone.end());
                    for (size_t n = 0; n < cone.size(); ++n) {
                        for (size_t i = 0; i < cone.size(); ++i) {
                            if (!cone[i]) continue;
                            for (int next : file->steps[i].succs) {
                                cone[size_t(next)] = 1;
                            }
                        }
                    }
                    return cone;
                };
                const auto a = coneFor(kUpA0Rz), b = coneFor(kUpSpace);
                const auto checkTrace = [&](const std::vector<char> &cone, bool idle) {
                    size_t body = 0;
                    for (int32_t index:reader->GetLastRunTraceForTesting()) {
                        CHECK(index>=0 && size_t(index)<file->steps.size());
                        if (index<0 || size_t(index)>=file->steps.size()) continue;
                        const auto &step = file->steps[size_t(index)];
                        if (idle) CHECK(!step.isHead && step.isSource);
                        else if (!step.isSource) { CHECK(cone[size_t(index)]); ++body; }
                    }
                    if (!idle) CHECK(body>0);
                };
                const GfMatrix4d space = _UpTranslate(1, 0, 10);
                for (const auto &key:std::vector<std::pair<SdfPath, VtValue>>{{kUpA0Rz, VtValue(30.0)}, {kUpSpace,
                    VtValue(space)}}) {
                    const auto &cone = key.first == kUpA0Rz?a:b;
                    const auto &other = key.first == kUpA0Rz?kUpLimbB:kUpLimbA;
                    imaging->upstream->Set(key.first, key.second);
                    checkTrace(cone, false);
                    for (int32_t index : reader->GetLastRunTraceForTesting()) {
                        if (index < 0 || size_t(index) >= file->steps.size() ||
                            file->steps[size_t(index)].isSource) continue;
                        const std::string label = reader->GetStepLabelForTesting(size_t(index));
                        const auto &own = key.first == kUpA0Rz ? kUpLimbA : kUpLimbB;
                        const bool foreign = _Names(label, other) && !_Names(label, own);
                        if (foreign) {
                            std::printf("    D2 %s %s: %s names only the other limb\n",
                                        fine ? "fine" : "default",
                                        key.first.GetText(), label.c_str());
                        }
                        CHECK(!foreign);
                    }
                    const auto standing = imaging->Pose();
                    compareNativeGraph({{key.first, key.second, {}}});
                    CHECK(_SameUpstreamPose(_UpstreamGeometry(_UpstreamReference(fixture, kUpLimbsRig,
                        {{key.first, key.second, {}}}, 1.0)), standing, "D2 placed"));
                    CHECK(!_SameUpstreamPose(authored, standing, "D2 nonvacuity", false));
                    CHECK(_SameUpstreamPose(_UpstreamPart(authored, other), _UpstreamPart(standing, other),
                        "D2 opposite limb"));
                    const size_t count = imaging->Evaluations();
                    imaging->upstream->Set(key.first, key.second);
                    CHECK(imaging->Evaluations() == count);
                    CHECK(playback->EvaluateAndPublishResult(UsdTimeCode(1.0)).ok);
                    checkTrace(cone, true);
                    compareNativeGraph({{key.first, key.second, {}}});
                    CHECK(_SameUpstreamPose(standing, imaging->Pose(),
                                            "D2 held standing"));
                    imaging->upstream->Clear(key.first);
                    checkTrace(cone, false);
                    CHECK(_SameUpstreamPose(authored, imaging->Pose(), "D2 lifted"));
                    compareNativeGraph({});
                    CHECK(playback->EvaluateAndPublishResult(UsdTimeCode(1.0)).ok);
                    checkTrace(cone, true);
                }
                std::vector<char> both = a;
                for (size_t i = 0;i<both.size();++i) both[i] = both[i] || b[i];
                imaging->upstream->Set(kUpA0Rz, VtValue(30.0));
                imaging->upstream->Set(kUpSpace, VtValue(space));
                checkTrace(both, false);
                compareNativeGraph({{kUpA0Rz, VtValue(30.0), {}}, {kUpSpace, VtValue(space), {}}});
                CHECK(_SameUpstreamPose(_UpstreamGeometry(_UpstreamReference(fixture, kUpLimbsRig, {{kUpA0Rz,
                    VtValue(30.0), {}}, {kUpSpace, VtValue(space), {}}}, 1.0)), imaging->Pose(), "D2 both"));
                CHECK(playback->EvaluateAndPublishResult(UsdTimeCode(1.0)).ok); checkTrace(both, true);
                compareNativeGraph({{kUpA0Rz, VtValue(30.0), {}},
                              {kUpSpace, VtValue(space), {}}});
                imaging->upstream->Clear(kUpA0Rz); checkTrace(both, false);
                imaging->upstream->Clear(kUpSpace); checkTrace(both, false);
                CHECK(_SameUpstreamPose(authored, imaging->Pose(), "D2 both lifted"));
            }
        }
    }
    std::error_code removed; std::filesystem::remove(binary, removed);
    std::printf("  runtime upstream cone and no-change-no-work (%s): checked\n",
                fine ? "fine" : "default");
}

// Admission through imaging: each key is ignored, the pose stays authored.
static void
_UpstreamAdmissionNegatives(const std::string &fixture)
{
    std::printf("  upstream admission negatives\n");
    std::unique_ptr<_UpstreamImaging> imaging =
        _OpenUpstreamImaging(fixture, kUpLimbsRig, 1.0);
    if (!imaging) {
        return;
    }
    const _UpstreamPose authored = imaging->Pose();
    // A value on BRoot's connected rest:space itself, and a float on a
    // double avar.
    imaging->upstream->Set(kUpBRootRestSpace,
                           VtValue(_UpTranslate(1, 0, 10)));
    imaging->upstream->Set(kUpA0Rz, VtValue(30.0f));
    CHECK(imaging->Evaluator()->HasUpstreamInputs());
    CHECK(imaging->Evaluator()->GetUpstreamInputPaths().empty());
    CHECK(_SameUpstreamPose(authored, imaging->Pose(),
                            "admission negatives"));
}

// Sources present at population are noted from the add, and a second
// chain bound to the same context supplies the values when it delivers the
// trigger, with a diagnostic for a key the chains publish differently (Q4).
namespace {
class _UpstreamConflictCounter : public TfDiagnosticMgr::Delegate {
public:
    void IssueStatus(TfStatus const &) override {}
    void IssueWarning(TfWarning const &warning) override
    {
        if (warning.GetCommentary().find("two scene-index chains") !=
            std::string::npos) {
            count.fetch_add(1);
        }
    }
    void IssueError(TfError const &) override {}
    void IssueFatalError(TfCallContext const &, std::string const &) override
    {
    }
    std::atomic<size_t> count{0};
};
}  // namespace

static void
_UpstreamAddsAndChains(const std::string &fixture)
{
    std::printf("  upstream adds and chains\n");
    std::unique_ptr<_UpstreamImaging> first = _OpenUpstreamImaging(
        fixture, kUpLimbsRig, 1.0, [](_UpstreamInputsSceneIndex &upstream) {
            upstream.Set(kUpA0Rz, VtValue(30.0));
        });
    if (!first) {
        return;
    }
    const _UpstreamPose at30 = _UpstreamReference(
        fixture, kUpLimbsRig, {{kUpA0Rz, VtValue(30.0), {}}}, 1.0);
    CHECK(_SameUpstreamPose(at30, first->Pose(), "seeded before population"));

    // A second chain over the same stage binds to the same context.
    TfRefPtr<_UpstreamInputsSceneIndex> secondUpstream;
    UsdImagingCreateSceneIndicesInfo info;
    info.stage = first->stage;
    info.overridesSceneIndexCallback =
        [&secondUpstream](const HdSceneIndexBaseRefPtr &input)
        -> HdSceneIndexBaseRefPtr {
        secondUpstream = _UpstreamInputsSceneIndex::New(input);
        return secondUpstream;
    };
    const UsdImagingSceneIndices second = UsdImagingCreateSceneIndices(info);
    CHECK(RigExecImagingRegistry::ForStage(first->stage, false) ==
          first->context);
    _UpstreamConflictCounter conflicts;
    TfDiagnosticMgr::GetInstance().AddDelegate(&conflicts);
    secondUpstream->SetTime(1.0);
    secondUpstream->Set(kUpA0Rz, VtValue(10.0));
    CHECK(conflicts.count.load() >= 1);
    TfDiagnosticMgr::GetInstance().RemoveDelegate(&conflicts);
    const _UpstreamPose at10 = _UpstreamReference(
        fixture, kUpLimbsRig, {{kUpA0Rz, VtValue(10.0), {}}}, 1.0);
    CHECK(_SameUpstreamPose(at10, first->Pose(), "second chain supplies"));
    // The chain that delivers the trigger supplies the values.
    first->SetTime(2.0);
    CHECK(_SameUpstreamPose(
        _UpstreamReference(fixture, kUpLimbsRig,
                           {{kUpA0Rz, VtValue(30.0), {}}}, 2.0),
        first->Pose(), "first chain's trigger"));
    secondUpstream->SetTime(3.0);
    second.stageSceneIndex->SetTime(UsdTimeCode(3.0));
    CHECK(_SameUpstreamPose(
        _UpstreamReference(fixture, kUpLimbsRig,
                           {{kUpA0Rz, VtValue(10.0), {}}}, 3.0),
        first->Pose(), "second chain's trigger"));
}

// A chain's last anchor does not change the signal it publishes. Compare
// both signals at the incoming trigger, not their independently saved T0.
static void
_UpstreamChainConflictTimes(const std::string &fixture)
{
    std::printf("  upstream chain conflict times\n");
    const std::map<double, VtValue> curve = {
        {1.0, VtValue(1.0)}, {3.0, VtValue(3.0)}};
    std::unique_ptr<_UpstreamImaging> first = _OpenUpstreamImaging(
        fixture, kUpLimbsRig, 1.0, [&](auto &upstream) {
            upstream.SetSparse(kUpA0Rz, curve);
        });
    if (!first) {
        return;
    }
    TfRefPtr<_UpstreamInputsSceneIndex> secondUpstream;
    UsdImagingCreateSceneIndicesInfo info;
    info.stage = first->stage;
    info.overridesSceneIndexCallback =
        [&secondUpstream](const HdSceneIndexBaseRefPtr &input)
        -> HdSceneIndexBaseRefPtr {
        secondUpstream = _UpstreamInputsSceneIndex::New(input);
        return secondUpstream;
    };
    const UsdImagingSceneIndices second = UsdImagingCreateSceneIndices(info);
    secondUpstream->SetTime(1.0);
    second.stageSceneIndex->SetTime(UsdTimeCode(1.0));
    secondUpstream->SetSparse(kUpA0Rz, curve);
    const size_t firstCalls = first->upstream->calls->getValue.load();
    const size_t secondCalls = secondUpstream->calls->getValue.load();
    _UpstreamConflictCounter conflicts;
    TfDiagnosticMgr::GetInstance().AddDelegate(&conflicts);
    secondUpstream->SetTime(2.0);
    second.stageSceneIndex->SetTime(UsdTimeCode(2.0));
    CHECK(conflicts.count.load() == 0);
    CHECK(_SameUpstreamPose(
        _UpstreamReference(fixture, kUpLimbsRig,
                           {{kUpA0Rz, VtValue(2.0), {}}}, 2.0),
        first->Pose(), "equal curves at second trigger"));
    first->SetTime(3.0);
    CHECK(conflicts.count.load() == 0);
    CHECK(_SameUpstreamPose(
        _UpstreamReference(fixture, kUpLimbsRig,
                           {{kUpA0Rz, VtValue(3.0), {}}}, 3.0),
        first->Pose(), "equal curves at first trigger"));
    TfDiagnosticMgr::GetInstance().RemoveDelegate(&conflicts);
    CHECK(first->upstream->calls->getValue.load() == firstCalls);
    CHECK(secondUpstream->calls->getValue.load() == secondCalls);

    // A is anchored at 1 with A(1)=1. B's new value at 2 is also 1,
    // but A(2)=2: comparing stored atT0 values would miss the conflict.
    secondUpstream->SetSparse(kUpA0Rz, {
        {1.0, VtValue(0.0)}, {3.0, VtValue(2.0)}});
    secondUpstream->SetTime(1.0);
    second.stageSceneIndex->SetTime(UsdTimeCode(1.0));
    first->SetTime(1.0);
    const size_t changedCalls = secondUpstream->calls->getValue.load();
    conflicts.count.store(0);
    TfDiagnosticMgr::GetInstance().AddDelegate(&conflicts);
    secondUpstream->SetTime(2.0);
    second.stageSceneIndex->SetTime(UsdTimeCode(2.0));
    CHECK(conflicts.count.load() >= 1);
    TfDiagnosticMgr::GetInstance().RemoveDelegate(&conflicts);
    CHECK(_SameUpstreamPose(
        _UpstreamReference(fixture, kUpLimbsRig,
                           {{kUpA0Rz, VtValue(1.0), {}}}, 2.0),
        first->Pose(), "different incoming curve wins"));
    CHECK(first->upstream->calls->getValue.load() == firstCalls);
    CHECK(secondUpstream->calls->getValue.load() == changedCalls);
}

// A source on a prim added after activation, outside every root the rig
// reads, reaches the rig once a later connection brings that prim into its
// read roots: an add is noted wherever it lands.
static void
_UpstreamAddedOutsideRoots(const std::string &fixture)
{
    std::printf("  upstream source added outside the read roots\n");
    std::unique_ptr<_UpstreamImaging> imaging =
        _OpenUpstreamImaging(fixture, kUpLimbsRig, 1.0);
    if (!imaging) {
        return;
    }
    const SdfPath driver("/Driver");
    const SdfPath driverSpace = driver.AppendProperty(TfToken("inputs:space"));
    const GfMatrix4d pulled = _UpTranslate(1, 0, 10);
    // The prim, authoring the value BRoot's rest already has, and then the
    // connection, each in the session layer.
    const auto define = [&](const UsdStageRefPtr &stage,
                            const GfMatrix4d &value) {
        UsdEditContext context(stage, stage->GetSessionLayer());
        const UsdPrim prim = stage->DefinePrim(driver, TfToken("Scope"));
        CHECK(prim.CreateAttribute(TfToken("inputs:space"),
                                   SdfValueTypeNames->Matrix4d, true)
                  .Set(value));
    };
    const auto connect = [&](const UsdStageRefPtr &stage) {
        UsdEditContext context(stage, stage->GetSessionLayer());
        CHECK(stage->GetAttributeAtPath(kUpBRootRestSpace)
                  .SetConnections({driverSpace}));
    };
    const auto reference = [&](const GfMatrix4d &value) {
        const UsdStageRefPtr stage = UsdStage::Open(fixture);
        define(stage, value);
        connect(stage);
        RigExecImagingBridge bridge(stage, kUpLimbsRig);
        std::vector<std::string> errors;
        CHECK(bridge.Compile(&errors));
        CHECK(bridge.EvaluateAndPublishResult(UsdTimeCode(2.0)).ok);
        return _CaptureUpstreamPose(bridge.GetStore()->Get());
    };
    imaging->upstream->Set(driverSpace, VtValue(pulled));
    define(imaging->stage, _UpTranslate(0, 0, 10));
    imaging->indices.stageSceneIndex->ApplyPendingUpdates();
    connect(imaging->stage);
    imaging->indices.stageSceneIndex->ApplyPendingUpdates();
    imaging->SetTime(2.0);
    const _UpstreamPose pose = imaging->Pose();
    CHECK(!_SameUpstreamPose(reference(_UpTranslate(0, 0, 10)), pose,
                             "outside roots: authored", false));
    CHECK(_SameUpstreamPose(reference(pulled), pose,
                            "outside roots: pulled"));
}

// A connected posed space is an ordinary bound input in the sole graph.
// This checks admission and source identity independently of cache policy.
static void
_CheckConnectedPosedSourceGraph(const _UpstreamImaging &imaging)
{
    const RigExecBakedProgram *program = imaging.Program();
    CHECK(program != nullptr);
    if (!program) return;
    const auto &B = program->GetStepGraph();
    std::string error;
    CHECK(RigExecBakedValidateStepGraph(B, &error));
    const SdfPath owner("/LimbsAsset/Rig/Controls/BRoot");
    const auto found = B.index.find(owner);
    CHECK(found != B.index.end());
    if (found == B.index.end()) return;
    CHECK(found->second >= 0 && size_t(found->second) < B.ladders.size());
    if (found->second < 0 || size_t(found->second) >= B.ladders.size()) return;
    CHECK(B.ladders[size_t(found->second)].posedSpaceConnected);
    CHECK(B.providerProgram.rawInputs.count(kUpSpace) == 1);
    SdfPathVector connections;
    CHECK(imaging.stage->GetAttributeAtPath(
        owner.AppendProperty(TfToken("posed:space"))).GetConnections(&connections));
    CHECK(connections == SdfPathVector{kUpSpace});
}

// Step 8: the frame cache on, at time 1. Never serves the other value's
// pose: the sparse path misses on an upstream difference, and the D7
// pose-only key folds the values.
static void
_UpstreamCacheSteps(const std::string &fixture, const SdfPath &attribute,
                    const VtValue &value, bool sparse, const std::string &what)
{
    std::printf("  upstream cache: %s\n", what.c_str());
    std::unique_ptr<_UpstreamImaging> imaging =
        _OpenUpstreamImaging(fixture, kUpLimbsRig, 1.0);
    if (!imaging) {
        return;
    }
    // A retained base at time 1 (the warm index's row), so the proof
    // mismatch a Set causes falls into the sparse plan.
    imaging->SetTime(2.0);
    imaging->SetTime(1.0);
    const _UpstreamPose p0 = imaging->Pose();
    if (sparse) {
        CHECK(imaging->Program() != nullptr);
        bool based = false;
        for (const auto &[time, key] :
             imaging->context->GetCompletedKeys(kUpLimbsRig)) {
            based = based || time == UsdTimeCode(1.0);
        }
        CHECK(based);
    } else {
        CHECK(TfGetenv("RIGEXEC_FRAME_CACHE", "") == "on");
        _CheckConnectedPosedSourceGraph(*imaging);
    }
    VtValue sourceBefore;
    const UsdAttribute rawSource = imaging->stage->GetAttributeAtPath(attribute);
    CHECK(rawSource && rawSource.Get(&sourceBefore, UsdTimeCode(1.0)));
    const size_t evaluations0 = imaging->Evaluations();
    imaging->upstream->Set(attribute, value);
    const _UpstreamPose p1 = imaging->Pose();
    CHECK(!_SameUpstreamPose(p0, p1, what, false));
    CHECK(_SameUpstreamPose(
        _UpstreamReference(fixture, kUpLimbsRig, {{attribute, value, {}}},
                           1.0),
        p1, what + ": set"));
    // A miss: evaluated, not served (the plan's base lookup may count as
    // a store hit; the evaluation is what a miss is).
    CHECK(imaging->Evaluations() == evaluations0 + 1);
    imaging->upstream->Clear(attribute);
    CHECK(_SameUpstreamPose(p0, imaging->Pose(), what + ": cleared"));
    imaging->upstream->Set(attribute, value);
    CHECK(_SameUpstreamPose(p1, imaging->Pose(), what + ": set again"));
    // Revisits of time 1 under each value serve that value's pose.
    imaging->SetTime(2.0);
    imaging->SetTime(1.0);
    CHECK(_SameUpstreamPose(p1, imaging->Pose(), what + ": revisited"));
    VtValue sourceAfter;
    CHECK(rawSource.Get(&sourceAfter, UsdTimeCode(1.0)));
    CHECK(sourceAfter == sourceBefore);
}

// Steps 9 and 9c: other frames warm from a time-varying source, with no
// data source call off the notice thread or off a sample time.
static void
_UpstreamWarmSteps(const std::string &fixture)
{
    std::printf("  upstream warming\n");
    std::unique_ptr<_UpstreamImaging> imaging =
        _OpenUpstreamImaging(fixture, kUpLimbsRig, 1.0);
    if (!imaging) {
        return;
    }
    if (!imaging->Program() || !RigExecBackgroundWarmingEnabled()) {
        std::printf("  (no baked program or no warming: skipping)\n");
        return;
    }
    RigExecImagingRegistry &context = *imaging->context;
    const SdfPath rig = kUpLimbsRig;
    const std::vector<double> range{1, 2, 3, 4, 5, 6};
    const std::vector<double> later{2, 3, 4, 5, 6};
    CHECK(context.SetWarmRange(rig, range));
    imaging->upstream->SetSampled(
        kUpA0Rz, [](double t) { return VtValue(5.0 * t); });
    const std::shared_ptr<_UpstreamCalls> calls = imaging->upstream->calls;
    const auto samples = [](double k, double lo, double hi) {
        _UpstreamAuthored authored{kUpA0Rz, VtValue(), {}};
        for (double f = lo; f <= hi; f += 1.0) {
            authored.samples[f] = VtValue(k * f);
        }
        return authored;
    };
    CHECK(_WarmRange(*imaging, later));
    CHECK(context.GetLastWarmSkipReason(rig) == RigExecWarmSkipReason::None);
    CHECK(!context.GetWarmBurstUsable(rig));

    // A warm pass with nothing changed calls no data source.
    const size_t callsBefore = calls->getValue.load();
    context.OnIdle();
    context.WaitUntilBackgroundIdle();
    CHECK(calls->getValue.load() == callsBefore);

    // Hits through the warm rows: no lookup sample, no evaluation, no
    // cancelled generation; each equals the authored samples.
    RigExecProfiler *profiler = context.MutableBridgeProfiler(rig);
    if (profiler) {
        profiler->SetEnabled(true);
    }
    const size_t lookups = _ScopeCount(profiler, "Imaging.LookupSampleDigest");
    const RigExecFrameGeneration generation =
        context.CurrentFrameGeneration(rig);
    const size_t evaluations = imaging->Evaluations();
    const size_t hits = context.GetFrameCacheStats(rig).hits;
    const size_t triggerCalls = calls->getValue.load();
    for (const double t : later) {
        imaging->SetTime(t);
        CHECK(_SameUpstreamPose(
            _UpstreamReference(fixture, rig, {samples(5.0, 1, 6)}, t),
            imaging->Pose(), "5t at " + std::to_string(int(t))));
    }
    // A trigger inside the window with no source change re-derives the
    // value at T0 from the table: no data source call.
    CHECK(calls->getValue.load() == triggerCalls);
    CHECK(imaging->Evaluations() == evaluations);
    CHECK(context.GetFrameCacheStats(rig).hits == hits + later.size());
    CHECK(_ScopeCount(profiler, "Imaging.LookupSampleDigest") == lookups);
    CHECK(context.CurrentFrameGeneration(rig) == generation);
    imaging->SetTime(1.0);

    // f = 7t: a dirty cancels the generation; the old rows no longer serve.
    const uint64_t serial = context.GetUpstreamSerial(rig);
    imaging->upstream->SetSampled(
        kUpA0Rz, [](double t) { return VtValue(7.0 * t); });
    CHECK(context.GetUpstreamSerial(rig) != serial);
    CHECK(context.CurrentFrameGeneration(rig) != generation);
    for (const RigExecWarmFrameState state :
         context.GetFrameStates(rig, later)) {
        CHECK(state != RigExecWarmFrameState::Cached);
    }
    const size_t missed = imaging->Evaluations();
    imaging->SetTime(3.0);
    CHECK(imaging->Evaluations() == missed + 1);
    // Not vacuous: a lookup past the warm rows does record the scope.
    CHECK(!profiler ||
          _ScopeCount(profiler, "Imaging.LookupSampleDigest") > lookups);
    CHECK(_SameUpstreamPose(
        _UpstreamReference(fixture, rig, {samples(7.0, 1, 6)}, 3.0),
        imaging->Pose(), "7t at 3 (live)"));
    imaging->SetTime(1.0);
    CHECK(_WarmRange(*imaging, later));
    for (const double t : later) {
        imaging->SetTime(t);
        CHECK(_SameUpstreamPose(
            _UpstreamReference(fixture, rig, {samples(7.0, 1, 6)}, t),
            imaging->Pose(), "7t at " + std::to_string(int(t))));
    }
    imaging->SetTime(1.0);

    // Sparse samples {1: 5, 3: 15, 5: 25}: frames 2, 4 and 6 reconstruct
    // (linear between samples, the last held), queried only at samples.
    const _UpstreamAuthored sparseAuthored{
        kUpA0Rz,
        VtValue(),
        {{1.0, VtValue(5.0)}, {3.0, VtValue(15.0)}, {5.0, VtValue(25.0)}}};
    imaging->upstream->SetSparse(kUpA0Rz, sparseAuthored.samples);
    CHECK(_WarmRange(*imaging, later));
    for (const double t : {2.0, 4.0, 6.0}) {
        imaging->SetTime(t);
        CHECK(_SameUpstreamPose(
            _UpstreamReference(fixture, rig, {sparseAuthored}, t),
            imaging->Pose(), "sparse at " + std::to_string(int(t))));
    }
    imaging->SetTime(1.0);

    // A wider range re-pulls: frames 7-10 warm with the source's values
    // there (the last sample held), not the value at 1.
    CHECK(context.SetWarmRange(rig, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}));
    const std::shared_ptr<const RigExecUpstreamTable> wide =
        context.GetUpstreamTable(rig);
    CHECK(wide && wide->windowHi == 10.0);
    const std::vector<double> wider{7, 8, 9, 10};
    CHECK(_WarmRange(*imaging, wider));
    const _UpstreamPose atOne =
        _UpstreamReference(fixture, rig, {{kUpA0Rz, VtValue(5.0), {}}}, 8.0);
    for (const double t : wider) {
        imaging->SetTime(t);
        const _UpstreamPose pose = imaging->Pose();
        CHECK(_SameUpstreamPose(
            _UpstreamReference(fixture, rig, {sparseAuthored}, t), pose,
            "sparse at " + std::to_string(int(t))));
        if (t == 8.0) {
            CHECK(!_SameUpstreamPose(atOne, pose, "held", false));
        }
    }
    imaging->SetTime(1.0);

    // A frame outside the window has no value for a varying source: the
    // factory skips it rather than warm it with another frame's value.
    const RigExecWarmFactoryResult outside = context.BuildWarmWork(
        rig, UsdTimeCode(14.0), context.CurrentFrameGeneration(rig),
        RigExecFrozenStepRunner());
    CHECK(!outside.work);
    CHECK(outside.skip == RigExecWarmSkipReason::Unsampleable);
    CHECK(outside.detail == "upstream not pulled for this time");

    // A uniform source keeps the burst route.
    imaging->upstream->Set(kUpA0Rz, VtValue(30.0));
    context.OnIdle();
    context.WaitUntilBackgroundIdle();
    CHECK(context.GetWarmBurstUsable(rig));
    CHECK(calls->offSample.load() == 0);
    CHECK(calls->offThread.load() == 0);
}

// Step 9c: arrays through the cache and warming (U1c's admission hook on).
static void
_UpstreamArraySteps(const std::string &fixture)
{
    std::printf("  upstream arrays\n");
    RigExecSetUpstreamArrayAdmissionForTesting(true);
    {
        _UpstreamCacheMode cache("on");
        std::unique_ptr<_UpstreamImaging> imaging =
            _OpenUpstreamImaging(fixture, kUpLimbsRig, 1.0);
        if (imaging && imaging->Program()) {
            const UsdAttribute attribute =
                imaging->stage->GetAttributeAtPath(kUpJointWeights);
            VtFloatArray authored;
            CHECK(attribute.Get(&authored));
            const auto weights = [&](float a) {
                VtFloatArray w(authored.size());
                for (size_t i = 0; i < w.size(); i += 2) {
                    w[i] = a;
                    w[i + 1] = 1.0f - a;
                }
                return w;
            };
            // Step 8's shape: authored, set (a miss), cleared, set again,
            // at a posed frame (at frame 1 every joint rests, and the skin
            // returns its rest points whatever the weights).
            imaging->SetTime(7.0);
            imaging->SetTime(6.0);
            const _UpstreamPose p0 = imaging->Pose();
            const VtValue value(weights(0.25f));
            imaging->upstream->Set(kUpJointWeights, value);
            const _UpstreamPose p1 = imaging->Pose();
            CHECK(!_SameUpstreamPose(p0, p1, "array", false));
            CHECK(_SameUpstreamPose(
                _UpstreamReference(fixture, kUpLimbsRig,
                                   {{kUpJointWeights, value, {}}}, 6.0),
                p1, "array: set"));
            imaging->upstream->Clear(kUpJointWeights);
            CHECK(_SameUpstreamPose(p0, imaging->Pose(), "array: cleared"));
            imaging->upstream->Set(kUpJointWeights, value);
            CHECK(_SameUpstreamPose(p1, imaging->Pose(), "array: again"));
            imaging->upstream->Clear(kUpJointWeights);
            imaging->SetTime(1.0);

            // Step 9's shape: three array samples; frames between them
            // reconstruct, the table holds the three samples only.
            if (RigExecBackgroundWarmingEnabled()) {
                RigExecImagingRegistry &context = *imaging->context;
                const std::vector<double> later{2, 3, 4, 5, 6};
                CHECK(context.SetWarmRange(kUpLimbsRig, {1, 2, 3, 4, 5, 6}));
                const std::map<double, VtValue> arraySamples{
                    {1.0, VtValue(weights(1.0f))},
                    {3.0, VtValue(weights(0.5f))},
                    {5.0, VtValue(weights(0.0f))}};
                imaging->upstream->SetSparse(kUpJointWeights, arraySamples);
                const std::shared_ptr<const RigExecUpstreamTable> table =
                    context.GetUpstreamTable(kUpLimbsRig);
                CHECK(table &&
                      RigExecUpstreamTableBufferCount(*table) == 3);
                CHECK(_WarmRange(*imaging, later));
                const size_t evaluations = imaging->Evaluations();
                for (const double t : {2.0, 4.0, 6.0}) {
                    imaging->SetTime(t);
                    // Linear between equal-length samples, the last held,
                    // authored per frame (time-sampled layouts do not bake).
                    std::vector<RigExecUpstreamTableSample> samples;
                    for (const auto &[at, v] : arraySamples) {
                        samples.push_back({at, v});
                    }
                    const VtValue expected =
                        RigExecReconstructUpstream(samples, t, false);
                    CHECK(_SameUpstreamPose(
                        _UpstreamReference(fixture, kUpLimbsRig,
                                           {{kUpJointWeights, expected, {}}},
                                           t),
                        imaging->Pose(),
                        "array sparse at " + std::to_string(int(t))));
                }
                CHECK(imaging->Evaluations() == evaluations);
                CHECK(imaging->upstream->calls->offSample.load() == 0);
                CHECK(imaging->upstream->calls->offThread.load() == 0);
            }
        }
    }
    RigExecSetUpstreamArrayAdmissionForTesting(false);
}

// Step 9b: a lift reaches a job from a snapshot frozen while the value
// stood. One session warms the authored frame; another places the value
// before its first freeze (so its snapshot inherits it), warms, lifts and
// warms again: that job's key and pose are the authored ones.
static void
_UpstreamLiftSteps(const std::string &stagePath, const SdfPath &rig,
                   const SdfPath &attribute, const VtValue &value,
                   double t, double later, const std::string &what)
{
    std::printf("  upstream lift: %s\n", what.c_str());
    if (!RigExecBackgroundWarmingEnabled()) {
        return;
    }
    const auto keyAt = [&](RigExecImagingRegistry &context) {
        RigExecFrameCacheKey key{0, 0};
        for (const auto &[time, k] : context.GetCompletedKeys(rig)) {
            if (time == UsdTimeCode(later)) {
                key = k;
            }
        }
        return key;
    };
    RigExecFrameCacheKey authoredKey{0, 0};
    _UpstreamPose authored;
    {
        std::unique_ptr<_UpstreamImaging> imaging =
            _OpenUpstreamImaging(stagePath, rig, t);
        if (!imaging || !imaging->Program()) {
            std::printf("  (no baked program: skipping)\n");
            return;
        }
        CHECK(imaging->context->SetWarmRange(rig, {t, later}));
        CHECK(_WarmRange(*imaging, {later}));
        authoredKey = keyAt(*imaging->context);
        imaging->SetTime(later);
        authored = imaging->Pose();
    }
    std::unique_ptr<_UpstreamImaging> imaging =
        _OpenUpstreamImaging(stagePath, rig, t);
    if (!imaging) {
        return;
    }
    RigExecImagingRegistry &context = *imaging->context;
    imaging->upstream->Set(attribute, value);
    CHECK(context.SetWarmRange(rig, {t, later}));
    CHECK(_WarmRange(*imaging, {later}));
    const RigExecFrameCacheKey placedKey = keyAt(context);
    CHECK(placedKey.controlDigest != authoredKey.controlDigest);

    imaging->upstream->Clear(attribute);
    CHECK(_WarmRange(*imaging, {later}));
    const RigExecFrameCacheKey liftedKey = keyAt(context);
    CHECK(liftedKey.controlDigest == authoredKey.controlDigest);
    const size_t evaluations = imaging->Evaluations();
    imaging->SetTime(later);
    CHECK(imaging->Evaluations() == evaluations);
    CHECK(_SameUpstreamPose(authored, imaging->Pose(), what + ": lifted"));
}

// TestUpstreamSceneIndexDrivesRigInputs: steps 1-5 and 7 (frame cache off),
// step 6 in its interim form (.rigexec playback; the full form and the
// runtime cone are the runtime session's), the posed variant, admission
// negatives, step 8 and its D7 variant, steps 9, 9b and 9c.
static void
TestUpstreamSceneIndexDrivesRigInputs(const std::string &examplesDir)
{
    if (_ImagingResourceDir().empty()) {
        std::printf("  (no imaging plugInfo; skipping upstream inputs)\n");
        return;
    }
    std::printf("progress: TestUpstreamSceneIndexDrivesRigInputs\n");
    std::fflush(stdout);
    const std::string fixture =
        examplesDir + "/../tests/fixtures/upstream_inputs.usda";
    const std::string posed =
        examplesDir + "/../tests/fixtures/upstream_inputs_posed.usda";
    {
        _UpstreamCacheMode cache("off");
        // A0's ComposeSubtree seeds the control case; BRoot's rest group
        // seeds the space case in the head tier.
        _UpstreamValueSteps(fixture, kUpA0Rz, VtValue(30.0), kUpLimbA,
                            kUpLimbB, "A0", "", "A0 avars:rz");
        _UpstreamValueSteps(fixture, kUpSpace,
                            VtValue(_UpTranslate(1, 0, 10)), kUpLimbB,
                            kUpLimbA, "", "BRoot", "Upstream inputs:space");
        _UpstreamPlaybackSteps(fixture);
        TestABakeExposesUpstreamAsInputs(fixture);
        TestTheRuntimeUpstreamCone(fixture);
        _UpstreamAdmissionNegatives(fixture);
        _UpstreamAddsAndChains(fixture);
        _UpstreamChainConflictTimes(fixture);
        _UpstreamAddedOutsideRoots(fixture);
        // Connected posed space follows the upstream input through the graph.
        std::unique_ptr<_UpstreamImaging> imaging =
            _OpenUpstreamImaging(posed, kUpLimbsRig, 1.0);
        if (imaging) {
            CHECK(TfGetenv("RIGEXEC_FRAME_CACHE", "") == "off");
            _CheckConnectedPosedSourceGraph(*imaging);
            const auto authored = imaging->Pose();
            VtValue sourceBefore;
            const UsdAttribute rawSource = imaging->stage->GetAttributeAtPath(kUpSpace);
            CHECK(rawSource && rawSource.Get(&sourceBefore, UsdTimeCode(1.0)));
            imaging->upstream->Set(kUpSpace, VtValue(_UpTranslate(1, 0, 10)));
            CHECK(_SameUpstreamPose(
                _UpstreamReference(
                    posed, kUpLimbsRig,
                    {{kUpSpace, VtValue(_UpTranslate(1, 0, 10)), {}}}, 1.0),
                imaging->Pose(), "posed: standing"));
            CHECK(imaging->Evaluator()->GetUpstreamInputPaths() ==
                  std::vector<SdfPath>{kUpSpace});
            CHECK(!_SameUpstreamPose(authored, imaging->Pose(), "posed: sensitivity", false));
            imaging->upstream->Clear(kUpSpace);
            CHECK(imaging->Evaluator()->GetUpstreamInputPaths().empty());
            CHECK(_SameUpstreamPose(authored, imaging->Pose(), "posed: lifted"));
            VtValue sourceAfter;
            CHECK(rawSource.Get(&sourceAfter, UsdTimeCode(1.0)));
            CHECK(sourceAfter == sourceBefore);
        }
    }
    {
        _UpstreamCacheMode cache("on");
        const bool baked = [&]() {
            std::unique_ptr<_UpstreamImaging> probe =
                _OpenUpstreamImaging(fixture, kUpLimbsRig, 1.0);
            return probe && probe->Program() != nullptr;
        }();
        if (baked) {
            _UpstreamCacheSteps(fixture, kUpA0Rz, VtValue(30.0), true,
                                "sparse path");
        }
        _UpstreamCacheSteps(posed, kUpSpace, VtValue(_UpTranslate(1, 0, 10)),
                            false, "pose-only key (D7)");
        _UpstreamWarmSteps(fixture);
        // A constant avar: the clone's inherited avar slot is undone only by
        // the worker's upstream arm (rule 2).
        _UpstreamLiftSteps(fixture, kUpLimbsRig, kUpA0Rz, VtValue(30.0), 1, 4,
                           "A0 avars:rz");
        _UpstreamLiftSteps(
            examplesDir + "/09_PropertyMathMovers.usda",
            SdfPath("/PropMathAsset/Rig"),
            SdfPath("/PropMathAsset/Rig/Movers/OffsetLift.inputs:value"),
            VtValue(GfVec3f(0, 7, 0)), 1012, 1015,
            "09 chain mover inputs:value");
        _UpstreamLiftSteps(
            examplesDir + "/biped/Biped.usda", SdfPath("/Biped/Rig"),
            SdfPath("/Biped/Rig/Movers/skin_body_geo/body_geo_skin."
                    "inputs:defaultWeight"),
            VtValue(0.5f), 1, 4, "biped skin inputs:defaultWeight");
    }
    _UpstreamArraySteps(fixture);
}

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecImaging <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = _SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    // The rig adapter's keyless registration, before any test builds a
    // UsdImaging chain: the adapter registry reads Plug metadata once, on
    // first use. Skipped (with the test that needs it) when the build did
    // not provide the generated plugInfo.
    const std::string imagingResources = _ImagingResourceDir();
    if (!imagingResources.empty() &&
        PlugRegistry::GetInstance().RegisterPlugins(
            imagingResources).empty()) {
        std::printf("FATAL: no imaging plugin found at %s\n",
                    imagingResources.c_str());
        return 2;
    }

    TestDrivenXformConditioning();
    TestPrePopulationAttachment();
    TestPopulatedWrappingAndResync();
    TestLateConsumerTraversal();
    TestNoticeStreamFiltering();
    TestNarrowLocators();
    TestFilterChainOverRetainedScene();
    TestBridgeOverShotStage(examplesDir);
    TestStandaloneControlGuide();
    TestCustomControlRestExtent();
    TestControlGuideSource();
    TestJointHierarchyGuides(examplesDir);
    TestSimpleRigControlAndDeformation(examplesDir);
    TestControlGuides(examplesDir);
    TestShaderMatrixPrimvars();
    TestShaderMatrixWorldSpace();
    TestControlGuidePoseDirtiness();
    TestGuidesFollowDrivenAssetRoot();
    TestSolverGuideRadius(examplesDir);
    TestExampleControlGuides(examplesDir);
    TestMotionCapabilityMatrix(examplesDir);
    TestCompleteMotionPublication(examplesDir);
    TestDrivenXformResetBoundaries(examplesDir);
    TestLegacyRenderIndexPickup(examplesDir);
    TestMultiRigAtomicActivation(examplesDir);
    TestEditTriggeredReevaluation(examplesDir);
    TestFailedActivationKeepsLiveNoticeOrdering();
    TestExternalReadEditsRepublish();
    TestConstraintDrivenXformPublishes(examplesDir);
    TestJointFreeRigPublishes(examplesDir);
    TestDisconnectingAConstraintClearsTheDrivenXform(examplesDir);
    TestRelationshipEditTriggersReevaluation(examplesDir);
    TestPropertyResultsDoNotPublishPrims(examplesDir);
    // Last: it publishes a fixture into the process-global registry store.
    TestGuideBoundsExport();
    TestTransformAuthorityWarnings(examplesDir);
    TestXformPreviewDelta();
    TestSnapshotBackedExtent(examplesDir);
    TestInterveningXformLeavesTheLocalExtentAlone(examplesDir);
    TestExtentIsPureFunctionOfStageAndTime(examplesDir);
    TestPurposeScopedBounds(examplesDir);
    TestAllPurposeRenderTags(examplesDir);
    TestPolicyRigActivationSurvivesItsOwnDerivation(examplesDir);
    // Last: it drives the real UsdImaging chain and deactivates on the way
    // out, leaving the process-global registry cleared.
    TestSetTimeDrivenEvaluation(examplesDir);
    // The real chain with an upstream scene index installed; every session
    // it opens is deactivated on the way out.
    TestUpstreamSceneIndexDrivesRigInputs(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecImaging: all tests passed\n");
    return 0;
}
