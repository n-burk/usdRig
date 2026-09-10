//
// RigExec Hydra scene-index tests (spec §10, §14.5 Hydra matrix):
// construction/pull goldens (pre-population attachment, populated
// wrapping with add-again resync, late-consumer traversal, legacy
// render-index pickup), live notice-stream filtering with locator
// preservation, narrow dependency-derived locators with
// ComputeDirtyLocators() sentinel expansion, motionBlurSupport
// capability matrix with one-sample non-motion profile and preflight
// failure without evaluation, derivative blocking, terminal-enumeration
// audit, and the real evaluator publishing the ArmShotAnim rig.
//
#include "rigExecImaging/bridge.h"
#include "rigExecImaging/registry.h"
#include "rigExecImaging/sceneIndices.h"
#include "rigExecMath/avarScale.h"
#include "rigExecMath/curvenet.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/references.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/imaging/hd/basisCurvesSchema.h"
#include "pxr/imaging/hd/basisCurvesTopologySchema.h"
#include "pxr/imaging/hd/coneSchema.h"
#include "pxr/imaging/hd/cubeSchema.h"
#include "pxr/imaging/hd/dataSourceLocator.h"
#include "pxr/imaging/hd/extentSchema.h"
#include "pxr/imaging/hd/legacyDisplayStyleSchema.h"
#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
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
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdUtils/stageCache.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
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
    chain.binding = RigExecBindingResolvingSceneIndex::New(chain.pruning);
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
//
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

    // ---- Nothing authored at all: the schema fallbacks draw a wire
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

    // ---- Every shape in every draw mode, one publication each.
    //
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

    // ---- Style and placement, on the default-shaped guide.
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

    // ---- purpose. The STOCK UsdGeomImageable attribute, not a RigExec
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

    // ---- guide:wireWidth, which is what makes a wire control clickable.
    //
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

// A constraint-driven ASSET ROOT carries the synthesized guides with it,
// and says so.
//
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
//
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
//
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

        // ---- The planar shapes, authored in memory on this same rig.
        //
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
TestPosedCurvenetGuides(const std::string &examplesDir)
{
    const UsdStageRefPtr stage = UsdStage::Open(
        examplesDir + "/12_CurvenetProfile.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath asset("/CurvenetAsset"), net("/CurvenetAsset/Geom/Net");
    const SdfPath curve = net.AppendChild(TfToken("rigGuideVol_0"));
    RigExecImagingBridge bridge(stage, asset.AppendChild(TfToken("Rig")));
    std::vector<std::string> errors;
    CHECK(bridge.Compile(&errors));
    auto firstResult = bridge.EvaluateAndPublishResult(UsdTimeCode(1001));
    CHECK(firstResult.ok);
    auto first = bridge.GetStore()->Get();
    CHECK(first && first->prims.count(net));
    if (!first || !first->prims.count(net)) return;
    const auto &initial = first->prims.at(net);
    CHECK(initial.hasVolumeGuides && initial.volumeGuides.size() == 1);
    if (initial.volumeGuides.empty()) return;
    CHECK(initial.volumeGuideAnchor == net);
    const VtVec3fArray restCurve = initial.volumeGuides[0].points;

    auto upstream = HdRetainedSceneIndex::New();
    GfMatrix4d assetWorld(1), netWorld(1);
    assetWorld.SetTranslate(GfVec3d(100, 0, 0));
    netWorld.SetTranslate(GfVec3d(125, 0, 0));
    const auto xformData = [](const GfMatrix4d &matrix) {
        const TfToken name = HdXformSchemaTokens->xform;
        const HdDataSourceBaseHandle data = HdXformSchema::Builder()
            .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(matrix))
            .Build();
        return HdRetainedContainerDataSource::New(1, &name, &data);
    };
    upstream->AddPrims({{asset, TfToken("xform"), xformData(assetWorld)},
                       {net, HdPrimTypeTokens->points, xformData(netWorld)}});
    auto results = RigExecResultsSceneIndex::New(upstream, bridge.GetStore());
    _RecordingObserver observer;
    results->AddObserver(HdSceneIndexObserverPtr(&observer));
    CHECK(results->GetPrim(curve).primType == HdPrimTypeTokens->basisCurves);
    CHECK(_GetPointsPrimvar(results->GetPrim(curve)) == restCurve);
    const auto guideMatrix = HdXformSchema::GetFromParent(
        results->GetPrim(curve).dataSource).GetMatrix();
    CHECK(guideMatrix && guideMatrix->GetTypedValue(0) == netWorld);

    const auto posedResult = bridge.EvaluateAndPublishResult(UsdTimeCode(1024));
    CHECK(posedResult.ok);
    results->NotifyGenerationPublished(posedResult.dirtied);
    const auto posed = bridge.GetStore()->Get();
    CHECK(posed && posed->prims.count(net));
    if (!posed || !posed->prims.count(net)) return;
    const auto &published = posed->prims.at(net);
    CHECK(published.hasPoints && published.volumeGuides.size() == 1);
    const auto &guide = published.volumeGuides[0];
    CHECK(guide.points != restCurve);
    CHECK(_GetPointsPrimvar(results->GetPrim(curve)) == guide.points);
    CHECK(std::find(observer.dirtied.begin(), observer.dirtied.end(), curve) !=
          observer.dirtied.end());
    // Every authored Bezier endpoint occurs exactly in the sampled posed
    // guide; using the design pool here would fail at the animated end.
    VtIntArray indices;
    stage->GetPrimAtPath(net).GetAttribute(TfToken("rigExec:splineIndices"))
        .Get(&indices);
    for (size_t i = 0; i < indices.size(); i += 4) {
        const GfVec3f knot = published.points[indices[i]];
        CHECK(std::find(guide.points.begin(), guide.points.end(), knot) !=
              guide.points.end());
    }
    // Preserve authored children occupying a generated name.
    upstream->AddPrims({_MeshShell(curve)});
    CHECK(results->GetPrim(curve).primType == HdPrimTypeTokens->mesh);
    upstream->RemovePrims({curve});
    CHECK(results->GetPrim(curve).primType == HdPrimTypeTokens->basisCurves);
    results->RemoveObserver(HdSceneIndexObserverPtr(&observer));
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
//
// The evaluator side is asserted in testRigExecArm; this is the imaging leg.
// hasXform was plumbed to HdXformSchema and had no writer at all, so the
// failure mode here is a published-once-then-frozen transform, which looks
// exactly like a static prop in the viewer.
// A rig with no joints publishes through the bridge like any other.
//
// The evaluator gate is only half of it: the bridge's guide fills all iterate
// joint frames, the binding epoch is built from the published prim set, and a
// rig whose entire output is one driven Xform exercises every one of those
// with an empty joint set.
// Disconnecting a constraint's target must un-drive the subtree.
//
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
//
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
//
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
    const auto it2 = snap ? snap->prims.find(turret)
                          : RigExecImagingSnapshot().prims.end();
    CHECK(snap && it2 != snap->prims.end());
    if (!snap || it2 == snap->prims.end()) return;
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
    //
    // Flattening comes FIRST because that is where UsdImaging puts it: the
    // chain's only HdFlatteningSceneIndex is built inside
    // UsdImagingNiPrototypePropagatingSceneIndex, which
    // UsdImagingCreateSceneIndices constructs before it appends plugin scene
    // indices -- and UsdImagingSceneIndexPlugin::AppendSceneIndex is the only
    // hook RigExec has. So RigExec sees world-space transforms and is solely
    // responsible for carrying a driven Xform's subtree along with it.
    //
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
    //
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
    //
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
//
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
    //
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
//
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

    // An empty generation draws nothing at all.
    store->Publish(nullptr);
    CHECK(RigExecImaging_GetAllGuideBoundsAssetSpace(b) == 0);
    CHECK(bounds(joint) == 0);
}

// The codeless schema's resource directory.
//
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
    //
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
//
// The extent is LOCAL and UsdGeomBBoxCache multiplies it by the prim's own
// local-to-world, which already contains that Xform. The published frames
// contain it too since 2026-09-10, so the snapshot branch has to divide it
// back out; if it does not, the box lands at the Xform applied twice --
// guide in one place, bounding box in another.
//
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
//
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
//
// The snapshot store is a process-global singleton and USD calls this
// callback with a stage and a time of its own choosing, so a lookup by
// path alone answers a query about one stage/frame with another's pose --
// plausible, wrong, and invisible downstream. Both halves are pinned here.
//
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

    TestDrivenXformConditioning();
    TestPrePopulationAttachment();
    TestPopulatedWrappingAndResync();
    TestLateConsumerTraversal();
    TestNoticeStreamFiltering();
    TestNarrowLocators();
    TestFilterChainOverRetainedScene();
    TestBridgeOverShotStage(examplesDir);
    TestStandaloneControlGuide();
    TestJointHierarchyGuides(examplesDir);
    TestSimpleRigControlAndDeformation(examplesDir);
    TestControlGuides(examplesDir);
    TestGuidesFollowDrivenAssetRoot();
    TestSolverGuideRadius(examplesDir);
    TestExampleControlGuides(examplesDir);
    TestMotionCapabilityMatrix(examplesDir);
    TestCompleteMotionPublication(examplesDir);
    TestDrivenXformResetBoundaries(examplesDir);
    TestLegacyRenderIndexPickup(examplesDir);
    TestMultiRigAtomicActivation(examplesDir);
    TestPosedCurvenetGuides(examplesDir);
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
    TestSnapshotBackedExtent(examplesDir);
    TestInterveningXformLeavesTheLocalExtentAlone(examplesDir);
    TestExtentIsPureFunctionOfStageAndTime(examplesDir);
    TestPurposeScopedBounds(examplesDir);
    TestAllPurposeRenderTags(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecImaging: all tests passed\n");
    return 0;
}
