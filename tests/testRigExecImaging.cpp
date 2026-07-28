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

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/imaging/hd/dataSourceLocator.h"
#include "pxr/imaging/hd/extentSchema.h"
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
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <string>

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
        CHECK(sawSphere && sawCone);

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
        CHECK(coneAdds == 1);
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

// Edit-driven re-evaluation (registry): any authored edit beneath the
// rig's asset re-evaluates at the last-set time and republishes;
// unrelated edits do not.
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
        .CreateAttribute(TfToken("guide:length"),
                         SdfValueTypeNames->Double)
        .Set(3.5);
    snapshot = registry.GetStore()->Get();
    CHECK(snapshot && snapshot->generation > generation1);

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

// A constraint-driven Xform reaches the snapshot as a transform, and it
// changes with time.
//
// The evaluator side is asserted in testRigExecArm; this is the imaging leg.
// hasXform was plumbed to HdXformSchema and had no writer at all, so the
// failure mode here is a published-once-then-frozen transform, which looks
// exactly like a static prop in the viewer.
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

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecImaging <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = TfAbsPath(
        examplesDir + "/../plugin/rigExecSchema/resources");
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
    TestMotionCapabilityMatrix(examplesDir);
    TestLegacyRenderIndexPickup(examplesDir);
    TestEditTriggeredReevaluation(examplesDir);
    TestConstraintDrivenXformPublishes(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecImaging: all tests passed\n");
    return 0;
}
