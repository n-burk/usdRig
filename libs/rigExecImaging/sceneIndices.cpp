//
// RigExec Hydra scene-index filters implementation (spec §10).
//
#include "sceneIndices.h"

#include "pxr/imaging/hd/coneSchema.h"
#include "pxr/imaging/hd/containerDataSourceEditor.h"
#include "pxr/imaging/hd/dataSource.h"
#include "pxr/imaging/hd/extentSchema.h"
#include "pxr/imaging/hd/overlayContainerDataSource.h"
#include "pxr/imaging/hd/primOriginSchema.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/purposeSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/sphereSchema.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hd/visibilitySchema.h"
#include "pxr/imaging/hd/xformSchema.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

// ---------------------------------------------------------------------------
// RigExecInternalPrimPruningSceneIndex
// ---------------------------------------------------------------------------

RigExecInternalPrimPruningSceneIndex::RigExecInternalPrimPruningSceneIndex(
    const HdSceneIndexBaseRefPtr &inputSceneIndex)
    : HdSingleInputFilteringSceneIndexBase(inputSceneIndex)
{
}

void
RigExecInternalPrimPruningSceneIndex::SetOwnedScopes(
    const std::set<SdfPath> &scopes)
{
    // Structural: the previously and newly owned scopes disappear/appear.
    HdSceneIndexObserver::RemovedPrimEntries removed;
    for (const SdfPath &scope : _ownedScopes) {
        removed.emplace_back(scope);
    }
    _ownedScopes = scopes;
    if (!removed.empty() && _IsObserved()) {
        _SendPrimsRemoved(removed);
    }
}

bool
RigExecInternalPrimPruningSceneIndex::_IsOwned(const SdfPath &path) const
{
    for (const SdfPath &scope : _ownedScopes) {
        if (path.HasPrefix(scope)) {
            return true;
        }
    }
    return false;
}

HdSceneIndexPrim
RigExecInternalPrimPruningSceneIndex::GetPrim(const SdfPath &primPath) const
{
    if (_IsOwned(primPath)) {
        return {TfToken(), nullptr};
    }
    return _GetInputSceneIndex()->GetPrim(primPath);
}

SdfPathVector
RigExecInternalPrimPruningSceneIndex::GetChildPrimPaths(
    const SdfPath &primPath) const
{
    SdfPathVector children =
        _GetInputSceneIndex()->GetChildPrimPaths(primPath);
    children.erase(
        std::remove_if(children.begin(), children.end(),
                       [this](const SdfPath &p) { return _IsOwned(p); }),
        children.end());
    return children;
}

void
RigExecInternalPrimPruningSceneIndex::_PrimsAdded(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::AddedPrimEntries &entries)
{
    if (!_IsObserved()) {
        return;
    }
    // Filter owned entries while preserving the order of unrelated ones
    // (spec §10.1).
    HdSceneIndexObserver::AddedPrimEntries filtered;
    filtered.reserve(entries.size());
    for (const auto &entry : entries) {
        if (!_IsOwned(entry.primPath)) {
            filtered.push_back(entry);
        }
    }
    if (!filtered.empty()) {
        _SendPrimsAdded(filtered);
    }
}

void
RigExecInternalPrimPruningSceneIndex::_PrimsRemoved(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::RemovedPrimEntries &entries)
{
    if (!_IsObserved()) {
        return;
    }
    HdSceneIndexObserver::RemovedPrimEntries filtered;
    filtered.reserve(entries.size());
    for (const auto &entry : entries) {
        if (!_IsOwned(entry.primPath)) {
            filtered.push_back(entry);
        }
    }
    if (!filtered.empty()) {
        _SendPrimsRemoved(filtered);
    }
}

void
RigExecInternalPrimPruningSceneIndex::_PrimsDirtied(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::DirtiedPrimEntries &entries)
{
    if (!_IsObserved()) {
        return;
    }
    HdSceneIndexObserver::DirtiedPrimEntries filtered;
    filtered.reserve(entries.size());
    for (const auto &entry : entries) {
        if (!_IsOwned(entry.primPath)) {
            filtered.push_back(entry);
        }
    }
    if (!filtered.empty()) {
        _SendPrimsDirtied(filtered);
    }
}

// ---------------------------------------------------------------------------
// RigExecBindingResolvingSceneIndex
// ---------------------------------------------------------------------------

RigExecBindingResolvingSceneIndex::RigExecBindingResolvingSceneIndex(
    const HdSceneIndexBaseRefPtr &inputSceneIndex)
    : HdSingleInputFilteringSceneIndexBase(inputSceneIndex)
{
}

void
RigExecBindingResolvingSceneIndex::SetBindingEpoch(BindingEpochConstPtr epoch)
{
    BindingEpochConstPtr previous = std::atomic_load(&_epoch);
    std::atomic_store(&_epoch, std::move(epoch));

    if (!_IsObserved()) {
        return;
    }
    // An output-set/binding change uses universal dirtiness on every
    // affected output prim (spec §10.4).
    HdSceneIndexObserver::DirtiedPrimEntries dirtied;
    BindingEpochConstPtr current = std::atomic_load(&_epoch);
    if (previous) {
        for (const SdfPath &p : previous->publishedPrims) {
            dirtied.emplace_back(p, HdDataSourceLocatorSet::UniversalSet());
        }
    }
    if (current) {
        for (const SdfPath &p : current->publishedPrims) {
            if (!previous || !previous->publishedPrims.count(p)) {
                dirtied.emplace_back(
                    p, HdDataSourceLocatorSet::UniversalSet());
            }
        }
    }
    if (!dirtied.empty()) {
        _SendPrimsDirtied(dirtied);
    }
}

HdSceneIndexPrim
RigExecBindingResolvingSceneIndex::GetPrim(const SdfPath &primPath) const
{
    // Compiled output addresses are stage paths under the USD backend:
    // the prim shell passes through; the results index overlays values.
    return _GetInputSceneIndex()->GetPrim(primPath);
}

SdfPathVector
RigExecBindingResolvingSceneIndex::GetChildPrimPaths(
    const SdfPath &primPath) const
{
    return _GetInputSceneIndex()->GetChildPrimPaths(primPath);
}

void
RigExecBindingResolvingSceneIndex::_PrimsAdded(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::AddedPrimEntries &entries)
{
    if (_IsObserved()) {
        _SendPrimsAdded(entries);
    }
}

void
RigExecBindingResolvingSceneIndex::_PrimsRemoved(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::RemovedPrimEntries &entries)
{
    if (_IsObserved()) {
        _SendPrimsRemoved(entries);
    }
}

void
RigExecBindingResolvingSceneIndex::_PrimsDirtied(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::DirtiedPrimEntries &entries)
{
    if (_IsObserved()) {
        _SendPrimsDirtied(entries);
    }
}

// ---------------------------------------------------------------------------
// RigExecResultsSceneIndex
// ---------------------------------------------------------------------------

RigExecResultsSceneIndex::RigExecResultsSceneIndex(
    const HdSceneIndexBaseRefPtr &inputSceneIndex,
    std::shared_ptr<RigExecSnapshotStore> store)
    : HdSingleInputFilteringSceneIndexBase(inputSceneIndex)
    , _store(std::move(store))
{
    // Seed the driven-transform history from whatever generation is already
    // published. A chain constructed AFTER activation (usdview builds its
    // viewport when it likes, and the plugin activates on stage open) would
    // otherwise believe nothing was ever driven, and the first loss or
    // Deactivate() would leave every driven subtree stale.
    // Guide counts are seeded for the same reason: a late observer's initial
    // traversal finds the synthesized children, so this index must already
    // believe it announced them or a later drop to zero sends no removals
    // and the stale guides survive downstream.
    if (const RigExecImagingSnapshotConstPtr snapshot =
            _store ? _store->Get() : nullptr) {
        for (const auto &[path, published] : snapshot->prims) {
            if (published.hasXform) {
                _announcedDrivenXforms.insert(path);
            }
            // Through _DesiredGuideCount, not the raw payload size: it also
            // requires the parent to exist upstream, which is what a
            // traversal actually finds. Seeding the raw count for a prim
            // whose upstream parent is absent would later emit removals for
            // children that never existed.
            _RefreshAnnouncedGuides(path);
        }
    }
}

namespace {

// Snapshot-backed sampled points source (spec 10.5): reports the retained
// frame-relative offsets and returns only cached values; pulls can never
// trigger evaluation.
class _SampledPointsDataSource final
    : public HdTypedSampledDataSource<VtVec3fArray> {
public:
    HD_DECLARE_DATASOURCE(_SampledPointsDataSource);

    VtValue GetValue(Time shutterOffset) override {
        return VtValue(GetTypedValue(shutterOffset));
    }

    VtVec3fArray GetTypedValue(Time shutterOffset) override {
        if (_offsets.empty()) {
            return _base;
        }
        size_t best = 0;
        for (size_t i = 1; i < _offsets.size(); ++i) {
            if (std::abs(_offsets[i] - shutterOffset) <
                std::abs(_offsets[best] - shutterOffset)) {
                best = i;
            }
        }
        return _samples[best];
    }

    bool GetContributingSampleTimesForInterval(
        Time startTime, Time endTime,
        std::vector<Time> *outSampleTimes) override {
        if (_offsets.size() < 2) {
            return false;
        }
        outSampleTimes->assign(_offsets.begin(), _offsets.end());
        return true;
    }

private:
    _SampledPointsDataSource(
        VtVec3fArray base, std::vector<float> offsets,
        std::vector<VtVec3fArray> samples)
        : _base(std::move(base))
        , _offsets(std::move(offsets))
        , _samples(std::move(samples)) {}

    VtVec3fArray _base;
    std::vector<float> _offsets;
    std::vector<VtVec3fArray> _samples;
};

HdTokenDataSourceHandle
_Token(const TfToken &token)
{
    return HdRetainedTypedSampledDataSource<TfToken>::New(token);
}

// ---------------------------------------------------------------------------
// Synthesized guide children (spec §10.3 extension): joints and aggregate
// solvers draw as guide geometry — a sphere at each posed frame origin and
// a cone along the frame's +X aim axis — matching the contract of
// OpenExec's UsdIrImagingJointScopeAdapter (purpose guide, constant
// displayColor/displayOpacity primvars).
// ---------------------------------------------------------------------------

const std::string _guideSpherePrefix("rigGuideSphere_");
const std::string _guideConePrefix("rigGuideCone_");

bool
_ParseGuideName(const TfToken &name, bool *isCone, size_t *index)
{
    const std::string &s = name.GetString();
    const std::string *prefix = nullptr;
    if (s.compare(0, _guideSpherePrefix.size(), _guideSpherePrefix) == 0) {
        prefix = &_guideSpherePrefix;
        *isCone = false;
    } else if (s.compare(0, _guideConePrefix.size(), _guideConePrefix) ==
               0) {
        prefix = &_guideConePrefix;
        *isCone = true;
    } else {
        return false;
    }
    const std::string digits = s.substr(prefix->size());
    if (digits.empty() || digits.size() > 9 ||
        digits.find_first_not_of("0123456789") != std::string::npos) {
        return false;
    }
    size_t value = 0;
    for (const char c : digits) {
        value = value * 10 + static_cast<size_t>(c - '0');
    }
    *index = value;
    return true;
}

TfToken
_GuideName(bool isCone, size_t index)
{
    return TfToken((isCone ? _guideConePrefix : _guideSpherePrefix) +
                   std::to_string(index));
}

HdContainerDataSourceHandle
_BuildGuidePrim(
    const RigExecPublishedPrim &published, bool isCone, size_t index,
    const HdContainerDataSourceHandle &parentDataSource,
    const GfMatrix4d &assetRootWorld)
{
    const GfMatrix4d &frame = published.guideFrames[index];
    const double length = published.guideLengths[index];
    const double radius = index < published.guideRadii.size()
        ? published.guideRadii[index] : 1.0;

    // The cone spans its axis symmetrically: translate so the base sits
    // at the frame origin, pointing along the frame's +X aim axis.
    GfMatrix4d xform = frame;
    if (isCone) {
        GfMatrix4d local(1.0);
        local.SetTranslate(GfVec3d(length / 2.0, 0, 0));
        xform = local * frame;
    }

    static const TfToken primvarTokens[] = {
        HdTokens->displayColor, HdTokens->displayOpacity};
    const HdDataSourceBaseHandle primvarValues[] = {
        HdPrimvarSchema::Builder()
            .SetPrimvarValue(
                HdRetainedTypedSampledDataSource<VtVec3fArray>::New(
                    VtVec3fArray{published.guideColor}))
            .SetInterpolation(_Token(HdPrimvarSchemaTokens->constant))
            .SetRole(_Token(HdPrimvarSchemaTokens->color))
            .Build(),
        HdPrimvarSchema::Builder()
            .SetPrimvarValue(
                HdRetainedTypedSampledDataSource<VtFloatArray>::New(
                    VtFloatArray{published.guideOpacity}))
            .SetInterpolation(_Token(HdPrimvarSchemaTokens->constant))
            .Build()};

    // Composed with the ASSET ROOT's world transform, and declared final.
    //
    // guideFrames are ASSET-space -- the space rig frames live in, since
    // rest:space and avars carry no stage placement. Publishing one directly
    // drew every guide at the world origin no matter where the asset was
    // placed, and nothing downstream composes it for us.
    //
    // The asset root, NOT the guide's parent. A joint may sit under an
    // intervening UsdGeomXform inside the asset (joints are discovered
    // anywhere beneath the rig), and that Xform's contribution is already
    // baked into the rig's own frames -- composing the parent's flattened
    // matrix would apply it a second time. Asset at +100 with an internal
    // Xform at +7 would put an identity guide frame at +107 instead of +100.
    //
    // resetXformStack is then true because the result is fully composed --
    // the same reason the driven-Xform path sets it (see GetPrim).
    xform = xform * assetRootWorld;

    TfTokenVector names;
    std::vector<HdDataSourceBaseHandle> values;
    names.push_back(HdXformSchemaTokens->xform);
    values.push_back(
        HdXformSchema::Builder()
            .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(
                xform))
            .SetResetXformStack(
                HdRetainedTypedSampledDataSource<bool>::New(true))
            .Build());
    names.push_back(HdPurposeSchema::GetSchemaToken());
    values.push_back(
        HdPurposeSchema::Builder()
            .SetPurpose(_Token(HdRenderTagTokens->guide))
            .Build());
    // INHERITED BY HAND from the parent this guide hangs off: visibility,
    // so hiding a joint hides its guides; and primOrigin, so picking one
    // selects the joint.
    //
    // These prims exist only in this scene index. Nothing upstream knows
    // about them -- we are downstream of the flattening that resolves
    // inherited state -- and nothing downstream can map them back to the
    // stage, because they have no USD counterpart at all. Both problems are
    // ours alone to solve, and both have the same shape.
    //
    // Without visibility a synthesized prim falls back to VISIBLE, so guides
    // floated over a hidden rig. Without primOrigin, picking reports the
    // guide's own path, usdview resolves it with
    // UsdStage::GetPrimAtPath (appController.py onPrimSelected), gets
    // nothing, and the click silently selects nothing -- forwarding the
    // parent's origin makes a guide select its joint instead.
    //
    // This is exactly what UsdImaging does for its own synthesized prims
    // (usdImaging/drawModeStandin.cpp `_PrimDataSource`, which forwards the
    // same two from the model prim). The parent's data source is already
    // flattened, so one read carries the whole inheritance chain, and
    // reusing its handles keeps any time sampling intact.
    if (parentDataSource) {
        if (const HdDataSourceBaseHandle visibility =
                HdContainerDataSource::Get(
                    parentDataSource,
                    HdVisibilitySchema::GetDefaultLocator())) {
            names.push_back(HdVisibilitySchema::GetSchemaToken());
            values.push_back(visibility);
        }
        if (const HdDataSourceBaseHandle origin =
                HdContainerDataSource::Get(
                    parentDataSource,
                    HdPrimOriginSchema::GetDefaultLocator())) {
            names.push_back(HdPrimOriginSchema::GetSchemaToken());
            values.push_back(origin);
        }
    }
    names.push_back(HdPrimvarsSchemaTokens->primvars);
    values.push_back(HdRetainedContainerDataSource::New(
        2, primvarTokens, primvarValues));
    // Sizing follows Ir (UsdIrImagingJointScopeAdapter) in shape -- cone
    // height from the guide length, drawn along the +X aim axis -- but the
    // radius is authored rather than left to Hydra's fallback of 1.0, which
    // is only the right size at arm scale. It MUST be set explicitly on both
    // primitives: an absent radius silently becomes 1.0
    // (hdsi/implicitSurfaceSceneIndex.cpp), which is the bug this replaced.
    const HdDataSourceBaseHandle radiusSource =
        HdRetainedTypedSampledDataSource<double>::New(radius);
    if (isCone) {
        names.push_back(HdConeSchema::GetSchemaToken());
        values.push_back(
            HdConeSchema::Builder()
                .SetHeight(HdRetainedTypedSampledDataSource<double>::New(
                    length))
                .SetRadius(HdDoubleDataSource::Cast(radiusSource))
                .SetAxis(_Token(HdConeSchemaTokens->X))
                .Build());
    } else {
        names.push_back(HdSphereSchema::GetSchemaToken());
        values.push_back(
            HdSphereSchema::Builder()
                .SetRadius(HdDoubleDataSource::Cast(radiusSource))
                .Build());
    }
    return HdRetainedContainerDataSource::New(
        names.size(), names.data(), values.data());
}

// Reject an inversion whose linear block is this badly conditioned. This is a
// reciprocal-condition proxy -- ||A|| * ||A^-1|| on the 3x3 -- which is
// dimensionless and scale-free, unlike a determinant threshold: a uniform
// scale of 1e-4 has determinant 1e-12 but condition number 1, while
// diag(1e20, 1e-20, 1) has determinant 1 and is numerically hopeless.
constexpr double _kMaxConditionProxy = 1e12;

// Largest absolute element of the 3x3 linear block.
double
_MaxAbsLinear(const GfMatrix4d &m)
{
    double result = 0.0;
    for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 3; ++c) {
            result = std::max(result, std::abs(m[r][c]));
        }
    }
    return result;
}

bool
_AllFinite(const GfMatrix4d &m)
{
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            if (!std::isfinite(m[r][c])) {
                return false;
            }
        }
    }
    return true;
}

// Inverts \p m, or fails. GfMatrix4d::GetInverse() does not report failure --
// it returns FLT_MAX * identity -- so an unchecked inverse turns a zero-scale
// prim (an ordinary way to hide geometry) into components around 1e77.
bool
_TryInvert(const GfMatrix4d &m, GfMatrix4d *inverse)
{
    if (!_AllFinite(m)) {
        return false;
    }
    *inverse = m.GetInverse();
    if (!_AllFinite(*inverse)) {
        return false;
    }
    // Written so a NaN proxy fails the test rather than passing it.
    const double proxy = _MaxAbsLinear(m) * _MaxAbsLinear(*inverse);
    if (!(proxy < _kMaxConditionProxy)) {
        return false;
    }

    // ...and verify the inverse actually inverts.
    //
    // GfMatrix4d::GetInverse() signals failure by returning
    // SetScale(FLT_MAX) (matrix4d.cpp:400). That sentinel is FINITE, and
    // when the source matrix is tiny the product of magnitudes stays small:
    // diag(0, 1e-30, 1e-30) yields a proxy of only ~3.4e8, well under the
    // conditioning bound, so neither check above rejects it. The residual
    // does -- m * FLT_MAX*I is nowhere near identity.
    //
    // The tolerance is loose on purpose: anything we accept has a proxy
    // below 1e12, so its worst-case residual is around 1e-16 * 1e12 = 1e-4,
    // while the sentinel misses by many orders of magnitude.
    const GfMatrix4d residual = m * (*inverse);
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            const double expected = (r == c) ? 1.0 : 0.0;
            if (!(std::abs(residual[r][c] - expected) <= 1e-3)) {
                return false;
            }
        }
    }
    return true;
}

// The xform matrix a prim's data source carries, or identity when it has
// none. Upstream of us that value is already world-space (flattened).
GfMatrix4d
_ReadXform(const HdContainerDataSourceHandle &dataSource)
{
    if (!dataSource) {
        return GfMatrix4d(1.0);
    }
    HdXformSchema schema = HdXformSchema::GetFromParent(dataSource);
    if (!schema) {
        return GfMatrix4d(1.0);
    }
    const HdMatrixDataSourceHandle matrix = schema.GetMatrix();
    return matrix ? matrix->GetTypedValue(0.0) : GfMatrix4d(1.0);
}

// Builds the sparse stronger root container for one published prim
// (spec §10.3.1): standard schemas only.
HdContainerDataSourceHandle
_BuildStrongRoot(const RigExecPublishedPrim &published)
{
    TfTokenVector names;
    std::vector<HdDataSourceBaseHandle> values;

    // NOTE: xform is deliberately absent here. Unlike points/normals/extent,
    // the published transform cannot be turned into a data source from the
    // snapshot alone -- it is a LOCAL matrix arriving at a scene index that
    // sits downstream of flattening, so it has to be combined with the
    // upstream world matrix. _ComputeDrivenXform does that in GetPrim, where
    // the input prim is in hand.

    TfTokenVector primvarNames;
    std::vector<HdDataSourceBaseHandle> primvarValues;
    if (published.hasPoints) {
        // points maps to the flat primvars/points/primvarValue leaf; a
        // stock built-in is always flat (spec §10.3.1).
        primvarNames.push_back(HdTokens->points);
        const HdSampledDataSourceHandle pointsSource =
            published.sampleOffsets.size() ==
                    published.pointsSamples.size() &&
                !published.sampleOffsets.empty()
                ? HdSampledDataSourceHandle(_SampledPointsDataSource::New(
                      published.points, published.sampleOffsets,
                      published.pointsSamples))
                : HdSampledDataSourceHandle(
                      HdRetainedTypedSampledDataSource<VtVec3fArray>::New(
                          published.points));
        primvarValues.push_back(
            HdPrimvarSchema::Builder()
                .SetPrimvarValue(pointsSource)
                .SetInterpolation(_Token(HdPrimvarSchemaTokens->vertex))
                .SetRole(_Token(HdPrimvarSchemaTokens->point))
                .Build());
        // Whenever the epoch owns/publishes final points, the overlay
        // blocks the complete upstream velocities and accelerations
        // entries: ownership, not value comparison (spec §10.5).
        primvarNames.push_back(HdTokens->velocities);
        primvarValues.push_back(HdBlockDataSource::New());
        primvarNames.push_back(HdTokens->accelerations);
        primvarValues.push_back(HdBlockDataSource::New());
    }
    if (published.hasNormals) {
        primvarNames.push_back(HdTokens->normals);
        primvarValues.push_back(
            HdPrimvarSchema::Builder()
                .SetPrimvarValue(
                    HdRetainedTypedSampledDataSource<VtVec3fArray>::New(
                        published.normals))
                .SetInterpolation(_Token(HdPrimvarSchemaTokens->vertex))
                .SetRole(_Token(HdPrimvarSchemaTokens->normal))
                .Build());
    }
    if (!primvarNames.empty()) {
        names.push_back(HdPrimvarsSchemaTokens->primvars);
        values.push_back(HdRetainedContainerDataSource::New(
            primvarNames.size(), primvarNames.data(), primvarValues.data()));
    }

    if (published.hasExtent) {
        names.push_back(HdExtentSchemaTokens->extent);
        values.push_back(
            HdExtentSchema::Builder()
                .SetMin(HdRetainedTypedSampledDataSource<GfVec3d>::New(
                    published.extentMin))
                .SetMax(HdRetainedTypedSampledDataSource<GfVec3d>::New(
                    published.extentMax))
                .Build());
    }

    if (names.empty()) {
        return nullptr;
    }
    return HdRetainedContainerDataSource::New(
        names.size(), names.data(), values.data());
}

}  // namespace

// The world transform \p primPath should have once every constraint-driven
// ancestor (and possibly itself) is accounted for, or nothing when no driven
// ancestor applies.
//
// Why this is a delta rather than a straight overwrite: this scene index is
// installed through UsdImagingSceneIndexPlugin::AppendSceneIndex, which
// UsdImagingCreateSceneIndices calls AFTER building
// UsdImagingNiPrototypePropagatingSceneIndex -- and the only
// HdFlatteningSceneIndex in the imaging chain lives inside that. So by the
// time prims reach us their transforms are world-space and every descendant's
// world transform has ALREADY been composed from the driven prim's original
// matrix. Publishing the revised local matrix onto the driven prim alone
// would move that prim (visible in usdview's scene index debugger) while
// every mesh parented under it kept its stale world transform (a static
// viewport) -- which is the entire point of driving an Xform.
//
// USD is row-vector, so world = local * parentWorld. For a driven prim A:
//
//   W_old = L_old * P          W_new = L_new * P          P = L_old^-1 * W_old
//   => W_new = L_new * L_old^-1 * W_old
//
// A descendant D has F_old = C * W_old for the accumulated locals C between
// D and A, so F_new = C * W_new = F_old * (W_old^-1 * W_new). That gives one
// delta, post-multiplied, applying uniformly to A and to everything beneath:
//
//   delta = W_old^-1 * L_new * L_old^-1 * W_old
//
// Nested driven ancestors compose by post-multiplying innermost-first; each
// delta is expressed in the un-revised world frame at its own level, which is
// exactly what the input scene index still reports.
bool
RigExecResultsSceneIndex::_ComputeDrivenXform(
    const SdfPath &primPath,
    const HdContainerDataSourceHandle &inputDataSource,
    const RigExecImagingSnapshot &snapshot,
    GfMatrix4d *result) const
{
    // Fast path. This runs for EVERY prim on EVERY pull, so a rig that drives
    // no transforms at all (a points-only rig, the common case) must not pay
    // for an ancestor walk of the whole scene.
    if (!snapshot.hasDrivenXforms) {
        return false;
    }

    const GfMatrix4d own = _ReadXform(inputDataSource);

    // KNOWN LIMITATION: resetXformStack is not honoured here, and cannot be.
    // A prim that resets the stack ignores its ancestors, so a driven
    // ancestor should not reach it -- but HdFlattenedXformDataSourceProvider
    // consumes the authored flag and stamps resetXformStack=true on EVERY
    // flattened prim to mark the matrix as already world-space. Downstream of
    // flattening the authored boundary is simply not recoverable, so a
    // resetXformStack descendant of a constraint-driven Xform will be carried
    // along with it rather than staying put.
    GfMatrix4d composed(1.0);
    bool driven = false;
    for (SdfPath current = primPath;
         !current.IsAbsoluteRootPath() && !current.IsEmpty();
         current = current.GetParentPath()) {
        const auto it = snapshot.prims.find(current);
        if (it == snapshot.prims.end() || !it->second.hasXform) {
            continue;
        }
        const GfMatrix4d worldOld =
            current == primPath
                ? own
                : _ReadXform(
                      _GetInputSceneIndex()->GetPrim(current).dataSource);

        // Skip an ancestor we cannot invert -- do NOT abandon the walk.
        // A collapsed dimension means no uniform right-delta exists for THAT
        // level, but a valid driven ancestor further up still has one, and
        // bailing out entirely would detach this prim from a transform that
        // its siblings do follow.
        GfMatrix4d worldOldInverse(1.0);
        GfMatrix4d baseInverse(1.0);
        if (!_TryInvert(worldOld, &worldOldInverse) ||
            !_TryInvert(it->second.xformBase, &baseInverse)) {
            continue;
        }

        composed = composed * (worldOldInverse * it->second.xform *
                               baseInverse * worldOld);
        driven = true;
    }
    if (!driven) {
        return false;
    }
    const GfMatrix4d resolved = own * composed;
    if (!_AllFinite(resolved)) {
        return false;
    }
    *result = resolved;
    return true;
}

HdSceneIndexPrim
RigExecResultsSceneIndex::GetPrim(const SdfPath &primPath) const
{
    // Synthesized guide children exist only in this index: pulls read the
    // current atomic snapshot and never compute (spec §10.3). Authored
    // prims win the name — only paths absent upstream are served as
    // guides, and only beneath a parent that still exists upstream.
    bool guideIsCone = false;
    size_t guideIndex = 0;
    if (primPath.IsPrimPath() && !primPath.IsAbsoluteRootPath() &&
        _ParseGuideName(primPath.GetNameToken(), &guideIsCone,
                        &guideIndex) &&
        !_GetInputSceneIndex()->GetPrim(primPath).dataSource) {
        const HdSceneIndexPrim parent =
            _GetInputSceneIndex()->GetPrim(primPath.GetParentPath());
        if (parent.dataSource) {
            if (const RigExecImagingSnapshotConstPtr snapshot =
                    _store->Get()) {
                const auto it =
                    snapshot->prims.find(primPath.GetParentPath());
                if (it != snapshot->prims.end() && it->second.hasGuides &&
                    guideIndex < it->second.guideFrames.size()) {
                    HdSceneIndexPrim prim;
                    prim.primType = guideIsCone ? HdPrimTypeTokens->cone
                                                : HdPrimTypeTokens->sphere;
                    prim.dataSource = _BuildGuidePrim(
                        it->second, guideIsCone, guideIndex,
                        parent.dataSource,
                        _ResolveAssetRootWorld(*snapshot));
                    return prim;
                }
            }
        }
        return HdSceneIndexPrim();
    }

    HdSceneIndexPrim prim = _GetInputSceneIndex()->GetPrim(primPath);
    // A null upstream root means no prim and therefore no RigExec
    // publication (spec §10.3.1).
    if (!prim.dataSource) {
        return prim;
    }
    const RigExecImagingSnapshotConstPtr snapshot = _store->Get();
    if (!snapshot) {
        return prim;
    }

    // The transform is resolved for EVERY prim, not just published ones: a
    // mesh parented under a constraint-driven Xform is never itself published
    // (it is carried by hierarchy, not deformed) yet its world transform
    // still has to change.
    GfMatrix4d drivenXform(1.0);
    if (_ComputeDrivenXform(primPath, prim.dataSource, *snapshot,
                            &drivenXform)) {
        static const TfToken xformName = HdXformSchemaTokens->xform;
        const HdDataSourceBaseHandle xformSource =
            HdXformSchema::Builder()
                .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(
                    drivenXform))
                // TRUE: this matrix is fully composed, exactly like the one
                // flattening produced upstream. Marking it false would invite
                // a later flattener to compose the parent in a second time,
                // and consumers that key off the flag (hdPrman's world-offset
                // index applies its offset only to matrices marked composed)
                // would skip it.
                .SetResetXformStack(
                    HdRetainedTypedSampledDataSource<bool>::New(true))
                .Build();
        prim.dataSource =
            HdOverlayContainerDataSource::OverlayedContainerDataSources(
                HdRetainedContainerDataSource::New(1, &xformName,
                                                   &xformSource),
                prim.dataSource);
    }

    const auto it = snapshot->prims.find(primPath);
    if (it == snapshot->prims.end()) {
        return prim;
    }
    if (HdContainerDataSourceHandle strongRoot =
            _BuildStrongRoot(it->second)) {
        // Container-on-container overlap composes recursively: owned
        // value leaves win while topology, descriptors, and unrelated
        // schemas continue to come from upstream (spec §10.3.1).
        prim.dataSource =
            HdOverlayContainerDataSource::OverlayedContainerDataSources(
                strongRoot, prim.dataSource);
    }
    return prim;
}

SdfPathVector
RigExecResultsSceneIndex::GetChildPrimPaths(const SdfPath &primPath) const
{
    SdfPathVector children =
        _GetInputSceneIndex()->GetChildPrimPaths(primPath);
    if (const RigExecImagingSnapshotConstPtr snapshot = _store->Get()) {
        const auto it = snapshot->prims.find(primPath);
        if (it != snapshot->prims.end() && it->second.hasGuides &&
            _GetInputSceneIndex()->GetPrim(primPath).dataSource) {
            for (size_t i = 0; i < it->second.guideFrames.size(); ++i) {
                for (const bool isCone : {false, true}) {
                    const SdfPath childPath =
                        primPath.AppendChild(_GuideName(isCone, i));
                    if (std::find(children.begin(), children.end(),
                                  childPath) == children.end()) {
                        children.push_back(childPath);
                    }
                }
            }
        }
    }
    return children;
}

// How many guide elements this prim should have in the current generation.
// This is also exactly what a traversing observer will find, because
// GetChildPrimPaths synthesizes the same set.
size_t
RigExecResultsSceneIndex::_DesiredGuideCount(const SdfPath &path) const
{
    const RigExecImagingSnapshotConstPtr snapshot = _store->Get();
    if (!snapshot) {
        return 0;
    }
    const auto it = snapshot->prims.find(path);
    if (it == snapshot->prims.end() || !it->second.hasGuides ||
        !_GetInputSceneIndex()->GetPrim(path).dataSource) {
        return 0;
    }
    return it->second.guideFrames.size();
}

// Records the guide count without emitting anything. Must run even while
// unobserved: the COUNT is what tells a later observer how many children it
// once had, and without it a drop to zero sends no removals at all.
void
RigExecResultsSceneIndex::_RefreshAnnouncedGuides(const SdfPath &path)
{
    const size_t desired = _DesiredGuideCount(path);
    if (desired == 0) {
        _announcedGuides.erase(path);
    } else {
        _announcedGuides[path] = desired;
    }
}

void
RigExecResultsSceneIndex::_SyncGuideChildren(
    const SdfPath &path,
    HdSceneIndexObserver::AddedPrimEntries *added,
    HdSceneIndexObserver::RemovedPrimEntries *removed)
{
    const size_t desired = _DesiredGuideCount(path);
    size_t announced = 0;
    const auto it = _announcedGuides.find(path);
    if (it != _announcedGuides.end()) {
        announced = it->second;
    }
    // Authored prims win colliding names: never claim (or remove) a path
    // that exists upstream.
    auto existsUpstream = [this](const SdfPath &childPath) {
        return static_cast<bool>(
            _GetInputSceneIndex()->GetPrim(childPath).dataSource);
    };
    for (size_t i = announced; i < desired; ++i) {
        const SdfPath spherePath = path.AppendChild(_GuideName(false, i));
        if (!existsUpstream(spherePath)) {
            added->emplace_back(spherePath, HdPrimTypeTokens->sphere);
        }
        const SdfPath conePath = path.AppendChild(_GuideName(true, i));
        if (!existsUpstream(conePath)) {
            added->emplace_back(conePath, HdPrimTypeTokens->cone);
        }
    }
    for (size_t i = desired; i < announced; ++i) {
        const SdfPath spherePath = path.AppendChild(_GuideName(false, i));
        if (!existsUpstream(spherePath)) {
            removed->emplace_back(spherePath);
        }
        const SdfPath conePath = path.AppendChild(_GuideName(true, i));
        if (!existsUpstream(conePath)) {
            removed->emplace_back(conePath);
        }
    }
    if (desired == 0) {
        _announcedGuides.erase(path);
    } else {
        _announcedGuides[path] = desired;
    }
}

// Dirties \p locators on every descendant of \p path present upstream.
void
RigExecResultsSceneIndex::_DirtySubtree(
    const SdfPath &path,
    const HdDataSourceLocatorSet &locators,
    HdSceneIndexObserver::DirtiedPrimEntries *entries) const
{
    for (const SdfPath &child :
             _GetInputSceneIndex()->GetChildPrimPaths(path)) {
        entries->emplace_back(child, locators);
        _DirtySubtree(child, locators, entries);
    }
}

// Brings the driven-transform history for \p path in line with the current
// generation. Must run whether or not anyone is observing: the history is
// what tells a LATER observer that a subtree needs correcting.
void
RigExecResultsSceneIndex::_RefreshDrivenXform(const SdfPath &path)
{
    if (_IsDrivenXform(path)) {
        _announcedDrivenXforms.insert(path);
    } else {
        _announcedDrivenXforms.erase(path);
    }
}

// The asset root's world transform, as THIS index reports it.
//
// Resolved through our own GetPrim rather than the input, because the asset
// root may itself be a constraint-driven Xform: reading upstream would place
// every guide at the asset's pre-revision position and leave them behind
// whenever the rig moves its own root.
GfMatrix4d
RigExecResultsSceneIndex::_ResolveAssetRootWorld(
    const RigExecImagingSnapshot &snapshot) const
{
    if (snapshot.assetRoot.IsEmpty()) {
        return GfMatrix4d(1.0);
    }
    return _ReadXform(GetPrim(snapshot.assetRoot).dataSource);
}

// Does the current generation drive this prim's transform?
bool
RigExecResultsSceneIndex::_IsDrivenXform(const SdfPath &path) const
{
    const RigExecImagingSnapshotConstPtr snapshot = _store->Get();
    if (!snapshot) {
        return false;
    }
    const auto it = snapshot->prims.find(path);
    return it != snapshot->prims.end() && it->second.hasXform;
}

void
RigExecResultsSceneIndex::NotifyGenerationPublished(
    const RigExecPublishedDirtyVector &dirtied)
{
    if (!_IsObserved() || dirtied.empty()) {
        // Track driven transforms even with nobody listening. Notices are
        // pointless while unobserved, but the HISTORY is not: an observer
        // attaching later caches whatever GetPrim returns, and only this
        // record can tell us its subtree needs correcting when the driven
        // transform is subsequently lost.
        for (const RigExecPublishedDirty &entry : dirtied) {
            _RefreshDrivenXform(entry.path);
            _RefreshAnnouncedGuides(entry.path);
        }
        return;
    }
    // Value changes start from the narrowest logical leaves (spec §10.4
    // locator table), then expand through every rebuilt ancestor with
    // ComputeDirtyLocators() because v0.1 rebuilds the retained container
    // handles per accepted snapshot (spec §10.3). Structural/output-set
    // changes use universal dirtiness.
    HdSceneIndexObserver::DirtiedPrimEntries entries;
    HdSceneIndexObserver::AddedPrimEntries addedGuides;
    HdSceneIndexObserver::RemovedPrimEntries removedGuides;
    entries.reserve(dirtied.size());
    for (const RigExecPublishedDirty &entry : dirtied) {
        if (entry.changes & RigExecChangeStructural) {
            entries.emplace_back(
                entry.path, HdDataSourceLocatorSet::UniversalSet());

            // A driven transform APPEARING or DISAPPEARING moves the whole
            // subtree just as much as one changing value does, and every
            // hasXform transition arrives here as structural rather than as
            // RigExecChangeXform: first publication, a provider gaining or
            // losing its constraint, recompilation, and Deactivate() (which
            // publishes an empty snapshot). Hydra dirtiness is not
            // hierarchical, so without this the descendants keep whichever
            // delta was last applied -- the original bug, in a new dress.
            //
            // _announcedDrivenXforms is what makes the disappearing case
            // work: by the time we see the removal the snapshot no longer
            // mentions the prim, so the only record that its subtree needs
            // correcting is the one we kept.
            const bool wasDriven =
                _announcedDrivenXforms.count(entry.path) != 0;
            const bool isDriven = _IsDrivenXform(entry.path);
            if (wasDriven || isDriven) {
                _DirtySubtree(entry.path,
                              HdDataSourceLocatorSet::UniversalSet(),
                              &entries);
            }
            _RefreshDrivenXform(entry.path);

            // Guide children follow the published set exactly. The ones that
            // SURVIVE a count change also need dirtying: a representation
            // change reports structural and suppresses RigExecChangeGuides,
            // so without this a guide whose frame, length, radius or styling
            // changed in the same generation keeps its old value forever.
            const size_t survivingGuides = std::min(
                _announcedGuides.count(entry.path)
                    ? _announcedGuides[entry.path] : 0,
                _DesiredGuideCount(entry.path));
            _SyncGuideChildren(entry.path, &addedGuides, &removedGuides);
            for (size_t i = 0; i < survivingGuides; ++i) {
                for (const bool isCone : {false, true}) {
                    const SdfPath childPath =
                        entry.path.AppendChild(_GuideName(isCone, i));
                    if (_GetInputSceneIndex()->GetPrim(childPath).dataSource) {
                        continue;
                    }
                    entries.emplace_back(
                        childPath, HdDataSourceLocatorSet::UniversalSet());
                }
            }
            continue;
        }
        if (entry.changes & RigExecChangeGuides) {
            // Same element count (count changes are structural): the
            // synthesized children rebuild wholesale per generation.
            const auto announcedIt = _announcedGuides.find(entry.path);
            if (announcedIt == _announcedGuides.end()) {
                // Never announced (late consumer): the adds carry the
                // fresh payload; no dirtying needed.
                _SyncGuideChildren(entry.path, &addedGuides,
                                   &removedGuides);
            } else {
                for (size_t i = 0; i < announcedIt->second; ++i) {
                    for (const bool isCone : {false, true}) {
                        const SdfPath childPath =
                            entry.path.AppendChild(_GuideName(isCone, i));
                        // Authored prims occupying a guide name are not
                        // ours to dirty.
                        if (_GetInputSceneIndex()
                                ->GetPrim(childPath)
                                .dataSource) {
                            continue;
                        }
                        entries.emplace_back(
                            childPath,
                            HdDataSourceLocatorSet::UniversalSet());
                    }
                }
            }
        }
        HdDataSourceLocatorSet leaves;
        if (entry.changes & RigExecChangeXform) {
            leaves.insert(HdDataSourceLocator(
                HdXformSchemaTokens->xform, HdXformSchemaTokens->matrix));

            // ...and every descendant, by hand.
            //
            // No flattening scene index sits downstream of us to propagate
            // this (see _ComputeDrivenXform: UsdImaging builds its flattening
            // BEFORE it appends plugin scene indices). We resolve each
            // descendant's world transform ourselves in GetPrim, so we are
            // also the only thing that can announce that it changed. Without
            // this the driven Xform moves and everything parented under it
            // stays put.
            //
            // The locator set is expanded the same way the provider's own is
            // below: GetPrim hands descendants a freshly built retained
            // container each generation, and a consumer that caches the
            // container handles needs the rebuilt chain invalidated, not just
            // the leaf (HdContainerDataSourceEditor::ComputeDirtyLocators).
            _announcedDrivenXforms.insert(entry.path);
            static const HdDataSourceLocatorSet xformSubtree =
                HdContainerDataSourceEditor::ComputeDirtyLocators(
                    HdDataSourceLocatorSet{
                        HdDataSourceLocator(HdXformSchemaTokens->xform,
                                            HdXformSchemaTokens->matrix)});
            _DirtySubtree(entry.path, xformSubtree, &entries);
        }
        if (entry.changes & RigExecChangePoints) {
            leaves.insert(HdDataSourceLocator(
                HdPrimvarsSchemaTokens->primvars, HdTokens->points,
                HdPrimvarSchemaTokens->primvarValue));
            // The epoch-owned structural blocks ride with owned points
            // (spec §10.4 locator table): the complete entries.
            leaves.insert(HdDataSourceLocator(
                HdPrimvarsSchemaTokens->primvars, HdTokens->velocities));
            leaves.insert(HdDataSourceLocator(
                HdPrimvarsSchemaTokens->primvars, HdTokens->accelerations));
        }
        if (entry.changes & RigExecChangeNormals) {
            leaves.insert(HdDataSourceLocator(
                HdPrimvarsSchemaTokens->primvars, HdTokens->normals,
                HdPrimvarSchemaTokens->primvarValue));
        }
        if (entry.changes & RigExecChangeExtent) {
            leaves.insert(HdDataSourceLocator(
                HdExtentSchemaTokens->extent, HdExtentSchemaTokens->min));
            leaves.insert(HdDataSourceLocator(
                HdExtentSchemaTokens->extent, HdExtentSchemaTokens->max));
        }
        if (leaves.IsEmpty()) {
            continue;
        }
        entries.emplace_back(
            entry.path,
            HdContainerDataSourceEditor::ComputeDirtyLocators(leaves));
    }
    if (!removedGuides.empty()) {
        _SendPrimsRemoved(removedGuides);
    }
    if (!addedGuides.empty()) {
        _SendPrimsAdded(addedGuides);
    }
    if (!entries.empty()) {
        _SendPrimsDirtied(entries);
    }
}

void
RigExecResultsSceneIndex::_PrimsAdded(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::AddedPrimEntries &entries)
{
    // Upstream topology reaches us even with nobody downstream listening --
    // a filtering scene index stays attached to its input regardless -- and
    // the announcement HISTORY has to keep up either way. A parent added
    // while unobserved makes guides synthesizable; if we skip recording
    // that, a later observer traverses N guides we believe we never
    // announced, and the next drop to zero emits no removals at all.
    if (!_IsObserved()) {
        for (const auto &entry : entries) {
            _RefreshDrivenXform(entry.primPath);
            _RefreshAnnouncedGuides(entry.primPath);
        }
        return;
    }
    _SendPrimsAdded(entries);
    // A re-added path may still be driven by the current generation, and
    // _PrimsRemoved pruned its history when it went away. Restore it, or a
    // later loss would find wasDriven == false and skip the subtree.
    for (const auto &entry : entries) {
        _RefreshDrivenXform(entry.primPath);
    }
    // Upstream (re)additions can re-enable synthesized guides beneath
    // the added parents (a re-added joint shell must re-announce its
    // guide children): reconcile against the current snapshot.
    HdSceneIndexObserver::AddedPrimEntries addedGuides;
    HdSceneIndexObserver::RemovedPrimEntries removedGuides;
    for (const auto &entry : entries) {
        _announcedGuides.erase(entry.primPath);  // re-add is a resync
        _SyncGuideChildren(entry.primPath, &addedGuides, &removedGuides);
    }
    if (!removedGuides.empty()) {
        _SendPrimsRemoved(removedGuides);
    }
    if (!addedGuides.empty()) {
        _SendPrimsAdded(addedGuides);
    }
}

void
RigExecResultsSceneIndex::_PrimsRemoved(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::RemovedPrimEntries &entries)
{
    // Same reasoning as _PrimsAdded: forget the history for a removed
    // subtree even while unobserved, or a stale count outlives the prims.
    if (!_IsObserved()) {
        for (const auto &entry : entries) {
            for (auto it = _announcedGuides.begin();
                 it != _announcedGuides.end();) {
                it = it->first.HasPrefix(entry.primPath)
                    ? _announcedGuides.erase(it) : std::next(it);
            }
            for (auto it = _announcedDrivenXforms.begin();
                 it != _announcedDrivenXforms.end();) {
                it = it->HasPrefix(entry.primPath)
                    ? _announcedDrivenXforms.erase(it) : std::next(it);
            }
        }
        return;
    }
    _SendPrimsRemoved(entries);
    HdSceneIndexObserver::AddedPrimEntries addedGuides;
    HdSceneIndexObserver::RemovedPrimEntries removedGuides;
    std::set<SdfPath> revealedParents;
    for (const auto &entry : entries) {
        // Subtree removal removes announced guide children with it:
        // forget their announcements.
        for (auto it = _announcedGuides.begin();
             it != _announcedGuides.end();) {
            if (it->first.HasPrefix(entry.primPath)) {
                it = _announcedGuides.erase(it);
            } else {
                ++it;
            }
        }
        // Likewise for driven transforms: a removed prim has no subtree left
        // to correct, and holding the path would resurrect it as a phantom
        // "was driven" on any later structural publication at that path.
        for (auto it = _announcedDrivenXforms.begin();
             it != _announcedDrivenXforms.end();) {
            if (it->HasPrefix(entry.primPath)) {
                it = _announcedDrivenXforms.erase(it);
            } else {
                ++it;
            }
        }
        // Removing an authored prim that occupied a guide name reveals
        // the synthesized guide again: collect the parent for exactly
        // one re-announcement (batched sibling collisions must not
        // duplicate adds).
        bool isCone = false;
        size_t index = 0;
        if (entry.primPath.IsPrimPath() &&
            !entry.primPath.IsAbsoluteRootPath() &&
            _ParseGuideName(entry.primPath.GetNameToken(), &isCone,
                            &index)) {
            revealedParents.insert(entry.primPath.GetParentPath());
        }
    }
    for (const SdfPath &parent : revealedParents) {
        _announcedGuides.erase(parent);
        _SyncGuideChildren(parent, &addedGuides, &removedGuides);
    }
    if (!removedGuides.empty()) {
        _SendPrimsRemoved(removedGuides);
    }
    if (!addedGuides.empty()) {
        _SendPrimsAdded(addedGuides);
    }
}

void
RigExecResultsSceneIndex::_PrimsDirtied(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::DirtiedPrimEntries &entries)
{
    if (!_IsObserved()) {
        return;
    }
    // A synthesized guide reads its visibility AND its parent's world
    // transform from that parent, so a parent whose either changed has to
    // drag its guide children along -- they are invisible to the flattener
    // upstream and to every observer downstream, so nothing else will.
    HdSceneIndexObserver::DirtiedPrimEntries forwarded(entries);
    static const HdDataSourceLocatorSet inheritedLocators =
        HdContainerDataSourceEditor::ComputeDirtyLocators(
            HdDataSourceLocatorSet{HdVisibilitySchema::GetDefaultLocator(),
                                   HdXformSchema::GetDefaultLocator()});
    for (const auto &entry : entries) {
        if (!entry.dirtyLocators.Intersects(
                HdVisibilitySchema::GetDefaultLocator()) &&
            !entry.dirtyLocators.Intersects(
                HdXformSchema::GetDefaultLocator())) {
            continue;
        }
        const auto announced = _announcedGuides.find(entry.primPath);
        if (announced == _announcedGuides.end()) {
            continue;
        }
        for (size_t i = 0; i < announced->second; ++i) {
            for (const bool isCone : {false, true}) {
                const SdfPath childPath =
                    entry.primPath.AppendChild(_GuideName(isCone, i));
                // Authored prims win colliding names, as everywhere else
                // here: one is not ours to dirty, and forwarding it would
                // duplicate the notice upstream already sent.
                if (_GetInputSceneIndex()->GetPrim(childPath).dataSource) {
                    continue;
                }
                forwarded.emplace_back(childPath, inheritedLocators);
            }
        }
    }
    _SendPrimsDirtied(forwarded);
}

}  // namespace rigExec
