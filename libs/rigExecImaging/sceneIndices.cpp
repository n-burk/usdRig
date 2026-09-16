//
// RigExec Hydra scene-index filters implementation (spec §10).
//
#include "sceneIndices.h"

#include "pxr/base/gf/range3d.h"
#include "pxr/imaging/hd/basisCurvesSchema.h"
#include "pxr/imaging/hd/basisCurvesTopologySchema.h"
#include "pxr/imaging/hd/coneSchema.h"
#include "pxr/imaging/hd/containerDataSourceEditor.h"
#include "pxr/imaging/hd/cubeSchema.h"
#include "pxr/imaging/hd/dataSource.h"
#include "pxr/imaging/hd/extentSchema.h"
#include "pxr/imaging/hd/legacyDisplayStyleSchema.h"
#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
#include "pxr/imaging/hd/overlayContainerDataSource.h"
#include "pxr/imaging/hd/primOriginSchema.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/purposeSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/sphereSchema.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/imaging/hd/visibilitySchema.h"
#include "pxr/imaging/hd/xformSchema.h"

#include <algorithm>
#include <cmath>
#include <tuple>

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
    // Guide topology is seeded for the same reason: a late observer's initial
    // traversal finds the synthesized children, so this index must already
    // believe it announced them or a later drop to zero sends no removals
    // and the stale guides survive downstream.
    if (const RigExecImagingSnapshotConstPtr snapshot =
            _store ? _store->Get() : nullptr) {
        for (const auto &[path, published] : snapshot->prims) {
            if (published.hasXform) {
                _announcedDrivenXforms.insert(path);
            }
            // Through _DesiredGuideTopology, not the raw payload size: it also
            // requires the parent to exist upstream, which is what a
            // traversal actually finds. Seeding the raw count for a prim
            // whose upstream parent is absent would later emit removals for
            // children that never existed.
            _RefreshAnnouncedGuides(path);
            _RefreshAnnouncedControlGuide(path);
            _RefreshAnnouncedVolumeGuides(path);
        }
    }
}

namespace {

// Snapshot-backed sampled points source (spec 10.5): reports the retained
// frame-relative offsets and returns only cached values; pulls can never
// trigger evaluation.
template <class T>
class _MotionDataSource final : public HdTypedSampledDataSource<T> {
public:
    HD_DECLARE_DATASOURCE(_MotionDataSource);
    using Time = HdSampledDataSource::Time;

    VtValue GetValue(Time shutterOffset) override {
        return VtValue(GetTypedValue(shutterOffset));
    }

    T GetTypedValue(Time shutterOffset) override {
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
    _MotionDataSource(T base, std::vector<float> offsets, std::vector<T> samples)
        : _base(std::move(base))
        , _offsets(std::move(offsets))
        , _samples(std::move(samples)) {}

    T _base;
    std::vector<float> _offsets;
    std::vector<T> _samples;
};

template <class T>
typename HdTypedSampledDataSource<T>::Handle
_MotionValue(const T &base, const std::vector<float> &offsets,
             const std::vector<T> &samples)
{
    if (!samples.empty() && samples.size() == offsets.size())
        return _MotionDataSource<T>::New(base, offsets, samples);
    return HdRetainedTypedSampledDataSource<T>::New(base);
}

HdTokenDataSourceHandle
_Token(const TfToken &token)
{
    return HdRetainedTypedSampledDataSource<TfToken>::New(token);
}

// ---------------------------------------------------------------------------
// Synthesized guide children (spec §10.3 extension): joints draw a sphere at
// each posed origin plus a cone to every nested child, while aggregate
// solvers draw a sphere/cone pair along each frame's +X aim axis. Both use
// purpose guide and constant displayColor/displayOpacity primvars.
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

// A guide element has a cone exactly when its derived/resolved length is
// positive. Zero is a sphere-only leaf-joint representation, not a
// degenerate cone.
bool
_GuideHasCone(const RigExecPublishedPrim &published, size_t index)
{
    return index < published.guideLengths.size() &&
           std::isfinite(published.guideLengths[index]) &&
           published.guideLengths[index] > 0.0;
}

bool
_GuideHasSphere(const RigExecPublishedPrim &published, size_t index)
{
    return index < published.guideFrames.size() &&
           (index >= published.guideDrawSpheres.size() ||
            published.guideDrawSpheres[index]);
}

// The constant styling every synthesized guide carries, from the authored
// guide:displayColor / guide:displayOpacity of the prim it hangs off, plus
// the guide's own points when it is an explicit primitive rather than one
// of Hydra's implicits, a constant width when it is a wire curve, and
// optional normals when an explicit mesh provides them.
//
// \p wireWidth of zero authors no widths at all, which is the hairline
// fallback and the only thing the joint sphere/cone guides ever want.
HdContainerDataSourceHandle
_BuildGuideStylePrimvars(
    const RigExecPublishedPrim &published,
    const VtVec3fArray &points = VtVec3fArray(),
    double wireWidth = 0.0,
    const VtVec3fArray &normals = VtVec3fArray(),
    const TfToken &normalsInterpolation = TfToken())
{
    TfTokenVector names{HdTokens->displayColor, HdTokens->displayOpacity};
    std::vector<HdDataSourceBaseHandle> values{
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
    if (!points.empty()) {
        names.push_back(HdTokens->points);
        values.push_back(
            HdPrimvarSchema::Builder()
                .SetPrimvarValue(
                    HdRetainedTypedSampledDataSource<VtVec3fArray>::New(
                        points))
                .SetInterpolation(_Token(HdPrimvarSchemaTokens->vertex))
                .SetRole(_Token(HdPrimvarSchemaTokens->point))
                .Build());
    }
    if (wireWidth > 0.0) {
        // Constant interpolation: one width for the whole curve set. The
        // value is in the curve's LOCAL space, which the guide's xform then
        // scales along with the points -- so a non-uniform guide scale
        // thickens the curve anisotropically, exactly as it stretches the
        // shape it belongs to.
        names.push_back(HdTokens->widths);
        values.push_back(
            HdPrimvarSchema::Builder()
                .SetPrimvarValue(
                    HdRetainedTypedSampledDataSource<VtFloatArray>::New(
                        VtFloatArray{static_cast<float>(wireWidth)}))
                .SetInterpolation(_Token(HdPrimvarSchemaTokens->constant))
                .Build());
    }
    if (!normals.empty()) {
        names.push_back(HdTokens->normals);
        values.push_back(
            HdPrimvarSchema::Builder()
                .SetPrimvarValue(
                    HdRetainedTypedSampledDataSource<VtVec3fArray>::New(
                        normals))
                .SetInterpolation(_Token(
                    normalsInterpolation.IsEmpty()
                        ? HdPrimvarSchemaTokens->vertex
                        : normalsInterpolation))
                .SetRole(_Token(HdPrimvarSchemaTokens->normal))
                .Build());
    }
    return HdRetainedContainerDataSource::New(
        names.size(), names.data(), values.data());
}

// INHERITED BY HAND from the parent a guide hangs off: visibility, so
// hiding a joint or control hides its guides; and primOrigin, so picking
// one selects that prim.
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
void
_AppendInheritedGuideState(
    const HdContainerDataSourceHandle &parentDataSource,
    TfTokenVector *names, std::vector<HdDataSourceBaseHandle> *values)
{
    if (!parentDataSource) {
        return;
    }
    if (const HdDataSourceBaseHandle visibility =
            HdContainerDataSource::Get(
                parentDataSource, HdVisibilitySchema::GetDefaultLocator())) {
        names->push_back(HdVisibilitySchema::GetSchemaToken());
        values->push_back(visibility);
    }
    if (const HdDataSourceBaseHandle origin =
            HdContainerDataSource::Get(
                parentDataSource, HdPrimOriginSchema::GetDefaultLocator())) {
        names->push_back(HdPrimOriginSchema::GetSchemaToken());
        values->push_back(origin);
    }
}

// The Hydra render tag for a published prim's resolved USD purpose.
//
// One mapping for joints, solvers, and controls alike: BBoxCache buckets a
// prim's extent by its purpose, so the tag its guide draws under has to be
// derived from the same value or the two disagree -- a guide that renders
// as a diagnostic while its bounds count as ordinary geometry, or worse
// the reverse.
const TfToken &
_GuideRenderTag(const RigExecPublishedPrim &published)
{
    // All four allowed purposes, not just guide-or-not: `render` and
    // `proxy` are legal on any Imageable, and collapsing them into
    // geometry would draw a proxy-purpose guide in the geometry pass while
    // BBoxCache filed its bounds under proxy -- the same disagreement this
    // mapping exists to prevent, just in a less-travelled corner.
    if (published.guidePurpose == UsdGeomTokens->guide) {
        return HdRenderTagTokens->guide;
    }
    if (published.guidePurpose == UsdGeomTokens->proxy) {
        return HdRenderTagTokens->proxy;
    }
    if (published.guidePurpose == UsdGeomTokens->render) {
        return HdRenderTagTokens->render;
    }
    return HdRenderTagTokens->geometry;
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
            .SetPurpose(_Token(_GuideRenderTag(published)))
            .Build());
    _AppendInheritedGuideState(parentDataSource, &names, &values);
    names.push_back(HdPrimvarsSchemaTokens->primvars);
    values.push_back(_BuildGuideStylePrimvars(published));
    // Sizing follows Ir (UsdIrImagingJointScopeAdapter) in shape -- cone
    // height from the derived link length, drawn along the link frame's +X
    // axis -- but the radius is authored rather than left to Hydra's fallback
    // of 1.0, which is only the right size at arm scale. It MUST be set on both
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

// ---------------------------------------------------------------------------
// Synthesized control guides (spec §10.3 extension): a RigExecControl draws
// ONE shape at its posed frame, chosen by guide:shape and guide:drawMode and
// sized by the evaluated frame-axis magnitudes times guide:scaleX/Y/Z. Same
// synthesis, announcement, and dirtying machinery as the joint sphere/cone
// children above — only the geometry and the child naming differ.
// ---------------------------------------------------------------------------

// The single fixed child name. No index suffix, unlike the joint guides:
// there is exactly one guide per control, so a name that had to be parsed
// back into a number would be carrying information that does not exist.
const TfToken _controlGuideName("rigGuideCtrl");

// One unit shape, ready to publish.
//
// Every shape here is a UNIT shape centred on the frame origin with
// half-extent 1: sphere and circle have radius 1, box and cube span ±1,
// diamond (an octahedron) has its vertices at ±1 on each axis, and pyramid
// has base corners (±1, -1, ±1) with apex (0, 1, 0). circle and box are the
// planar pair — normal +Y, drawn in the local XZ plane — while cube is the
// 3D box.
//
// Nothing in here depends on the pose or on the effective scale (both live in
// the xform), so the whole table is built once and every control that draws
// a given shape shares the same arrays.
struct _ControlGuideShape {
    TfToken shape;
    TfToken drawMode;
    TfToken primType;
    /// Empty for the implicit primitives: Hydra's own sphere and cube carry
    /// their dimensions in their schemas rather than in points.
    VtVec3fArray points;
    /// curveVertexCounts for basisCurves, faceVertexCounts for meshes.
    VtIntArray counts;
    /// faceVertexIndices; empty for basisCurves, whose vertices are
    /// consumed consecutively.
    VtIntArray indices;
    /// The shape's own local bound, published as HdExtentSchema so a
    /// renderer that culls or frames on extent sees a guide of the right
    /// size. Meaningless (and unpublished) for the implicits.
    GfVec3d extentMin{0};
    GfVec3d extentMax{0};
};

// std::acos(-1) rather than M_PI: the latter is not a standard C++ macro
// and needs _USE_MATH_DEFINES on MSVC, which this tree builds under.
const double _kPi = std::acos(-1.0);
// Ring resolution shared by every round guide: 32 segments, which is 33
// points once the ring is closed by repeating the first.
constexpr int _kGuideRingSegments = 32;

// Appends one unit ring in the plane spanned by axes \p axisA and \p axisB,
// traversed in \p direction (-1 reverses it, which is what makes a planar
// mesh face +Y rather than -Y).
//
// \p close repeats the first point at the end. Linear nonperiodic curves
// have no wrap, so restating the start vertex is what closes a ring; that
// costs one vertex and avoids the linear-periodic path entirely.
void
_AppendGuideRing(
    int segments, int axisA, int axisB, double direction, bool close,
    VtVec3fArray *points)
{
    for (int i = 0; i < segments + (close ? 1 : 0); ++i) {
        const double theta =
            2.0 * _kPi * double(i % segments) / double(segments);
        GfVec3f p(0.0f);
        p[axisA] = static_cast<float>(std::cos(theta));
        p[axisB] = static_cast<float>(direction * std::sin(theta));
        points->push_back(p);
    }
}

// The four corners of the unit box at height \p y, traversed in the same
// +Y-facing rotational sense the rings use, so the planar box mesh and the
// planar circle mesh face the same way.
void
_AppendGuideBoxRing(float y, bool close, VtVec3fArray *points)
{
    static const float corners[4][2] = {
        {-1, -1}, {-1, 1}, {1, 1}, {1, -1}};  // (x, z)
    for (int i = 0; i < (close ? 5 : 4); ++i) {
        points->push_back(
            GfVec3f(corners[i % 4][0], y, corners[i % 4][1]));
    }
}

std::vector<_ControlGuideShape>
_BuildControlGuideShapes()
{
    const TfToken kSphere("sphere");
    const TfToken kCircle("circle");
    const TfToken kBox("box");
    const TfToken kCube("cube");
    const TfToken kDiamond("diamond");
    const TfToken kPyramid("pyramid");
    const TfToken kWire("wire");
    const TfToken kGeometry("geometry");

    // The octahedron and the pyramid, shared by their wire and geometry
    // forms so the two cannot disagree about where the corners are.
    const GfVec3f kPlusX(1, 0, 0), kMinusX(-1, 0, 0);
    const GfVec3f kPlusY(0, 1, 0), kMinusY(0, -1, 0);
    const GfVec3f kPlusZ(0, 0, 1), kMinusZ(0, 0, -1);

    std::vector<_ControlGuideShape> shapes;
    auto add = [&shapes](const TfToken &shape, const TfToken &drawMode,
                         const TfToken &primType, VtVec3fArray points,
                         VtIntArray counts, VtIntArray indices) {
        _ControlGuideShape entry;
        entry.shape = shape;
        entry.drawMode = drawMode;
        entry.primType = primType;
        entry.points = std::move(points);
        entry.counts = std::move(counts);
        entry.indices = std::move(indices);
        GfRange3d bound;
        for (const GfVec3f &p : entry.points) {
            bound.UnionWith(GfVec3d(p));
        }
        if (!bound.IsEmpty()) {
            entry.extentMin = bound.GetMin();
            entry.extentMax = bound.GetMax();
        }
        shapes.push_back(std::move(entry));
    };

    // ---- wire: linear nonperiodic basisCurves. The width comes from
    // guide:wireWidth at publication (see _BuildControlGuidePrim); the
    // topology below is width-free unit geometry.
    {
        VtVec3fArray points;
        _AppendGuideRing(_kGuideRingSegments, /* X */ 0, /* Z */ 2, -1.0,
                         /* close = */ true, &points);
        add(kCircle, kWire, HdPrimTypeTokens->basisCurves, points,
            VtIntArray{_kGuideRingSegments + 1}, VtIntArray());
    }
    {
        // Three orthogonal great circles: the classic wire sphere.
        VtVec3fArray points;
        _AppendGuideRing(_kGuideRingSegments, 0, 2, -1.0, true, &points);
        _AppendGuideRing(_kGuideRingSegments, 0, 1, 1.0, true, &points);
        _AppendGuideRing(_kGuideRingSegments, 1, 2, 1.0, true, &points);
        add(kSphere, kWire, HdPrimTypeTokens->basisCurves, points,
            VtIntArray{_kGuideRingSegments + 1, _kGuideRingSegments + 1,
                       _kGuideRingSegments + 1},
            VtIntArray());
    }
    {
        VtVec3fArray points;
        _AppendGuideBoxRing(0.0f, /* close = */ true, &points);
        add(kBox, kWire, HdPrimTypeTokens->basisCurves, points,
            VtIntArray{5}, VtIntArray());
    }
    {
        // Two rings plus the four vertical pillars that join them.
        VtVec3fArray points;
        _AppendGuideBoxRing(-1.0f, true, &points);
        _AppendGuideBoxRing(1.0f, true, &points);
        VtVec3fArray corners;
        _AppendGuideBoxRing(0.0f, /* close = */ false, &corners);
        for (const GfVec3f &corner : corners) {
            points.push_back(GfVec3f(corner[0], -1.0f, corner[2]));
            points.push_back(GfVec3f(corner[0], 1.0f, corner[2]));
        }
        add(kCube, kWire, HdPrimTypeTokens->basisCurves, points,
            VtIntArray{5, 5, 2, 2, 2, 2}, VtIntArray());
    }
    {
        // Three 4-segment rings in the coordinate planes pass through the
        // ±axis vertices, and their edges are EXACTLY the octahedron's
        // twelve: each ring contributes the four edges of one equator, and
        // the three equators share no edge.
        VtVec3fArray points;
        _AppendGuideRing(4, 0, 1, 1.0, true, &points);
        _AppendGuideRing(4, 1, 2, 1.0, true, &points);
        _AppendGuideRing(4, 2, 0, 1.0, true, &points);
        add(kDiamond, kWire, HdPrimTypeTokens->basisCurves, points,
            VtIntArray{5, 5, 5}, VtIntArray());
    }
    {
        // Base ring plus one segment from each base corner to the apex.
        VtVec3fArray points;
        _AppendGuideBoxRing(-1.0f, true, &points);
        VtVec3fArray corners;
        _AppendGuideBoxRing(-1.0f, /* close = */ false, &corners);
        for (const GfVec3f &corner : corners) {
            points.push_back(corner);
            points.push_back(kPlusY);
        }
        add(kPyramid, kWire, HdPrimTypeTokens->basisCurves, points,
            VtIntArray{5, 2, 2, 2, 2}, VtIntArray());
    }

    // ---- geometry: the two shapes Hydra already draws as implicits stay
    // implicits (exactly as the joint guides use them), and the rest are
    // meshes with no authored normals -- flat shading is what a guide wants.
    add(kSphere, kGeometry, HdPrimTypeTokens->sphere, VtVec3fArray(),
        VtIntArray(), VtIntArray());
    add(kCube, kGeometry, HdPrimTypeTokens->cube, VtVec3fArray(),
        VtIntArray(), VtIntArray());
    {
        // One n-gon face. Hydra triangulates it; authoring a fan here would
        // only add a centre vertex nothing needs.
        VtVec3fArray points;
        _AppendGuideRing(_kGuideRingSegments, 0, 2, -1.0,
                         /* close = */ false, &points);
        VtIntArray indices(_kGuideRingSegments);
        for (int i = 0; i < _kGuideRingSegments; ++i) {
            indices[i] = i;
        }
        add(kCircle, kGeometry, HdPrimTypeTokens->mesh, points,
            VtIntArray{_kGuideRingSegments}, indices);
    }
    {
        VtVec3fArray points;
        _AppendGuideBoxRing(0.0f, /* close = */ false, &points);
        add(kBox, kGeometry, HdPrimTypeTokens->mesh, points, VtIntArray{4},
            VtIntArray{0, 1, 2, 3});
    }
    {
        // Octahedron: four faces around +Y, four around -Y. Wound
        // counter-clockwise seen from outside, so the geometric normals a
        // renderer derives point outward even though doubleSided makes the
        // guide legible either way.
        const VtVec3fArray points{kPlusX, kMinusX, kPlusY,
                                  kMinusY, kPlusZ, kMinusZ};
        add(kDiamond, kGeometry, HdPrimTypeTokens->mesh, points,
            VtIntArray{3, 3, 3, 3, 3, 3, 3, 3},
            VtIntArray{0, 2, 4, 4, 2, 1, 1, 2, 5, 5, 2, 0,
                       0, 4, 3, 4, 1, 3, 1, 5, 3, 5, 0, 3});
    }
    {
        // Four side triangles plus the base quad, which faces -Y.
        //
        // The winding follows _AppendGuideBoxRing's rotational sense --
        // (-1,-1) -> (-1,1) -> (1,1) -> (1,-1) in (x, z), which is
        // counter-clockwise seen from +Y. So a side triangle takes its two
        // base corners in ring order and then the apex, and the base quad
        // takes the ring REVERSED, because the base is the one face whose
        // outward direction is -Y.
        VtVec3fArray points;
        _AppendGuideBoxRing(-1.0f, /* close = */ false, &points);
        points.push_back(kPlusY);
        add(kPyramid, kGeometry, HdPrimTypeTokens->mesh, points,
            VtIntArray{3, 3, 3, 3, 4},
            VtIntArray{0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4, 3, 2, 1, 0});
    }
    return shapes;
}

// The unit shape for one published shape/drawMode pair, or nothing when the
// pair names no shape this version draws.
//
// allowedTokens is documentation, not enforcement -- USD will happily
// compose `guide:shape = "teapot"` -- so an unrecognized pair has to mean
// something definite. It means no guide at all, and it means that in ONE
// place: every desired-state question about a control guide comes through
// here, so the child that GetChildPrimPaths announces and the child GetPrim
// can build are the same child by construction.
const _ControlGuideShape *
_FindControlGuideShape(const TfToken &shape, const TfToken &drawMode)
{
    static const std::vector<_ControlGuideShape> shapes =
        _BuildControlGuideShapes();
    for (const _ControlGuideShape &entry : shapes) {
        if (entry.shape == shape && entry.drawMode == drawMode) {
            return &entry;
        }
    }
    return nullptr;
}

HdContainerDataSourceHandle
_BuildControlGuidePrim(
    const RigExecPublishedPrim &published, const _ControlGuideShape &shape,
    const HdContainerDataSourceHandle &parentDataSource,
    const GfMatrix4d &assetRootWorld)
{
    // S(effectiveScale) * rigid frame * asset root placement.
    //
    // USD is row-vector, so the leftmost factor applies first: the effective
    // per-axis scale sizes the unit shape in ITS own axes, and only
    // then is the result placed by the frame (the same ordering the cone's
    // base offset uses in _BuildGuidePrim). Scaling after the frame would
    // apply the control's X scale along the world X axis rather than its own.
    //
    // The frame is orthonormalized upstream (RigExec's bridge rigidizes it).
    // controlGuideScale already contains the removed evaluated frame-axis
    // magnitudes multiplied by the positive authored guide multipliers.
    //
    // Composed with the ASSET ROOT's world transform, and declared final,
    // for exactly the reasons spelled out in _BuildGuidePrim: control frames
    // are ASSET-space, and the asset root -- not the control's namespace
    // parent -- is what places them.
    GfMatrix4d scale(1.0);
    scale.SetScale(published.controlGuideScale);
    const GfMatrix4d xform =
        scale * published.controlGuideFrame * assetRootWorld;

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
    // Purpose comes from the CONTROL's own resolved UsdGeomImageable
    // purpose -- the stock attribute, whose fallback RigExecControl leaves
    // at `default` while joints and solvers override it to `guide`. Using
    // the standard attribute rather than a RigExec token is what keeps the
    // drawn render tag and UsdGeomBBoxCache's classification of the same
    // prim in agreement by construction.
    names.push_back(HdPurposeSchema::GetSchemaToken());
    values.push_back(
        HdPurposeSchema::Builder()
            .SetPurpose(_Token(_GuideRenderTag(published)))
            .Build());
    _AppendInheritedGuideState(parentDataSource, &names, &values);
    names.push_back(HdPrimvarsSchemaTokens->primvars);
    // Width belongs to the wire draw mode alone: the meshes and the
    // implicits have no curves to widen, and authoring a stray widths
    // primvar on them would be a primvar a renderer has to decide what to
    // do with.
    values.push_back(_BuildGuideStylePrimvars(
        published, shape.points,
        shape.primType == HdPrimTypeTokens->basisCurves
            ? published.controlGuideWireWidth : 0.0));

    if (shape.primType == HdPrimTypeTokens->basisCurves &&
        published.controlGuideWireWidth > 0.0) {
        // Refine the curves, or the width above is decoration.
        //
        // Storm honours a curve's width only once the curve is REFINED:
        // HdStBasisCurves::_SupportsRefinement is `refineLevel > 0`, and
        // below that a linear basisCurves draws as one-pixel GL lines with
        // widths ignored outright. usdview's default complexity is "low",
        // which is refineLevel 0 -- so an authored width changed nothing at
        // all where it mattered most, and the control stayed as unclickable
        // as the hairline it replaced (measured: 8 hits out of 1681
        // single-pixel picks at low, 97 at high).
        //
        // Asking for refinement on the guide itself decouples that from the
        // viewer's global complexity setting, which is a display preference
        // about the ASSET and has no business deciding whether the rig's
        // controls can be clicked.
        names.push_back(HdLegacyDisplayStyleSchemaTokens->displayStyle);
        values.push_back(HdRetainedContainerDataSource::New(
            HdLegacyDisplayStyleSchemaTokens->refineLevel,
            HdRetainedTypedSampledDataSource<int>::New(1)));
    }

    if (shape.primType == HdPrimTypeTokens->basisCurves) {
        names.push_back(HdBasisCurvesSchema::GetSchemaToken());
        values.push_back(
            HdBasisCurvesSchema::Builder()
                .SetTopology(
                    HdBasisCurvesTopologySchema::Builder()
                        .SetCurveVertexCounts(
                            HdRetainedTypedSampledDataSource<
                                VtIntArray>::New(shape.counts))
                        .SetBasis(_Token(HdTokens->linear))
                        .SetType(_Token(HdTokens->linear))
                        .SetWrap(_Token(HdTokens->nonperiodic))
                        .Build())
                .Build());
    } else if (shape.primType == HdPrimTypeTokens->mesh) {
        names.push_back(HdMeshSchema::GetSchemaToken());
        values.push_back(
            HdMeshSchema::Builder()
                .SetTopology(
                    HdMeshTopologySchema::Builder()
                        .SetFaceVertexCounts(
                            HdRetainedTypedSampledDataSource<
                                VtIntArray>::New(shape.counts))
                        .SetFaceVertexIndices(
                            HdRetainedTypedSampledDataSource<
                                VtIntArray>::New(shape.indices))
                        .SetOrientation(_Token(
                            HdMeshTopologySchemaTokens->rightHanded))
                        .Build())
                // Guides are looked at from every side and have no
                // interior, so a back face is never the wrong thing to
                // draw.
                .SetDoubleSided(
                    HdRetainedTypedSampledDataSource<bool>::New(true))
                .Build());
    } else if (shape.primType == HdPrimTypeTokens->sphere) {
        // Unit radius: the authored scale sizes it, exactly like the
        // explicit shapes. Set EXPLICITLY even though 1.0 is also the
        // implicit-surface fallback — the joint guides learned that lesson
        // (see _BuildGuidePrim).
        names.push_back(HdSphereSchema::GetSchemaToken());
        values.push_back(
            HdSphereSchema::Builder()
                .SetRadius(
                    HdRetainedTypedSampledDataSource<double>::New(1.0))
                .Build());
    } else if (shape.primType == HdPrimTypeTokens->cube) {
        // Size 2 spans ±1: the same unit box the wire cube draws.
        names.push_back(HdCubeSchema::GetSchemaToken());
        values.push_back(
            HdCubeSchema::Builder()
                .SetSize(HdRetainedTypedSampledDataSource<double>::New(2.0))
                .Build());
    }
    // The shape's LOCAL bound. The xform carries the scale and the pose, so
    // this never varies per control; the implicits publish none, since
    // Hydra derives theirs from the schema.
    if (!shape.points.empty()) {
        names.push_back(HdExtentSchemaTokens->extent);
        values.push_back(
            HdExtentSchema::Builder()
                .SetMin(HdRetainedTypedSampledDataSource<GfVec3d>::New(
                    shape.extentMin))
                .SetMax(HdRetainedTypedSampledDataSource<GfVec3d>::New(
                    shape.extentMax))
                .Build());
    }
    return HdRetainedContainerDataSource::New(
        names.size(), names.data(), values.data());
}

// ---------------------------------------------------------------------------
// Synthesized volume weight guides (spec §4.1 volumetric extension, drawn
// side): one child per published iso-surface, announced and dirtied by the
// same protocol as the joint and control guides above. Only the geometry
// and the child naming differ — the payload arrives fully built from the
// bridge (see RigExecVolumeGuideElement), because a curve weight's guide is
// derived from the AUTHORED curve and has no unit form to table.
// ---------------------------------------------------------------------------

const std::string _volumeGuidePrefix("rigGuideVol_");

bool
_ParseVolumeGuideName(const TfToken &name, size_t *index)
{
    const std::string &s = name.GetString();
    if (s.compare(0, _volumeGuidePrefix.size(), _volumeGuidePrefix) != 0) {
        return false;
    }
    const std::string digits = s.substr(_volumeGuidePrefix.size());
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
_VolumeGuideName(size_t index)
{
    return TfToken(_volumeGuidePrefix + std::to_string(index));
}

HdContainerDataSourceHandle
_BuildVolumeGuidePrim(
    const RigExecPublishedPrim &published, size_t index,
    const HdContainerDataSourceHandle &parentDataSource,
    const GfMatrix4d &assetRootWorld)
{
    const RigExecVolumeGuideElement &element = published.volumeGuides[index];

    TfTokenVector names;
    std::vector<HdDataSourceBaseHandle> values;
    // Composed with the ASSET ROOT's world transform and declared final,
    // for exactly the reasons _BuildGuidePrim spells out: volume frames
    // are asset-space and nothing downstream composes them for us.
    names.push_back(HdXformSchemaTokens->xform);
    values.push_back(
        HdXformSchema::Builder()
            .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(
                element.xform * assetRootWorld))
            .SetResetXformStack(
                HdRetainedTypedSampledDataSource<bool>::New(true))
            .Build());
    names.push_back(HdPurposeSchema::GetSchemaToken());
    values.push_back(
        HdPurposeSchema::Builder()
            .SetPurpose(_Token(_GuideRenderTag(published)))
            .Build());
    _AppendInheritedGuideState(parentDataSource, &names, &values);
    names.push_back(HdPrimvarsSchemaTokens->primvars);
    values.push_back(_BuildGuideStylePrimvars(
        published, element.points, element.wireWidth, element.normals,
        element.normalsInterpolation));

    if (element.primType == HdPrimTypeTokens->basisCurves) {
        if (element.wireWidth > 0.0) {
            // Storm honours a curve width only once the curve is REFINED
            // (HdStBasisCurves::_SupportsRefinement), and usdview's default
            // complexity is refineLevel 0 -- so ask for refinement here
            // rather than let a viewer preference about the ASSET decide
            // whether the rig's guides are visible. Same reasoning, same
            // fix, as the control guides.
            names.push_back(HdLegacyDisplayStyleSchemaTokens->displayStyle);
            values.push_back(HdRetainedContainerDataSource::New(
                HdLegacyDisplayStyleSchemaTokens->refineLevel,
                HdRetainedTypedSampledDataSource<int>::New(1)));
        }
        names.push_back(HdBasisCurvesSchema::GetSchemaToken());
        values.push_back(
            HdBasisCurvesSchema::Builder()
                .SetTopology(
                    HdBasisCurvesTopologySchema::Builder()
                        .SetCurveVertexCounts(
                            HdRetainedTypedSampledDataSource<VtIntArray>::New(
                                element.counts))
                        .SetBasis(_Token(HdTokens->linear))
                        .SetType(_Token(HdTokens->linear))
                        .SetWrap(_Token(HdTokens->nonperiodic))
                        .Build())
                .Build());
    } else if (element.primType == HdPrimTypeTokens->mesh) {
        names.push_back(HdMeshSchema::GetSchemaToken());
        values.push_back(
            HdMeshSchema::Builder()
                .SetTopology(
                    HdMeshTopologySchema::Builder()
                        .SetFaceVertexCounts(
                            HdRetainedTypedSampledDataSource<VtIntArray>::New(
                                element.counts))
                        .SetFaceVertexIndices(
                            HdRetainedTypedSampledDataSource<VtIntArray>::New(
                                element.indices))
                        .SetOrientation(
                            _Token(HdMeshTopologySchemaTokens->rightHanded))
                        .Build())
                .SetDoubleSided(
                    HdRetainedTypedSampledDataSource<bool>::New(
                        element.doubleSided))
                .Build());
    } else if (element.primType == HdPrimTypeTokens->sphere) {
        // Unit radius: the iso-surface radius rides in the xform. Set
        // EXPLICITLY because an absent radius silently becomes 1.0
        // (hdsi/implicitSurfaceSceneIndex.cpp) -- which happens to be the
        // same number here, and would stop being so the moment anyone
        // changed the convention.
        names.push_back(HdSphereSchema::GetSchemaToken());
        values.push_back(
            HdSphereSchema::Builder()
                .SetRadius(HdRetainedTypedSampledDataSource<double>::New(1.0))
                .Build());
    }
    // The element's LOCAL bound, so a renderer that culls or frames on
    // extent sees a guide of the right size. The implicits publish none:
    // Hydra derives theirs from the schema.
    if (!element.points.empty()) {
        GfRange3d bound;
        for (const GfVec3f &p : element.points) {
            bound.UnionWith(GfVec3d(p));
        }
        if (!bound.IsEmpty()) {
            names.push_back(HdExtentSchemaTokens->extent);
            values.push_back(
                HdExtentSchema::Builder()
                    .SetMin(HdRetainedTypedSampledDataSource<GfVec3d>::New(
                        bound.GetMin()))
                    .SetMax(HdRetainedTypedSampledDataSource<GfVec3d>::New(
                        bound.GetMax()))
                    .Build());
        }
    }
    return HdRetainedContainerDataSource::New(
        names.size(), names.data(), values.data());
}

// The influence overlay's colour ramp: neutral grey where the field is off,
// saturated red where it is fully on.
//
// One named function so the look is tunable in one place, and so the test
// can assert the two endpoints rather than a magic triple. Linear in RGB
// rather than perceptually uniform on purpose -- the artist is reading
// WHERE the boundary is, and a linear ramp puts the visual midpoint at
// w = 0.5, which is the number they are about to type into falloffMin.
//
// Red because that is what the R&H "Voodoo" influence display used and
// what RigExecVolumeWeight's guide:displayColor already defaults to: the
// volume and the region it grabs read as one object.
GfVec3f
_WeightOverlayColor(float weight)
{
    static const GfVec3f kOff(0.55f, 0.55f, 0.55f);
    static const GfVec3f kOn(1.0f, 0.05f, 0.05f);
    // Clamped rather than extrapolated: rigExec:rangePolicy already lets a
    // volume publish out-of-range weights under `clamp`, and a colour
    // channel outside [0, 1] is a renderer's problem, not a signal.
    const float t = weight < 0.0f ? 0.0f : (weight > 1.0f ? 1.0f : weight);
    return kOff + (kOn - kOff) * t;
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
_ReadXform(const HdContainerDataSourceHandle &dataSource, float shutterOffset = 0.0f)
{
    if (!dataSource) {
        return GfMatrix4d(1.0);
    }
    HdXformSchema schema = HdXformSchema::GetFromParent(dataSource);
    if (!schema) {
        return GfMatrix4d(1.0);
    }
    const HdMatrixDataSourceHandle matrix = schema.GetMatrix();
    return matrix ? matrix->GetTypedValue(shutterOffset) : GfMatrix4d(1.0);
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
        const HdSampledDataSourceHandle pointsSource = _MotionValue(
            published.points, published.sampleOffsets, published.pointsSamples);
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
                    _MotionValue(published.normals, published.sampleOffsets,
                                 published.normalsSamples))
                .SetInterpolation(_Token(HdPrimvarSchemaTokens->vertex))
                .SetRole(_Token(HdPrimvarSchemaTokens->normal))
                .Build());
    }

    // The influence overlay, as a VERTEX displayColor primvar.
    //
    // A primvar and not a material, deliberately. This is a DIAGNOSTIC
    // that has to appear the instant a rigger selects a volume and vanish
    // the instant they deselect it, over whatever the asset is already
    // shaded with -- and it must not disturb that shading in any way a
    // later render could inherit. Binding a material would mean
    // synthesizing a material network prim, authoring a binding onto the
    // mesh, and putting the asset's real binding back afterwards: three
    // pieces of state to keep consistent, on the drawing path, for a
    // picture that is deliberately unlit and flat. displayColor is the one
    // channel every Hydra renderer already reads without any of that, and
    // it is what the joint and control guides are styled with two hundred
    // lines above -- there with CONSTANT interpolation, here with VERTEX,
    // because the whole point is that the value varies per point.
    //
    // Only emitted when the overlay is on for THIS prim: with it off the
    // container below is exactly what it always was, so a normal render is
    // untouched rather than merely unchanged-looking.
    //
    // displayOpacity is deliberately NOT published alongside. Fading the
    // mesh by weight would make a zero-weight region translucent, which
    // reads as "not there" rather than "not grabbed", and it would
    // overwrite an authored per-point opacity that the asset may be
    // relying on. The gradient carries the whole signal.
    if (published.hasWeightOverlay && !published.weightOverlay.empty()) {
        VtVec3fArray colors(published.weightOverlay.size());
        for (size_t i = 0; i < published.weightOverlay.size(); ++i) {
            colors[i] = _WeightOverlayColor(published.weightOverlay[i]);
        }
        primvarNames.push_back(HdTokens->displayColor);
        primvarValues.push_back(
            HdPrimvarSchema::Builder()
                .SetPrimvarValue(
                    HdRetainedTypedSampledDataSource<VtVec3fArray>::New(
                        colors))
                .SetInterpolation(_Token(HdPrimvarSchemaTokens->vertex))
                .SetRole(_Token(HdPrimvarSchemaTokens->color))
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
                .SetMin(_MotionValue(published.extentMin, published.sampleOffsets,
                                     published.extentMinSamples))
                .SetMax(_MotionValue(published.extentMax, published.sampleOffsets,
                                     published.extentMaxSamples))
                .Build());
    }

    if (names.empty()) {
        return nullptr;
    }
    return HdRetainedContainerDataSource::New(
        names.size(), names.data(), values.data());
}

}  // namespace

bool
RigExecControlGuideIsDrawn(const TfToken &shape, const TfToken &drawMode)
{
    return _FindControlGuideShape(shape, drawMode) != nullptr;
}

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
    GfMatrix4d *result, float shutterOffset) const
{
    // Fast path. This runs for EVERY prim on EVERY pull, so a rig that drives
    // no transforms at all (a points-only rig, the common case) must not pay
    // for an ancestor walk of the whole scene.
    if (!snapshot.hasDrivenXforms) {
        return false;
    }

    const GfMatrix4d own = _ReadXform(inputDataSource, shutterOffset);

    // Source-stage reset boundaries travel in the immutable snapshot because
    // flattening stamps resetXformStack=true on every world-space matrix.
    // Stop after a boundary's own revision; its ancestors cannot reach it.
    GfMatrix4d composed(1.0);
    bool driven = false;
    for (SdfPath current = primPath;
         !current.IsAbsoluteRootPath() && !current.IsEmpty();
         current = current.GetParentPath()) {
        const auto it = snapshot.prims.find(current);
        if (it == snapshot.prims.end() || !it->second.hasXform) {
            if (snapshot.xformResetPaths.count(current)) break;
            continue;
        }
        const GfMatrix4d worldOld =
            current == primPath
                ? own
                : _ReadXform(
                      _GetInputSceneIndex()->GetPrim(current).dataSource, shutterOffset);
        const auto &published = it->second;
        GfMatrix4d revised = published.xform, base = published.xformBase;
        if (!published.sampleOffsets.empty() &&
            published.xformSamples.size() == published.sampleOffsets.size() &&
            published.xformBaseSamples.size() == published.sampleOffsets.size()) {
            size_t sample = 0;
            for (size_t i = 1; i < published.sampleOffsets.size(); ++i) {
                if (std::abs(published.sampleOffsets[i] - shutterOffset) <
                    std::abs(published.sampleOffsets[sample] - shutterOffset)) sample = i;
            }
            revised = published.xformSamples[sample];
            base = published.xformBaseSamples[sample];
        }

        // Skip an ancestor we cannot invert -- do NOT abandon the walk.
        // A collapsed dimension means no uniform right-delta exists for THAT
        // level, but a valid driven ancestor further up still has one, and
        // bailing out entirely would detach this prim from a transform that
        // its siblings do follow.
        GfMatrix4d worldOldInverse(1.0);
        GfMatrix4d baseInverse(1.0);
        if (!_TryInvert(worldOld, &worldOldInverse) ||
            !_TryInvert(base, &baseInverse)) {
            if (snapshot.xformResetPaths.count(current)) break;
            continue;
        }

        composed = composed * (worldOldInverse * revised *
                               baseInverse * worldOld);
        driven = true;
        if (snapshot.xformResetPaths.count(current)) break;
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
                    guideIndex < it->second.guideFrames.size() &&
                    (guideIsCone
                         ? _GuideHasCone(it->second, guideIndex)
                         : _GuideHasSphere(it->second, guideIndex))) {
                    HdSceneIndexPrim prim;
                    prim.primType = guideIsCone ? HdPrimTypeTokens->cone
                                                : HdPrimTypeTokens->sphere;
                    prim.dataSource = _BuildGuidePrim(
                        it->second, guideIsCone, guideIndex,
                        parent.dataSource,
                        _ResolveAssetRootWorld(
                            it->second.assetRoot.IsEmpty()
                                ? snapshot->assetRoot
                                : it->second.assetRoot,
                            *snapshot));
                    return prim;
                }
            }
        }
        return HdSceneIndexPrim();
    }
    // The control guide follows the same rules as its sphere/cone siblings:
    // authored prims win the name, and the guide is only served while its
    // control still exists upstream.
    if (primPath.IsPrimPath() && !primPath.IsAbsoluteRootPath() &&
        primPath.GetNameToken() == _controlGuideName &&
        !_GetInputSceneIndex()->GetPrim(primPath).dataSource) {
        const HdSceneIndexPrim parent =
            _GetInputSceneIndex()->GetPrim(primPath.GetParentPath());
        if (parent.dataSource) {
            if (const RigExecImagingSnapshotConstPtr snapshot =
                    _store->Get()) {
                const auto it =
                    snapshot->prims.find(primPath.GetParentPath());
                if (it != snapshot->prims.end() &&
                    it->second.hasControlGuide) {
                    if (const _ControlGuideShape *shape =
                            _FindControlGuideShape(
                                it->second.controlGuideShape,
                                it->second.controlGuideDrawMode)) {
                        HdSceneIndexPrim prim;
                        prim.primType = shape->primType;
                        prim.dataSource = _BuildControlGuidePrim(
                            it->second, *shape, parent.dataSource,
                            _ResolveAssetRootWorld(
                                it->second.assetRoot.IsEmpty()
                                    ? snapshot->assetRoot
                                    : it->second.assetRoot,
                                *snapshot));
                        return prim;
                    }
                }
            }
        }
        return HdSceneIndexPrim();
    }
    // The volume weight iso-surfaces, on the same three rules: authored
    // prims win the name, the parent must still exist upstream, and the
    // element must exist in the current generation.
    size_t volumeIndex = 0;
    if (primPath.IsPrimPath() && !primPath.IsAbsoluteRootPath() &&
        _ParseVolumeGuideName(primPath.GetNameToken(), &volumeIndex) &&
        !_GetInputSceneIndex()->GetPrim(primPath).dataSource) {
        const HdSceneIndexPrim parent =
            _GetInputSceneIndex()->GetPrim(primPath.GetParentPath());
        if (parent.dataSource) {
            if (const RigExecImagingSnapshotConstPtr snapshot =
                    _store->Get()) {
                const auto it =
                    snapshot->prims.find(primPath.GetParentPath());
                if (it != snapshot->prims.end() &&
                    it->second.hasVolumeGuides &&
                    volumeIndex < it->second.volumeGuides.size()) {
                    HdSceneIndexPrim prim;
                    prim.primType =
                        it->second.volumeGuides[volumeIndex].primType;
                    prim.dataSource = _BuildVolumeGuidePrim(
                        it->second, volumeIndex, parent.dataSource,
                        _ResolveAssetRootWorld(
                            !it->second.volumeGuideAnchor.IsEmpty()
                                ? it->second.volumeGuideAnchor
                                : (it->second.assetRoot.IsEmpty()
                                    ? snapshot->assetRoot : it->second.assetRoot),
                            *snapshot));
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
        std::vector<float> offsets;
        for (SdfPath current = primPath; !current.IsEmpty() &&
                 !current.IsAbsoluteRootPath(); current = current.GetParentPath()) {
            const auto it = snapshot->prims.find(current);
            if (it != snapshot->prims.end() && it->second.hasXform) {
                offsets.insert(offsets.end(), it->second.sampleOffsets.begin(),
                               it->second.sampleOffsets.end());
            }
        }
        // Preserve upstream motion as well as RigExec's explicit samples.
        const auto appendUpstreamSamples = [&offsets](const HdContainerDataSourceHandle &root) {
            const auto matrix = HdXformSchema::GetFromParent(root).GetMatrix();
            if (!matrix) return;
            std::vector<float> samples;
            if (matrix->GetContributingSampleTimesForInterval(
                    offsets.empty() ? 0.0f : *std::min_element(offsets.begin(), offsets.end()),
                    offsets.empty() ? 0.0f : *std::max_element(offsets.begin(), offsets.end()),
                    &samples)) offsets.insert(offsets.end(), samples.begin(), samples.end());
        };
        appendUpstreamSamples(prim.dataSource);
        for (SdfPath current = primPath.GetParentPath(); !current.IsEmpty() &&
                 !current.IsAbsoluteRootPath(); current = current.GetParentPath()) {
            const auto it = snapshot->prims.find(current);
            if (it != snapshot->prims.end() && it->second.hasXform)
                appendUpstreamSamples(_GetInputSceneIndex()->GetPrim(current).dataSource);
        }
        std::sort(offsets.begin(), offsets.end());
        offsets.erase(std::unique(offsets.begin(), offsets.end()), offsets.end());
        std::vector<GfMatrix4d> samples;
        for (float offset : offsets) {
            GfMatrix4d value = _ReadXform(prim.dataSource, offset);
            _ComputeDrivenXform(primPath, prim.dataSource, *snapshot, &value, offset);
            samples.push_back(value);
        }
        static const TfToken xformName = HdXformSchemaTokens->xform;
        const HdDataSourceBaseHandle xformSource =
            HdXformSchema::Builder()
                .SetMatrix(_MotionValue(drivenXform, offsets, samples))
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
    const RigExecImagingSnapshotConstPtr snapshot = _store->Get();
    if (!snapshot) {
        return children;
    }
    const auto it = snapshot->prims.find(primPath);
    if (it == snapshot->prims.end() ||
        (!it->second.hasGuides && !it->second.hasControlGuide &&
         !it->second.hasVolumeGuides) ||
        !_GetInputSceneIndex()->GetPrim(primPath).dataSource) {
        return children;
    }
    // Authored prims win colliding names, so a synthesized child is only
    // added where the upstream traversal did not already report one.
    auto append = [&children](const SdfPath &childPath) {
        if (std::find(children.begin(), children.end(), childPath) ==
            children.end()) {
            children.push_back(childPath);
        }
    };
    if (it->second.hasGuides) {
        for (size_t i = 0; i < it->second.guideFrames.size(); ++i) {
            if (_GuideHasSphere(it->second, i)) {
                append(primPath.AppendChild(_GuideName(false, i)));
            }
            if (_GuideHasCone(it->second, i)) {
                append(primPath.AppendChild(_GuideName(true, i)));
            }
        }
    }
    // One fixed-name child per guide-bearing control — and only for a
    // shape/drawMode pair this version can actually build, so a traversal
    // never finds a child GetPrim would refuse to serve.
    if (it->second.hasControlGuide &&
        _FindControlGuideShape(it->second.controlGuideShape,
                               it->second.controlGuideDrawMode)) {
        append(primPath.AppendChild(_controlGuideName));
    }
    // One child per published iso-surface. Every element the bridge
    // published is drawable by construction (its primType names a prim
    // this builds), so unlike the control case there is no shape table to
    // consult first.
    if (it->second.hasVolumeGuides) {
        for (size_t i = 0; i < it->second.volumeGuides.size(); ++i) {
            append(primPath.AppendChild(_VolumeGuideName(i)));
        }
    }
    return children;
}

namespace {
constexpr uint8_t _GuideSphereBit = 1u;
constexpr uint8_t _GuideConeBit = 2u;
}

// One topology mask per guide payload element. This is exactly what traversal
// finds, including the upstream-parent gate.
std::vector<uint8_t>
RigExecResultsSceneIndex::_DesiredGuideTopology(const SdfPath &path) const
{
    const RigExecImagingSnapshotConstPtr snapshot = _store->Get();
    if (!snapshot) {
        return {};
    }
    const auto it = snapshot->prims.find(path);
    if (it == snapshot->prims.end() || !it->second.hasGuides ||
        !_GetInputSceneIndex()->GetPrim(path).dataSource) {
        return {};
    }
    std::vector<uint8_t> result(it->second.guideFrames.size(), 0u);
    for (size_t i = 0; i < result.size(); ++i) {
        if (_GuideHasSphere(it->second, i)) {
            result[i] |= _GuideSphereBit;
        }
        if (_GuideHasCone(it->second, i)) {
            result[i] |= _GuideConeBit;
        }
    }
    return result;
}

// Records the guide topology without emitting anything. Must run even while
// unobserved: this history tells a later observer which children once existed,
// including whether an element has a sphere, a cone, or both.
void
RigExecResultsSceneIndex::_RefreshAnnouncedGuides(const SdfPath &path)
{
    const std::vector<uint8_t> desired = _DesiredGuideTopology(path);
    if (desired.empty()) {
        _announcedGuides.erase(path);
    } else {
        _announcedGuides[path] = desired;
    }
}

// The prim type this prim's control guide should have in the current
// generation, or an empty token for none. Requires the parent upstream for
// the same reason _DesiredGuideTopology does: this is what a traversing
// observer finds, and GetChildPrimPaths synthesizes exactly this.
TfToken
RigExecResultsSceneIndex::_DesiredControlGuideType(const SdfPath &path) const
{
    const RigExecImagingSnapshotConstPtr snapshot = _store->Get();
    if (!snapshot) {
        return TfToken();
    }
    const auto it = snapshot->prims.find(path);
    if (it == snapshot->prims.end() || !it->second.hasControlGuide ||
        !_GetInputSceneIndex()->GetPrim(path).dataSource) {
        return TfToken();
    }
    const _ControlGuideShape *shape = _FindControlGuideShape(
        it->second.controlGuideShape, it->second.controlGuideDrawMode);
    return shape ? shape->primType : TfToken();
}

// Records the announcement without emitting anything; must run even while
// unobserved, exactly as _RefreshAnnouncedGuides must.
void
RigExecResultsSceneIndex::_RefreshAnnouncedControlGuide(const SdfPath &path)
{
    const TfToken desired = _DesiredControlGuideType(path);
    if (desired.IsEmpty()) {
        _announcedControlGuides.erase(path);
    } else {
        _announcedControlGuides[path] = desired;
    }
}

// The prim types this prim's volume guide children should have in the
// current generation, in index order; empty for none.
//
// A VECTOR of types rather than a count, because a volume guide is both:
// the number of iso-surfaces varies (a sphere whose falloffMin is zero
// draws one surface, not two) AND guide:drawMode decides what kind of prim
// each one is. Answering both questions from one desired-state function is
// what keeps GetChildPrimPaths, GetPrim, and the announcements agreeing --
// the same reason _DesiredControlGuideType returns a type.
//
// Requires the parent upstream for the same reason the other two do: this
// is what a traversing observer actually finds.
std::vector<TfToken>
RigExecResultsSceneIndex::_DesiredVolumeGuideTypes(const SdfPath &path) const
{
    const RigExecImagingSnapshotConstPtr snapshot = _store->Get();
    if (!snapshot) {
        return {};
    }
    const auto it = snapshot->prims.find(path);
    if (it == snapshot->prims.end() || !it->second.hasVolumeGuides ||
        !_GetInputSceneIndex()->GetPrim(path).dataSource) {
        return {};
    }
    std::vector<TfToken> types;
    types.reserve(it->second.volumeGuides.size());
    for (const RigExecVolumeGuideElement &element : it->second.volumeGuides) {
        types.push_back(element.primType);
    }
    return types;
}

// Records the announcement without emitting; must run even while
// unobserved, exactly as its two siblings must.
void
RigExecResultsSceneIndex::_RefreshAnnouncedVolumeGuides(const SdfPath &path)
{
    std::vector<TfToken> desired = _DesiredVolumeGuideTypes(path);
    if (desired.empty()) {
        _announcedVolumeGuides.erase(path);
    } else {
        _announcedVolumeGuides[path] = std::move(desired);
    }
}

void
RigExecResultsSceneIndex::_SyncVolumeGuideChildren(
    const SdfPath &path,
    HdSceneIndexObserver::AddedPrimEntries *added,
    HdSceneIndexObserver::RemovedPrimEntries *removed)
{
    const std::vector<TfToken> desired = _DesiredVolumeGuideTypes(path);
    const auto it = _announcedVolumeGuides.find(path);
    const std::vector<TfToken> announced =
        it != _announcedVolumeGuides.end() ? it->second
                                           : std::vector<TfToken>();
    if (desired.empty() && announced.empty()) {
        // Nothing believed on either side. Worth its own exit: every
        // published joint and control comes through here too, and this
        // spares them an upstream pull for a child nobody wants.
        return;
    }
    // Authored prims win colliding names: never claim (or remove) a path
    // that exists upstream.
    auto existsUpstream = [this](const SdfPath &childPath) {
        return static_cast<bool>(
            _GetInputSceneIndex()->GetPrim(childPath).dataSource);
    };
    for (size_t i = 0; i < desired.size(); ++i) {
        // Re-added on a TYPE change as well as on a first appearance: an
        // author switching guide:drawMode from wire to geometry keeps the
        // child but turns a basisCurves into a mesh, and a fresh
        // PrimsAdded is how a consumer learns the prim it cached is a
        // different kind of prim now.
        if (i < announced.size() && announced[i] == desired[i]) {
            continue;
        }
        const SdfPath childPath = path.AppendChild(_VolumeGuideName(i));
        if (!existsUpstream(childPath)) {
            added->emplace_back(childPath, desired[i]);
        }
    }
    for (size_t i = desired.size(); i < announced.size(); ++i) {
        const SdfPath childPath = path.AppendChild(_VolumeGuideName(i));
        if (!existsUpstream(childPath)) {
            removed->emplace_back(childPath);
        }
    }
    if (desired.empty()) {
        _announcedVolumeGuides.erase(path);
    } else {
        _announcedVolumeGuides[path] = desired;
    }
}

void
RigExecResultsSceneIndex::_SyncControlGuideChild(
    const SdfPath &path,
    HdSceneIndexObserver::AddedPrimEntries *added,
    HdSceneIndexObserver::RemovedPrimEntries *removed)
{
    const TfToken desired = _DesiredControlGuideType(path);
    const auto it = _announcedControlGuides.find(path);
    const bool announced = it != _announcedControlGuides.end();
    if (desired.IsEmpty() && !announced) {
        // Nothing to announce and nothing to take back. Worth its own exit:
        // every published joint comes through here too, and this spares it
        // an upstream pull for a child neither side believes in.
        return;
    }
    const SdfPath childPath = path.AppendChild(_controlGuideName);
    // Authored prims win colliding names: never claim (or remove) a path
    // that exists upstream.
    const bool existsUpstream = static_cast<bool>(
        _GetInputSceneIndex()->GetPrim(childPath).dataSource);
    if (!desired.IsEmpty()) {
        // Re-added on a TYPE change as well as on a first appearance: a
        // control switched from wire to geometry keeps its child but stops
        // being a basisCurves, and a fresh add is how a consumer is told
        // the prim it cached is a different kind of prim now.
        if (!existsUpstream && (!announced || it->second != desired)) {
            added->emplace_back(childPath, desired);
        }
        _announcedControlGuides[path] = desired;
    } else {
        if (announced && !existsUpstream) {
            removed->emplace_back(childPath);
        }
        _announcedControlGuides.erase(path);
    }
}

void
RigExecResultsSceneIndex::_SyncGuideChildren(
    const SdfPath &path,
    HdSceneIndexObserver::AddedPrimEntries *added,
    HdSceneIndexObserver::RemovedPrimEntries *removed)
{
    const std::vector<uint8_t> desired = _DesiredGuideTopology(path);
    std::vector<uint8_t> announced;
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
    const size_t common = std::min(announced.size(), desired.size());
    for (size_t i = announced.size(); i < desired.size(); ++i) {
        const SdfPath spherePath = path.AppendChild(_GuideName(false, i));
        if ((desired[i] & _GuideSphereBit) && !existsUpstream(spherePath)) {
            added->emplace_back(spherePath, HdPrimTypeTokens->sphere);
        }
        if (desired[i] & _GuideConeBit) {
            const SdfPath conePath =
                path.AppendChild(_GuideName(true, i));
            if (!existsUpstream(conePath)) {
                added->emplace_back(conePath, HdPrimTypeTokens->cone);
            }
        }
    }
    for (size_t i = desired.size(); i < announced.size(); ++i) {
        const SdfPath spherePath = path.AppendChild(_GuideName(false, i));
        if ((announced[i] & _GuideSphereBit) &&
            !existsUpstream(spherePath)) {
            removed->emplace_back(spherePath);
        }
        if (announced[i] & _GuideConeBit) {
            const SdfPath conePath =
                path.AppendChild(_GuideName(true, i));
            if (!existsUpstream(conePath)) {
                removed->emplace_back(conePath);
            }
        }
    }
    // A surviving element can independently gain or lose its sphere or cone.
    // Those are child-topology changes, not merely dirty values.
    for (size_t i = 0; i < common; ++i) {
        for (const auto &[bit, isCone, primType] : {
                 std::tuple<uint8_t, bool, TfToken>(
                     _GuideSphereBit, false, HdPrimTypeTokens->sphere),
                 std::tuple<uint8_t, bool, TfToken>(
                     _GuideConeBit, true, HdPrimTypeTokens->cone)}) {
            if ((announced[i] & bit) == (desired[i] & bit)) {
                continue;
            }
            const SdfPath childPath =
                path.AppendChild(_GuideName(isCone, i));
            if (existsUpstream(childPath)) {
                continue;
            }
            if (desired[i] & bit) {
                added->emplace_back(childPath, primType);
            } else {
                removed->emplace_back(childPath);
            }
        }
    }
    if (desired.empty()) {
        _announcedGuides.erase(path);
    } else {
        _announcedGuides[path] = desired;
    }
}

// Dirties \p locators on the synthesized guide children announced under
// \p path.
//
// These are descendants that exist ONLY in this scene index, so the
// upstream walk in _DirtySubtree cannot reach them -- and unlike an
// ordinary descendant, a guide does not merely inherit its ancestors'
// transform: _BuildGuidePrim composes the ASSET ROOT's world matrix into
// the guide's own matrix and then declares the stack reset. So when a
// constraint drives the asset root, every guide's transform changes while
// its published payload stays byte-identical -- RigExecChangeGuides never
// fires, no ancestor's dirtiness propagates down to a reset-stack prim, and
// a cached renderer is never told to pull. The asset moves and the guides
// stay behind.
void
RigExecResultsSceneIndex::_DirtyAnnouncedGuideChildren(
    const SdfPath &path,
    const HdDataSourceLocatorSet &locators,
    HdSceneIndexObserver::DirtiedPrimEntries *entries) const
{
    // Authored prims win colliding names, as everywhere else here: one is
    // not ours to dirty, and the upstream walk already covers it.
    auto emit = [&](const SdfPath &childPath) {
        if (!_GetInputSceneIndex()->GetPrim(childPath).dataSource) {
            entries->emplace_back(childPath, locators);
        }
    };
    const auto guides = _announcedGuides.find(path);
    if (guides != _announcedGuides.end()) {
        for (size_t i = 0; i < guides->second.size(); ++i) {
            emit(path.AppendChild(_GuideName(false, i)));
            if (guides->second[i]) {
                emit(path.AppendChild(_GuideName(true, i)));
            }
        }
    }
    if (_announcedControlGuides.count(path) != 0) {
        emit(path.AppendChild(_controlGuideName));
    }
    const auto volumes = _announcedVolumeGuides.find(path);
    if (volumes != _announcedVolumeGuides.end()) {
        for (size_t i = 0; i < volumes->second.size(); ++i) {
            emit(path.AppendChild(_VolumeGuideName(i)));
        }
    }
}

// Dirties \p locators on every descendant of \p path present upstream, plus
// the synthesized guide children that are present only here.
void
RigExecResultsSceneIndex::_DirtySubtree(
    const SdfPath &path,
    const HdDataSourceLocatorSet &locators,
    HdSceneIndexObserver::DirtiedPrimEntries *entries) const
{
    _DirtyAnnouncedGuideChildren(path, locators, entries);
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
    const SdfPath &assetRoot,
    const RigExecImagingSnapshot &snapshot) const
{
    if (assetRoot.IsEmpty()) {
        return GfMatrix4d(1.0);
    }
    // Resolved against the snapshot the CALLER is holding, not through our
    // own GetPrim -- which would take a second, independent load of the
    // store.
    //
    // One pull must see one generation. The publisher swaps generations
    // atomically at any moment, so a guide that reads its frame from
    // generation N and its asset-root placement from N+1 draws a pose that
    // never existed: the control where it was, offset by where the asset
    // has since moved to. It is a narrow window and a plausible-looking
    // result, which is exactly the kind that survives review.
    //
    // This reproduces what GetPrim would report for the asset root -- the
    // driven-transform overlay when one applies, the upstream flattened
    // matrix otherwise -- because _ComputeDrivenXform already takes the
    // generation to resolve against as a parameter.
    const HdSceneIndexPrim prim =
        _GetInputSceneIndex()->GetPrim(assetRoot);
    GfMatrix4d driven(1.0);
    if (_ComputeDrivenXform(assetRoot, prim.dataSource, snapshot,
                            &driven)) {
        return driven;
    }
    return _ReadXform(prim.dataSource);
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
            _RefreshAnnouncedControlGuide(entry.path);
            _RefreshAnnouncedVolumeGuides(entry.path);
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
    std::vector<SdfPath> xformRoots;
    entries.reserve(dirtied.size());
    for (const RigExecPublishedDirty &entry : dirtied) {
        if (entry.changes & RigExecChangeStructural) {
            entries.emplace_back(
                entry.path, HdDataSourceLocatorSet::UniversalSet());

            // An influence overlay APPEARING or DISAPPEARING changes which
            // primvars this prim owns, and that is a RESYNC, not a dirty.
            //
            // HdSceneIndexAdapterSceneDelegate caches each rprim's primvar
            // DESCRIPTORS and rebuilds them from PrimsAdded; a dirty --
            // even the universal one emitted just above -- re-pulls values
            // for the descriptors it already has. A primvar that was not
            // there at sync time is therefore never asked for, and the
            // mesh renders grey with a perfectly correct red displayColor
            // sitting in the scene index one hop upstream.
            //
            // Measured in a real usdview by differencing the framebuffer
            // across the on/off transition: with this re-announcement, 2.8%
            // of the frame gets redder; without it, 0.0% does, while every
            // scene-index assertion still passes. tests/
            // testUsdviewVolumeWeightOverlay.py is that measurement, and it
            // compares the two frames rather than counting red in one --
            // RigExec's own guide geometry is red and much larger on screen
            // than the mesh, so an absolute count says almost nothing.
            //
            // Re-announcing the prim with its upstream type is the same
            // move _SyncGuideChildren already makes for a synthesized
            // child whose prim type changed, and it is cheap: it happens
            // only on the transition, not per frame.
            {
                const bool wasOverlaid =
                    _announcedWeightOverlays.count(entry.path) != 0;
                bool isOverlaid = false;
                if (const RigExecImagingSnapshotConstPtr snapshot =
                        _store ? _store->Get() : nullptr) {
                    const auto it = snapshot->prims.find(entry.path);
                    isOverlaid = it != snapshot->prims.end() &&
                                 it->second.hasWeightOverlay &&
                                 !it->second.weightOverlay.empty();
                }
                if (wasOverlaid != isOverlaid) {
                    if (isOverlaid) {
                        _announcedWeightOverlays.insert(entry.path);
                    } else {
                        _announcedWeightOverlays.erase(entry.path);
                    }
                    const TfToken primType =
                        _GetInputSceneIndex()->GetPrim(entry.path).primType;
                    if (!primType.IsEmpty()) {
                        addedGuides.emplace_back(entry.path, primType);
                    }
                }
            }

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
            if (wasDriven || isDriven || (entry.changes & RigExecChangeXform)) {
                _DirtySubtree(entry.path,
                              HdDataSourceLocatorSet::UniversalSet(),
                              &entries);
            }
            _RefreshDrivenXform(entry.path);

            // Guide children follow the published set exactly. The ones that
            // SURVIVE a topology change also need dirtying: a representation
            // change reports structural and suppresses RigExecChangeGuides,
            // so without this a guide whose frame, derived length, radius, or
            // styling changed in the same generation keeps its old value.
            const std::vector<uint8_t> announcedGuides =
                _announcedGuides.count(entry.path)
                    ? _announcedGuides[entry.path]
                    : std::vector<uint8_t>();
            const std::vector<uint8_t> desiredGuides =
                _DesiredGuideTopology(entry.path);
            const size_t survivingGuides =
                std::min(announcedGuides.size(), desiredGuides.size());
            _SyncGuideChildren(entry.path, &addedGuides, &removedGuides);
            for (size_t i = 0; i < survivingGuides; ++i) {
                auto dirty = [&](bool isCone) {
                    const SdfPath childPath =
                        entry.path.AppendChild(_GuideName(isCone, i));
                    if (_GetInputSceneIndex()->GetPrim(childPath).dataSource) {
                        return;
                    }
                    entries.emplace_back(
                        childPath, HdDataSourceLocatorSet::UniversalSet());
                };
                if ((announcedGuides[i] & _GuideSphereBit) &&
                    (desiredGuides[i] & _GuideSphereBit)) {
                    dirty(false);
                }
                if ((announcedGuides[i] & _GuideConeBit) &&
                    (desiredGuides[i] & _GuideConeBit)) {
                    dirty(true);
                }
            }

            // The control guide child, on the same reasoning: it survives a
            // structural entry whenever it was announced and is still
            // wanted, and a survivor whose frame or scale moved in the same
            // generation gets no RigExecChangeGuides of its own.
            const bool controlGuideSurvives =
                _announcedControlGuides.count(entry.path) != 0 &&
                !_DesiredControlGuideType(entry.path).IsEmpty();
            _SyncControlGuideChild(entry.path, &addedGuides, &removedGuides);
            if (controlGuideSurvives) {
                const SdfPath childPath =
                    entry.path.AppendChild(_controlGuideName);
                if (!_GetInputSceneIndex()->GetPrim(childPath).dataSource) {
                    entries.emplace_back(
                        childPath, HdDataSourceLocatorSet::UniversalSet());
                }
            }

            // The volume guide children, on the same reasoning again: an
            // element that survives a structural entry with its prim type
            // intact gets no add of its own, so a surface whose radius or
            // placement moved in the same generation would otherwise keep
            // its old geometry forever.
            const size_t survivingVolumes = std::min(
                _announcedVolumeGuides.count(entry.path)
                    ? _announcedVolumeGuides[entry.path].size() : size_t(0),
                _DesiredVolumeGuideTypes(entry.path).size());
            _SyncVolumeGuideChildren(entry.path, &addedGuides,
                                     &removedGuides);
            for (size_t i = 0; i < survivingVolumes; ++i) {
                const SdfPath childPath =
                    entry.path.AppendChild(_VolumeGuideName(i));
                if (!_GetInputSceneIndex()->GetPrim(childPath).dataSource) {
                    entries.emplace_back(
                        childPath, HdDataSourceLocatorSet::UniversalSet());
                }
            }
            continue;
        }
        if (entry.changes & RigExecChangeGuides) {
            // The control guide keeps its prim type across a value change
            // (shape and drawMode edits report structural), so this is the
            // frame/scale/styling case: dirty the child that already exists,
            // or announce it if this consumer never heard about it.
            const SdfPath controlGuidePath =
                entry.path.AppendChild(_controlGuideName);
            if (_announcedControlGuides.count(entry.path) == 0) {
                _SyncControlGuideChild(entry.path, &addedGuides,
                                       &removedGuides);
            } else if (!_GetInputSceneIndex()
                            ->GetPrim(controlGuidePath)
                            .dataSource) {
                entries.emplace_back(
                    controlGuidePath,
                    HdDataSourceLocatorSet::UniversalSet());
            }

            // Reconcile even when the element count is unchanged: a joint
            // length crossing zero adds or removes its cone while its sphere
            // and frame survive.
            const auto announcedIt = _announcedGuides.find(entry.path);
            if (announcedIt == _announcedGuides.end()) {
                // Never announced (late consumer): the adds carry the
                // fresh payload; no dirtying needed.
                _SyncGuideChildren(entry.path, &addedGuides,
                                   &removedGuides);
            } else {
                const std::vector<uint8_t> announcedGuides =
                    announcedIt->second;
                const std::vector<uint8_t> desiredGuides =
                    _DesiredGuideTopology(entry.path);
                _SyncGuideChildren(entry.path, &addedGuides,
                                   &removedGuides);
                const size_t survivingGuides = std::min(
                    announcedGuides.size(), desiredGuides.size());
                for (size_t i = 0; i < survivingGuides; ++i) {
                    auto dirty = [&](bool isCone) {
                        const SdfPath childPath =
                            entry.path.AppendChild(_GuideName(isCone, i));
                        // Authored prims occupying a guide name are not
                        // ours to dirty.
                        if (_GetInputSceneIndex()
                                ->GetPrim(childPath)
                                .dataSource) {
                            return;
                        }
                        entries.emplace_back(
                            childPath,
                            HdDataSourceLocatorSet::UniversalSet());
                    };
                    if ((announcedGuides[i] & _GuideSphereBit) &&
                        (desiredGuides[i] & _GuideSphereBit)) {
                        dirty(false);
                    }
                    if ((announcedGuides[i] & _GuideConeBit) &&
                        (desiredGuides[i] & _GuideConeBit)) {
                        dirty(true);
                    }
                }
            }

            // Volume guide elements keep their prim types across a value
            // change (drawMode edits report structural), so this is the
            // radius/placement/styling case.
            const auto announcedVolumes =
                _announcedVolumeGuides.find(entry.path);
            if (announcedVolumes == _announcedVolumeGuides.end()) {
                // Never announced (late consumer): the adds carry the
                // fresh payload; no dirtying needed.
                _SyncVolumeGuideChildren(entry.path, &addedGuides,
                                         &removedGuides);
            } else {
                for (size_t i = 0; i < announcedVolumes->second.size(); ++i) {
                    const SdfPath childPath =
                        entry.path.AppendChild(_VolumeGuideName(i));
                    if (_GetInputSceneIndex()
                            ->GetPrim(childPath)
                            .dataSource) {
                        continue;
                    }
                    entries.emplace_back(
                        childPath, HdDataSourceLocatorSet::UniversalSet());
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
            _RefreshDrivenXform(entry.path);
            // Walked once, after the loop, from the topmost roots only: see
            // `xformRoots` below.
            xformRoots.push_back(entry.path);
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
        if (entry.changes & RigExecChangeWeightOverlay) {
            // The narrowest leaf, exactly like points and normals: the
            // overlay only ever reaches this arm while it is already ON
            // for this prim (an on/off transition changes the owned leaf
            // set and arrives as structural instead), so the primvar the
            // consumer cached is present and only its value moved.
            leaves.insert(HdDataSourceLocator(
                HdPrimvarsSchemaTokens->primvars, HdTokens->displayColor,
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
    // THE DRIVEN-TRANSFORM SUBTREES, topmost roots only, once each.
    //
    // Walking each driven Xform's subtree as it was met re-walked every
    // nested one again for every driven ancestor above it -- a rig whose
    // whole control and joint hierarchy moves with its root (dragging the
    // hips) dirtied the same prims over and over and asked the upstream
    // scene index for their children each time: 33 ms of every hips drag
    // sample on the biped. A root nested under another root in this batch
    // is already inside that walk, and it dirties exactly the same locator
    // set, so it is skipped. Every prim still gets the same locators.
    if (!xformRoots.empty()) {
        static const HdDataSourceLocatorSet xformSubtree =
            HdContainerDataSourceEditor::ComputeDirtyLocators(
                HdDataSourceLocatorSet{
                    HdDataSourceLocator(HdXformSchemaTokens->xform,
                                        HdXformSchemaTokens->matrix)});
        std::sort(xformRoots.begin(), xformRoots.end());
        xformRoots.erase(std::unique(xformRoots.begin(), xformRoots.end()),
                         xformRoots.end());
        SdfPath walked;
        for (const SdfPath &root : xformRoots) {
            // Sorted, so a descendant always follows its nearest walked
            // ancestor directly or after that ancestor's other descendants.
            if (!walked.IsEmpty() && root.HasPrefix(walked)) {
                continue;
            }
            _DirtySubtree(root, xformSubtree, &entries);
            walked = root;
        }
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
            _RefreshAnnouncedControlGuide(entry.primPath);
            _RefreshAnnouncedVolumeGuides(entry.primPath);
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
        _announcedControlGuides.erase(entry.primPath);
        _SyncControlGuideChild(entry.primPath, &addedGuides, &removedGuides);
        _announcedVolumeGuides.erase(entry.primPath);
        _SyncVolumeGuideChildren(entry.primPath, &addedGuides,
                                 &removedGuides);
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
    // subtree even while unobserved, or stale topology outlives the prims.
    if (!_IsObserved()) {
        for (const auto &entry : entries) {
            for (auto it = _announcedGuides.begin();
                 it != _announcedGuides.end();) {
                it = it->first.HasPrefix(entry.primPath)
                    ? _announcedGuides.erase(it) : std::next(it);
            }
            for (auto it = _announcedControlGuides.begin();
                 it != _announcedControlGuides.end();) {
                it = it->first.HasPrefix(entry.primPath)
                    ? _announcedControlGuides.erase(it) : std::next(it);
            }
            for (auto it = _announcedVolumeGuides.begin();
                 it != _announcedVolumeGuides.end();) {
                it = it->first.HasPrefix(entry.primPath)
                    ? _announcedVolumeGuides.erase(it) : std::next(it);
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
    std::set<SdfPath> revealedControlParents;
    std::set<SdfPath> revealedVolumeParents;
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
        for (auto it = _announcedControlGuides.begin();
             it != _announcedControlGuides.end();) {
            if (it->first.HasPrefix(entry.primPath)) {
                it = _announcedControlGuides.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = _announcedVolumeGuides.begin();
             it != _announcedVolumeGuides.end();) {
            if (it->first.HasPrefix(entry.primPath)) {
                it = _announcedVolumeGuides.erase(it);
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
            !entry.primPath.IsAbsoluteRootPath()) {
            if (_ParseGuideName(entry.primPath.GetNameToken(), &isCone,
                                &index)) {
                revealedParents.insert(entry.primPath.GetParentPath());
            } else if (entry.primPath.GetNameToken() == _controlGuideName) {
                revealedControlParents.insert(
                    entry.primPath.GetParentPath());
            } else if (_ParseVolumeGuideName(entry.primPath.GetNameToken(),
                                             &index)) {
                revealedVolumeParents.insert(entry.primPath.GetParentPath());
            }
        }
    }
    // Kept apart so a parent whose sphere guide was revealed does not also
    // get its control guide re-announced (and vice versa): a redundant add
    // is a resync downstream, for a prim that never went anywhere.
    for (const SdfPath &parent : revealedParents) {
        _announcedGuides.erase(parent);
        _SyncGuideChildren(parent, &addedGuides, &removedGuides);
    }
    for (const SdfPath &parent : revealedControlParents) {
        _announcedControlGuides.erase(parent);
        _SyncControlGuideChild(parent, &addedGuides, &removedGuides);
    }
    for (const SdfPath &parent : revealedVolumeParents) {
        _announcedVolumeGuides.erase(parent);
        _SyncVolumeGuideChildren(parent, &addedGuides, &removedGuides);
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
        // Authored prims win colliding names, as everywhere else here: one
        // is not ours to dirty, and forwarding it would duplicate the
        // notice upstream already sent.
        auto forward = [&](const SdfPath &childPath) {
            if (!_GetInputSceneIndex()->GetPrim(childPath).dataSource) {
                forwarded.emplace_back(childPath, inheritedLocators);
            }
        };
        if (_announcedControlGuides.count(entry.primPath) != 0) {
            forward(entry.primPath.AppendChild(_controlGuideName));
        }
        const auto volumes = _announcedVolumeGuides.find(entry.primPath);
        if (volumes != _announcedVolumeGuides.end()) {
            for (size_t i = 0; i < volumes->second.size(); ++i) {
                forward(entry.primPath.AppendChild(_VolumeGuideName(i)));
            }
        }
        const auto announced = _announcedGuides.find(entry.primPath);
        if (announced == _announcedGuides.end()) {
            continue;
        }
        for (size_t i = 0; i < announced->second.size(); ++i) {
            if (announced->second[i] & _GuideSphereBit) {
                forward(entry.primPath.AppendChild(_GuideName(false, i)));
            }
            if (announced->second[i] & _GuideConeBit) {
                forward(entry.primPath.AppendChild(_GuideName(true, i)));
            }
        }
    }
    _SendPrimsDirtied(forwarded);
}

// ---------------------------------------------------------------------------
// RigExecXformOverrideSceneIndex
// ---------------------------------------------------------------------------

RigExecXformOverrideSceneIndex::RigExecXformOverrideSceneIndex(
    const HdSceneIndexBaseRefPtr &inputSceneIndex)
    : HdSingleInputFilteringSceneIndexBase(inputSceneIndex)
{
}

bool
RigExecXformOverrideSceneIndex::HasWorldDeltas() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return !_deltas.empty();
}

const GfMatrix4d *
RigExecXformOverrideSceneIndex::_FindDelta(const SdfPath &path) const
{
    // Nearest previewed ancestor, or the prim itself. Walking up rather than
    // testing every entry keeps the cost proportional to namespace depth
    // instead of to how many prims are being previewed, and "nearest" is the
    // answer that matches flattening: the closest previewed ancestor's move is
    // the one already composed into this prim's matrix.
    if (_deltas.empty()) {
        return nullptr;
    }
    for (SdfPath walk = path; !walk.IsEmpty() && !walk.IsAbsoluteRootPath();
         walk = walk.GetParentPath()) {
        const auto it = _deltas.find(walk);
        if (it != _deltas.end()) {
            return &it->second;
        }
    }
    return nullptr;
}

void
RigExecXformOverrideSceneIndex::_DirtyXformSubtree(
    const SdfPath &path,
    HdSceneIndexObserver::DirtiedPrimEntries *entries) const
{
    static const HdDataSourceLocatorSet locators{
        HdXformSchema::GetDefaultLocator()};
    entries->emplace_back(path, locators);
    // Descendants carry the previewed ancestor baked in, so each one's matrix
    // changes too. The walk is over the UPSTREAM children: a preview adds no
    // prims and removes none.
    for (const SdfPath &child :
         _GetInputSceneIndex()->GetChildPrimPaths(path)) {
        _DirtyXformSubtree(child, entries);
    }
}

void
RigExecXformOverrideSceneIndex::SetWorldDeltas(
    const std::map<SdfPath, GfMatrix4d> &deltas)
{
    std::set<SdfPath> changed;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // The union of what was previewed and what now is, minus the entries
        // that are identical in both. A mouse sample that moved one prim
        // dirties one subtree, and a released drag dirties exactly what it
        // stops previewing.
        for (const auto &[path, delta] : _deltas) {
            const auto it = deltas.find(path);
            if (it == deltas.end() || it->second != delta) {
                changed.insert(path);
            }
        }
        for (const auto &[path, delta] : deltas) {
            const auto it = _deltas.find(path);
            if (it == _deltas.end() || it->second != delta) {
                changed.insert(path);
            }
        }
        if (changed.empty()) {
            return;
        }
        _deltas = deltas;
    }
    if (!_IsObserved()) {
        return;
    }
    HdSceneIndexObserver::DirtiedPrimEntries entries;
    for (const SdfPath &path : changed) {
        _DirtyXformSubtree(path, &entries);
    }
    if (!entries.empty()) {
        _SendPrimsDirtied(entries);
    }
}

HdSceneIndexPrim
RigExecXformOverrideSceneIndex::GetPrim(const SdfPath &primPath) const
{
    HdSceneIndexPrim prim = _GetInputSceneIndex()->GetPrim(primPath);
    if (!prim.dataSource) {
        return prim;
    }
    GfMatrix4d delta(1.0);
    {
        std::lock_guard<std::mutex> lock(_mutex);
        const GfMatrix4d *found = _FindDelta(primPath);
        if (!found) {
            return prim;
        }
        delta = *found;
    }
    // Post-multiplied onto the matrix that arrived, which is already the
    // composed world one. resetXformStack stays TRUE for the same reason the
    // driven-xform overlay sets it: the result is fully composed, and a
    // downstream consumer that re-composed a parent into it would double the
    // ancestors.
    static const TfToken xformName = HdXformSchemaTokens->xform;
    const HdDataSourceBaseHandle xformSource =
        HdXformSchema::Builder()
            .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(
                _ReadXform(prim.dataSource) * delta))
            .SetResetXformStack(
                HdRetainedTypedSampledDataSource<bool>::New(true))
            .Build();
    prim.dataSource =
        HdOverlayContainerDataSource::OverlayedContainerDataSources(
            HdRetainedContainerDataSource::New(1, &xformName, &xformSource),
            prim.dataSource);
    return prim;
}

void
RigExecXformOverrideSceneIndex::_PrimsAdded(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::AddedPrimEntries &entries)
{
    if (_IsObserved()) {
        _SendPrimsAdded(entries);
    }
}

void
RigExecXformOverrideSceneIndex::_PrimsRemoved(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::RemovedPrimEntries &entries)
{
    if (_IsObserved()) {
        _SendPrimsRemoved(entries);
    }
}

void
RigExecXformOverrideSceneIndex::_PrimsDirtied(
    const HdSceneIndexBase &,
    const HdSceneIndexObserver::DirtiedPrimEntries &entries)
{
    if (_IsObserved()) {
        _SendPrimsDirtied(entries);
    }
}

}  // namespace rigExec
