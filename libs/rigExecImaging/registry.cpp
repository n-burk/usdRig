//
// RigExec imaging registry and C activation surface.
//
#include "registry.h"

#include "pxr/base/gf/bbox3d.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/type.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usdGeom/boundable.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/usd/usdGeom/boundableComputeExtent.h"
#include "pxr/usd/usdUtils/stageCache.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rigExec {

RigExecImagingRegistry &
RigExecImagingRegistry::GetInstance()
{
    static RigExecImagingRegistry instance;
    return instance;
}

RigExecImagingRegistry::RigExecImagingRegistry()
    : _store(std::make_shared<RigExecSnapshotStore>())
{
}

void
RigExecImagingRegistry::RegisterChain(
    const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
    const RigExecBindingResolvingSceneIndexRefPtr &binding,
    const RigExecResultsSceneIndexRefPtr &results)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _chains.push_back(
        {TfWeakPtr<RigExecInternalPrimPruningSceneIndex>(
             get_pointer(pruning)),
         TfWeakPtr<RigExecBindingResolvingSceneIndex>(get_pointer(binding)),
         TfWeakPtr<RigExecResultsSceneIndex>(get_pointer(results))});
    // A chain constructed after activation adopts the current epoch scope.
    if (!_generatedScope.IsEmpty()) {
        pruning->SetOwnedScopes({_generatedScope});
    }
}

bool
RigExecImagingRegistry::Activate(
    const UsdStageRefPtr &stage, const SdfPath &rigPath,
    UsdTimeCode initialTime, std::vector<std::string> *errors)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _bridge = std::make_unique<RigExecImagingBridge>(stage, rigPath, _store);
    if (!_bridge->Compile(errors)) {
        _bridge.reset();
        return false;
    }
    _generatedScope = _bridge->GetGeneratedScope();
    for (Chain &chain : _chains) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes({_generatedScope});
        }
    }
    // Edit-driven re-evaluation: listen on the source stage so property
    // edits republish at the current time (the rig's inputs all live
    // beneath the asset root; cross-asset writes are rejected).
    _assetRoot = rigPath.GetParentPath();
    _lastTime = initialTime;
    TfNotice::Revoke(_changeKey);
    _changeKey = TfNotice::Register(
        TfCreateWeakPtr(this), &RigExecImagingRegistry::_OnObjectsChanged,
        stage);
    _Broadcast(_bridge->EvaluateAndPublishResult(initialTime));
    return true;
}

bool
RigExecImagingRegistry::SetTime(UsdTimeCode time)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_bridge) {
        return false;
    }
    _lastTime = time;
    const RigExecImagingBridge::PublishResult result =
        _bridge->EvaluateAndPublishResult(time);
    _Broadcast(result);
    return result.ok;
}

void
RigExecImagingRegistry::_OnObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &)
{
    SdfPath assetRoot;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_bridge || _assetRoot.IsEmpty()) {
            return;
        }
        assetRoot = _assetRoot;
    }
    // Any edit touching the asset can factor into the final frame
    // (solvers, joints, movers, controls, weights, driver geometry,
    // guide styling): re-evaluate at the current time. The evaluator's
    // epoch digest turns structural edits into recompiles; value edits
    // flow through exec invalidation on the shared layers.
    auto touchesAsset = [&assetRoot](const SdfPath &path) {
        const SdfPath primPath = path.GetPrimPath();
        return primPath.HasPrefix(assetRoot) ||
               assetRoot.HasPrefix(primPath);
    };
    bool relevant = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (touchesAsset(path)) {
            relevant = true;
            break;
        }
    }
    if (!relevant) {
        for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
            if (touchesAsset(path)) {
                relevant = true;
                break;
            }
        }
    }
    if (relevant) {
        UsdTimeCode time;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            time = _lastTime;
        }
        SetTime(time);
    }
}

void
RigExecImagingRegistry::Deactivate()
{
    std::lock_guard<std::mutex> lock(_mutex);
    TfNotice::Revoke(_changeKey);
    _changeKey = TfNotice::Key();
    _assetRoot = SdfPath();
    _bridge.reset();
    _generatedScope = SdfPath();
    RigExecImagingBridge::PublishResult cleared;
    cleared.ok = true;
    cleared.dirtied = _store->Publish(nullptr);
    _Broadcast(cleared);
}

void
RigExecImagingRegistry::_Broadcast(
    const RigExecImagingBridge::PublishResult &result)
{
    if (!result.ok) {
        return;
    }
    // Prune chains whose scene index graphs were destroyed.
    _chains.erase(
        std::remove_if(_chains.begin(), _chains.end(),
                       [](const Chain &c) { return !c.results; }),
        _chains.end());
    for (Chain &chain : _chains) {
        if (result.epoch && chain.binding) {
            chain.binding->SetBindingEpoch(result.epoch);
        }
        if (chain.results) {
            chain.results->NotifyGenerationPublished(result.dirtied);
        }
    }
}

}  // namespace rigExec

// ---------------------------------------------------------------------------
// C activation surface.
// ---------------------------------------------------------------------------

using rigExec::RigExecImagingRegistry;

namespace {

// Grows \p range to cover everything one published prim draws, in ASSET
// space. Returns whether it contributed anything at all.
bool
_AccumulateGuideBounds(
    const rigExec::RigExecPublishedPrim &published, PXR_NS::GfRange3d *range)
{
    bool any = false;

    // Joint and solver guides: a sphere at each frame origin plus a cone
    // reaching guideLength along that frame's +X aim axis. Bounding the
    // cone by a second sphere at its tip is conservative by design -- it
    // costs a little empty space at the tip and needs no cone math, and a
    // frame-selection box that is slightly generous is invisible while one
    // that clips is not.
    for (size_t i = 0; i < published.guideFrames.size(); ++i) {
        const PXR_NS::GfMatrix4d &frame = published.guideFrames[i];
        const double radius = i < published.guideRadii.size()
            ? published.guideRadii[i] : 1.0;
        const double length = i < published.guideLengths.size()
            ? published.guideLengths[i] : 0.0;
        const PXR_NS::GfVec3d origin = frame.ExtractTranslation();
        // Row-vector convention: row 0 is the frame's X basis, and the
        // frame is orthonormalized upstream, so it is already unit length.
        const PXR_NS::GfVec3d tip = origin + length * frame.GetRow3(0);
        const PXR_NS::GfVec3d extent(radius, radius, radius);
        range->UnionWith(origin - extent);
        range->UnionWith(origin + extent);
        range->UnionWith(tip - extent);
        range->UnionWith(tip + extent);
        any = true;
    }

    // The control guide: every shape is documented as unit-sized and
    // centred on the frame origin with half-extent 1 (spec §10.3
    // extension), so the unit cube bounds all six of them. Deliberately
    // NOT the exact per-shape extent: that table lives in the scene index,
    // which this registry must not depend on, and using it would make the
    // framing distance jump around as an author retypes guide:shape
    // between a sphere and a flat circle.
    //
    // It must bound what is actually DRAWN, though, so it asks the scene
    // index's own predicate whether anything is: an unrecognized
    // shape/drawMode pair synthesizes no prim, and reporting a box for it
    // would frame a host's camera on empty space.
    if (published.hasControlGuide &&
        rigExec::RigExecControlGuideIsDrawn(published.controlGuideShape,
                                            published.controlGuideDrawMode)) {
        // Wire curves are drawn with a width, in the guide's own local
        // pre-scale units, so the drawn geometry reaches half a width
        // past the unit shape BEFORE the per-axis scale applies.
        double halfWidth = 0.0;
        if (published.controlGuideDrawMode == PXR_NS::TfToken("wire") &&
            std::isfinite(published.controlGuideWireWidth) &&
            published.controlGuideWireWidth > 0.0) {
            halfWidth = published.controlGuideWireWidth * 0.5;
        }
        const PXR_NS::GfVec3d &scale = published.controlGuideScale;
        const PXR_NS::GfVec3d half(scale[0] * (1.0 + halfWidth),
                                   scale[1] * (1.0 + halfWidth),
                                   scale[2] * (1.0 + halfWidth));
        // The frame is rigid, so aligning the transformed box is exact.
        range->UnionWith(
            PXR_NS::GfBBox3d(PXR_NS::GfRange3d(-half, half),
                             published.controlGuideFrame)
                .ComputeAlignedRange());
        any = true;
    }
    return any;
}

// The union of everything published beneath \p path that shares its
// PURPOSE, including \p path itself.
//
// A Boundable's extent is authoritative for its whole subtree:
// UsdGeomBBoxCache stops descending at one ("Boundables should always
// provide their own extent and do not require participation from
// descendants", bboxCache.cpp). RigExec nests providers as a matter of
// course -- a joint chain is joints under joints -- so an extent covering
// only its own guide silently drops every descendant from any ancestor's
// bound, and framing a rig framed its first joint.
//
// Purpose-scoped, though, because one extent carries ONE purpose: the
// cache files this box under the boundable's own resolved purpose. Folding
// a default-purpose control nested under a guide-purpose joint into that
// box would file the control's bounds under `guide`, so a viewer with
// guides off would frame around geometry it is not showing -- and a viewer
// with guides on would frame around it twice. Differing-purpose
// descendants are excluded here and warned about at compile.
bool
_AccumulateSubtreeGuideBounds(
    const rigExec::RigExecImagingSnapshot &snapshot, const PXR_NS::SdfPath &path,
    const PXR_NS::TfToken &purpose, PXR_NS::GfRange3d *range)
{
    bool any = false;
    // The published set is path-keyed and sorted, so the subtree is one
    // contiguous run beginning at the prim itself.
    for (auto it = snapshot.prims.lower_bound(path);
         it != snapshot.prims.end() && it->first.HasPrefix(path); ++it) {
        if (it->second.guidePurpose != purpose) {
            continue;
        }
        any = _AccumulateGuideBounds(it->second, range) || any;
    }
    return any;
}

// A prim's resolved render purpose -- the same value the bridge publishes
// and the same one UsdGeomBBoxCache files its extent under.
PXR_NS::TfToken
_ResolvedPurpose(const PXR_NS::UsdPrim &prim)
{
    if (const PXR_NS::UsdGeomImageable imageable =
            PXR_NS::UsdGeomImageable(prim)) {
        const PXR_NS::TfToken purpose = imageable.ComputePurpose();
        if (!purpose.IsEmpty()) {
            return purpose;
        }
    }
    return PXR_NS::UsdGeomTokens->default_;
}

// The authored REST frame of one RigExec transform provider, composed the
// same way RigExecJointRestSpace does it in computations.cpp: the rest
// avars as a local delta preceding the authored rest:space, orthonormalized
// (rest spaces always are, per the Ir contract).
//
// Duplicated rather than shared with the evaluator on purpose. This runs
// with no compiled rig and no exec system -- the whole point of the rest
// fallback is to answer for a stage nobody has evaluated -- so it can only
// read authored attributes, which is precisely what it does.
PXR_NS::GfMatrix4d
_AuthoredRestSpace(const PXR_NS::UsdPrim &prim, const PXR_NS::UsdTimeCode &time,
                   bool *rigid)
{
    auto scalar = [&prim, &time](const char *name) {
        double value = 0.0;
        if (const PXR_NS::UsdAttribute a =
                prim.GetAttribute(PXR_NS::TfToken(name))) {
            a.Get(&value, time);
        }
        return value;
    };
    static const PXR_NS::GfVec3d axes[3] = {
        PXR_NS::GfVec3d(1, 0, 0), PXR_NS::GfVec3d(0, 1, 0),
        PXR_NS::GfVec3d(0, 0, 1)};
    const double angles[3] = {scalar("rest:rx"), scalar("rest:ry"),
                              scalar("rest:rz")};
    PXR_NS::GfMatrix4d local(1.0);
    for (int index = 0; index < 3; ++index) {  // XYZ, the rest-avar order
        if (angles[index] != 0.0) {
            local = local * PXR_NS::GfMatrix4d(
                                PXR_NS::GfRotation(axes[index], angles[index]),
                                PXR_NS::GfVec3d(0));
        }
    }
    PXR_NS::GfMatrix4d translate(1.0);
    translate.SetTranslate(
        PXR_NS::GfVec3d(scalar("rest:tx"), scalar("rest:ty"),
                        scalar("rest:tz")));
    local = local * translate;

    PXR_NS::GfMatrix4d space(1.0);
    if (const PXR_NS::UsdAttribute a =
            prim.GetAttribute(PXR_NS::TfToken("rest:space"))) {
        a.Get(&space, time);
    }
    PXR_NS::GfMatrix4d rest = local * space;
    // Reported, not swallowed: the bridge refuses to publish a guide whose
    // frame cannot be orthonormalized, so an extent computed from one would
    // bound a guide the renderer declines to draw.
    *rigid = rest.Orthonormalize(/* issueWarning = */ false);
    return rest;
}

// The bounds a provider's guide would draw AT REST, from authored
// attributes alone.
//
// This is the answer for a stage that has never been evaluated -- opened in
// a host that has not activated RigExec, or queried before the first
// generation is published. It is deliberately the rest pose rather than
// nothing: a bounding box that collapses the moment the rig is not running
// makes framing fail exactly where a user reaches for it first.
bool
_AccumulateRestGuideBounds(
    const PXR_NS::UsdPrim &prim, const PXR_NS::UsdTimeCode &time,
    PXR_NS::GfRange3d *range)
{
    auto number = [&prim, &time](const char *name, double fallback) {
        double value = fallback;
        if (const PXR_NS::UsdAttribute a =
                prim.GetAttribute(PXR_NS::TfToken(name))) {
            a.Get(&value, time);
        }
        return value;
    };
    bool rigid = false;
    const PXR_NS::GfMatrix4d rest = _AuthoredRestSpace(prim, time, &rigid);
    if (!rigid) {
        return false;
    }
    const PXR_NS::TfToken type = prim.GetTypeName();

    if (type == "RigExecControl") {
        auto token = [&prim](const char *name, const char *fallback) {
            PXR_NS::TfToken value(fallback);
            if (const PXR_NS::UsdAttribute a =
                    prim.GetAttribute(PXR_NS::TfToken(name))) {
                a.Get(&value);
            }
            return value;
        };
        // Same predicate the scene index draws through: an unrecognized
        // shape/drawMode pair synthesizes nothing and must bound nothing.
        const PXR_NS::TfToken shape = token("guide:shape", "circle");
        const PXR_NS::TfToken drawMode = token("guide:drawMode", "wire");
        if (!rigExec::RigExecControlGuideIsDrawn(shape, drawMode)) {
            return false;
        }
        const PXR_NS::GfVec3d scale(number("guide:scaleX", 1.0),
                                    number("guide:scaleY", 1.0),
                                    number("guide:scaleZ", 1.0));
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(scale[i]) || scale[i] <= 0.0) {
                return false;  // draws nothing, so it bounds nothing
            }
        }
        double halfWidth = 0.0;
        if (drawMode == PXR_NS::TfToken("wire")) {
            const double width = number("guide:wireWidth", 0.05);
            if (std::isfinite(width) && width > 0.0) {
                halfWidth = width * 0.5;
            }
        }
        // The unit shape, for the same reason the snapshot path uses it:
        // every guide shape is documented as unit-sized with half-extent 1.
        const PXR_NS::GfVec3d half(scale[0] * (1.0 + halfWidth),
                                   scale[1] * (1.0 + halfWidth),
                                   scale[2] * (1.0 + halfWidth));
        range->UnionWith(
            PXR_NS::GfBBox3d(PXR_NS::GfRange3d(-half, half), rest)
                .ComputeAlignedRange());
        return true;
    }
    if (type == "RigExecJoint") {
        const double radius = number("guide:radius", 1.0);
        const double length = number("guide:length", 0.0);
        if (!std::isfinite(radius) || radius <= 0.0 ||
            !std::isfinite(length)) {
            return false;
        }
        const PXR_NS::GfVec3d origin = rest.ExtractTranslation();
        const PXR_NS::GfVec3d tip = origin + length * rest.GetRow3(0);
        const PXR_NS::GfVec3d extent(radius, radius, radius);
        range->UnionWith(origin - extent);
        range->UnionWith(origin + extent);
        range->UnionWith(tip - extent);
        range->UnionWith(tip + extent);
        return true;
    }
    return false;
}

// Writes \p range out as min xyz then max xyz.
void
_WriteBounds(const PXR_NS::GfRange3d &range, double outMinMax[6])
{
    const PXR_NS::GfVec3d min = range.GetMin();
    const PXR_NS::GfVec3d max = range.GetMax();
    for (int i = 0; i < 3; ++i) {
        outMinMax[i] = min[i];
        outMinMax[i + 3] = max[i];
    }
}

// UsdGeomBoundable::ComputeExtentFromPlugins entry point for every RigExec
// transform provider (spec §10.3 extension, host-durability redesign).
//
// This is what makes framing a control work in EVERY host rather than in
// the one whose Python we could reach. UsdGeomBBoxCache is what usdview,
// Solaris, and mayaUsd all consult, and it asks a Boundable for its extent;
// before this, RigExec types were not Boundable and reported nothing, so a
// rig had no bounds anywhere and framing a control moved the camera not at
// all. The usdview adapter used to monkeypatch computeWorldBound to paper
// over that -- a patch for one host, replaced by this.
//
// The extent BAKES the posed frame, because the prim carries no stage
// transform of its own: rest:space plus avars are the only transform
// authority (the Ir alignment), and they are asset-relative. That is
// correct exactly while no Xformable sits between the asset root and the
// provider, which the compiler validates and warns about.
bool
_ComputeRigExecGuideExtent(
    const PXR_NS::UsdGeomBoundable &boundable, const PXR_NS::UsdTimeCode &time,
    const PXR_NS::GfMatrix4d *transform, PXR_NS::VtVec3fArray *extent)
{
    const PXR_NS::UsdPrim prim = boundable.GetPrim();
    if (!prim || !extent) {
        return false;
    }
    PXR_NS::GfRange3d range;
    bool found = false;

    // The live generation wins -- but ONLY if it describes this stage at
    // this time. USD calls this with a stage and a time of its own
    // choosing, and the store is a process-global singleton, so an
    // ungated lookup by path answers a query about stage A frame 12 with
    // stage B's frame 30 pose: plausible, wrong, and undetectable
    // downstream. Identity is the stage OBJECT -- two stages commonly share
    // a root layer and differ only by session layer. Anything else falls through to the rest pose, which
    // makes this callback a pure function of (stage, time).
    if (const rigExec::RigExecImagingSnapshotConstPtr snapshot =
            RigExecImagingRegistry::GetInstance().GetStore()->Get()) {
        if (snapshot->Describes(prim.GetStage(), time)) {
            found = _AccumulateSubtreeGuideBounds(
                *snapshot, prim.GetPath(), _ResolvedPurpose(prim), &range);
        }
    }
    // ...otherwise the rest pose, from authored attributes alone, so an
    // un-evaluated stage still frames. The subtree rule applies here too:
    // BBoxCache stops descending at a Boundable, so a provider's extent
    // has to speak for the providers nested under it.
    if (!found) {
        const PXR_NS::TfToken purpose = _ResolvedPurpose(prim);
        for (const PXR_NS::UsdPrim &descendant :
             PXR_NS::UsdPrimRange(prim)) {
            // Purpose-scoped for the same reason the snapshot path is: one
            // extent carries one purpose.
            if (_ResolvedPurpose(descendant) != purpose) {
                continue;
            }
            found = _AccumulateRestGuideBounds(descendant, time, &range) ||
                    found;
        }
    }
    if (!found || range.IsEmpty()) {
        return false;
    }
    if (transform) {
        range = PXR_NS::GfBBox3d(range, *transform).ComputeAlignedRange();
    }
    *extent = PXR_NS::VtVec3fArray{PXR_NS::GfVec3f(range.GetMin()),
                                   PXR_NS::GfVec3f(range.GetMax())};
    return true;
}

}  // namespace

PXR_NAMESPACE_OPEN_SCOPE

// Keyed by TfType rather than by the templated overload: the RigExec schema
// is CODELESS, so there is no C++ class to name as a template argument. The
// type still exists -- Plug declares it from the schema plugInfo, and its
// ancestor chain reaches UsdGeomBoundable -- which is all
// ComputeExtentFromPlugins needs to find this.
//
// Registered on the abstract base, not on each concrete type: the lookup
// walks a prim's ancestor types, so one registration answers for every
// provider that inherits it.
//
// Inside PXR_NAMESPACE_OPEN_SCOPE, like every other TF_REGISTRY_FUNCTION in
// this tree -- the macro's tag type has to resolve the way usdGeom's own
// subscription resolves it, and at global scope with a PXR_NS:: qualifier
// it registers into a registry nobody subscribes to, which fails silently.
TF_REGISTRY_FUNCTION(UsdGeomBoundable)
{
    // RigExecXformable covers every joint and control through its
    // ancestors; the aggregate solvers draw guides too but inherit
    // Boundable directly, so they are named individually.
    for (const char *name : {"RigExecXformable",
                             "RigExecFkChain",
                             "RigExecTwoBoneIk",
                             "RigExecBlendPointFrames",
                             "RigExecTwistDistribution",
                             "RigExecRibbon"}) {
        const TfType type = TfType::FindByName(name);
        if (type.IsUnknown()) {
            // The schema plugin is not registered in this process, so
            // nothing can be Boundable anyway. Not worth shouting about.
            continue;
        }
        UsdGeomRegisterComputeExtentFunction(
            type, ::_ComputeRigExecGuideExtent);
    }
}

PXR_NAMESPACE_CLOSE_SCOPE

extern "C" {

int
RigExecImaging_Activate(
    long long stageCacheId, const char *rigPath, double initialFrame)
{
    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdUtilsStageCache::Get().Find(
        PXR_NS::UsdStageCache::Id::FromLongInt(
            static_cast<long int>(stageCacheId)));
    if (!stage) {
        std::printf("rigExecImaging: no stage for cache id %lld\n",
                    stageCacheId);
        return 1;
    }

    PXR_NS::SdfPath path;
    if (rigPath && rigPath[0]) {
        path = PXR_NS::SdfPath(rigPath);
    } else {
        // Discover the first RigExecRig prim on the stage.
        for (const PXR_NS::UsdPrim &prim : stage->Traverse()) {
            if (prim.GetTypeName() == "RigExecRig") {
                path = prim.GetPath();
                break;
            }
        }
    }
    if (path.IsEmpty()) {
        std::printf("rigExecImaging: no RigExecRig prim found\n");
        return 2;
    }

    std::vector<std::string> errors;
    if (!RigExecImagingRegistry::GetInstance().Activate(
            stage, path, PXR_NS::UsdTimeCode(initialFrame), &errors)) {
        for (const std::string &e : errors) {
            std::printf("rigExecImaging: %s\n", e.c_str());
        }
        return 3;
    }
    std::printf("rigExecImaging: activated %s\n", path.GetText());
    return 0;
}

int
RigExecImaging_SetTime(double frame)
{
    return RigExecImagingRegistry::GetInstance().SetTime(
               PXR_NS::UsdTimeCode(frame))
        ? 0 : 1;
}

void
RigExecImaging_Deactivate()
{
    RigExecImagingRegistry::GetInstance().Deactivate();
}

long long
RigExecImaging_GetGeneration()
{
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    return snapshot ? static_cast<long long>(snapshot->generation) : 0;
}

int
RigExecImaging_GetGuideBoundsAssetSpace(
    const char *primPath, double outMinMax[6])
{
    if (!primPath || !primPath[0] || !outMinMax) {
        return 0;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    if (!snapshot) {
        return 0;
    }
    // An arbitrary caller-supplied string reaches SdfPath here, and its
    // constructor is loud about a malformed one. Ask first.
    if (!PXR_NS::SdfPath::IsValidPathString(primPath)) {
        return 0;
    }
    const auto it = snapshot->prims.find(PXR_NS::SdfPath(primPath));
    if (it == snapshot->prims.end()) {
        return 0;
    }
    PXR_NS::GfRange3d range;
    if (!_AccumulateGuideBounds(it->second, &range) || range.IsEmpty()) {
        return 0;
    }
    _WriteBounds(range, outMinMax);
    return 1;
}

int
RigExecImaging_GetAllGuideBoundsAssetSpace(double outMinMax[6])
{
    if (!outMinMax) {
        return 0;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        RigExecImagingRegistry::GetInstance().GetStore()->Get();
    if (!snapshot) {
        return 0;
    }
    PXR_NS::GfRange3d range;
    bool any = false;
    for (const auto &[path, published] : snapshot->prims) {
        any = _AccumulateGuideBounds(published, &range) || any;
    }
    if (!any || range.IsEmpty()) {
        return 0;
    }
    _WriteBounds(range, outMinMax);
    return 1;
}

}  // extern "C"
