//
// RigExec Hydra publication bridge implementation.
//
#include "bridge.h"

#include "rigExecMath/pointFrame.h"

#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"

#include <cmath>

namespace rigExec {

namespace {

// The rigid ASSET-space placement matrix of one posed frame, or nothing
// when the frame cannot supply one.
//
// Guides are rigid: the frame's scale/shear is stripped so the authored
// guide dimensions (a joint's length/radius, a control's per-axis scale)
// are the SOLE dimensional scale. An affine basis would apply the bone
// scale a second time -- to the cone height and base offset for a joint,
// to the whole shape for a control.
//
// Shared by the joint/solver payload and the control guides so the two
// cannot drift apart: a guide drawn from a frame one of them rejects and
// the other accepts would be a difference nobody could see coming.
bool
_RigidGuideMatrix(const RigExecPointFrame &frame, GfMatrix4d *result)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    static const RigExecPointFrame identity;
    GfMatrix4d m(1.0);
    if (!RigExecPointsToMatrix(identity.points, frame.points, &m)) {
        return false;
    }
    // A basis that cannot be orthonormalized (collapsed/collinear axes)
    // draws nothing rather than publishing a broken placement.
    if (!m.Orthonormalize(/* issueWarning = */ false)) {
        return false;
    }
    // Proper rigid rotation: a reflected frame keeps its +X aim but has
    // its Z basis flipped so the determinant is positive.
    if (m.GetDeterminant3() < 0) {
        for (int c = 0; c < 3; ++c) {
            m[2][c] = -m[2][c];
        }
    }
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!std::isfinite(m[r][c])) {
                return false;
            }
        }
    }
    *result = m;
    return true;
}

// One guide element from a posed frame: rig-space placement matrix, cone
// length, and primitive radius. Joints use the authored guide:length
// exactly (Ir contract: zero draws no cone); solver elements — which have
// no authored length — fall back to the frame's aim landmark distance.
//
// A non-positive radius draws nothing at all. Unlike length, where zero
// legitimately means "sphere but no bone", a zero-radius guide is invisible
// either way, so publishing it would only cost the viewer geometry it
// cannot see.
bool
_AppendGuideFrame(
    const RigExecPointFrame &frame, double authoredLength,
    double authoredRadius, bool useAimFallback,
    RigExecPublishedPrim *published)
{
    GfMatrix4d m(1.0);
    if (!_RigidGuideMatrix(frame, &m)) {
        return false;
    }
    double length = authoredLength;
    if (length <= 0.0) {
        length = useAimFallback
            ? (frame.X() - frame.Origin()).GetLength() : 0.0;
    }
    if (!std::isfinite(length)) {
        return false;
    }
    if (!std::isfinite(authoredRadius) || authoredRadius <= 0.0) {
        return false;
    }
    published->guideFrames.push_back(m);
    published->guideLengths.push_back(length);
    published->guideRadii.push_back(authoredRadius);
    return true;
}

void
_ReadGuideStyle(
    const UsdPrim &prim, UsdTimeCode time, RigExecPublishedPrim *published)
{
    if (!prim) {
        return;
    }
    if (UsdAttribute a = prim.GetAttribute(TfToken("guide:displayColor"))) {
        a.Get(&published->guideColor, time);
    }
    if (UsdAttribute a =
            prim.GetAttribute(TfToken("guide:displayOpacity"))) {
        a.Get(&published->guideOpacity, time);
    }
}

}  // namespace

RigExecImagingBridge::RigExecImagingBridge(
    const UsdStageRefPtr &stage, const SdfPath &rigPath)
    : RigExecImagingBridge(
          stage, rigPath, std::make_shared<RigExecSnapshotStore>())
{
}

RigExecImagingBridge::RigExecImagingBridge(
    const UsdStageRefPtr &stage, const SdfPath &rigPath,
    std::shared_ptr<RigExecSnapshotStore> store)
    : _stage(stage)
    , _rigPath(rigPath)
    , _evaluator(std::make_unique<RigExecRigEvaluator>(stage, rigPath))
    , _store(std::move(store))
{
}

bool
RigExecImagingBridge::Compile(std::vector<std::string> *errors)
{
    return _evaluator->Compile(errors);
}

SdfPath
RigExecImagingBridge::GetGeneratedScope() const
{
    return _rigPath.AppendChild(TfToken("__RigExecGenerated"));
}

// Constraint-driven transforms, published onto the prim itself so parented
// geometry rides along. Base and revision travel together: the scene index
// sits downstream of Hydra's flatten and turns the pair into a world-space
// delta (see RigExecPublishedPrim::xformBase).
void
RigExecImagingBridge::_FillProviderXforms(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    for (const auto &[providerPath, matrix] : pose.providerXforms) {
        const auto baseIt = pose.providerBaseXforms.find(providerPath);
        if (baseIt == pose.providerBaseXforms.end()) {
            // The evaluator writes both or neither; a revision without its
            // base cannot be turned into a delta, so publishing it would move
            // the prim by an arbitrary amount.
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[providerPath];
        published.xform = matrix;
        published.xformBase = baseIt->second;
        published.hasXform = true;
        snapshot->hasDrivenXforms = true;
    }
}

void
RigExecImagingBridge::_FillGuides(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    // Guide frames are asset-space; the consumer needs to know which prim
    // that space is anchored to in order to place them.
    snapshot->assetRoot = _rigPath.GetParentPath();

    for (const auto &[jointPath, frame] : pose.jointFramesFinal) {
        const UsdPrim prim = _stage->GetPrimAtPath(jointPath);
        double authoredLength = 0;
        // Schema default, so an unauthored joint keeps Hydra's own fallback
        // radius and every existing rig looks exactly as it did.
        double authoredRadius = 1.0;
        if (prim) {
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("guide:length"))) {
                a.Get(&authoredLength, pose.time);
            }
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("guide:radius"))) {
                a.Get(&authoredRadius, pose.time);
            }
        }
        RigExecPublishedPrim &published = snapshot->prims[jointPath];
        if (_AppendGuideFrame(frame, authoredLength, authoredRadius,
                              /* useAimFallback = */ false, &published)) {
            published.hasGuides = true;
            _ReadGuideStyle(prim, pose.time, &published);
        }
    }
    for (const auto &[solverPath, frames] : pose.solverFrames) {
        RigExecPublishedPrim &published = snapshot->prims[solverPath];
        bool any = false;
        for (const RigExecPointFrame &frame : frames) {
            // Solver elements have no prim of their own to author on, so
            // they keep the unit radius they have always had.
            any = _AppendGuideFrame(frame, 0.0, 1.0,
                                    /* useAimFallback = */ true,
                                    &published) ||
                  any;
        }
        if (any) {
            published.hasGuides = true;
            _ReadGuideStyle(_stage->GetPrimAtPath(solverPath), pose.time,
                            &published);
        } else if (!published.hasPoints && !published.hasNormals &&
                   !published.hasExtent && !published.hasXform) {
            snapshot->prims.erase(solverPath);
        }
    }
}

// Controls draw one synthesized shape each at their posed frame
// (spec §10.3 extension): shape, draw mode, and per-axis scale are
// authored on the control, and the results scene index turns the published
// payload into the child prim.
void
RigExecImagingBridge::_FillControlGuides(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    // Set here as well as in _FillGuides: control guide frames are
    // asset-space too, and a rig can publish these and no joint guides at
    // all (every joint frame degenerate, say), in which case this is the
    // only place the anchor gets recorded.
    snapshot->assetRoot = _rigPath.GetParentPath();

    for (const auto &[controlPath, frame] : pose.controlFrames) {
        GfMatrix4d placement(1.0);
        if (!_RigidGuideMatrix(frame, &placement)) {
            continue;
        }
        const UsdPrim prim = _stage->GetPrimAtPath(controlPath);
        // The schema fallbacks, restated. Normally GetAttribute resolves
        // them for us, but a stage composed without the codeless schema
        // plugin registered has no fallback to find, and an empty shape
        // token names no shape at all -- so a rig would silently stop
        // drawing control guides rather than draw the documented default.
        TfToken shape("circle");
        TfToken drawMode("wire");
        GfVec3d scale(1.0, 1.0, 1.0);
        if (prim) {
            if (UsdAttribute a = prim.GetAttribute(TfToken("guide:shape"))) {
                a.Get(&shape, pose.time);
            }
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("guide:drawMode"))) {
                a.Get(&drawMode, pose.time);
            }
            static const TfToken scaleAttrs[3] = {
                TfToken("guide:scaleX"), TfToken("guide:scaleY"),
                TfToken("guide:scaleZ")};
            for (int axis = 0; axis < 3; ++axis) {
                if (UsdAttribute a = prim.GetAttribute(scaleAttrs[axis])) {
                    a.Get(&scale[axis], pose.time);
                }
            }
        }
        // A non-finite or non-positive scale on ANY axis draws nothing,
        // mirroring the joint guide:radius rule: a flattened shape is
        // invisible from most angles and degenerate from the rest, so
        // publishing it would only cost the viewer geometry it cannot
        // meaningfully see.
        if (!std::isfinite(scale[0]) || !std::isfinite(scale[1]) ||
            !std::isfinite(scale[2]) || scale[0] <= 0.0 ||
            scale[1] <= 0.0 || scale[2] <= 0.0) {
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[controlPath];
        published.hasControlGuide = true;
        published.controlGuideFrame = placement;
        published.controlGuideShape = shape;
        published.controlGuideDrawMode = drawMode;
        published.controlGuideScale = scale;
        _ReadGuideStyle(prim, pose.time, &published);
    }
}

bool
RigExecImagingBridge::EvaluateAndPublish(UsdTimeCode time)
{
    const PublishResult result = EvaluateAndPublishResult(time);
    if (!result.ok) {
        return false;
    }
    if (_binding && result.epoch) {
        _binding->SetBindingEpoch(result.epoch);
    }
    if (_results) {
        _results->NotifyGenerationPublished(result.dirtied);
    }
    return true;
}

RigExecImagingBridge::PublishResult
RigExecImagingBridge::EvaluateAndPublishResult(UsdTimeCode time)
{
    PublishResult result;
    // 1. Evaluation always completes before publication (spec §8.2).
    const RigExecRigPose pose = _evaluator->Evaluate(time);
    if (!pose.valid) {
        return result;
    }

    // 2. Build the complete immutable generation from the exact native
    // results: points and normals as flat primvars, extent as min/max
    // (spec §10.2). Only standard data crosses this boundary.
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    snapshot->generation = ++_generation;
    for (const auto &[propertyPath, value] : pose.movedProperties) {
        const SdfPath primPath = propertyPath.GetPrimPath();
        const TfToken property = propertyPath.GetNameToken();
        RigExecPublishedPrim &published = snapshot->prims[primPath];
        if (property == "points" && value.IsHolding<VtVec3fArray>()) {
            published.hasPoints = true;
            published.points = value.UncheckedGet<VtVec3fArray>();
        } else if (property == "normals" &&
                   value.IsHolding<VtVec3fArray>()) {
            published.hasNormals = true;
            published.normals = value.UncheckedGet<VtVec3fArray>();
        } else if (property == "extent" && value.IsHolding<VtVec3fArray>()) {
            const auto extent = value.UncheckedGet<VtVec3fArray>();
            if (extent.size() == 2) {
                published.hasExtent = true;
                published.extentMin = GfVec3d(extent[0]);
                published.extentMax = GfVec3d(extent[1]);
            }
        }
    }
    // Joints and aggregate solvers publish guide payloads (drawn by the
    // results scene index like OpenExec's IrJointScope guides).
    _FillProviderXforms(pose, snapshot.get());
    _FillGuides(pose, snapshot.get());
    _FillControlGuides(pose, snapshot.get());

    // 3. A structural recompile publishes a replacement binding epoch
    // before value notices (spec §10.4).
    const size_t epochDigest = _evaluator->GetBindingEpochDigest();
    if (epochDigest != _publishedEpochDigest) {
        auto epoch = std::make_shared<
            RigExecBindingResolvingSceneIndex::BindingEpoch>();
        epoch->id = epochDigest;
        for (const auto &[primPath, published] : snapshot->prims) {
            epoch->publishedPrims.insert(primPath);
        }
        result.epoch = std::move(epoch);
        _publishedEpochDigest = epochDigest;
    }

    // 4. Atomic snapshot swap; the caller sends the coalesced precise
    // dirtied notices from the notice owner (spec §8.2, §10.4).
    result.dirtied = _store->Publish(std::move(snapshot));
    result.ok = true;
    return result;
}

bool
RigExecImagingBridge::PreflightMotionProfile(
    const std::vector<float> &shutterOffsets, MotionBlurSupport support,
    std::string *whyNot)
{
    if (shutterOffsets.empty()) {
        if (whyNot) {
            *whyNot = "render preflight requires at least one explicit "
                      "shutter offset";
        }
        return false;
    }
    // motionBlurSupport = false permits one sample (spec §10.3.1); the
    // capability bit never supplies offsets, and an absent bit leaves the
    // application's explicit render profile authoritative.
    if (support == MotionBlurSupport::False && shutterOffsets.size() > 1) {
        if (whyNot) {
            *whyNot = "renderer advertises motionBlurSupport = false; a "
                      "multi-sample motion profile fails preflight";
        }
        return false;
    }
    return true;
}

RigExecImagingBridge::PublishResult
RigExecImagingBridge::EvaluateAndPublishSamples(
    UsdTimeCode baseTime, const std::vector<float> &shutterOffsets,
    MotionBlurSupport support)
{
    PublishResult result;
    // Preflight fails deterministically before any evaluation
    // (spec §10.5): capability mismatch or an invalid profile never
    // computes and never publishes.
    if (!PreflightMotionProfile(shutterOffsets, support) ||
        baseTime.IsDefault()) {
        return result;
    }
    // The offset nearest zero supplies the primary (non-sampled) outputs.
    size_t baseIndex = 0;
    for (size_t i = 1; i < shutterOffsets.size(); ++i) {
        if (std::abs(shutterOffsets[i]) <
            std::abs(shutterOffsets[baseIndex])) {
            baseIndex = i;
        }
    }

    // Evaluation of every required offset completes before anything
    // publishes (spec 8.2, 10.5): all samples are captured, then one
    // complete immutable generation swaps in.
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    std::map<SdfPath, std::vector<VtVec3fArray>> pointsPerPrim;
    for (size_t i = 0; i < shutterOffsets.size(); ++i) {
        const UsdTimeCode sampleTime(
            baseTime.GetValue() + double(shutterOffsets[i]));
        const RigExecRigPose pose = _evaluator->Evaluate(sampleTime);
        if (!pose.valid) {
            return result;  // an incomplete sample set never publishes
        }
        if (i == baseIndex) {
            // Guides are single-sampled at the base offset.
            _FillProviderXforms(pose, snapshot.get());
            _FillGuides(pose, snapshot.get());
            _FillControlGuides(pose, snapshot.get());
        }
        for (const auto &[propertyPath, value] : pose.movedProperties) {
            const SdfPath primPath = propertyPath.GetPrimPath();
            const TfToken property = propertyPath.GetNameToken();
            if (property == "points" && value.IsHolding<VtVec3fArray>()) {
                pointsPerPrim[primPath].push_back(
                    value.UncheckedGet<VtVec3fArray>());
            }
            if (i == baseIndex) {
                RigExecPublishedPrim &published = snapshot->prims[primPath];
                if (property == "points" &&
                    value.IsHolding<VtVec3fArray>()) {
                    published.hasPoints = true;
                    published.points = value.UncheckedGet<VtVec3fArray>();
                } else if (property == "normals" &&
                           value.IsHolding<VtVec3fArray>()) {
                    published.hasNormals = true;
                    published.normals = value.UncheckedGet<VtVec3fArray>();
                } else if (property == "extent" &&
                           value.IsHolding<VtVec3fArray>()) {
                    const auto extent =
                        value.UncheckedGet<VtVec3fArray>();
                    if (extent.size() == 2) {
                        published.hasExtent = true;
                        published.extentMin = GfVec3d(extent[0]);
                        published.extentMax = GfVec3d(extent[1]);
                    }
                }
            }
        }
    }
    for (auto &[primPath, samples] : pointsPerPrim) {
        if (samples.size() != shutterOffsets.size()) {
            return result;  // missing required samples fail preflight
        }
        RigExecPublishedPrim &published = snapshot->prims[primPath];
        published.sampleOffsets = shutterOffsets;
        published.pointsSamples = std::move(samples);
    }
    snapshot->generation = ++_generation;

    const size_t epochDigest = _evaluator->GetBindingEpochDigest();
    if (epochDigest != _publishedEpochDigest) {
        auto epoch = std::make_shared<
            RigExecBindingResolvingSceneIndex::BindingEpoch>();
        epoch->id = epochDigest;
        for (const auto &[primPath, published] : snapshot->prims) {
            epoch->publishedPrims.insert(primPath);
        }
        result.epoch = std::move(epoch);
        _publishedEpochDigest = epochDigest;
    }
    result.dirtied = _store->Publish(std::move(snapshot));
    result.ok = true;
    return result;
}

}  // namespace rigExec
