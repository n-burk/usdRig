#include "fallbackStage.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/copyUtils.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/usd/property.h"
#include <set>

namespace rigExec {
namespace {
bool SameOptions(const RigExecFallbackStageOptions &a,
                 const RigExecFallbackStageOptions &b)
{
    return a.preferProgram == b.preferProgram &&
        a.publishWeightFields == b.publishWeightFields &&
        a.solverGuidesEnabled == b.solverGuidesEnabled &&
        a.explicitMode == b.explicitMode && a.mode == b.mode;
}
}

bool RigExecFallbackStageMirror::HasFailed(
    uint64_t revision, UsdTimeCode time,
    const RigExecFallbackStageOptions &options) const
{
    const auto failures = std::atomic_load(&_failures);
    return failures && failures->revision == revision &&
        SameOptions(failures->options, options) &&
        failures->times.count(time.GetValue());
}

RigExecFallbackStageSource::RigExecFallbackStageSource(const UsdStageRefPtr &stage)
{
    _CaptureBaseline(stage);
}

bool RigExecFallbackStageSource::_CaptureBaseline(const UsdStageRefPtr &stage)
{
    _snapshot.reset();
    if (!stage) return false;
    auto snapshot = std::make_shared<RigExecFallbackStageSnapshot>();
    snapshot->baseline = stage->Flatten(/*addSourceFileComment=*/false);
    if (!snapshot->baseline) return false;
    snapshot->revision = ++_revision;
    snapshot->interpolation = stage->GetInterpolationType();
    _snapshot = std::move(snapshot);
    return true;
}

bool RigExecFallbackStageSource::CaptureChanges(
    const UsdStageRefPtr &stage, const UsdNotice::ObjectsChanged &notice)
{
    if (!_snapshot || !notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return _CaptureBaseline(stage);
    }
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath()) return _CaptureBaseline(stage);
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (!path.IsPropertyPath() && !notice.GetChangedFields(path).empty()) {
            return _CaptureBaseline(stage);
        }
    }
    std::set<SdfPath> properties;
    for (const SdfPath &path : notice.GetResyncedPaths()) properties.insert(path);
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (path.IsPropertyPath()) properties.insert(path);
    }
    if (properties.empty() && _snapshot->interpolation == stage->GetInterpolationType()) {
        return true; // inert ancestor overs only
    }
    auto snapshot = std::make_shared<RigExecFallbackStageSnapshot>(*_snapshot);
    const UsdStageRefPtr capture = UsdStage::CreateInMemory();
    const SdfLayerRefPtr layer = capture->GetRootLayer();
    ++_revision;
    for (const SdfPath &path : properties) {
        RigExecFallbackStagePatch patch;
        patch.layer = layer;
        patch.revision = _revision;
        const UsdProperty property = stage->GetPropertyAtPath(path);
        if (!property) {
            patch.removed = true;
            snapshot->patches[path] = std::move(patch);
            continue;
        }
        const UsdPrim parent = capture->OverridePrim(path.GetPrimPath());
        if (!property.FlattenTo(parent)) {
            _snapshot.reset();
            return false;
        }
        snapshot->patches[path] = std::move(patch);
    }
    snapshot->revision = _revision;
    snapshot->interpolation = stage->GetInterpolationType();
    _snapshot = std::move(snapshot);
    return true;
}

bool RigExecFallbackStageMirror::Evaluate(
    const std::shared_ptr<const RigExecFallbackStageSnapshot> &snapshot,
    UsdTimeCode time, RigExecRigPose *pose,
    const RigExecFallbackStageOptions &options)
{
    if (!snapshot || !snapshot->baseline || !pose) return false;
    std::lock_guard<std::mutex> lock(_mutex);
    // A newer queued job may reach this serialized mirror first. Never apply
    // an older snapshot on top of it (the scheduler also fences publication).
    if (_stage && snapshot->revision < _revision) return false;
    const auto failureToken = std::atomic_load(&_failures);
    const auto failed = [&]() {
        auto failures = std::make_shared<Failures>();
        if (failureToken && failureToken->revision == snapshot->revision &&
            SameOptions(failureToken->options, options)) {
            *failures = *failureToken;
        }
        failures->revision = snapshot->revision;
        failures->options = options;
        failures->times.insert(time.GetValue());
        auto expected = failureToken;
        std::atomic_compare_exchange_strong(
            &_failures, &expected, std::shared_ptr<const Failures>(failures));
        return false;
    };
    if (!_stage || _baseline != snapshot->baseline) {
        _evaluator.reset();
        _stage.Reset();
        const SdfLayerRefPtr root = SdfLayer::CreateAnonymous("fallback-worker.usda");
        root->TransferContent(snapshot->baseline);
        _stage = UsdStage::Open(root);
        if (!_stage) return failed();
        _baseline = snapshot->baseline;
        _revision = 0;
    }
    {
        // Only this worker owns the mutable root. Notice invalidation finishes
        // before the following pull, including edits revealing weaker values.
        SdfChangeBlock changes;
        const SdfLayerRefPtr root = _stage->GetRootLayer();
        for (const auto &[path, patch] : snapshot->patches) {
            if (patch.revision <= _revision) continue;
            if (!patch.removed) {
                SdfCreatePrimInLayer(root, path.GetPrimPath());
                if (!SdfCopySpec(patch.layer, path, root, path)) return failed();
            } else {
                const SdfPropertySpecHandle property = root->GetPropertyAtPath(path);
                const SdfPrimSpecHandle parent = root->GetPrimAtPath(path.GetPrimPath());
                if (parent && property) parent->RemoveProperty(property);
            }
        }
    }
    _revision = snapshot->revision;
    _stage->SetInterpolationType(snapshot->interpolation);
    if (_evaluator &&
        (_options.preferProgram != options.preferProgram ||
         _options.explicitMode != options.explicitMode)) {
        // Explicit mode cannot be relinquished through SetEvaluationMode.
        _evaluator.reset();
    }
    if (!_evaluator) {
        _evaluator = std::make_unique<RigExecRigEvaluator>(
            _stage, _rig, options.preferProgram);
        if (options.explicitMode) _evaluator->SetEvaluationMode(options.mode);
        if (!_evaluator->Compile()) {
            _evaluator.reset();
            return failed();
        }
    }
    if (options.explicitMode) _evaluator->SetEvaluationMode(options.mode);
    _evaluator->SetPublishWeightFields(options.publishWeightFields);
    _evaluator->SetSolverGuidesEnabled(options.solverGuidesEnabled);
    _options = options;
    *pose = _evaluator->Evaluate(time);
    return pose->valid ? true : failed();
}

} // namespace rigExec
