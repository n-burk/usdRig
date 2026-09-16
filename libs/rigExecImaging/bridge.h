//
// RigExec Hydra publication bridge (spec §8.2 generation-fenced flow).
//
// Owns the evaluator-side of the imaging chain: evaluation completes
// first, then the complete immutable generation is atomically published
// to the snapshot store, and only then are precise dirtied notices sent.
// Hydra pulls never compute or wait.
//
#ifndef RIGEXEC_IMAGING_BRIDGE_H
#define RIGEXEC_IMAGING_BRIDGE_H

#include "sceneIndices.h"
#include "snapshotStore.h"

#include "rigExec/rigEvaluator.h"

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rigExec {

/// Drives one rig's evaluation into the imaging chain.
class RigExecImagingBridge {
public:
    RigExecImagingBridge(
        const UsdStageRefPtr &stage, const SdfPath &rigPath);

    /// Constructs over an externally owned store (the application may
    /// create the store before any scene index or bridge exists).
    RigExecImagingBridge(
        const UsdStageRefPtr &stage, const SdfPath &rigPath,
        std::shared_ptr<RigExecSnapshotStore> store);

    /// The outcome of one evaluate-and-publish step, for callers that
    /// broadcast notices to multiple filter chains.
    struct PublishResult {
        bool ok = false;
        RigExecPublishedDirtyVector dirtied;
        /// Non-null when the binding epoch changed this generation.
        RigExecBindingResolvingSceneIndex::BindingEpochConstPtr epoch;
    };

    /// The renderer's HdSceneIndexCreateArgsSchema.motionBlurSupport
    /// capability bit (spec §10.3.1): false permits one sample, true
    /// allows the explicit render-preflight sample set, and when absent
    /// the application's explicit render profile is authoritative.
    enum class MotionBlurSupport { Absent, False, True };

    /// Validates a render-motion profile against the renderer capability
    /// bit before any evaluation (spec §10.3.1, §10.5): a multi-sample
    /// profile fails preflight when the renderer advertises
    /// motionBlurSupport = false, and an empty offset set never renders.
    static bool PreflightMotionProfile(
        const std::vector<float> &shutterOffsets,
        MotionBlurSupport support,
        std::string *whyNot = nullptr);

    /// Evaluate-then-publish without notifying any scene index; the
    /// caller forwards the result to its chains.
    PublishResult EvaluateAndPublishResult(UsdTimeCode time);

    /// Render-motion publication: evaluates the rig at
    /// baseTime + offset for every explicit frame-relative shutter offset,
    /// retains points, normals, extents and driven transforms per offset,
    /// and publishes them under one complete generation fence. Diagnostic
    /// guides use the offset nearest zero. Preflight (capability check and offset
    /// validation) fails before any evaluation; an incomplete sample set
    /// never publishes.
    PublishResult EvaluateAndPublishSamples(
        UsdTimeCode baseTime, const std::vector<float> &shutterOffsets,
        MotionBlurSupport support = MotionBlurSupport::Absent);

    /// Compiles the rig; returns false with messages on failure.
    bool Compile(std::vector<std::string> *errors = nullptr);

    /// The store the results scene index reads from.
    const std::shared_ptr<RigExecSnapshotStore> &GetStore() const {
        return _store;
    }

    /// The reserved generated scope this rig owns (for pruning).
    SdfPath GetGeneratedScope() const;

    size_t GetBindingEpochDigest() const {
        return _evaluator->GetBindingEpochDigest();
    }

    /// Wires the filters that receive epoch swaps and generation notices.
    void SetSceneIndices(
        const RigExecBindingResolvingSceneIndexRefPtr &binding,
        const RigExecResultsSceneIndexRefPtr &results) {
        _binding = binding;
        _results = results;
    }

    /// Serialized evaluate-then-publish for one explicit time
    /// (spec §8.2): compute the complete generation, atomically swap the
    /// snapshot, then send coalesced dirtied notices.
    bool EvaluateAndPublish(UsdTimeCode time);

    /// Selects the weight object whose resolved field is painted onto its
    /// weighted geometry as the influence overlay; an empty path turns
    /// the overlay off.
    ///
    /// ONE at a time on purpose. Two gradients composited onto one mesh
    /// are a picture of neither field, and a rigger placing a volume is
    /// asking about exactly one of them.
    ///
    /// Takes effect on the next publication -- this only records the
    /// selection, because publishing is the caller's serialization point
    /// (the registry republishes at the current time right after).
    void SetWeightOverlay(const SdfPath &weightPrimPath) {
        _weightOverlay = weightPrimPath;
        // Resolving the per-point influence field is the overlay's whole
        // cost, and this bridge is its only consumer: pay it only while an
        // overlay is actually selected.
        _evaluator->SetPublishWeightFields(!_weightOverlay.IsEmpty());
    }

    const SdfPath &GetWeightOverlay() const { return _weightOverlay; }

    /// Uncommitted manipulation values for this rig, with nothing authored
    /// (RigExecRigEvaluator::SetInteractiveOverrides). Records only, like
    /// SetWeightOverlay: publishing is the caller's serialization point.
    void SetInteractiveOverrides(std::vector<RigExecValueOverride> overrides) {
        _evaluator->SetInteractiveOverrides(std::move(overrides));
    }

    void ClearInteractiveOverrides() {
        _evaluator->ClearInteractiveOverrides();
    }

    /// The stage the rig evaluates against, for a caller that has to read the
    /// authored value an override is standing in for.
    const UsdStageRefPtr &GetEvaluationStage() const {
        return _evaluator->GetEvaluationStage();
    }

private:
    /// Publishes guide payloads (joints and aggregate solvers draw as
    /// guide geometry like OpenExec's IrJointScope) into the snapshot.
    void _FillProviderXforms(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    void _FillGuides(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    /// Publishes the single synthesized guide shape each control draws at
    /// its posed frame (spec §10.3 extension).
    /// Records the stage identity and sample time on a generation.
    void _StampGeneration(
        UsdTimeCode time, RigExecImagingSnapshot *snapshot) const;

    void _FillControlGuides(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    /// Publishes the wire (or solid) falloffMin/falloffMax iso-surfaces
    /// every placed influence volume draws, so a rigger can see the shape
    /// being placed rather than infer it from the deformation.
    void _FillVolumeGuides(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    /// Samples native curvenets from their evaluated control-point pools.
    void _FillCurvenetGuides(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    /// Copies the selected weight object's resolved field onto the
    /// geometry prim it weights (spec §10.3 influence-overlay extension).
    void _FillWeightOverlay(
        const RigExecRigPose &pose,
        RigExecImagingSnapshot *snapshot) const;

    UsdStageRefPtr _stage;
    SdfPath _rigPath;
    /// Weight object currently painted as the influence overlay; empty
    /// means off, which is the default and the ordinary render.
    SdfPath _weightOverlay;
public:
    /// The evaluator's profiler, which the imaging layer also records its
    /// own publish phases into, so one summary covers a whole viewport
    /// update. Recording is off unless RIGEXEC_IMAGING_PROFILE is set.
    const RigExecProfiler &GetProfiler() const
    {
        return _evaluator->GetProfiler();
    }
    RigExecProfiler *MutableProfiler() const
    {
        return const_cast<RigExecProfiler *>(&_evaluator->GetProfiler());
    }
    /// The evaluation mode actually answering this rig (baked, dynamic...),
    /// for diagnostics a viewer shows. See RigExecRigEvaluator.
    const RigExecRigEvaluator &GetEvaluator() const { return *_evaluator; }

    /// Forgets every cached guide input. Called for any stage notice that
    /// touches the rig and on every recompile: the caches hold AUTHORED
    /// styling, and only an edit can move that.
    void InvalidateGuideCaches();

private:
    /// A prim's guide styling, read once and republished every generation.
    ///
    /// Measured on the biped: rebuilding the guides for ~400 joints and
    /// controls re-read every one of these off the stage on EVERY mouse
    /// move of a drag -- 14-16 ms of each move, more than the rig's own
    /// evaluation. They are authored values that do not move while a
    /// control is dragged, so each is read once. An attribute that CAN move
    /// between two generations -- connected (the opacity an IK/FK switch
    /// drives) or time-varying -- is marked live and read exactly as before.
    struct _GuideInputs {
        UsdPrim prim;
        bool styleReady = false;
        TfToken purpose;
        UsdAttribute colorAttr;
        bool colorLive = false;
        bool hasColor = false;
        GfVec3f color;
        UsdAttribute opacityAttr;
        bool opacityLive = false;
        bool hasOpacity = false;
        float opacity = 1.0f;
        bool controlReady = false;
        bool controlLive = false;
        TfToken shape;
        TfToken drawMode;
        GfVec3d scale;
        double wireWidth = 0.05;
        GfVec3d offset = GfVec3d(0.0);
        bool radiusReady = false;
        bool radiusLive = false;
        double radius = 1.0;
    };
    _GuideInputs &_GuideInputsFor(const SdfPath &path) const;
    void _ReadGuideStyleCached(_GuideInputs &inputs,
                               const RigExecRigPose &pose,
                               RigExecPublishedPrim *published) const;
    double _GuideRadius(_GuideInputs &inputs, UsdTimeCode time) const;
    const std::map<SdfPath, std::vector<SdfPath>> &
    _JointChildren(const RigExecRigPose &pose) const;

    std::unique_ptr<RigExecRigEvaluator> _evaluator;
    std::shared_ptr<RigExecSnapshotStore> _store;
    RigExecBindingResolvingSceneIndexRefPtr _binding;
    RigExecResultsSceneIndexRefPtr _results;
    uint64_t _generation = 0;
    size_t _publishedEpochDigest = 0;
    mutable std::unordered_map<SdfPath, _GuideInputs, SdfPath::Hash>
        _guideInputs;
    mutable std::map<SdfPath, std::vector<SdfPath>> _jointChildren;
    /// Joint path -> whether a RigExecControl is above it (through joints): the
    /// hidden pivots nested in a control hierarchy. Stage-edit scoped like
    /// the other guide caches.
    mutable std::unordered_map<SdfPath, bool, SdfPath::Hash>
        _controlSpaceJoints;
    bool _IsControlSpaceJoint(const SdfPath &path) const;
    mutable size_t _jointChildrenKey = 0;
    mutable bool _jointChildrenValid = false;
    /// The evaluator's stage-edit serial the guide caches were filled under.
    mutable uint64_t _guideCacheSerial = 0;
    /// Drops the guide caches if any stage edit has happened since they
    /// were filled. Called at the top of every fill that reads them.
    void _SyncGuideCaches() const;
};

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_BRIDGE_H
