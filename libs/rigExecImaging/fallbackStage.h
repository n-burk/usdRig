// Immutable scene capture and privately owned execution for background warming.
#ifndef RIGEXEC_IMAGING_FALLBACK_STAGE_H
#define RIGEXEC_IMAGING_FALLBACK_STAGE_H

#include "rigExec/rigEvaluator.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/notice.h"
#include <memory>
#include <map>
#include <mutex>
#include <set>
#include <vector>

namespace rigExec {

struct RigExecFallbackStagePatch {
    SdfLayerRefPtr layer;
    bool removed = false;
    uint64_t revision = 0;
};

// Layers are private captures, immutable after publication; no source stage
// or source-layer handle crosses into a job.
struct RigExecFallbackStageSnapshot {
    SdfLayerRefPtr baseline;
    // Latest complete opinion per property, bounded by edited property count
    // rather than the number of gestures. Undo replaces an entry too.
    std::map<SdfPath, RigExecFallbackStagePatch> patches;
    uint64_t revision = 0;
    UsdInterpolationType interpolation = UsdInterpolationTypeLinear;
};

// Called only on the source stage's owner thread, under registry ownership.
class RigExecFallbackStageSource {
public:
    explicit RigExecFallbackStageSource(const UsdStageRefPtr &stage);
    bool CaptureChanges(const UsdStageRefPtr &stage,
                        const UsdNotice::ObjectsChanged &notice);
    std::shared_ptr<const RigExecFallbackStageSnapshot> GetSnapshot() const {
        return _snapshot;
    }
private:
    bool _CaptureBaseline(const UsdStageRefPtr &stage);
    uint64_t _revision = 0;
    std::shared_ptr<const RigExecFallbackStageSnapshot> _snapshot;
};

// Captured from the live evaluator on its owner thread. Jobs do not consult
// the live bridge to resolve presentation fields or an explicitly chosen mode.
struct RigExecFallbackStageOptions {
    bool preferProgram = false;
    bool publishWeightFields = true;
    bool solverGuidesEnabled = true;
    bool explicitMode = false;
    RigExecEvaluationMode mode = RigExecEvaluationMode::Dynamic;
};

// Only worker threads enter Evaluate. The UI never acquires this mutex or
// accesses the private stage/evaluator. Shared ownership lets jobs outlive
// their session without holding the source stage or imaging bridge.
class RigExecFallbackStageMirror {
public:
    explicit RigExecFallbackStageMirror(const SdfPath &rig) : _rig(rig) {}
    bool Evaluate(const std::shared_ptr<const RigExecFallbackStageSnapshot> &snapshot,
                  UsdTimeCode time, RigExecRigPose *pose,
                  const RigExecFallbackStageOptions &options = {});
    // Atomic status read; never takes the private evaluator's mutex. A new
    // source revision or changed evaluation options permits another attempt.
    bool HasFailed(uint64_t revision, UsdTimeCode time,
                   const RigExecFallbackStageOptions &options) const;
    void ClearFailures() {
        // A unique empty token also fences a failure still being evaluated;
        // storing null would permit an old null token to match after reset.
        std::atomic_store(&_failures, std::make_shared<const Failures>());
    }
private:
    struct Failures {
        uint64_t revision = 0;
        RigExecFallbackStageOptions options;
        std::set<double> times;
    };
    SdfPath _rig;
    std::mutex _mutex;
    SdfLayerRefPtr _baseline;
    UsdStageRefPtr _stage;
    std::unique_ptr<RigExecRigEvaluator> _evaluator;
    uint64_t _revision = 0;
    RigExecFallbackStageOptions _options;
    std::shared_ptr<const Failures> _failures;
};

} // namespace rigExec
#endif
